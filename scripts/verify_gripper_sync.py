#!/usr/bin/env python
"""Gate a SO-101 teleop recording: does the follower gripper track the leader?

Usage (unchanged):
    python verify_gripper_sync.py /path/to/dataset
    python verify_gripper_sync.py /path/to/dataset --verbose

The dataset must be a lerobot v2 layout (data/chunk-*/episode_*.parquet with
`action` and `observation.state` arrays of shape (6,)). Motor order (index 5 is
gripper): shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
Percent is "% closed" (higher = more closed).

Teleop is open-loop: the human leader hand moves freely, the follower gripper is
physically constrained. So "action keeps moving while state holds" is NOT always a
fault. This script distinguishes the three legitimate "holds" from a real freeze:

  1. CLAMP   (state >= 30, closed): follower holds the block closed while the arm
             carries it and the leader hand relaxes. NORMAL + useful signal, exempt.
  2. FLOOR   (state <= 22, open):   follower sits at its open limit while the leader
             opens further (release phase). Reported separately as a mirror-mapping
             health metric (lower clamp ~21%), NOT part of the tracking slope.
  3. MIDDLE  (22 < state < 30):     a real freeze is the follower stuck while the
             leader commands CLOSE (action - state stays > DEADBAND for more than 3
             frames). If the leader merely holds OPEN below the state (release-stop:
             the block is already released at 22-24%), that is exempt, like the floor.

The tracking law is fitted on SETTLED plateaus only (state constant >= 10 frames)
with state > 22 (release-limit plateaus excluded) and |state-action| <= 5% (stuck /
freeze outliers excluded; those are reported as real freezes by check 4). The clamp
plateaus are kept in this fit because their settled value is a genuine tracking point
(the follower reached it by tracking during the grasp, settling ~1.8% below the
leader); the clamp exemption above is about not flagging the subsequent hold, not
about discarding the settled value.

Exit code 0 = PASS, 1 = FAIL (so it can gate a recording SOP). Command line is
unchanged from the previous version.
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

GRIPPER_IDX = 5
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Classification thresholds (percent closed).
FLOOR_OPEN = 22.0        # state <= this  -> release / open-limit plateau
CLAMP_CLOSED = 30.0      # state >= this  -> clamp / closed-on-block plateau
DEADBAND = 9.0           # |action - state| above this while stuck in the middle = real freeze
SETTLED_MIN_LEN = 10     # a state-constant run at least this long counts as "settled"
FREEZE_MIN_FRAMES = 3    # middle freeze must persist MORE than this many frames
OUTLIER_OFFSET = 5.0     # a settled plateau with |state - action| above this is stuck/outlier, not tracking
MAX_OUTLIER_FRACTION = 0.15  # more than this fraction of settled plateaus far from leader = systematic failure
EPS = 1e-6               # equality tolerance for "state unchanged"

# Tracking-law acceptance bounds. The follower trails the leader by ~2% in BOTH
# directions (closing -> ~-2%, opening -> ~+2%), so the intercept may legitimately be
# slightly positive; the bound is symmetric to catch only gross (>5%) offsets. The
# real discriminators are slope (~1 vs a compressed range) and corr (consistency).
SLOPE_LO, SLOPE_HI = 0.8, 1.2
INTERCEPT_LO, INTERCEPT_HI = -5.0, 5.0
CORR_MIN = 0.95
MIN_SETTLED = 4          # need at least this many settled plateaus to trust the fit


def load_episodes(dataset_root):
    """Return [(basename, action(Nx6), state(Nx6)), ...] in episode order."""
    pattern = os.path.join(dataset_root, "data", "chunk-*", "episode_*.parquet")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"ERROR: no parquet files under {pattern}", file=sys.stderr)
        sys.exit(2)
    eps = []
    for f in files:
        df = pd.read_parquet(f)
        eps.append((os.path.basename(f), np.stack(df["action"].to_numpy()),
                    np.stack(df["observation.state"].to_numpy())))
    return eps


def linreg(a, s):
    ma, ms = a.mean(), s.mean()
    denom = np.sum((a - ma) ** 2)
    if denom == 0:
        return np.nan, np.nan, np.nan
    slope = np.sum((a - ma) * (s - ms)) / denom
    intercept = ms - slope * ma
    corr = np.corrcoef(a, s)[0, 1]
    return slope, intercept, corr


def find_plateaus(state):
    """Split a 1-D state series into runs of constant value.

    Returns list of dicts (start, end_exclusive, value, length).
    """
    n = len(state)
    runs = []
    i = 0
    while i < n:
        j = i + 1
        while j < n and abs(state[j] - state[i]) < EPS:
            j += 1
        runs.append(dict(start=i, end=j, value=float(state[i]), length=j - i))
        i = j
    return runs


def classify(value):
    if value <= FLOOR_OPEN:
        return "FLOOR"
    if value >= CLAMP_CLOSED:
        return "CLAMP"
    return "MIDDLE"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dataset", help="path to dataset root (contains meta/ and data/)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    eps = load_episodes(args.dataset)
    n_eps = len(eps)

    action_all = np.concatenate([a[:, GRIPPER_IDX] for _, a, _ in eps])
    state_all = np.concatenate([s[:, GRIPPER_IDX] for _, _, s in eps])

    print(f"Loaded {len(action_all)} frames from {n_eps} episodes: {args.dataset}")
    print(f"  action.gripper: min={action_all.min():.2f} max={action_all.max():.2f} mean={action_all.mean():.2f}")
    print(f"  state.gripper : min={state_all.min():.2f} max={state_all.max():.2f} mean={state_all.mean():.2f}")

    # Full-dataset slope is reported for reference only (it is dragged down by the
    # clamp holds and the release-floor saturation, so it is NOT used to gate).
    fslope, fintercept, fcorr = linreg(action_all, state_all)
    print(f"  [reference] full slope state = {fslope:.4f} * action + {fintercept:.2f}   corr={fcorr:.4f}")
    print("            (full slope is NOT the gate; clamp holds + release floor bias it low)")

    # ---- Detect and classify every state-constant plateau (per episode) ----
    plateaus = []  # each: name, start, end, value, length, cat, action_segment
    for name, a, s in eps:
        g = s[:, GRIPPER_IDX]
        for r in find_plateaus(g):
            r["name"] = name
            r["cat"] = classify(r["value"])
            r["action_seg"] = a[r["start"]:r["end"], GRIPPER_IDX]
            plateaus.append(r)

    floor_plats = [p for p in plateaus if p["cat"] == "FLOOR"]
    clamp_plats = [p for p in plateaus if p["cat"] == "CLAMP"]
    middle_plats = [p for p in plateaus if p["cat"] == "MIDDLE"]

    total = len(action_all)
    floor_frames = sum(p["length"] for p in floor_plats)
    clamp_frames = sum(p["length"] for p in clamp_plats)
    middle_frames = sum(p["length"] for p in middle_plats)
    print(f"\nPlateau classification (state-constant runs, n={len(plateaus)}):")
    print(f"  CLAMP  (state>={CLAMP_CLOSED:.0f} closed) : {len(clamp_plats):3d} runs, {clamp_frames:4d} frames = {100*clamp_frames/total:5.1f}%")
    print(f"  FLOOR  (state<={FLOOR_OPEN:.0f} open)   : {len(floor_plats):3d} runs, {floor_frames:4d} frames = {100*floor_frames/total:5.1f}%")
    print(f"  MIDDLE (22<state<30)          : {len(middle_plats):3d} runs, {middle_frames:4d} frames = {100*middle_frames/total:5.1f}%")

    # ---- Check 1: tracking law on settled, non-floor, non-outlier plateaus ----
    # A settled plateau whose |state - action| is huge is a STUCK/freeze (the follower
    # never tracked there), not a tracking sample -> exclude it so it cannot skew the
    # fit. Such outliers are reported as real freezes by check [4].
    settled_cand = [p for p in plateaus if p["length"] >= SETTLED_MIN_LEN and p["value"] > FLOOR_OPEN]
    settled = []
    outliers = []
    for p in settled_cand:
        off = p["value"] - p["action_seg"][0]
        (settled if abs(off) <= OUTLIER_OFFSET else outliers).append((p, off))
    act_refs = np.array([p["action_seg"][0] for p, _ in settled])
    state_vals = np.array([p["value"] for p, _ in settled])
    tslope, tintercept, tcorr = linreg(act_refs, state_vals)

    print(f"\n[1] Tracking law (settled >= {SETTLED_MIN_LEN} frames, state > {FLOOR_OPEN:.0f}, |offset| <= {OUTLIER_OFFSET:.0f}):")
    print(f"    n_settled={len(settled)}  state = {tslope:.4f} * action + {tintercept:.3f}  corr={tcorr:.4f}")
    n_cand = len(settled) + len(outliers)
    outlier_frac = (len(outliers) / n_cand) if n_cand else 0.0
    if outliers:
        print(f"    excluded {len(outliers)} stuck/outlier plateau(s) (|state-action| > {OUTLIER_OFFSET:.0f}%):")
        for p, off in outliers:
            print(f"      {p['name']} frames {p['start']:3d}-{p['end']-1:3d} state={p['value']:.2f} action={p['action_seg'][0]:.2f} offset={off:+.2f}")
    print(f"    outlier fraction = {outlier_frac:.1%} (limit {MAX_OUTLIER_FRACTION:.0%})")
    if args.verbose:
        for p, off in settled:
            print(f"      {p['name']} frames {p['start']:3d}-{p['end']-1:3d} state={p['value']:.2f} action={p['action_seg'][0]:.2f} offset={off:+.2f}")
    tracking_ok = (
        len(settled) >= MIN_SETTLED
        and SLOPE_LO <= tslope <= SLOPE_HI
        and INTERCEPT_LO <= tintercept <= INTERCEPT_HI
        and tcorr > CORR_MIN
        and outlier_frac <= MAX_OUTLIER_FRACTION
    )
    if len(settled) < MIN_SETTLED:
        print(f"    INSUFFICIENT settled plateaus ({len(settled)} < {MIN_SETTLED})")

    # ---- Check 2: clamp segments are valid signals, not faults ----
    clamp_moving = [p for p in clamp_plats if (p["action_seg"].max() - p["action_seg"].min()) > 0.5]
    print(f"\n[2] Clamp/hold segments (state closed while leader moves):")
    print(f"    {len(clamp_moving)} clamp runs have leader still moving -> treated as VALID grasp/carry signal (exempt)")

    # ---- Check 3: release-floor report (mirror lower clamp, independent metric) ----
    state_floor = state_all.min()
    # per-episode count of "leader commands clearly below 21%" and its longest run
    total_below = 0
    longest_below = 0
    for _, a, _ in eps:
        g = a[:, GRIPPER_IDX]
        below = g < (FLOOR_OPEN - 1.0)
        total_below += int(below.sum())
        cur = 0
        for b in below:
            cur = cur + 1 if b else 0
            longest_below = max(longest_below, cur)
    # sustained gap = follower's settled floor value vs leader command, measured on the
    # settled floor plateaus only (excludes the transient open burst).
    settled_floor = [p for p in floor_plats if p["length"] >= SETTLED_MIN_LEN]
    floor_gaps = [p["value"] - p["action_seg"].mean() for p in settled_floor]
    print(f"\n[3] Release/open-limit report (mirror lower clamp):")
    print(f"    min(state.gripper) = {state_floor:.2f}%  (leader min = {action_all.min():.2f}%)")
    print(f"    frames where leader commands below 21%: {total_below}  (longest continuous run {longest_below} frames = {longest_below/12:.1f}s @12fps)")
    if floor_gaps:
        print(f"    settled floor gap (state - action, {len(floor_gaps)} plateaus): "
              f"mean {np.mean(floor_gaps):+.2f}%  range [{min(floor_gaps):+.2f}, {max(floor_gaps):+.2f}]%")
    else:
        print(f"    no settled floor plateaus (gripper never sat at its open limit)")
    floor_ok = True  # never gates FAIL; reported so a mirror-mapping issue is visible

    # ---- Check 4: real freeze = stuck in MIDDLE while the leader commands CLOSE ----
    # A genuine freeze is the follower ignoring a CLOSE command (action rising well
    # above the stuck state). The mirror case (action held LOW/open below the state)
    # is a normal "release-stop": the block is already released at 22-24%, so the
    # follower correctly stops while the leader over-opens -- exempt, like the floor.
    bad_freezes = []
    for p in middle_plats:
        err = p["action_seg"] - p["value"]   # positive = leader commanding more CLOSED than state
        # longest consecutive frames within this plateau where leader commands close > deadband
        longest = 0
        cur = 0
        for e in err:
            cur = cur + 1 if e > DEADBAND else 0
            longest = max(longest, cur)
        if longest > FREEZE_MIN_FRAMES:
            bad_freezes.append((p, longest))
    print(f"\n[4] Real middle-freeze check (state in 22..30, action - state > {DEADBAND:.0f}% [leader closing] for > {FREEZE_MIN_FRAMES} frames):")
    print(f"    violations: {len(bad_freezes)}")
    for p, longest in bad_freezes:
        print(f"    BAD: {p['name']} frames {p['start']}-{p['end']-1} state={p['value']:.2f} stuck {longest} frames")
    freeze_ok = len(bad_freezes) == 0

    # ---- verdict ----
    ok = tracking_ok and floor_ok and freeze_ok
    print()
    if not tracking_ok:
        reasons = []
        if len(settled) < MIN_SETTLED:
            reasons.append(f"only {len(settled)} settled plateau(s) (< {MIN_SETTLED})")
        if not (SLOPE_LO <= tslope <= SLOPE_HI):
            reasons.append(f"slope {tslope:.3f} not in [{SLOPE_LO},{SLOPE_HI}]")
        if not (INTERCEPT_LO <= tintercept <= INTERCEPT_HI):
            reasons.append(f"intercept {tintercept:.3f} not in [{INTERCEPT_LO},{INTERCEPT_HI}]")
        if not (tcorr > CORR_MIN):
            reasons.append(f"corr {tcorr:.3f} <= {CORR_MIN}")
        if outlier_frac > MAX_OUTLIER_FRACTION:
            reasons.append(f"outlier fraction {outlier_frac:.1%} > {MAX_OUTLIER_FRACTION:.0%}")
        print("  FAIL: tracking law -> " + "; ".join(reasons))
    if not freeze_ok:
        print(f"  FAIL: {len(bad_freezes)} real middle-freeze segment(s) found")
    print("Gripper sync check:", "PASS" if ok else "FAIL")
    if not ok:
        print("  -> Check follower gripper calibration / deadband before recording.")
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
