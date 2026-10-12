#!/usr/bin/env python3
"""
verify_dual_camera.py — round2 双相机（ASC60C global + W19 wrist）开录前自检。

观测项（不猜，全部实测打印）：
  1. global  /dev/video10（v4l2loopback，由 ros2_camera_bridge.py 写入）能否打开、
     能否按 lerobot 配置设 fps=12 并通过校验、尺寸 640x480。
  2. wrist   W19 by-id 稳定路径能否打开、四色编码、尺寸、原生 fps（不设 CAP_PROP_FPS，
     因为 W19 忽略该设置、lerobot _validate_fps 会崩）。
  3. 双相机**同时**交错读各 N 帧：帧率 / 尺寸 / 有无互斥。
  4. 各存一帧到 --outdir，供人眼确认构图（global 看全局、wrist 看夹爪+物块）。

用法：
  python verify_dual_camera.py                # 默认设备 + 默认输出 /tmp
  python verify_dual_camera.py --wrist /dev/video4 --outdir /tmp

前置：global 这路需要桥接已在写（ros2_camera_bridge.py 合成帧或 ROS 帧均可）。
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import cv2


def fourcc_str(v: int) -> str:
    v = int(v)
    if v == 0:
        return "NONE"
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def open_global(path: str, fps: int):
    """按 lerobot 配置打开 global（fps=12 需通过校验）。"""
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, "open failed"
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, float(fps))
    got = cap.get(cv2.CAP_PROP_FPS)
    ok = math.isclose(fps, got, rel_tol=1e-3)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    info = f"{fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))} {w}x{h} fps_set={fps}->{got:.2f} isclose={ok}"
    return cap, info


def open_wrist(path: str):
    """按 lerobot 配置打开 wrist（不设 fps，设 640x480）。"""
    cap = cv2.VideoCapture(path, cv2.CAP_ANY)  # by-id 路径走 CAP_ANY（实测解析到 V4L2 backend）
    if not cap.isOpened():
        return None, "open failed"
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    info = f"{fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))} {w}x{h} native_fps={cap.get(cv2.CAP_PROP_FPS):.2f}"
    return cap, info


def read_n(cap, n: int):
    ok = 0
    shape = None
    for _ in range(n):
        ret, f = cap.read()
        if ret and f is not None:
            ok += 1
            shape = f.shape
    return ok, shape


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--global-dev", default="/dev/video10", dest="global_dev",
                    help="global 相机设备（ASC60C 桥接）")
    ap.add_argument("--wrist", default="/dev/v4l/by-id/<W19相机by-id路径>",
                    help="wrist 相机设备（W19 by-id 稳定路径）")
    ap.add_argument("--fps", type=int, default=12, help="global 相机 dict 里的 fps（默认 12）")
    ap.add_argument("--outdir", default="/tmp", help="存帧目录（global_check.png / wrist_check.png）")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("[1] global (ASC60C /dev/video10)")
    g, ginfo = open_global(args.global_dev, args.fps)
    if g is None:
        print(f"  FAIL: {ginfo}  (桥接没起？先跑 ros2_camera_bridge.py)")
        return
    gok, gshape = read_n(g, 5)
    print(f"  {ginfo}")
    print(f"  warmup read: {gok}/5 ok, shape={gshape}")

    print("=" * 60)
    print("[2] wrist (W19)")
    w, winfo = open_wrist(args.wrist)
    if w is None:
        print(f"  FAIL: {winfo}")
        g.release()
        return
    wok, wshape = read_n(w, 5)
    print(f"  {winfo}")
    print(f"  warmup read: {wok}/5 ok, shape={wshape}")

    print("=" * 60)
    print("[3] 双相机同时交错读 40 次")
    NF = 40
    g_ok = w_ok = 0
    g_shapes, w_shapes = set(), set()
    g_last = w_last = None
    t0 = time.time()
    for _ in range(NF):
        rg, fg = g.read()
        rw, fw = w.read()
        if rg and fg is not None:
            g_ok += 1
            g_shapes.add(fg.shape)
            g_last = fg
        if rw and fw is not None:
            w_ok += 1
            w_shapes.add(fw.shape)
            w_last = fw
    dt = time.time() - t0
    print(f"  global: {g_ok}/{NF} ok, shapes={g_shapes}, rate={g_ok / dt:.1f} fps")
    print(f"  wrist : {w_ok}/{NF} ok, shapes={w_shapes}, rate={w_ok / dt:.1f} fps")
    print(f"  (耗时 {dt:.2f}s)")

    ok = True
    if g_ok < NF or w_ok < NF:
        print("  FAIL: 有相机丢帧，检查桥接/USB/带宽")
        ok = False
    if (480, 640, 3) not in g_shapes or (480, 640, 3) not in w_shapes:
        print("  FAIL: 尺寸不是 640x480x3")
        ok = False

    print("=" * 60)
    print("[4] 存帧确认构图")
    if g_last is not None:
        gp = outdir / "global_check.png"
        cv2.imwrite(str(gp), g_last)
        print(f"  global -> {gp}")
    if w_last is not None:
        wp = outdir / "wrist_check.png"
        cv2.imwrite(str(wp), w_last)
        print(f"  wrist  -> {wp}")

    g.release()
    w.release()

    print("=" * 60)
    if ok:
        print("双相机自检 PASS：可开录。")
    else:
        print("双相机自检 FAIL：先排查再开录。")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
