#!/usr/bin/env python3
"""
identify_arm_ports.py — 实测判定主臂(握柄/teleop) vs 从臂(执行/robot) 各挂在哪个 ttyACM。

用法（lerobot 环境）:
    python3 identify_arm_ports.py [--seconds 60] [--only-baseline]

原理:
    两个臂的电机都是 ID 1..6 (sts3215)。分别对 /dev/ttyACM0 和 /dev/ttyACM1
    建立 FeetechMotorsBus 连接，读 6 个关节的 Present_Position（raw，不归一化）。
    用户大幅掰动「主臂握柄/扳机」= 第 6 关节 (gripper)，哪个端口的第 6 关节
    raw 值发生大幅变化，那个端口就是主臂(leader/teleop)；另一个就是从臂(follower/robot)。
"""
import argparse
import sys
import time

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]


def connect(port):
    bus = FeetechMotorsBus(port=port, motors=MOTORS, calibration={})
    bus.connect()
    return bus


def read_all(bus):
    out = {}
    for name in MOTORS:
        out[name] = bus.read("Present_Position", name, normalize=False)
    return out


def fmt_vals(vals):
    return " ".join(f"{k}={vals[k]}" for k in MOTORS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=60, help="监控时长（秒）")
    ap.add_argument("--only-baseline", action="store_true", help="只读一次基线后退出")
    args = ap.parse_args()

    buses = {}
    for p in PORTS:
        try:
            buses[p] = connect(p)
            print(f"[OK] 已连接 {p}", flush=True)
        except Exception as e:
            print(f"[FAIL] 连接 {p} 失败: {e!r}", flush=True)

    if not buses:
        print("两个端口都连不上，检查供电/接线。", file=sys.stderr)
        return 1

    # 基线：连读 3 次取最后一次稳定值
    baseline = {}
    for _ in range(3):
        for p, bus in buses.items():
            try:
                baseline[p] = read_all(bus)
            except Exception as e:
                print(f"[WARN] 读 {p} 基线失败: {e!r}", flush=True)
        time.sleep(0.2)

    print("\n=== 基线 Present_Position (raw) ===", flush=True)
    for p, vals in baseline.items():
        print(f"{p}: {fmt_vals(vals)}", flush=True)
    print("(关节顺序: shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper)",
          flush=True)

    if args.only_baseline:
        for p, bus in buses.items():
            try:
                bus.disconnect(disable_torque=False)
            except Exception:
                pass
        return 0

    print(f"\n=== 开始监控 {args.seconds}s，请现在大幅掰动主臂(握柄)的握柄/扳机(第6关节) ===",
          flush=True)
    print("时间戳 | ttyACM0 第6关节(变化量) | ttyACM1 第6关节(变化量)", flush=True)

    max_delta = {p: 0.0 for p in PORTS}
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        row = time.strftime("%H:%M:%S")
        for p in PORTS:
            if p not in buses:
                row += " | (未连接)"
                continue
            try:
                cur = read_all(buses[p])
                d6 = cur["gripper"] - baseline[p]["gripper"]
                max_delta[p] = max(max_delta[p], abs(d6))
                row += f" | j6={cur['gripper']:>4} (Δ{d6:+})"
            except Exception as e:
                row += f" | 读失败:{e!r}"
        print(row, flush=True)
        time.sleep(0.3)

    print("\n=== 监控结束，第6关节(gripper)最大变化量 ===", flush=True)
    for p in PORTS:
        print(f"{p}: max|Δj6| = {max_delta[p]:.0f}", flush=True)

    # 判定
    active = [p for p in PORTS if p in buses and max_delta[p] >= 50]
    if len(active) == 1:
        master = active[0]
        slave = [p for p in buses if p != master][0]
        print(f"\n[判定] 主臂(leader/teleop) = {master}", flush=True)
        print(f"[判定] 从臂(follower/robot) = {slave}", flush=True)
    elif len(active) == 0:
        print("\n[判定] 两个端口第6关节都没动（>=50 raw），无法判定。请重新跑并实际掰动握柄。",
              flush=True)
    else:
        print(f"\n[判定] 两个端口都动了（{active}），异常，检查是否两臂联动或接线混乱。",
              flush=True)

    for p, bus in buses.items():
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
