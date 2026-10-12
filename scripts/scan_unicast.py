#!/usr/bin/env python3
"""单播扫描：逐个 ID 点名读位置，验证总线上哪些舵机已串联应答。

为什么用它而不是 --scan：
  STS3215 是半双工总线，广播 ping 会让多颗舵机同时应答 → 信号冲突 → "NOTHING found"。
  单颗时广播没问题，串联 ≥2 颗后必须用单播（逐个 ID 读）才不会冲突。

用法（先 source ~/miniconda3/bin/activate lerobot）：
  python scan_unicast.py --port /dev/ttyACM0
"""
import argparse

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

NAME = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def _norm_mode(id_: int) -> MotorNormMode:
    return MotorNormMode.RANGE_0_100 if id_ == 6 else MotorNormMode.RANGE_M100_100


def main():
    p = argparse.ArgumentParser(description="单播扫描总线上已应答的舵机")
    p.add_argument("--port", required=True, help="如 /dev/ttyACM0")
    args = p.parse_args()

    motors = {n: Motor(i, "sts3215", _norm_mode(i)) for i, n in NAME.items()}
    bus = FeetechMotorsBus(port=args.port, motors=motors)
    bus.connect(handshake=False)

    print(f"=== 单播逐 ID 扫描 {args.port} ===")
    found = []
    for name, m in motors.items():
        try:
            pos = bus.read("Present_Position", name, normalize=False)
            found.append(m.id)
            print(f"  ID {m.id} ({name}): ✅ 应答, 位置原始={pos}")
        except Exception:
            print(f"  ID {m.id} ({name}): ❌ 无应答")

    print(f"\n在线 ID: {sorted(found)}（共 {len(found)}/6 颗）")
    bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
