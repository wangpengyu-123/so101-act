#!/usr/bin/env python3
"""给总线上唯一的一颗舵机设 ID（无需官方交互式命令，逐颗设）。

用法（先 source ~/miniconda3/bin/activate lerobot）：
  python set_one_motor.py --port /dev/ttyACM0 --id 1     # 把当前总线上唯一那颗设为 ID1(shoulder_pan)
  python set_one_motor.py --port /dev/ttyACM0 --id 6     # 设为 ID6(gripper)

前提：总线上**只接了一颗**舵机（其他串联线都拔掉），这是 Feetech 总线的物理约束。
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
    # 前 5 关节 [-100,100]，gripper(6) [0,100]（只影响归一化，写 ID 无关）
    return MotorNormMode.RANGE_0_100 if id_ == 6 else MotorNormMode.RANGE_M100_100


def main():
    p = argparse.ArgumentParser(description="给总线上唯一一颗舵机设 ID")
    p.add_argument("--port", required=True, help="如 /dev/ttyACM0")
    p.add_argument("--id", type=int, required=True, choices=range(1, 7),
                   help="目标 ID：1=shoulder_pan 2=shoulder_lift 3=elbow_flex 4=wrist_flex 5=wrist_roll 6=gripper")
    args = p.parse_args()

    name = NAME[args.id]
    motors = {n: Motor(i, "sts3215", _norm_mode(i)) for i, n in NAME.items()}
    bus = FeetechMotorsBus(port=args.port, motors=motors)

    # setup_motor：自动扫描总线唯一那颗 → 解锁 EEPROM → 写 ID → 写波特率
    bus.setup_motor(name)
    print(f"[OK] 总线上唯一那颗舵机已设为 '{name}' = ID {args.id}")
    # 只关串口，不写 torque（舵机 ID 已改，旧 ID 列表会找不到设备报假错）
    bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
