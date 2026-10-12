#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SO-ARM101 单舵机测试工具（装机前用）

复用 lerobot 0.3.4 的 Feetech 驱动（FeetechMotorsBus），只做最小测试：
  - --scan            扫描总线上所有舵机 ID 并读回位置
  - --id N --read     读单个舵机当前位置（附电压/温度/电流/负载）
  - --id N --step S   对单个舵机发相对小步进 S（前后各读一次位置，观察转向）
  - --id N --torque on|off   开关力矩（用于「手顶有力 / 自由转动顺滑」测试）
  - --arm-type {leader,follower}  指定测哪条臂（主臂/从臂各自都是 ID 1~6）
  - --dry-run         无硬件也能跑通 import + 参数解析 + 打印

双臂说明（SO-ARM101 DIY 散件）：
  主臂 Leader   = HX-10HM × 6，lerobot 类型 so101_leader，独立 USB 端口
  从臂 Follower = HX-30HM × 6，lerobot 类型 so101_follower，独立 USB 端口
  两条臂的舵机 ID 各自都是 1~6（gripper=6 / shoulder_pan=1），分开接各自端口分别测试。

单位说明：
  位置/步进都使用舵机的【原始编码单位】0..4095（sts3215 12bit 编码器，一整圈 360°）。
  1 个原始单位 ≈ 360/4096 ≈ 0.088°（在输出轴/舵盘上）。默认步进 30 ≈ 2.6°。
  归一化值（前 5 关节 [-100,100]、gripper [0,100]）需要先做关节标定（lerobot-calibrate），
  本脚本不依赖标定，因此一律用原始单位。

运行环境：
  source ~/miniconda3/bin/activate lerobot
  python ~/so101_hardware/servo_test.py --dry-run
"""

import argparse
import sys
import time

# ---------------------------------------------------------------------------
# 文件顶部常量（可用命令行参数覆盖）
# ---------------------------------------------------------------------------
DEFAULT_PORT = "/dev/ttyUSB0"       # 默认串口（真机接线后 ls /dev/ttyUSB* 或 /dev/ttyACM* 确认）
DEFAULT_BAUDRATE = 1_000_000        # 默认波特率 1M（lerobot 驱动 default_baudrate）
DEFAULT_STEP = 30                   # 默认相对步进（原始单位，约 2.6°）
DEFAULT_SETTLE_S = 0.5              # 步进后等待电机到位的时间（秒）
FULL_RESOLUTION = 4096              # sts3215 满量程 0..4095

# ID -> 关节名（与 lerobot so101_follower 的 motors 定义一致，顺序不能乱）
JOINT_NAME_BY_ID = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

# 型号号 -> 型号名（feetech/tables.py 的 MODEL_NUMBER_TABLE）
MODEL_NUMBER_TO_MODEL = {
    777: "sts3215",
    2825: "sts3250",
    11272: "sm8512bl",
    1284: "scs0009",
}

# 双臂类型（--arm-type 用）。两条臂舵机 ID 都是 1~6，仅电机型号/用途不同。
ARM_TYPES = {
    "leader": {
        "label": "主臂 Leader",
        "motor": "HX-10HM（低减速比/顺滑）",
        "lerobot_type": "so101_leader",
    },
    "follower": {
        "label": "从臂 Follower",
        "motor": "HX-30HM（高减速比/有劲）",
        "lerobot_type": "so101_follower",
    },
}

# ---------------------------------------------------------------------------
# 驱动导入（try/except 包住，方便 --dry-run 报告导入状态）
# ---------------------------------------------------------------------------
try:
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    DRIVER_OK = True
    DRIVER_IMPORT_ERROR = None
except Exception as exc:  # noqa: BLE001
    DRIVER_OK = False
    DRIVER_IMPORT_ERROR = exc


def _name_for_id(id_: int) -> str:
    return JOINT_NAME_BY_ID.get(id_, f"id_{id_}")


def _norm_mode_for_id(id_: int) -> MotorNormMode:
    # 前 5 关节 [-100,100]，gripper(6) [0,100]。仅影响归一化，本脚本读原始值不受影响。
    if id_ == 6:
        return MotorNormMode.RANGE_0_100
    return MotorNormMode.RANGE_M100_100


def _build_bus(port: str, ids: list[int]) -> FeetechMotorsBus:
    """按给定 ID 列表构造 bus（全部按 sts3215 协议处理）。"""
    motors = {}
    for i in ids:
        motors[_name_for_id(i)] = Motor(i, "sts3215", _norm_mode_for_id(i))
    return FeetechMotorsBus(port=port, motors=motors)


def _single_bus(port: str, id_: int) -> FeetechMotorsBus:
    return _build_bus(port, [id_])


def _arm_header(args) -> str:
    """根据 --arm-type 返回一行臂类型描述，用于各命令输出头部。"""
    if args.arm_type in ARM_TYPES:
        info = ARM_TYPES[args.arm_type]
        return f"臂类型: {info['label']}（{info['motor']}，lerobot 类型 {info['lerobot_type']}）"
    return "臂类型: 未指定（leader=主臂 HX-10HM / follower=从臂 HX-30HM，两条臂舵机 ID 均 1~6）"


def _print_motor_row(id_: int, name: str, raw: int) -> None:
    deg = raw / FULL_RESOLUTION * 360.0
    print(f"  ID {id_:<2}  {name:<14}  位置原始={raw:>4} (0..4095)  ≈{deg:>7.2f}°")


def _read_health(bus: FeetechMotorsBus, name: str) -> dict:
    regs = ["Present_Voltage", "Present_Temperature", "Present_Current", "Present_Load"]
    out = {}
    for reg in regs:
        try:
            out[reg] = bus.read(reg, name, normalize=False)
        except Exception:  # noqa: BLE001
            out[reg] = None
    return out


def _print_health(d: dict) -> None:
    v = d.get("Present_Voltage")
    t = d.get("Present_Temperature")
    c = d.get("Present_Current")
    load = d.get("Present_Load")
    volts = f"{v / 10:.1f}V" if v is not None else "N/A"     # 0.1V/单位
    temp = f"{t}°C" if t is not None else "N/A"              # 1°C/单位
    cur = f"{c}mA" if c is not None else "N/A"               # 1mA/单位
    load_s = f"{load}" if load is not None else "N/A"        # 0.1%/单位, ±1000
    print(f"  电压={volts}  温度={temp}  电流={cur}  负载(原始)={load_s}")
    print("  （空载时电流/负载应很小；若电流明显偏高或异响，立即断电检查）")


# ---------------------------------------------------------------------------
# 各子命令
# ---------------------------------------------------------------------------
def cmd_scan(args) -> int:
    print(f"扫描串口 {args.port} ...")
    print(_arm_header(args))

    # 先按默认波特率快扫
    bus = FeetechMotorsBus(port=args.port, motors={})
    found = {}
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(args.baudrate)
        ids_models = bus.broadcast_ping()
        if ids_models:
            found[args.baudrate] = sorted(ids_models.keys())
    finally:
        if bus.is_connected:
            bus.port_handler.closePort()

    if not found:
        print(f"默认波特率 {args.baudrate} 未发现舵机，改用全波特率扫描（约 10~15 秒，请等待）...")
        found = FeetechMotorsBus.scan_port(args.port)  # {baudrate: [ids]}

    if not found:
        print("✗ 未发现任何舵机。请检查：")
        print("  1) 供电是否接好（舵机控制板电源）")
        print("  2) USB 线、3pin 数据线是否插对")
        print("  3) 串口路径是否正确（ls /dev/ttyUSB* /dev/ttyACM*）")
        print("  4) 串口权限（如无权限：sudo chmod 666 <串口>）")
        return 1

    print(f"共发现 {sum(len(v) for v in found.values())} 个舵机（分属 {len(found)} 个波特率）。")
    for baudrate in sorted(found):
        ids = sorted(found[baudrate])
        print(f"\n=== 波特率 {baudrate} @ {args.port}：{len(ids)} 个舵机 ===")
        for i in ids:
            print(f"  ID {i:<2} → {_name_for_id(i)}")
        print("  读回位置：")
        _read_positions(args.port, baudrate, ids)

    print("\n提示：全链 6 个 ID 应为 1~6 互不重复、全部应答。若 ID 数量不对或重复，回到单舵机逐个核对。")
    return 0


def _read_positions(port: str, baudrate: int, ids: list[int]) -> None:
    bus = _build_bus(port, ids)
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(baudrate)
        positions = bus.sync_read("Present_Position", normalize=False)
        for i in sorted(ids):
            name = _name_for_id(i)
            model_nb = bus.ping(i)
            model = MODEL_NUMBER_TO_MODEL.get(model_nb, f"型号号={model_nb}")
            raw = positions[name]
            deg = raw / FULL_RESOLUTION * 360.0
            print(f"    ID {i:<2}  {name:<14}  型号={model:<14}  位置原始={raw:>4}  ≈{deg:>7.2f}°")
    finally:
        if bus.is_connected:
            bus.port_handler.closePort()


def cmd_read(args) -> int:
    name = _name_for_id(args.id)
    bus = _single_bus(args.port, args.id)
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(args.baudrate)
        raw = bus.read("Present_Position", name, normalize=False)
        print(f"ID {args.id} ({name}) @ {args.port} 波特率 {args.baudrate}")
        print(_arm_header(args))
        _print_motor_row(args.id, name, raw)
        _print_health(_read_health(bus, name))
    finally:
        if bus.is_connected:
            # 读位置不改变力矩状态（避免把之前 --torque on 的保持力关掉）
            bus.disconnect(disable_torque=False)
    return 0


def cmd_step(args) -> int:
    name = _name_for_id(args.id)
    step = args.step
    bus = _single_bus(args.port, args.id)
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(args.baudrate)

        # 确保处于位置模式（EPROM 写入需先关力矩）
        bus.disable_torque()
        bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
        bus.enable_torque()

        before = bus.read("Present_Position", name, normalize=False)
        target = before + step
        clamped = False
        if target < 0 or target > FULL_RESOLUTION - 1:
            clamped = True
            target = max(0, min(FULL_RESOLUTION - 1, target))

        print(f"ID {args.id} ({name}) @ {args.port} 波特率 {args.baudrate}")
        print(_arm_header(args))
        print(f"  步进 {step:+d} 原始单位（≈{step / FULL_RESOLUTION * 360:+.2f}°），位置 {before} → {target}"
              + ("（已钳位到 0..4095 边界）" if clamped else ""))
        if abs(step) > 200:
            print(f"  ⚠ 步进 |{step}| > 200（≈{abs(step) / FULL_RESOLUTION * 360:.1f}°）偏大，确认输出轴附近无障碍物再执行！")
        if args.id == 6:
            print("  ℹ gripper（夹爪）建议用小步进（如 --step 15），且全程空载、别装桨板。")

        bus.write("Goal_Position", name, target, normalize=False)
        time.sleep(args.settle)
        after = bus.read("Present_Position", name, normalize=False)
        delta = after - before
        deg_delta = delta / FULL_RESOLUTION * 360.0

        print(f"  结果: 前={before}  后={after}  实际位移={delta:+d}  (≈{deg_delta:+.2f}°)")
        if step == 0:
            print("  步进为 0，未移动（仅验证读写正常）。")
        elif delta > 0:
            print("  转向: 原始值增大（正方向）。请对照实物记下「+step 使输出轴朝哪边转」，填入对照表。")
        elif delta < 0:
            print("  转向: 原始值减小（负方向）。请对照实物记下方向。")
        else:
            print("  转向: 位置未变化。检查：力矩是否开启、是否被硬限位顶住、供电是否足够。")
    finally:
        if bus.is_connected:
            bus.disconnect()
    return 0


def cmd_torque(args) -> int:
    name = _name_for_id(args.id)
    bus = _single_bus(args.port, args.id)
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(args.baudrate)

        if args.torque == "on":
            bus.disable_torque()
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            bus.enable_torque()
            print(f"ID {args.id} ({name}): 力矩已开启，正保持当前位置。")
            print(_arm_header(args))
            if args.arm_type == "leader":
                print("  → HX-10HM 主臂：低减速比，保持力矩偏小但应能稳定保持；手顶应有抵抗力。")
            elif args.arm_type == "follower":
                print("  → HX-30HM 从臂：高减速比，手顶应有明显力矩、能顶住。")
            else:
                print("  → 用手顶一下输出轴：HX-30HM 应有明显力矩、能顶住；若顶不动/电流异响，检查限位与安装。")
            bus.disconnect(disable_torque=False)
            print("  （力矩状态保持到断电为止；如需释放再跑 --torque off）")
        else:
            bus.disable_torque()
            print(f"ID {args.id} ({name}): 力矩已关闭，输出轴可自由转动。")
            print(_arm_header(args))
            print("  → 手动转输出轴：检查是否顺滑、有无卡齿/异响。")
            bus.disconnect(disable_torque=False)
            print("  （自由状态保持到断电为止；如需再锁住跑 --torque on）")
    except Exception:
        if bus.is_connected:
            bus.disconnect()
        raise
    return 0


def cmd_dry_run(args) -> int:
    print("=" * 64)
    print("DRY RUN（无硬件模式，仅校验 import + 参数解析 + 打印）")
    print("=" * 64)
    if DRIVER_OK:
        print("驱动 import: OK")
        print(f"  类名      : FeetechMotorsBus (lerobot.motors.feetech)")
        print(f"  默认波特率: {FeetechMotorsBus.default_baudrate}")
        print(f"  默认超时  : {FeetechMotorsBus.default_timeout} ms")
        print(f"  协议      : sts3215 (protocol 0)")
    else:
        print(f"驱动 import: 失败 -> {DRIVER_IMPORT_ERROR}")
        print("请先 source ~/miniconda3/bin/activate lerobot")
        return 1

    print(f"串口参数: port={args.port}, baudrate={args.baudrate}")
    print(_arm_header(args))
    print(f"ID→关节: {JOINT_NAME_BY_ID}")
    print("将执行的动作:")
    actions = []
    if args.scan:
        actions.append(f"[scan] 扫描所有舵机 ID 并读回位置")
    if args.id is not None:
        if args.torque:
            actions.append(f"[torque] ID {args.id} -> {args.torque}")
        if args.step is not None:
            actions.append(
                f"[step]  ID {args.id} 步进 {args.step:+d} 原始单位 (≈{args.step / FULL_RESOLUTION * 360:+.2f}°)"
            )
        if args.read and args.step is None:
            actions.append(f"[read]  ID {args.id} 读当前位置")
    if not actions:
        actions.append("(未指定动作，仅校验参数)")
    for a in actions:
        print("  " + a)
    print("DRY RUN 通过 ✓（未连接任何硬件）")
    return 0


# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="servo_test.py",
        description="SO-ARM101 单舵机测试工具（复用 lerobot FeetechMotorsBus 驱动）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "ID → 关节对照：\n"
            "  1=shoulder_pan  2=shoulder_lift  3=elbow_flex\n"
            "  4=wrist_flex    5=wrist_roll    6=gripper\n\n"
            "示例：\n"
            "  python servo_test.py --dry-run\n"
            "  python servo_test.py --scan --port /dev/ttyUSB0 --arm-type follower\n"
            "  python servo_test.py --scan --port /dev/ttyACM0 --arm-type leader\n"
            "  python servo_test.py --id 3 --read --arm-type follower\n"
            "  python servo_test.py --id 1 --step 30 --arm-type follower   # 正小步进\n"
            "  python servo_test.py --id 1 --step=-30      # 负小步进（负值用 = 号）\n"
            "  python servo_test.py --id 2 --torque on --arm-type leader    # 手顶测力矩\n"
            "  python servo_test.py --id 2 --torque off     # 自由转动测顺滑\n"
        ),
    )
    p.add_argument("--port", default=DEFAULT_PORT, help=f"串口路径（默认 {DEFAULT_PORT}，Ubuntu 下也常是 /dev/ttyACM0）")
    p.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help=f"波特率（默认 {DEFAULT_BAUDRATE}）")
    p.add_argument(
        "--arm-type",
        choices=["leader", "follower"],
        default=None,
        help="指定测哪条臂：leader=主臂(HX-10HM/so101_leader) / follower=从臂(HX-30HM/so101_follower)。两条臂的舵机 ID 各自都是 1~6，仅电机型号与用途不同。",
    )
    p.add_argument("--scan", action="store_true", help="扫描总线上所有舵机 ID 并读回位置")
    p.add_argument("--id", type=int, default=None, metavar="N", help="目标舵机 ID（1~6）")
    p.add_argument("--read", action="store_true", help="读单个舵机当前位置")
    p.add_argument(
        "--step",
        type=int,
        nargs="?",
        const=DEFAULT_STEP,
        default=None,
        metavar="S",
        help=f"相对步进（原始单位 0..4095，默认 {DEFAULT_STEP} ≈ 2.6°；负值用 --step=-20）",
    )
    p.add_argument("--torque", choices=["on", "off"], default=None, help="开关力矩（用于手顶/自由转动测试）")
    p.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S, help=f"步进后等待到位秒数（默认 {DEFAULT_SETTLE_S}）")
    p.add_argument("--dry-run", action="store_true", help="无硬件校验 import/参数/打印")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.dry_run:
        return cmd_dry_run(args)

    if not DRIVER_OK:
        print(f"错误：无法导入 lerobot 驱动 -> {DRIVER_IMPORT_ERROR}")
        print("请先 source ~/miniconda3/bin/activate lerobot")
        return 1

    if args.scan:
        return cmd_scan(args)

    if args.id is not None and not 1 <= args.id <= 6:
        print(f"错误：--id 必须是 1~6（1=shoulder_pan 2=shoulder_lift 3=elbow_flex 4=wrist_flex 5=wrist_roll 6=gripper），收到 {args.id}")
        return 2

    if args.id is None:
        if args.read or args.step is not None or args.torque:
            print("错误：--read / --step / --torque 需要配合 --id N")
            return 2
        build_parser().print_help()
        return 0

    if args.torque:
        return cmd_torque(args)
    if args.step is not None:
        return cmd_step(args)
    if args.read:
        return cmd_read(args)

    build_parser().print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
