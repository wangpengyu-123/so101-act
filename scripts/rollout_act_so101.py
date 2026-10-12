#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rollout_act_so101.py — SO-101 ACT 真机 rollout 评测脚本（round3 训练产物）

加载本地 ACT checkpoint，连从臂 + 双相机，按「观测 → 推理 → 40 步开环执行 → 重新观测」
循环，单轮时长上限 round_time_s（默认 40s）。可对多个候选 checkpoint 各跑 N 轮，
输出成功/失败统计表（成功与否由人工判断：红块被放到棕色盒子上）。

关键设定（已定，勿改；依据见 config/act_config.yml 与 docs/踩坑全记录.md）：
- 训练 chunk_size=100 保持不动；推理 n_action_steps=40（40/12fps≈3.33s 开环后重新观测）。
  机制（读自 lerobot/src/lerobot/policies/act/modeling_act.py）：
    * ACTPolicy.__init__/reset() 建 _action_queue，maxlen=config.n_action_steps（第 108 行）
    * select_action()（第 111-132 行）：队列空 → predict_action_chunk(batch) 得 (1,100,6)，
      切片 [:, :self.config.n_action_steps]（第 128 行）→ transpose 填队列（40 个 (1,6)），
      之后每步 popleft 1 个，直到队列空再重新推理。
    * 覆盖方法：加载后 policy.config.n_action_steps = 40 再 policy.reset()（reset 用新值
      重建 deque；40 ≤ chunk_size=100，不违反 configuration_act.py:153 的校验）。
- 夹爪动作 clamp 到 [0, gripper_max]（默认 90；数据实测范围 [0.48, 81.03]，
  避开硬限位 100 —— 历史教训：顶硬限位会过载红灯 + 掉线）；关节 clamp [-100,100]。
- 评估 SOP（每轮开始前，脚本会提示）：
    1. 臂摆到「抓取准备簇」：shoulder_lift -80~-98、elbow 88~100、wrist_roll≈0、夹爪张开
    2. 红块归位到起始位置
    3. 回车开始本轮
- 成功判据：红块被放到棕色盒子（目标）上。

安全设计：
- 默认 dry-run（只推理不发电命令）；真机执行必须显式 --execute 并输入 GO 确认。
- Ctrl+C 急停：立即停止动作流（不再写 Goal_Position），舵机保持当前位姿（扭矩仍在）；
  再按一次 Ctrl+C 强制退出（串口随进程关闭，舵机保持最后目标位置）。
  正常跑完全部轮次则断开并释放扭矩（与录制行为一致，可 --keep-torque-on-exit 保留）。
- 动作节拍 12fps（~83ms）deadline 调度；单轮时长上限 --round-time-s（默认 40s）。
- 相机读取走 lerobot OpenCVCamera（录制同款配置 640x480，global fps12 / wrist fps30）。
- 不修改 lerobot 源码；不动训练产物（只读加载）。

用法示例（conda env: lerobot）：
  # 干跑验证（默认，不碰电机；双相机 + 策略前向整链）
  python rollout_act_so101.py --checkpoints ~/wallx-workspace/wall-x/lerobot/outputs/train/act_so101_round3/checkpoints/055000 --rounds 1 --round-time-s 8

  # 真机评测：候选 55000/60000/80000/100000 各 5 轮，录视频供人工判成功
  python rollout_act_so101.py --execute --record \
      --checkpoints \
        ~/wallx-workspace/wall-x/lerobot/outputs/train/act_so101_round3/checkpoints/055000 \
        ~/wallx-workspace/wall-x/lerobot/outputs/train/act_so101_round3/checkpoints/060000 \
        ~/wallx-workspace/wall-x/lerobot/outputs/train/act_so101_round3/checkpoints/080000 \
        ~/wallx-workspace/wall-x/lerobot/outputs/train/act_so101_round3/checkpoints/100000 \
      --rounds 5 --round-time-s 40

  # 或从清单文件读 checkpoint（一行一个路径，# 注释）
  python rollout_act_so101.py --execute --record --checkpoints-file ~/so101_hardware/eval_checkpoints.txt --rounds 5
"""

import argparse
import json
import logging
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.robots import make_robot_from_config
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

# ---------------------------------------------------------------------------
# 常量（与数据集 meta/info.json 的 feature names 一致，顺序不能乱）
# ---------------------------------------------------------------------------
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
JOINT_LIMIT = (-100.0, 100.0)   # 关节 1-5 归一化量程 RANGE_M100_100
GRIPPER_MIN = 0.0               # gripper 归一化量程 RANGE_0_100
DEFAULT_FOLLOWER_PORT = "/dev/serial/by-id/<从臂-by-id串口路径>"
DEFAULT_GLOBAL_CAM = "/dev/video10"
DEFAULT_WRIST_CAM = "/dev/v4l/by-id/<W19相机by-id路径>"
DEFAULT_OUTPUT_DIR = str(Path.home() / "so101_hardware" / "outputs" / "rollout")
CKPT_DIRNAME = "pretrained_model"

log = logging.getLogger("so101_rollout")

STOP = threading.Event()  # 急停标志（Ctrl+C 置位）


def _estop_handler(signum, frame):
    if STOP.is_set():
        # 第二次 Ctrl+C：强制退出（串口随进程关闭，舵机保持最后目标位置、扭矩仍在）
        log.warning("再次收到中断信号，强制退出（串口随进程关闭，舵机保持最后目标位置）。")
        sys.exit(130)
    STOP.set()
    log.warning("急停！已停止动作流：不再写 Goal_Position，舵机保持当前位姿（扭矩仍在）。")
    log.warning("再按一次 Ctrl+C 强制退出；正常退出路径会断开并释放扭矩。")


# ---------------------------------------------------------------------------
# checkpoint 路径解析
# ---------------------------------------------------------------------------
def resolve_checkpoints(paths: list[str]) -> list[Path]:
    out = []
    for p in paths:
        p = Path(p).expanduser()
        if not p.exists():
            raise FileNotFoundError(f"checkpoint 路径不存在: {p}")
        if p.is_dir():
            if p.name == CKPT_DIRNAME and (p / "config.json").exists():
                out.append(p)
            elif (p / CKPT_DIRNAME / "config.json").exists():
                out.append(p / CKPT_DIRNAME)
            else:
                raise FileNotFoundError(f"{p} 下没找到 {CKPT_DIRNAME}/config.json")
        else:
            raise FileNotFoundError(f"checkpoint 路径不是目录: {p}")
    # 去重保序
    seen, uniq = set(), []
    for p in out:
        r = p.resolve()
        if r not in seen:
            seen.add(r)
            uniq.append(p)
    return uniq


def checkpoint_step(ckpt_dir: Path) -> str:
    """055000/pretrained_model -> 055000"""
    return ckpt_dir.parent.name


# ---------------------------------------------------------------------------
# 策略加载 + n_action_steps 覆盖
# ---------------------------------------------------------------------------
def load_policy(ckpt_dir: Path, n_action_steps: int, device: str) -> ACTPolicy:
    log.info("加载策略: %s (device=%s)", ckpt_dir, device)
    t0 = time.time()
    policy = ACTPolicy.from_pretrained(str(ckpt_dir))
    policy.to(device)
    policy.eval()
    dt = time.time() - t0
    log.info("加载完成，耗时 %.1fs。原始 n_action_steps=%s, chunk_size=%s",
             dt, policy.config.n_action_steps, policy.config.chunk_size)

    assert policy.config.temporal_ensemble_coeff is None, "本脚本假设不用 temporal ensembling"
    if n_action_steps > policy.config.chunk_size:
        raise ValueError(f"n_action_steps={n_action_steps} 不能大于 chunk_size={policy.config.chunk_size}")

    policy.config.n_action_steps = n_action_steps
    policy.reset()  # 用新的 n_action_steps 重建 action queue
    q = getattr(policy, "_action_queue", None)
    log.info("n_action_steps 已覆盖为 %s（checkpoint 原始值 100）。action queue maxlen=%s",
             policy.config.n_action_steps, None if q is None else q.maxlen)

    if device.startswith("cuda"):
        log.info("GPU 显存占用: %.2f GB", torch.cuda.memory_allocated() / 1024**3)
    return policy


# ---------------------------------------------------------------------------
# 观测采集（相机 + 状态）
# ---------------------------------------------------------------------------
class ObservationSource:
    """真机模式：lerobot SO101Follower（bus + cameras 同一实例）；
    dry-run 模式：只开双相机（不碰电机串口），状态用固定值。"""

    def __init__(self, args, dry_run: bool):
        self.args = args
        self.dry_run = dry_run
        self.wrist_fallback = False
        cam_cfgs = {
            "global": OpenCVCameraConfig(
                index_or_path=args.global_camera,
                fps=args.global_fps if args.global_fps > 0 else None,
                width=640, height=480,
            ),
            "wrist": OpenCVCameraConfig(
                index_or_path=args.wrist_camera,
                fps=args.wrist_fps if args.wrist_fps > 0 else None,
                width=640, height=480,
            ),
        }
        if dry_run:
            self.robot = None
            self.cameras = make_cameras_from_configs(cam_cfgs)
        else:
            if not args.port or "<" in args.port:
                raise RuntimeError(
                    "未提供从臂串口：请传 --port /dev/serial/by-id/ 下的 by-id 路径"
                    "（如 /dev/serial/by-id/usb-1a86_USB_Single_Serial_<SERIAL>-if00，"
                    "用 ls -l /dev/serial/by-id/ 查）")
            robot_cfg = SO101FollowerConfig(
                port=args.port,
                id=args.robot_id,
                cameras=cam_cfgs,
                max_relative_target=args.max_step if args.max_step > 0 else None,
                disable_torque_on_disconnect=not args.keep_torque_on_exit,
            )
            self.robot = make_robot_from_config(robot_cfg)
            self.cameras = None  # 连接后从 robot.cameras 取

        self.dry_state: np.ndarray | None = None
        self._warned_fallback = False

    # -- 连接 --
    def connect(self, dry_state_mean: np.ndarray):
        if self.robot is not None:
            log.info("连接从臂 %s (robot id=%s) ...", self.args.port, self.args.robot_id)
            self.robot.connect()
            self.cameras = self.robot.cameras
            log.info("从臂已连接，标定一致: %s", self.robot.bus.is_calibrated)
        else:
            self.dry_state = np.asarray(dry_state_mean, dtype=np.float64)
            log.info("dry-run：不连从臂。使用固定状态 = %s",
                     np.array2string(self.dry_state, precision=1, floatmode="fixed"))

        # 连接相机（真机模式下 robot.connect 已连；这里保证 dry-run 也连）
        connected = {}
        for name, cam in self.cameras.items():
            if not cam.is_connected:
                try:
                    cam.connect()
                    connected[name] = True
                except Exception as e:
                    connected[name] = False
                    log.error("相机 %s (%s) 打开失败: %s", name, cam.index_or_path, e)
            else:
                connected[name] = True

        if not connected.get("global"):
            raise RuntimeError(f"global 相机 {self.args.global_camera} 打不开（ASC60C 桥接链在跑吗？）")
        if not connected.get("wrist"):
            if self.dry_run:
                log.warning("wrist 相机 %s 不可用。dry-run 回退：wrist 帧用 global 帧替代（仅用于链路验证）。",
                            self.args.wrist_camera)
                self.wrist_fallback = True
            else:
                raise RuntimeError(f"wrist 相机 {self.args.wrist_camera} 打不开（W19 插了吗？）")

    # -- 状态 --
    def read_state(self) -> np.ndarray:
        if self.robot is not None:
            pos = self.robot.bus.sync_read("Present_Position")  # normalize=True
            return np.array([pos[n] for n in JOINT_ORDER], dtype=np.float64)
        return self.dry_state.copy()

    # -- 相机帧（每 tick 都读，供策略/录视频共用）--
    def read_frames(self) -> tuple[np.ndarray, np.ndarray]:
        g = self.cameras["global"].async_read(timeout_ms=500)
        if self.wrist_fallback:
            w = g
        else:
            try:
                w = self.cameras["wrist"].async_read(timeout_ms=500)
            except Exception as e:
                if not self._warned_fallback and self.dry_run:
                    self._warned_fallback = True
                    log.warning("wrist 读帧失败（%s），dry-run 回退用 global 帧替代。", e)
                    self.wrist_fallback = True
                    w = g
                else:
                    raise
        return g, w

    # -- 动作执行（dry-run 只记录不写）--
    def send_action(self, action6: np.ndarray) -> str:
        goal = {f"{n}.pos": float(action6[i]) for i, n in enumerate(JOINT_ORDER)}
        if self.robot is None:
            return "dry-run(不写舵机)"
        self.robot.send_action(goal)
        return "sent"

    def disconnect(self):
        if self.robot is not None:
            try:
                self.robot.disconnect()
                log.info("从臂已断开（disable_torque_on_disconnect=%s）。",
                         not self.args.keep_torque_on_exit)
            except Exception as e:
                log.warning("从臂断开异常: %s", e)
        for name, cam in (self.cameras or {}).items():
            try:
                cam.disconnect()
            except Exception:
                pass
        log.info("相机已释放。")


# ---------------------------------------------------------------------------
# 策略 batch 构建
# ---------------------------------------------------------------------------
def build_batch(state: np.ndarray, global_img: np.ndarray, wrist_img: np.ndarray, device: str):
    def img_tensor(im: np.ndarray) -> torch.Tensor:
        # np (H,W,3) uint8 RGB -> (1,3,H,W) float [0,1]；策略内部按 ImageNet mean/std 归一化
        t = torch.from_numpy(np.ascontiguousarray(im)).to(device).float().div_(255.0)
        return t.permute(2, 0, 1).contiguous().unsqueeze(0)

    return {
        "observation.state": torch.tensor(state, dtype=torch.float32, device=device).unsqueeze(0),
        "observation.images.global": img_tensor(global_img),
        "observation.images.wrist": img_tensor(wrist_img),
    }


def clamp_action(raw: np.ndarray, gripper_max: float) -> np.ndarray:
    a = np.asarray(raw, dtype=np.float64).copy()
    a[:5] = np.clip(a[:5], *JOINT_LIMIT)
    a[5] = np.clip(a[5], GRIPPER_MIN, gripper_max)
    return a


# ---------------------------------------------------------------------------
# 单轮 rollout
# ---------------------------------------------------------------------------
class RoundResult:
    def __init__(self, checkpoint: str, index: int):
        self.checkpoint = checkpoint
        self.round = index
        self.success = None       # None=未判（dry-run/--no-prompt）
        self.duration_s = 0.0
        self.n_steps = 0
        self.n_infers = 0
        self.estop = False
        self.n_clipped = 0
        self.final_state = None
        self.final_action = None

    def to_dict(self):
        d = dict(self.__dict__)
        d["final_state"] = None if self.final_state is None else [round(float(x), 3) for x in self.final_state]
        d["final_action"] = None if self.final_action is None else [round(float(x), 3) for x in self.final_action]
        return d


def run_round(policy, src: ObservationSource, args, ckpt_label: str, round_idx: int,
              writers: dict | None, result: RoundResult, dry_run: bool):
    """一轮 rollout：观测→推理→执行 n_action_steps 步开环→重新观测，直到时长上限或急停。"""
    if not args.no_prompt and not dry_run:
        print("\n" + "=" * 70)
        print(f"[第 {round_idx} 轮开始前准备] checkpoint={ckpt_label}")
        print("  1. 机械臂摆到「抓取准备簇」：shoulder_lift -80~-98、elbow 88~100、wrist_roll≈0、夹爪张开")
        print("  2. 红块归位到起始位置")
        try:
            input("  准备好后按 Enter 开始本轮（Ctrl-C 急停）... ")
        except EOFError:
            STOP.set()

    policy.reset()  # 清空 action queue，本轮第一次 select_action 会重新推理
    queue = getattr(policy, "_action_queue", None)
    batch = None
    t0 = time.perf_counter()
    step_dt = 1.0 / args.fps
    next_tick = t0
    behind_logged = False

    while not STOP.is_set() and (time.perf_counter() - t0) < args.round_time_s:
        # --- 读相机帧（每 tick，供策略/录视频共用）---
        try:
            g_img, w_img = src.read_frames()
        except Exception as e:
            log.error("读帧失败: %s → 急停", e)
            STOP.set()
            result.estop = True
            break

        # --- 队列空 → 读状态 + 重新推理（n_action_steps 步开环的起点）---
        if queue is None or len(queue) == 0:
            state = src.read_state()
            batch = build_batch(state, g_img, w_img, args.device)
            result.n_infers += 1
            log.info("[ckpt=%s round=%d] 队列空，重新推理（第 %d 次，step=%d）...",
                     ckpt_label, round_idx, result.n_infers, result.n_steps)

        # --- 推理取一步动作 ---
        action = policy.select_action(batch)          # (1,6)，队列空时触发一次前向
        raw6 = action.detach().cpu().numpy()[0].astype(np.float64)
        if not np.isfinite(raw6).all():
            log.error("动作含 NaN/Inf：%s → 急停", raw6)
            STOP.set()
            result.estop = True
            break

        clamped = clamp_action(raw6, args.gripper_max)
        if not np.allclose(clamped, raw6):
            result.n_clipped += 1
            if result.n_clipped <= 3:
                log.warning("动作越界被 clamp：raw=%s -> %s",
                            np.array2string(raw6, precision=1, floatmode="fixed"),
                            np.array2string(clamped, precision=1, floatmode="fixed"))

        status = src.send_action(clamped)
        result.n_steps += 1
        result.final_action = clamped

        # --- 录视频（每 tick 一帧，BGR）---
        if writers is not None:
            writers["global"].write(cv2.cvtColor(g_img, cv2.COLOR_RGB2BGR))
            writers["wrist"].write(cv2.cvtColor(w_img, cv2.COLOR_RGB2BGR))

        # --- 12fps 节拍（deadline 调度）---
        next_tick += step_dt
        delay = next_tick - time.perf_counter()
        if delay > 0:
            if STOP.wait(delay):
                result.estop = True
                break
        elif time.perf_counter() - next_tick > 3 * step_dt and not behind_logged:
            behind_logged = True
            log.warning("节拍落后（读帧/推理耗时 > %.0fms），可能影响 12fps 执行节奏", 1000 * step_dt)

    result.duration_s = time.perf_counter() - t0
    if STOP.is_set():
        result.estop = True
    # 末状态（真机模式读一次用于日志）
    try:
        if src.robot is not None:
            result.final_state = src.read_state()
    except Exception as e:
        log.warning("读末状态失败: %s", e)

    log.info("[ckpt=%s round=%d] 结束: 时长 %.1fs, 步数 %d, 推理次数 %d (步/推理=%s), clamp 次数 %d%s",
             ckpt_label, round_idx, result.duration_s, result.n_steps, result.n_infers,
             "n/a" if result.n_infers == 0 else f"{result.n_steps / result.n_infers:.1f}",
             result.n_clipped, " [急停]" if result.estop else "")


# ---------------------------------------------------------------------------
# 视频写入
# ---------------------------------------------------------------------------
def open_writers(round_dir: Path, fps: int):
    round_dir.mkdir(parents=True, exist_ok=True)
    writers = {}
    for name in ("global", "wrist"):
        p = round_dir / f"{name}.mp4"
        vw = cv2.VideoWriter(str(p), cv2.VideoWriter_fourcc(*"mp4v"), fps, (640, 480))
        if not vw.isOpened():
            raise RuntimeError(f"无法创建视频文件 {p}")
        writers[name] = vw
    return writers


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="SO-101 ACT 真机 rollout 评测（默认 dry-run，不发电命令）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--checkpoints", nargs="+", default=[], metavar="PATH",
                   help="候选 checkpoint 目录（可带或不带 pretrained_model/ 后缀），可多个")
    p.add_argument("--checkpoints-file", default=None,
                   help="候选 checkpoint 清单文件（一行一个路径，# 开头为注释）")
    p.add_argument("--rounds", type=int, default=5, help="每个 checkpoint 跑几轮（默认 5）")
    p.add_argument("--round-time-s", type=float, default=40.0, help="单轮时长上限秒（默认 40）")
    p.add_argument("--n-action-steps", type=int, default=40, help="推理开环步数（默认 40，勿改）")
    p.add_argument("--fps", type=float, default=12.0, help="动作执行节拍 fps（默认 12，勿改）")
    p.add_argument("--gripper-max", type=float, default=90.0,
                   help="夹爪动作上限 clamp（默认 90；数据实测最大 81，硬限位 100 会过载红灯）")
    p.add_argument("--max-step", type=float, default=0.0,
                   help="真机执行时单步每关节相对步进上限（归一化单位，0=关闭；建议 5~10）")
    p.add_argument("--execute", action="store_true", help="【危险】真机执行模式（默认 dry-run）")
    p.add_argument("--dry-run", action="store_true", help="显式 dry-run（默认即 dry-run，此参数仅占位）")
    p.add_argument("--record", action="store_true", help="每轮存双相机视频（mp4，供人工判成功）")
    p.add_argument("--port", default=DEFAULT_FOLLOWER_PORT, help="从臂串口（by-id 稳定路径）")
    p.add_argument("--robot-id", default="my_awesome_follower_arm", help="从臂 robot id（对应标定文件）")
    p.add_argument("--global-camera", default=DEFAULT_GLOBAL_CAM, help="global 相机（/dev/video10 桥接）")
    p.add_argument("--wrist-camera", default=DEFAULT_WRIST_CAM, help="wrist 相机（W19 by-id）")
    p.add_argument("--global-fps", type=int, default=12,
                   help="global 相机 fps 设置（录制同款 12；0=不设置，用设备默认）")
    p.add_argument("--wrist-fps", type=int, default=30,
                   help="wrist 相机 fps 设置（录制同款 30；0=不设置，用设备默认）")
    p.add_argument("--keep-torque-on-exit", action="store_true",
                   help="正常退出时不断开扭矩（默认断开释放，与录制行为一致）")
    p.add_argument("--no-prompt", action="store_true",
                   help="跳过每轮前的摆臂/归位提示和成功询问（无人值守，成功记为未判）")
    p.add_argument("--assume-yes", action="store_true", help="跳过 --execute 的 GO 确认（危险）")
    p.add_argument("--dry-run-state", default=None,
                   help="dry-run 用的 6 关节状态 'a,b,c,d,e,f'（默认取 checkpoint 的 state 均值）")
    p.add_argument("--device", default="cuda", help="推理设备（默认 cuda）")
    p.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="输出目录")
    p.add_argument("--verbose", "-v", action="store_true", help="每步打印动作")
    return p.parse_args()


def main():
    args = parse_args()
    run_dir = Path(args.output_dir) / time.strftime("run_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(run_dir / "rollout.log", encoding="utf-8"),
        ],
    )
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    signal.signal(signal.SIGINT, _estop_handler)
    signal.signal(signal.SIGTERM, _estop_handler)

    # ---- 收集 checkpoint ----
    ckpt_paths = list(args.checkpoints)
    if args.checkpoints_file:
        for line in Path(args.checkpoints_file).read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                ckpt_paths.append(line)
    if not ckpt_paths:
        log.error("没有指定 checkpoint（--checkpoints 或 --checkpoints-file）")
        return 2
    ckpt_dirs = resolve_checkpoints(ckpt_paths)
    for d in ckpt_dirs:
        log.info("候选 checkpoint: step=%s path=%s", checkpoint_step(d), d)

    dry_run = not args.execute
    log.info("=" * 70)
    if dry_run:
        log.info("dry-run 模式：不连接从臂、不发送任何电机命令。推理/相机/节拍全链照跑。")
    else:
        log.warning("真机执行模式！将向从臂舵机写 Goal_Position。")
        log.warning("请确认：手臂活动范围内无人无物，电源正常，随时可 Ctrl+C 急停。")
        if not args.assume_yes:
            try:
                ans = input("输入 GO 并回车开始真机执行，其他输入退出: ").strip()
            except EOFError:
                ans = ""
            if ans != "GO":
                log.warning("已取消真机执行。")
                return 1
    log.info("单轮时长上限 %.0fs，节拍 %.1ffps（%.0fms/步），n_action_steps=%d，gripper clamp 上限 %.1f",
             args.round_time_s, args.fps, 1000 / args.fps, args.n_action_steps, args.gripper_max)
    log.info("输出目录: %s", run_dir)
    log.info("=" * 70)

    all_results: list[RoundResult] = []
    exit_code = 0

    for ckpt_dir in ckpt_dirs:
        ckpt_label = checkpoint_step(ckpt_dir)
        if STOP.is_set():
            log.warning("急停已触发，跳过剩余 checkpoint。")
            break

        # ---- 加载策略 ----
        try:
            policy = load_policy(ckpt_dir, args.n_action_steps, args.device)
        except Exception as e:
            log.error("加载 checkpoint %s 失败: %s", ckpt_label, e)
            exit_code = 2
            continue

        # ---- dry-run 状态 ----
        dry_state_mean = policy.normalize_inputs.buffer_observation_state.mean.detach().cpu().numpy()
        if args.dry_run_state:
            vals = [float(x) for x in args.dry_run_state.split(",")]
            if len(vals) != 6:
                log.error("--dry-run-state 需要 6 个逗号分隔的数")
                return 2
            dry_state_mean = np.asarray(vals)

        # ---- 连接（从臂 + 双相机）----
        src = ObservationSource(args, dry_run=dry_run)
        try:
            src.connect(dry_state_mean)
        except Exception as e:
            log.error("连接失败: %s", e)
            exit_code = 2
            continue

        # ---- 前向 smoke 验证：形状 (1,100,6) + 有限值 ----
        try:
            frames = src.read_frames()
            state = src.read_state()
            smoke_batch = build_batch(state, *frames, args.device)
            with torch.no_grad():
                chunk = policy.predict_action_chunk(smoke_batch)
            assert tuple(chunk.shape) == (1, policy.config.chunk_size, 6), f"形状异常: {chunk.shape}"
            assert torch.isfinite(chunk).all(), "输出含 NaN/Inf"
            log.info("前向 smoke ✓：chunk 形状 %s，全有限值。示例 action[0]=%s",
                     tuple(chunk.shape),
                     np.array2string(chunk[0, 0].cpu().numpy(), precision=2, floatmode="fixed"))
            policy.reset()
        except Exception as e:
            log.error("前向 smoke 失败: %s", e)
            src.disconnect()
            exit_code = 2
            continue

        # ---- N 轮 rollout ----
        ckpt_dir_out = run_dir / ckpt_label
        ckpt_dir_out.mkdir(parents=True, exist_ok=True)
        for i in range(1, args.rounds + 1):
            if STOP.is_set():
                log.warning("急停已触发，终止后续轮次。")
                break
            result = RoundResult(ckpt_label, i)
            writers = None
            if args.record:
                writers = open_writers(ckpt_dir_out / f"round_{i:02d}", args.fps)
            try:
                run_round(policy, src, args, ckpt_label, i, writers, result, dry_run=dry_run)
            except KeyboardInterrupt:
                STOP.set()
                result.estop = True
            except Exception as e:
                log.error("[ckpt=%s round=%d] 异常: %s（急停）", ckpt_label, i, e)
                STOP.set()
                result.estop = True
            finally:
                if writers is not None:
                    for w in writers.values():
                        w.release()
            all_results.append(result)

            if result.estop:
                break

            # ---- 成功询问（真机模式）----
            if not dry_run and not args.no_prompt:
                try:
                    ans = input(f"[ckpt={ckpt_label} round={i}] 本轮成功吗？（红块放到棕盒子上）y/n，q 退出: ").strip().lower()
                except EOFError:
                    ans = "q"
                if ans == "y":
                    result.success = True
                elif ans == "n":
                    result.success = False
                elif ans == "q":
                    result.success = None
                    log.warning("用户退出评测。")
                    break
                else:
                    result.success = None
            if writers is not None:
                log.info("[ckpt=%s round=%d] 视频已存 %s", ckpt_label, i, ckpt_dir_out / f"round_{i:02d}")

        src.disconnect()
        del policy
        if args.device.startswith("cuda"):
            torch.cuda.empty_cache()

    # ---- 汇总 ----
    results_path = run_dir / "results.json"
    with open(results_path, "w", encoding="utf-8") as f:
        json.dump([r.to_dict() for r in all_results], f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 78)
    print("评测汇总")
    print("=" * 78)
    print(f"{'checkpoint':<12}{'round':>6}{'success':>9}{'dur_s':>8}{'steps':>7}{'infers':>8}{'clipped':>8}")
    agg = {}
    for r in all_results:
        print(f"{r.checkpoint:<12}{r.round:>6}{str(r.success):>9}{r.duration_s:>8.1f}"
              f"{r.n_steps:>7}{r.n_infers:>8}{r.n_clipped:>8}")
        a = agg.setdefault(r.checkpoint, {"ok": 0, "fail": 0, "unjudged": 0, "rounds": 0})
        a["rounds"] += 1
        if r.success is True:
            a["ok"] += 1
        elif r.success is False:
            a["fail"] += 1
        else:
            a["unjudged"] += 1
    print("-" * 78)
    for ck, a in agg.items():
        rate = f"{a['ok']}/{a['rounds']}" if a["unjudged"] == 0 else f"{a['ok']} 成功 / {a['fail']} 失败 / {a['unjudged']} 未判"
        print(f"{ck:<12} 共 {a['rounds']} 轮: {rate}")
    print(f"\n明细与视频目录: {run_dir}")
    print(f"结果 JSON: {results_path}")
    if STOP.is_set():
        print("注意：过程中触发过急停。")
    return exit_code if exit_code else (130 if STOP.is_set() else 0)


if __name__ == "__main__":
    sys.exit(main())
