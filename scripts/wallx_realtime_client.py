#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wallx_realtime_client.py — SO-ARM101 真机 ←→ 云端 wall-x serving 实时推理客户端

链路：/dev/video10 相机帧 + /dev/ttyACM1 从臂 6 关节 → ws://localhost:32195 云端推理
     → predict_action[32,26] 取前 6 维 → （--execute 时）写回从臂舵机。

单位约定（全部经代码+数据实测确认，见项目汇报）：
  - 与云端交互的状态/动作单位 = lerobot 的归一化单位：
      关节 1-5（shoulder_pan..wrist_roll）: RANGE_M100_100（标定范围百分比，-100..100）
      关节 6（gripper）:              RANGE_0_100（0..100）
    这正好是训练数据集 so101_table_pick_place 的原始单位（parquet 实测 + norm_stats 印证）。
  - 舵机原始编码 0..4095 与归一化单位的换算由 lerobot FeetechMotorsBus 用同一份
    标定文件（my_awesome_follower_arm.json）自动完成：
      norm = ((raw - range_min)/(range_max - range_min)) * 200 - 100   (RANGE_M100_100)
      norm = ((raw - range_min)/(range_max - range_min)) * 100         (RANGE_0_100)
    （drive_mode=0，无符号翻转；bus.sync_read/sync_write 默认 normalize=True 即走这套换算）
  - 云端 serving 侧：状态用 norm_stats.json 的 q01/q99 做 min-max 归一化到 [-1,1]，
    模型输出动作再按 q01/q99 反归一化回上述单位后返回（--no-serialize-actions 原样返回
    [32,26]，前 6 维=6 关节，后 20 维=action_padding 恒 0）。

安全设计：
  - 默认 dry-run：只打印动作，绝不碰舵机。
  - 只有显式传 --execute 才写舵机；写之前打印目标位置（归一化+原始值）。
  - --max-delta 限制单次执行每关节相对当前值的最大步进（默认 5.0 归一化单位，
    ≈ 关节量程的 2.5%；传 0 关闭）。
  - 执行帧选择：32 帧里挑与当前关节状态最接近的一帧（ACT 惯例），再叠加 max-delta 限幅。
  - Ctrl-C 立即停止发送（急停）；意外退出不再写 Goal_Position，舵机保持当前位姿。
  - 云端返回 NaN/Inf 或形状不对 → 丢弃并报警，不执行。

用法（conda env lerobot）：
  # 真机 dry-run 联调（只读不写，打印动作与延迟）：
  python wallx_realtime_client.py --frames 5
  # 相机缺失时用离线帧测试推理链路：
  python wallx_realtime_client.py --frames 3 --image /tmp/frame.jpg
  # 真机执行（用户在场时）：
  python wallx_realtime_client.py --execute
  # 完整参数见 --help
"""

import argparse
import base64
import json
import logging
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

import msgpack
import msgpack_numpy

msgpack_numpy.patch()

import websockets
import websockets.sync.client as ws_sync

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import MotorCalibration

log = logging.getLogger("wallx_client")

# ---------------------------------------------------------------------------
# 常量（与 lerobot so101_follower / 训练配置一致，顺序不能乱）
# ---------------------------------------------------------------------------
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
N_JOINTS = 6
ACTION_HORIZON = 32
ACTION_DIM = 26          # 6 关节 + 20 padding（config_full.yml: so101_joint_pos 6 + action_padding 20）
DEFAULT_CALIB = str(Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/my_awesome_follower_arm.json")
DEFAULT_INSTRUCTION = "Grab the red block and place it on the target"
JOINT_RANGE = (-100.0, 100.0)   # 关节 1-5 归一化量程
GRIPPER_RANGE = (0.0, 100.0)    # gripper 归一化量程


# ---------------------------------------------------------------------------
# 从臂读写（复用 lerobot FeetechMotorsBus 与录制时同一份标定）
# ---------------------------------------------------------------------------
class FollowerArm:
    def __init__(self, port: str, calib_path: str = DEFAULT_CALIB):
        cal = json.load(open(calib_path))
        self.calib = {
            name: MotorCalibration(
                id=v["id"], drive_mode=v["drive_mode"], homing_offset=v["homing_offset"],
                range_min=v["range_min"], range_max=v["range_max"],
            )
            for name, v in cal.items()
        }
        motors = {
            name: Motor(i, "sts3215",
                        MotorNormMode.RANGE_0_100 if i == 6 else MotorNormMode.RANGE_M100_100)
            for i, name in enumerate(JOINT_ORDER, start=1)
        }
        self.bus = FeetechMotorsBus(port=port, motors=motors, calibration=self.calib)

    def connect(self):
        self.bus.connect()
        log.info("从臂已连接 %s, EEPROM 标定与文件一致: %s",
                 self.bus.port, self.bus.is_calibrated)
        return self

    def read_state(self) -> np.ndarray:
        """读 6 关节当前值（归一化单位，与训练数据同单位）。"""
        pos = self.bus.sync_read("Present_Position")  # normalize=True
        return np.array([pos[n] for n in JOINT_ORDER], dtype=np.float64)

    def write_state(self, target: np.ndarray) -> np.ndarray:
        """把 6 维归一化目标写成 Goal_Position（bus 内部换算到 0..4095）。返回实际写入值。"""
        goal = {n: float(target[i]) for i, n in enumerate(JOINT_ORDER)}
        self.bus.sync_write("Goal_Position", goal)  # normalize=True
        return target

    def disconnect(self):
        try:
            self.bus.disconnect()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 相机
# ---------------------------------------------------------------------------
class Camera:
    """读 /dev/video10（v4l2loopback，640x480 RGB3）。

    注意：若 ASC60C 相机本体掉线，桥接进程不再写入新帧，read 会阻塞；
    用 CAP_PROP_READ_TIMEOUT_MSEC 限时避免卡死。
    """

    def __init__(self, path: str, read_timeout_ms: int = 1500, jpeg_quality: int = 95):
        self.path = path
        self.jpeg_quality = jpeg_quality
        self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开相机 {path}（相机链是否在跑？）")
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        if read_timeout_ms > 0:
            self.cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, read_timeout_ms)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        log.info("相机已打开 %s: %dx%d", path, w, h)

    def grab_b64_jpeg(self) -> tuple[str, float]:
        """读一帧 → JPEG → base64 字符串。返回 (b64, 耗时ms)。"""
        t0 = time.time()
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"读帧失败/超时（{self.path} 无新帧）")
        # 统一缩放到训练尺寸 640x480（ASC60C 原生 UVC 是 1280x1040）
        if frame.shape[0] != 480 or frame.shape[1] != 640:
            frame = cv2.resize(frame, (640, 480))
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError("JPEG 编码失败")
        dt_ms = (time.time() - t0) * 1000
        return base64.b64encode(buf.tobytes()).decode("ascii"), dt_ms

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass


def load_image_override(path: str) -> str:
    """--image 离线帧：读文件（jpg/png）→ 按训练同尺寸 640x480 缩放 → base64。"""
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"读不到图像文件 {path}")
    if img.shape[0] != 480 or img.shape[1] != 640:
        img = cv2.resize(img, (640, 480))
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError("JPEG 编码失败")
    return base64.b64encode(buf.tobytes()).decode("ascii")


# ---------------------------------------------------------------------------
# 云端推理
# ---------------------------------------------------------------------------
class WallxCloud:
    def __init__(self, ws_url: str, timeout: float = 60.0):
        self.ws_url = ws_url
        self.timeout = timeout
        self.ws = None

    def connect(self):
        self.ws = ws_sync.connect(self.ws_url, open_timeout=15, close_timeout=5)
        meta = msgpack.unpackb(self.ws.recv(self.timeout), raw=False)
        log.info("已连接 %s, metadata=%s", self.ws_url, meta)
        return meta

    def infer(self, state: np.ndarray, frame_b64: str, wrist_b64: str, instruction: str) -> dict:
        """发一帧观测（global+wrist 双相机），收 predict_action[32,26]。返回 {predict_action, server_timing, rtt_ms}。"""
        req = {
            "state": {"so101_joint_pos": state.tolist()},
            "views": {"global_view": frame_b64, "wrist_view": wrist_b64},
            "instruction": instruction,
        }
        t0 = time.time()
        self.ws.send(msgpack.packb(req))
        resp = msgpack.unpackb(self.ws.recv(self.timeout), raw=False)
        rtt_ms = (time.time() - t0) * 1000
        if not isinstance(resp, dict) or "predict_action" not in resp:
            raise RuntimeError(f"云端响应异常: {type(resp)} {str(resp)[:300]}")
        pa = np.asarray(resp["predict_action"], dtype=np.float64)
        if pa.ndim == 3 and pa.shape[0] == 1:      # 服务端带 batch 维 (1,32,26)
            pa = pa[0]
        if pa.shape != (ACTION_HORIZON, ACTION_DIM):
            raise RuntimeError(f"predict_action 形状异常: {pa.shape}，期望 ({ACTION_HORIZON},{ACTION_DIM})")
        if not np.isfinite(pa[:, :N_JOINTS]).all():
            raise RuntimeError("predict_action 含 NaN/Inf，丢弃")
        out = {
            "predict_action": pa,
            "server_timing": resp.get("server_timing", {}),
            "rtt_ms": rtt_ms,
        }
        if "subtask" in resp:
            out["subtask"] = resp["subtask"]
        return out

    def close(self):
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# 动作处理（安全裁剪 + 执行帧选择）
# ---------------------------------------------------------------------------
def clamp_joints(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64).copy()
    x[:5] = np.clip(x[:5], *JOINT_RANGE)
    x[5] = np.clip(x[5], *GRIPPER_RANGE)
    return x


def select_and_clamp(action: np.ndarray, current: np.ndarray, max_delta: float):
    """选 32 帧中与当前状态最接近的一帧，并做相对步进限幅。

    返回 (target, chosen_frame_index, raw_delta_norm)。
    """
    act = np.asarray(action, dtype=np.float64)[:, :N_JOINTS]
    act = clamp_joints(act)
    dist = np.linalg.norm(act - current[None, :], axis=1)
    idx = int(np.argmin(dist))
    target = act[idx].copy()
    if max_delta > 0:
        delta = np.clip(target - current, -max_delta, max_delta)
        target = current + delta
        target = clamp_joints(target)
    return target, idx


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="SO-ARM101 真机 ↔ 云端 wall-x serving 实时客户端")
    p.add_argument("--ws-url", default="ws://localhost:32195", help="serving 地址（默认本地转发端口；远端部署可自行转发）")
    p.add_argument("--port", default="/dev/ttyACM1", help="从臂串口")
    p.add_argument("--calib", default=DEFAULT_CALIB, help="从臂标定文件（与录制时同一份）")
    p.add_argument("--camera", default="/dev/video10", help="global 相机（v4l2loopback 虚拟相机）")
    p.add_argument("--wrist-camera", default="/dev/v4l/by-id/<W19相机by-id路径>",
                   help="wrist 相机（W19 UVC）")
    p.add_argument("--image", default=None, help="离线帧文件（相机缺失时替代真机相机测试链路）")
    p.add_argument("--instruction", default=DEFAULT_INSTRUCTION, help="任务指令文本")
    p.add_argument("--frames", type=int, default=0, help="推理轮数（0=持续循环直到 Ctrl-C）")
    p.add_argument("--interval", type=float, default=0.0, help="每轮间隔秒数（dry-run 建议 0）")
    p.add_argument("--execute", action="store_true", help="【危险】真发舵机（默认 dry-run 只打印）")
    p.add_argument("--max-delta", type=float, default=5.0,
                   help="--execute 时单轮每关节最大步进（归一化单位；0=不限幅；默认 5.0）")
    p.add_argument("--execute-interval", type=float, default=0.0,
                   help="--execute 时写完 Goal_Position 后的等待秒数（0=不等待，连续追目标，平滑）")
    p.add_argument("--chunk-size", type=int, default=8,
                   help="--execute 时每次推理后连续执行的帧数（8=一次走8帧再重新推理）")
    p.add_argument("--timeout", type=float, default=60.0, help="云端响应超时秒数")
    p.add_argument("--verbose", "-v", action="store_true", help="打印每帧动作全部 32 帧前 3 帧")
    return p.parse_args()


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    if args.execute:
        log.warning("=" * 70)
        log.warning("  --execute 真机模式：将向从臂舵机写 Goal_Position！")
        log.warning("  请确认：手臂活动范围内无人/无物，随时可 Ctrl-C 急停。")
        log.warning("=" * 70)
    else:
        log.info("dry-run 模式：只打印动作，不碰舵机。")

    # 1. 从臂
    arm = FollowerArm(args.port, args.calib).connect()

    # 2. 相机（真机相机缺失时可用 --image 兜底）
    cam = None
    wrist_cam = None
    image_override = None
    if args.image:
        image_override = load_image_override(args.image)
        log.info("使用离线帧 %s（%d 字符 base64）", args.image, len(image_override))
    else:
        cam = Camera(args.camera)
        log.info("global 相机已就绪")
        wrist_cam = Camera(args.wrist_camera)
        log.info("wrist 相机已就绪")

    # 3. 云端
    cloud = WallxCloud(args.ws_url, args.timeout)
    meta = cloud.connect()

    last_frame_b64 = image_override
    last_wrist_b64 = image_override
    cam_errors = 0
    stopped = False
    round_no = 0

    def stop_handler(signum, frame):
        nonlocal stopped
        stopped = True
        log.warning("收到 Ctrl-C，立即停止发送动作（舵机保持当前位姿）")

    import signal
    signal.signal(signal.SIGINT, stop_handler)

    try:
        while not stopped:
            if args.frames > 0 and round_no >= args.frames:
                break
            round_no += 1
            t_round = time.time()

            # --- 读状态 ---
            state = arm.read_state()
            t_state = (time.time() - t_round) * 1000

            # --- 读帧（global + wrist 双相机）---
            frame_ms = -1.0
            wrist_ms = -1.0
            if cam is not None:
                try:
                    last_frame_b64, frame_ms = cam.grab_b64_jpeg()
                    cam_errors = 0
                except RuntimeError as e:
                    cam_errors += 1
                    if last_frame_b64 is None:
                        log.error("没有可用帧且读帧失败（%s），退出。相机链需要恢复。", e)
                        return 2
                    log.warning("读帧失败（%s），复用上一帧（连续 %d 次）", e, cam_errors)
            if wrist_cam is not None:
                try:
                    last_wrist_b64, wrist_ms = wrist_cam.grab_b64_jpeg()
                except RuntimeError as e:
                    log.warning("wrist 读帧失败（%s），复用上一帧", e)
            if last_frame_b64 is None or last_wrist_b64 is None:
                log.error("没有可用帧。")
                return 2

            # --- 云端推理 ---
            try:
                resp = cloud.infer(state, last_frame_b64, last_wrist_b64, args.instruction)
            except Exception as e:
                log.error("云端推理失败: %s", e)
                break

            action = resp["predict_action"]
            st = resp["server_timing"]
            infer_ms = st.get("infer_ms", float("nan"))

            # --- 动作处理：连续 chunk 执行 ---
            act = np.asarray(action, dtype=np.float64)[:, :N_JOINTS]
            act = clamp_joints(act)
            dist = np.linalg.norm(act - state[None, :], axis=1)
            start_idx = int(np.argmin(dist))

            if args.execute:
                # 连续执行 chunk（不等待，舵机连续追目标 → 平滑）
                cur = state
                chunk_targets = []
                chunk_raw = []
                for k in range(args.chunk_size):
                    idx = min(start_idx + k, ACTION_HORIZON - 1)
                    t = act[idx].copy()
                    if args.max_delta > 0:
                        d = np.clip(t - cur, -args.max_delta, args.max_delta)
                        t = cur + d
                    t = clamp_joints(t)
                    raw = {
                        n: int(((tv + 100) / 200) * (arm.calib[n].range_max - arm.calib[n].range_min) + arm.calib[n].range_min)
                        if i < 5 else int((tv / 100) * (arm.calib[n].range_max - arm.calib[n].range_min) + arm.calib[n].range_min)
                        for i, (n, tv) in enumerate(zip(JOINT_ORDER, t))
                    }
                    arm.write_state(t)
                    cur = t
                    chunk_targets.append(t)
                    chunk_raw.append(raw)
                    if args.execute_interval > 0:
                        time.sleep(args.execute_interval)
                log.info("─" * 78)
                log.info("第 %d 轮 连续执行 %d 帧 (起点 index=%d/31)", round_no, len(chunk_targets), start_idx)
                log.info("  起始状态: %s", np.array2string(state, precision=1, floatmode="fixed"))
                log.info("  首帧目标=%s", np.array2string(chunk_targets[0], precision=1, floatmode="fixed"))
                log.info("  末帧目标=%s", np.array2string(chunk_targets[-1], precision=1, floatmode="fixed"))
                log.info("  末帧原始=%s", str(chunk_raw[-1]))
            else:
                # dry-run：单帧打印（保持原逻辑）
                target, chosen = select_and_clamp(action, state, 0.0)
                delta_norm = target - state
                log.info("─" * 78)
                log.info("第 %d 轮  状态: %s", round_no, np.array2string(state, precision=1, floatmode="fixed"))
                log.info("帧: %s  base64=%d字符  (读帧 %.1fms | 状态读取 %.1fms)",
                         "真机相机" if cam is not None else "离线帧", len(last_frame_b64), frame_ms, t_state)
                log.info("云端: infer_ms=%.0f  客户端RTT=%.0fms%s", infer_ms, resp["rtt_ms"],
                         f"  prev_total_ms={st['prev_total_ms']:.0f}" if "prev_total_ms" in st else "")
                log.info("动作[0,:6]=%s", np.array2string(action[0, :N_JOINTS], precision=1, floatmode="fixed"))
                log.info("执行帧 index=%d/31  目标=%s", chosen, np.array2string(target, precision=1, floatmode="fixed"))
                log.info("相对步进=%s", np.array2string(delta_norm, precision=2, floatmode="fixed"))
                log.info("  [dry-run] 不写舵机。加 --execute 才真机执行。")

            if args.interval > 0 and not stopped:
                time.sleep(args.interval)
    finally:
        cloud.close()
        if cam is not None:
            cam.close()
        arm.disconnect()
        log.info("已断开。%s", "（Ctrl-C 急停后舵机保持当前位姿，未断电）" if stopped else "")


if __name__ == "__main__":
    sys.exit(main())
