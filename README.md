# so101-act

SO-ARM101 单臂 ACT 抓取管线（数据采集 → 训练 → 真机 rollout）— code, configs, and field notes.
An end-to-end ACT (Action Chunking with Transformers) pipeline on the low-cost SO-ARM101 arm:
dual-arm teleoperation data collection, LeRobot ACT training, and real-robot open-loop rollout.

**中文要点**：低成本 SO-ARM101 六维单臂 + 双相机（ASC60C 全局 + W19 腕部）遥操作采集 → LeRobot ACT 训练（100k 步）→ 真机开环 rollout。训练 loss 6.671→0.052；真机分布内成功率约 90%，位置外推（OOD）3–4cm 即失败——小数据 ACT 的典型泛化边界。

## Pipeline

```
┌──────────────────────────────┐   ┌──────────────────────────────┐   ┌──────────────────────────────┐
│ 1. Data collection           │   │ 2. ACT training              │   │ 3. Real-robot rollout        │
│    dual-arm teleoperation    │ → │    LeRobot ACT               │ → │    open-loop execution       │
│    leader SO-ARM101 (handle) │   │    chunk_size=100, use_vae   │   │    n_action_steps=40         │
│    follower SO-ARM101 (acts) │   │    lr 1e-5, batch 8, 100k    │   │    12 fps, gripper clamp     │
│    50 eps × 20–30 s, 12 fps  │   │    dual-cam 480×640          │   │    multi-checkpoint eval     │
└──────────────────────────────┘   └──────────────────────────────┘   └──────────────────────────────┘
   docs/补录数据SOP_v2.md             config/act_config.yml              scripts/rollout_act_so101.py
   docs/录制执行手册.md               config/act_smoke.sh                scripts/eval_checkpoints.txt
                                      train loss 6.671 → 0.052           ≈90% in-distribution success
```

## Hardware

- **Robot**: SO-ARM101 (SO-100 系低成本臂), 6 维 = 5 关节 + 1 夹爪, STS3215 串口总线舵机 (ID 1→6, shoulder_pan→gripper)。
- **Leader**: a second SO-ARM101 used as the teleoperation handle (joint-space mirroring).
- **Cameras**:
  - **ASC60C** 深度相机 → global view, 640×480, exposed to LeRobot as `/dev/video10` via `ros2_camera_bridge.py` + v4l2loopback（补丁见 `scripts/相机桥接使用说明.md`）。
  - **W19** UVC webcam → wrist view.
- **Compute**: NVIDIA CUDA GPU (trained on a 4060-class card; config assumes CUDA, batch 8 ≈ 4.2 GB).

## Results

| Metric | Value |
|---|---|
| Train loss (100k steps) | **6.671 → 0.052** |
| Real-robot success, in-distribution | **≈90%** |
| Real-robot success, OOD (3–4 cm block offset) | **fails** — small-data ACT generalization limit |
| Dataset | 50 episodes × 20–30 s, dual-cam, 12 fps, randomized block positions |

> 数字溯源：`≈90%` 为真机 rollout 的人工判据成功率——4 个候选 checkpoint
> （`scripts/eval_checkpoints.txt`：55k/60k/80k/100k）各跑 5 轮
> （`scripts/rollout_act_so101.py` 默认 `--rounds 5`），按 best-checkpoint 口径报告；
> 成功与否由现场人工判据「红块被放到棕色盒子上」判定（见 `scripts/rollout_act_so101_README.md`）。
> 逐轮明细存于本地 `run_*/results.json`、训练 loss 曲线引自本地训练日志，均未随仓库提交。

Comparison with open-source references:

- **Sherry Chen's SO-100 ACT**（low-cost robot 系列）: 50 episodes → 臂只会在桌子上啄（pecking, unusable）; 125 episodes → 90% in-distribution / 75% OOD。数据量门槛与本项目结论一致。
- **Japanese SO-ARM101 repro**（另一份 SO-ARM101 开源复现）: train loss 0.073，同量级。

## Repository layout

```
├── README.md
├── LICENSE                          # Apache-2.0
├── scripts/
│   ├── rollout_act_so101.py         # real-robot rollout / checkpoint evaluation
│   ├── rollout_act_so101_README.md
│   ├── eval_checkpoints.txt         # candidate checkpoint list
│   ├── ros2_camera_bridge.py        # ASC60C ROS2 → v4l2loopback /dev/video10
│   ├── 相机桥接使用说明.md
│   ├── wallx_realtime_client.py     # real-robot client for wall-oss serving (WebSocket)
│   ├── verify_gripper_sync.py       # gripper 标定/同步验证
│   ├── servo_test.py / set_one_motor.py / scan_unicast.py   # STS3215 调试
│   ├── identify_arm_ports.py        # leader/follower 串口识别 (by-id)
│   ├── dual_cam_live_server_v2.py / verify_dual_camera.py   # 双相机预览/验证
│   └── cam_live_server.py / w19_live_server.py              # 单相机 MJPEG 预览
├── config/
│   ├── act_config.yml               # ACT training config（来自 wall-x lerobot fork）
│   └── act_smoke.sh                 # 5-min 2-epoch smoke train（来自 lerobot fork）
└── docs/
    ├── 补录数据SOP_v2.md            # round3 数据补录 SOP（50 集双相机）
    ├── 录制执行手册.md              # teleop 录制手册（lerobot 0.3.4）
    └── 踩坑全记录.md                # 全链路坑记录
```

> Note: `config/act_config.yml` and `config/act_smoke.sh` are adapted from the
> [wall-x/lerobot](https://github.com/wall-x/lerobot) fork (LeRobot 0.3.4, editable install)
> and inherit LeRobot's Apache-2.0 license.

## Reproduction

### 1. Data collection (dual-arm teleoperation)

Setup: LeRobot 0.3.4 (editable), custom `so101_follower` / `so101_leader` robot classes,
cameras bridged to `/dev/video10` (ASC60C) and a W19 by-id node. Full SOP: `docs/录制执行手册.md` + `docs/补录数据SOP_v2.md`.

```bash
python -m lerobot.record \
  --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=my_awesome_follower_arm \
  --robot.cameras='{"global": {"type": "opencv", "index_or_path": "/dev/video10", "width": 640, "height": 480, "fps": 12},
                    "wrist":  {"type": "opencv", "index_or_path": "/dev/v4l/by-id/...W19...", "width": 640, "height": 480, "fps": 12}}' \
  --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=my_awesome_leader_arm \
  --dataset.repo_id=<user>/so101_pick_place \
  --dataset.root=/path/to/new/dataset_dir \
  --dataset.fps=12 --dataset.num_episodes=50 --dataset.episode_time_s=30 \
  --dataset.single_task="Grab the red block and place it on the target" \
  --dataset.push_to_hub=false
```

Key pitfalls (all hit and documented): ports must be resolved by-id at record time
(they drift), `repo_id` must contain `/`, `root` must not pre-exist,
`push_to_hub=false` is mandatory, ENTER-controlled episodes, camera read-timeout raised to 5000 ms.

### 2. ACT training

First a 2-epoch smoke run (validates load + forward before spending GPU hours), then the full run.

```bash
./config/act_smoke.sh /path/to/dataset_dir          # 2 epochs, ~5 min

# full run: edit dataset.repo_id / dataset.root / output_dir in config/act_config.yml, then:
lerobot-train --config-path config/act_config.yml   # (wall-x lerobot fork CLI)
```

Verified ACT hyperparameters (matching open-source ACT defaults, see comments in `config/act_config.yml`):
`chunk_size=100`, `use_vae=true`, `n_obs_steps=1`, `lr=1e-5`, `batch_size=8`, `steps=100000`,
`save_freq=5000`, `vision_backbone=resnet18` (ImageNet pretrained).

### 3. Real-robot rollout

```bash
# dry-run first (cameras + policy forward, no arm):
python3 rollout_act_so101.py --checkpoints <ckpt_dir> --rounds 1 --round-time-s 8

# real evaluation across candidate checkpoints:
python3 rollout_act_so101.py --execute --record \
  --checkpoints-file eval_checkpoints.txt --rounds 5 --round-time-s 40
```

Key settings: inference `n_action_steps=40` (≤ chunk 100), 12 fps deadline scheduling,
gripper clamped to [0, 90] (hardware limit is 100 — overdrive = red light + dropout),
optional per-step delta clamp `--max-step 5~10`. See `scripts/rollout_act_so101_README.md`.

> Note: paths in scripts/docs use `~/`, `$HOME`, or `<...>` placeholders (dataset ids, by-id
> device serials, etc.); replace them with the actual values on your machine.

## Acknowledgements

- [LeRobot](https://github.com/huggingface/lerobot) (Apache-2.0) — training/evaluation framework; used via the [wall-x lerobot](https://github.com/wall-x/lerobot) fork (0.3.4).
- [ACT](https://arxiv.org/abs/2304.13705) — *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware* (Zhao et al., RSS 2023), the policy architecture.
- [SO-100 / SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) and Sherry Chen's low-cost-robot ACT repos — hardware design and reference training recipes.
- SO-ARM101 vendor (自变量机器人 / X2Robot) — hardware support.

## License

Apache-2.0 (see [LICENSE](LICENSE)). `config/` files derived from LeRobot retain its Apache-2.0 license.
