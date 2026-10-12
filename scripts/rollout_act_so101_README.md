# rollout_act_so101.py — SO-101 ACT 真机 rollout 评测

对本地 ACT checkpoint 在真机从臂上跑 rollout：观测 → 推理 → 执行 n_action_steps=40 步开环
（40/12fps≈3.33s）→ 重新观测，单轮时长上限默认 40s；对多个候选 checkpoint 各跑 N 轮，
输出成功/失败统计表。成功判据（人工）：红块被放到棕色盒子上。

## 关键设定（已定，勿改）
- 训练 chunk_size=100 不动；推理 `n_action_steps=40`。机制见
  `lerobot/src/lerobot/policies/act/modeling_act.py`：`select_action` 在内部
  `_action_queue`（maxlen=n_action_steps，第 108 行）空时调 `predict_action_chunk`，
  取 `[:, :n_action_steps]`（第 128 行）填队列，之后每步弹 1 个动作。脚本加载后
  `policy.config.n_action_steps=40` + `policy.reset()` 覆盖生效（40≤chunk_size=100）。
- 夹爪动作 clamp 到 `[0, gripper_max=90]`（数据实测夹爪范围 [0.48, 81.03]；
  顶硬限位 100 会过载红灯 + 掉线，历史教训）。关节 clamp [-100,100]。
- 动作节拍 12fps（~83ms/步）deadline 调度；执行模式可选 `--max-step 5~10`
  追加每步相对步进限幅。

## 用法（conda env: lerobot）

```bash
source ~/miniconda3/bin/activate lerobot
cd ~/so101_hardware

# 干跑验证（默认 dry-run：不连从臂、不发电命令；双相机+策略前向整链）
python3 rollout_act_so101.py \
  --checkpoints ~/wallx-workspace/wall-x/lerobot/outputs/train/act_so101_round3/checkpoints/055000 \
  --rounds 1 --round-time-s 8

# 真机评测：4 个候选 checkpoint 各 5 轮，录双相机视频供人工判成功
python3 rollout_act_so101.py --execute --record \
  --checkpoints-file ~/so101_hardware/eval_checkpoints.txt \
  --rounds 5 --round-time-s 40
```

## 真跑时用户步骤清单
1. **检查相机链**（ASC60C 易掉线）：`v4l2-ctl -d /dev/video10 --info` 有设备后，
   用 cv2 读一帧确认有画面（读不到 = 链死了，按 `相机桥接使用说明.md` 重启
   ASC60C + ros2_camera_bridge.py）。W19 插好，`ls /dev/v4l/by-id/` 有 W19 节点。
2. **从臂上电**：确认从臂串口 `ls /dev/serial/by-id/` 存在（从臂的 by-id 后缀即 `<SERIAL_B>`，见 docs/补录数据SOP_v2.md）。
3. 红块归位到起始位置；机械臂摆到**抓取准备簇**：shoulder_lift -80~-98、
   elbow 88~100、wrist_roll≈0、夹爪张开（脚本每轮开始前也会提示）。
4. 确认手臂活动范围内无人无物。
5. 跑命令（见上）。脚本会要求输入 `GO` 才开始；每轮开始前按 Enter 确认场景就绪。
6. **每轮结束后**：看现场（或该轮视频 `outputs/rollout/run_*/<step>/round_NN/*.mp4`），
   回答脚本的 y/n 成功询问；下一轮开始前重新摆准备簇 + 红块归位。
7. 结束后看汇总表（终端 + `run_*/results.json`）。

## 急停
- 单次 Ctrl+C：立即停止动作流（不再写 Goal_Position），舵机保持当前位姿（扭矩仍在）。
- 再次 Ctrl+C：强制退出（串口随进程关闭，舵机保持最后目标位置）。
- 正常跑完：断开串口并释放扭矩（录制同款行为）；`--keep-torque-on-exit` 可保留扭矩。

## 常用参数
| 参数 | 默认 | 说明 |
|---|---|---|
| --rounds | 5 | 每个 checkpoint 的轮数 |
| --round-time-s | 40 | 单轮时长上限 |
| --n-action-steps | 40 | 开环步数（勿改） |
| --fps | 12 | 执行节拍（勿改） |
| --gripper-max | 90 | 夹爪 clamp 上限 |
| --max-step | 0 | 每步相对步进限幅（0=关，建议真跑 5~10） |
| --record | 关 | 每轮存双相机 mp4 |
| --no-prompt | 关 | 跳过场景就绪/成功询问（无人值守） |
| --assume-yes | 关 | 跳过 GO 确认（危险） |
| --checkpoints-file | - | 候选清单文件（见 eval_checkpoints.txt） |
