# SO-ARM101 真机补录数据 SOP v2

> 目标：录出 **≥50 集**、每集 **20-30s**、**红块位置随机化**、**统一 6 步动作序列**、
> gripper 全程有「开→闭→开」的高质量抓取数据，替换掉之前 31 集（9s/集、12fps）被判坏的数据。
>
> 本 SOP 的所有命令均抄自已验证文档：
> - `round2_recording_sop.md`（双相机命令、设备对照、启动顺序、后处理）
> - `录制执行手册.md`（ENTER 控制、verify、已知坑）
> - `真机录制checklist.md`（关节顺序、归一化、物块规格）
> - `verify_dual_camera.py` / `verify_real_dataset.py` / `unify_camera_keys_round2.py` /
>   `compute_norm_stats_round2.py`（脚本源码）
> - `~/wallx-workspace/wall-x/lerobot/src/lerobot/record.py`（record 命令参数语义，本机 0.3.4 源码）
>
> 本文只覆盖「补录」这条线；通用坑与背景见 `录制执行手册.md` 和 `round2_recording_sop.md`。

---

## 0. 一句话结论

- 数据集新目录：`~/so101_walloss/so101_real_round3`（**必须不存在**，fresh）。
- 每集 20-30s：`--dataset.episode_time_s=30` 是「到点自动停」的硬上限，做完 6 步按 ENTER 手动结束。
- 一共 ≥50 集：推荐 **5 批 × 10 集**，批与批之间用 `--resume=true` 追加，每批结束跑一次自检。
- 命令其余参数（fps=12、端口、双相机 dict、single_task、push_to_hub=false）沿用 round2，**别动**。

---

## 1. 开录前 5 分钟检查（逐项打勾，任意一条不过就停）

### 1.1 设备都插上、端口复核（重插会变，必须现场复核，别假设）

```bash
# 三个串口 + 相机设备都在（没插就先插，插完再跑）
ls -l /dev/ttyACM0 /dev/ttyACM1
ls -l /dev/serial/by-id/                # 复核两臂的稳定路径
ls -l /dev/v4l/by-id/ | grep W19        # W19 腕部相机 by-id 稳定路径
v4l2-ctl --list-devices                 # 复核相机设备映射（别照旧结论）
```

- **两臂串口映射（已实测锁定；ttyACM 编号会漂移，命令里一律用 by-id）**：
  - 主臂（你手里拿的遥操作握柄）= by-id `usb-1a86_USB_Single_Serial_<SERIAL_A>-if00`（本次实测 = `ttyACM1`）；
    从臂（会动的执行臂）= by-id `usb-1a86_USB_Single_Serial_<SERIAL_B>-if00`（本次实测 = `ttyACM0`）。
  - ⚠️ **ttyACM0/1 会随插拔/换线对调**，别记「主臂=ttyACM0」这种旧结论；**命令里一律填 by-id 路径**，别填 `/dev/ttyACM*`。
  - 每次插上后 `ls -l /dev/serial/by-id/` 复核一次：<SERIAL_A> 必须对应主臂（握柄）、<SERIAL_B> 必须对应从臂（执行）。
    拿不准就动一下握柄，看哪只臂跟动。
- **相机映射（round2 实测）**：
  - `global`（ASC60C）走 `/dev/video10`（ros2 桥接虚拟相机，640×480）。
  - `wrist`（W19）走 by-id `/dev/v4l/by-id/<W19相机by-id路径>`。
  - W19 的 `/dev/video*` 编号会漂（曾被挤到 video4/video5），所以 **wrist 一律用 by-id 路径**。

### 1.2 lerobot 版本（必须 0.3.4，不是 0.6.1）

```bash
source ~/miniconda3/bin/activate lerobot
python -c "import lerobot; print(lerobot.__version__)"    # 必须打印 0.3.4
```

### 1.3 双相机自检（两路都要出帧、能同时读）

```bash
source ~/miniconda3/bin/activate lerobot
python ~/so101_hardware/verify_dual_camera.py
```

- 输出末尾必须 `双相机自检 PASS：可开录。`；并且它会各存一帧到 `/tmp/global_check.png`、
  `/tmp/wrist_check.png`，**打开人眼看一眼构图**：global 覆盖整张工作台（物块、目标位、整臂），
  wrist 覆盖夹爪+物块+目标局部。
- 若 global 这路打不开：桥接没起，先按 §4.1 起 ros2 桥接再跑。

### 1.4 gripper 记录自检（录数据前必跑，缺一步都不许开录）

> round2 失败的根因之一：`state.gripper = 0.486×action.gripper + 22.2`（gripper 是坏信号）。
> 以下两步缺一不可：**录前先重标定硬件**，**录完每批再跑同步门禁**。

**① 录数据前：硬件重标定 + 自检（从臂夹爪）**

```bash
~/miniconda3/envs/lerobot/bin/python ~/wallx-workspace/wall-x/lerobot/scripts/recalibrate_so101_gripper.py --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_<SERIAL_B>-if00 --id my_awesome_follower_arm
```

- 作用：重测从臂夹爪真实行程，修正错误的 `range_min`/`range_max`（round2 失败的根因）。
- 脚本内置自检：`cmd 5 → present ∈ [-5,15]`、`cmd 95 → present ∈ [85,105]` 都满足才 PASS；
  任一 FAIL（退出码 1）就停下，先查夹爪连杆/力矩，别开录。
- ⚠️ 端口是 by-id `<SERIAL_B>`（从臂/夹爪），不是 `<SERIAL_A>`（主臂）；插上后先 `ls -l /dev/serial/by-id/` 复核 `<SERIAL_B>` 对应从臂。

**② 录完数据后：gripper 同步门禁（每批录完 / 全部录完都跑）**

```bash
~/miniconda3/envs/lerobot/bin/python ~/so101_walloss/verify_gripper_sync.py ~/so101_walloss/so101_real_round3 --verbose
```

- 判据：gripper 的 `slope ∈ [0.9,1.1]` 且 `|intercept| < 5` 且 `corr > 0.90` 且 `lag ≤ 1` 才 PASS（退出码 0）；
  FAIL（退出码 1）就说明这批 gripper 又坏了，别拿去训练，回①重标定后补录。

### 1.5 磁盘空间（可选，确认够放 50 集双相机数据）

```bash
df -h ~
```

- 数据写到 `~/so101_walloss/so101_real_round3`，先 `df -h ~` 确认该分区剩余空间足够。
  若录到一半 `df` 报警，先停（ESC），把后续批次换到数据盘（改 `--dataset.root`）。

### 1.6 关掉所有实时预览服务（新坑，检查实测发现）

- **录数据期间绝不能开预览**：`dual_cam_live_server_v2.py` 会持续读 `/dev/video4`（W19），
  抢占腕部相机，导致 lerobot 打不开 wrist；旧的 `cam_live_server.py` 会读 `/dev/video10`（global），同样干扰。
- 开录前确认没有预览进程在跑：
  ```bash
  ps aux | grep -E "cam_live_server|dual_cam_live_server" | grep -v grep   # 应无输出
  ```
- 要看构图，就录前看 `verify_dual_camera.py` 存的静态图 `/tmp/global_check.png`、`/tmp/wrist_check.png`，
  **别开实时预览**。

---

## 2. 红块位置随机化方案（照这个摆，别随手放）

> 旧数据失败的根因之一是「红块位置固定 → 角落成功率暴跌」。这一节给出**可执行的格点清单**，
> 每集换一个初始位 + 一个目标位，确保位置覆盖而不是无意识都放正前方。

### 2.1 先画好 9 个格点（开录前 5 分钟画）

- 物块规格：直径 **3.6cm 红色物块**（与仿真一致，来源 `录制执行手册.md` §6）。
- 以「从臂底座正前方、台面上可抓取区」为基准，画一个 **3 列 × 3 排** 网格，格点间距 **约 6cm**，
  整体覆盖约 12cm × 12cm（落在 ~192cm² 可抓取区内，来源 `真机录制checklist.md` §6）。
- 用纸胶带/记号笔把 9 个点标上编号（1~9），目标位和初始位共用这 9 个点。
- **四周留 2-3cm 边距**：别把点画到可达区最边缘，否则臂会顶到肘/肩限位（旧数据 27/31 集 elbow 顶 100、
  28/31 集 shoulder_lift 顶 -99，就是这个坑）。

```
（俯视台面，臂底座在下方）

           远 F   [7] 远左    [8] 远中    [9] 远右
           中 M   [4] 中左    [5] 中中    [6] 中右
           近 N   [1] 近左    [2] 近中    [3] 近右

           列      左 L        中 C        右 R
```

### 2.2 录前必做：9 个点全部「够得到 + 看得清」

录第一集之前，把臂手动摆到 9 个点逐一确认：
1. 每个点都**够得到**，且肘不顶到极限（物理最直/最紧都留一点余量）；
2. 每个点的红块在 **global 和 wrist 两路画面里都清晰**（不遮挡、不反光）。
3. 有哪个点够不到/看不清，就把网格整体往可达区中心收一点，重新标点。

### 2.3 每集怎么摆（一个 9 集循环，重复约 5 轮半 = 50 集）

- 第 k 集（1 起算）的**初始位** = 第 `((k-1) mod 9) + 1` 号点；**目标位** = 下表的「目标点」。
- 初始位与目标位**永远不是同一个点**，且近/远/左/右、短距离/长距离都覆盖到。

| 集号 | 初始位（红块起点） | 目标位 | 覆盖的方向 |
|------|--------------------|--------|------------|
| 1 | 1 近左 | 9 远右 | 长对角线 |
| 2 | 2 近中 | 6 中右 | 短右移 |
| 3 | 3 近右 | 7 远左 | 长对角线 |
| 4 | 4 中左 | 8 远中 | 前伸 |
| 5 | 5 中中 | 3 近右 | 短对角 |
| 6 | 6 中右 | 7 远左 | 对角 |
| 7 | 7 远左 | 2 近中 | 回拉 |
| 8 | 8 远中 | 4 中左 | 侧移回拉 |
| 9 | 9 远右 | 1 近左 | 长对角线 |

- 第 10 集回到第 1 行（初始位=1，目标位=9），如此循环。50 集 = 5 轮完整 + 5 集，
  保证每个初始位、每个目标位各被用到 ≥5 次。
- **复位段**（录完一集后提示 `Reset the environment`）：把红块摆到**下一集对应的新初始位**，
  目标位也按表换掉。别图省事留在原地。

---

## 3. 统一动作序列（每集必须走全这 6 步，一步都不能省）

> 旧数据 11/31 集「夹爪几乎不动」、动作被稀释。本次**每一集都严格按下面的顺序走**，
> 保证夹爪有完整的「张开 → 闭合 → 张开」循环。

**每集开始前的 home 位姿**（摆好再按 ENTER 开录）：
夹爪张开（gripper ≈ 100）、手臂抬高到不挡 global 视野、末端在可达区上方 10-20cm（来源 `round2_recording_sop.md` §5）。

| 步 | 动作 | 怎么判定「到位了」 | gripper 目标值 |
|----|------|--------------------|----------------|
| 1 | **移到红块上方** | wrist 里红块位于夹爪两指正中间；global 里末端悬停在红块正上方约 2-3cm | 保持张开 ≈100 |
| 2 | **张开夹爪** | 开口明显大于红块 3.6cm（两指间距 > 物块直径） | ≥80（尽量接近 100） |
| 3 | **下放** | 垂直下降，直到夹爪两指落到红块两侧、红块被「含」进开口、末端不再下降 | 保持张开 |
| 4 | **闭合夹爪** | 缓慢闭合夹住红块；此后抬起红块不滑落 | ≤20（接近 0，别继续死夹，电流减半补丁防堵转） |
| 5 | **抬起** | 先垂直抬升到红块完全离台（约 5cm+、夹爪下方无障碍），再水平移到目标位上方；移动中红块不掉 | 保持闭合 ≤20 |
| 6 | **放到目标** | 下降到目标位 → 张开夹爪（gripper 回到 ≥80/接近 100）→ 红块落在目标位 → 等半秒确认放稳 → 抬夹爪退回，回 home 位姿 | ≥80（放稳后接近 100） |

- **每集 gripper 必须走「开(≈100)→闭(≤20)→开(≈100)」完整循环**，swing（max-min）要大；
  任何一集只要夹爪基本没动，这一集就算坏集（见 §6）。
- 动作连贯、顺滑，避免单帧大突跳（verify 第 4 项会抓 >0.3 rad 单帧跳变，来源 `录制执行手册.md` §6）。

### 3.1 每集怎么开始 / 怎么结束 / 怎么进下一集（ENTER 控制）

- 终端出现 `Press ENTER to start recording episode N...`（**N 从 0 起算**，`episode 0` 是第 1 集，
  `episode 49` 是第 50 集；若用了 `--resume`，N 接着已有集数往后排）→ 摆好初始位 + home 位姿、
  **双手离开现场** → 按 **ENTER** 开录。
- 6 步全部完成、红块放稳后（约 20-30s）→ 按 **ENTER** 结束这一集。
  - 若动作太慢、快到 30s 还没做完，别拖：做到哪算哪，让它到 30s 自动停（这一集大概率判坏，按 §6 补录）。
- 出现 `Reset the environment` → 把红块摆到**下一集的新初始位**、目标位也换掉 → 按 **ENTER** 跳过复位段。
- **ENTER 一下一下按，别连按**（连按第二下会被下一条 `input()` 吃掉，导致下一集不确认直接开录）。
- 必须**在真实终端里跑**（stdin 是 TTY），别 `</dev/null` 或管道。

---

## 4. 完整录制命令

### 4.1 启动顺序（严格按此，反了 lerobot 打不开 video10，来源 `round2_recording_sop.md` §3）

```bash
# ① ASC60C ROS2 节点（新开终端）
cd ~/ascamera_ws
source /opt/ros/humble/setup.bash
# <ASC60C-SDK路径> = ASC60C SDK 源码目录（含 linux_ros/ 子目录），按本机实际解压位置替换：
export LD_LIBRARY_PATH=<ASC60C-SDK路径>/linux_ros/ros2/ascamera/libs/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
source ~/ascamera_ws/install/setup.bash
ros2 launch ascamera ascamera.launch.py

# ② 桥接（再开一个终端，必须先于 lerobot）
source /opt/ros/humble/setup.bash
python3 ~/so101_hardware/ros2_camera_bridge.py
```

### 4.2 主命令（一次性录 50 集）

```bash
source ~/miniconda3/bin/activate lerobot

python -m lerobot.record \
  --robot.type=so101_follower \
  --robot.port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_<SERIAL_B>-if00 \
  --robot.id=my_awesome_follower_arm \
  --robot.cameras='{
    "global": {"type": "opencv", "index_or_path": "/dev/video10", "width": 640, "height": 480, "fps": 12},
    "wrist":  {"type": "opencv", "index_or_path": "/dev/v4l/by-id/<W19相机by-id路径>", "width": 640, "height": 480, "fps": 30}
  }' \
  --teleop.type=so101_leader \
  --teleop.port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_<SERIAL_A>-if00 \
  --teleop.id=my_awesome_leader_arm \
  --dataset.repo_id=<user>/so101_real_round3 \
  --dataset.root=~/so101_walloss/so101_real_round3 \
  --dataset.fps=12 \
  --dataset.num_episodes=50 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=30 \
  --dataset.single_task="Grab the red block and place it on the target" \
  --dataset.push_to_hub=false
```

### 4.3 哪些参数必须改（对比 round2 命令），哪些别动

| 参数 | round2 值 | 本次值 | 说明 |
|------|-----------|--------|------|
| `--dataset.num_episodes` | 30 | **50** | 本次目标 ≥50 集 |
| `--dataset.episode_time_s` | 600 | **30** | 每集 20-30s；30 是「到点自动停」硬上限，做完按 ENTER 手动结束 |
| `--dataset.reset_time_s` | 600 | **30** | 复位段上限；摆完按 ENTER 跳过，30s 不按也会自动进下一集 |
| `--dataset.repo_id` | <user>/so101_real_round2 | **<user>/so101_real_round3** | 必须带斜杠 |
| `--dataset.root` | .../so101_real_round2 | **.../so101_real_round3** | **必须是不存在的目录** |
| `--dataset.fps` | 12 | **12（别动）** | 名义 fps；实际循环 ~6-7fps 是 round1/2 已确认的正常现象，存名义时间戳，训练可用 |
| `--robot.port` / `--teleop.port` | ttyACM1 / ttyACM0（旧） | **by-id：robot=`<SERIAL_B>` / teleop=`<SERIAL_A>`** | 从臂(执行/robot)=by-id `<SERIAL_B>`、主臂(握柄/teleop)=by-id `<SERIAL_A>`；ttyACM 编号会漂移（换线后已对调），命令一律填 by-id，别写 /dev/ttyACM* |
| 相机 dict | 同左 | **同左（别动）** | wrist 必须写 `"fps": 30`（W19 原生 30fps），global 写 `"fps": 12`；相机 dict 里的 `fps` 与 `--dataset.fps=12` 是两个独立量，wrist=30 不冲突 |
| `--dataset.single_task` | 同左 | 同左 | 别改 |
| `--dataset.push_to_hub` | false | **false（别动）** | 默认 true 会录完推 HF 报错 |

### 4.4 推荐改法：5 批 × 10 集（每 10 集能停下来自检，坏集早发现早补）

`num_episodes` 的语义是「**本次运行要录的集数**」（源码：`recorded_episodes` 每次从 0 起数），
`--resume=true` 是「**接着已有数据集追加**」。所以分批这么跑：

- **第 1 批**（fresh，目录还不存在，**不能带 resume**）：
  ```bash
  python -m lerobot.record ... --dataset.num_episodes=10 ... --dataset.root=~/so101_walloss/so101_real_round3 --dataset.push_to_hub=false
  ```
  （除 `--dataset.num_episodes=10` 外，其余参数与 §4.2 完全一致。）
- **第 2~5 批**（目录已存在，加 `--resume=true`，每批 10 集）：
  ```bash
  python -m lerobot.record ... --resume=true --dataset.num_episodes=10 ... --dataset.root=~/so101_walloss/so101_real_round3 --dataset.push_to_hub=false
  ```
- 每批结束（录满 10 集自动退出）→ 按 §5 跑一次自检 → 把坏集记下来 → 下一批多录补回。
- **注意**：`--resume=true` 只能用在「root 已存在」的数据集上；fresh 目录带 `--resume=true` 会直接报错
  （resume 分支走 `LeRobotDataset(...)` 去加载已有数据集，而 fresh 分支走 `LeRobotDataset.create(...)` 新建）。

---

## 5. 录制中每集 / 每 10 集的自检

### 5.1 每集（当集，录的时候 + 按 ENTER 结束前）

- 目视 6 步走全没走全：夹爪有没有真「开→闭→开」、红块有没有真拿起来、放稳到目标位。
- 时长：约 20-30s（**不是 9s**）。做完也别急着按 ENTER，动作放慢/到位，确保 ≥20s。
- 肘/肩别顶限位：目视臂别推到物理极限。
- 开录瞬间 + 复位段别让手入镜。

### 5.2 每 10 集（批次结束，停录后跑）

```bash
source ~/miniconda3/bin/activate lerobot
python ~/so101_walloss/verify_real_dataset.py --src ~/so101_walloss/so101_real_round3
```

**每 10 集还要跑 gripper 同步门禁**（不能只看 verify_real_dataset.py：它的 check[3] 只查
gripper ∈ [0,100]，旧坏数据 gripper 落在 [16.7,69.3] 也在区间内会误 PASS，必须靠这个抓）：

```bash
~/miniconda3/envs/lerobot/bin/python ~/so101_walloss/verify_gripper_sync.py ~/so101_walloss/so101_real_round3 --verbose
```

- 退出码 0 = gripper 同步 PASS（slope≈1、intercept≈0、corr>0.90、lag≤1）；退出码 1 = 这批 gripper 又坏了，
  回 §1.4 ① 重标定再补录，别拿去训练。

**看哪些指标（重要，别被 check[5] 误导）**：

| 检查项 | 通过口径（双相机数据） |
|--------|------------------------|
| [1] 基本统计 | fps=12 会记 **WARN**（<30Hz 遥操作可接受），不是 FAIL |
| [2] state/action 维度=6 + joint 顺序 | **必须 PASS** |
| [3] 关节数值范围 | **必须 PASS**：前 5 关节全在 [-100,100]、gripper 全在 [0,100]（软容差带内只记 INFO）；出现 body 越界=FAIL |
| [4] 相邻帧瞬移(掉帧) | WARN 可接受（阈值 0.3 rad 逐关节换算）；若大量瞬移帧说明动作突跳/掉帧，回去查 |
| [5] 相机 key | **双相机固定 FAIL，是已知误报**（脚本硬编码只认单相机 global），忽略这一项 |
| [6]/[6a]/[6b] | INFO + wrist_roll 甜点（不在甜点窗记 WARN，不致命） |
| [7] norm_stats | 缺失记 **WARN（预期）**，训练前再生成 |

- **结论只看**：`[2]`、`[3]` 必须 PASS，body 无越界；`[1]`fps、`[4]`、`[7]` 的 WARN 都是预期。
- **相机 key 的正确性**不靠 verify，靠 §7 的 `unify_camera_keys_round2.py --verify`
  （改名后必须打印 `global.rgb + wrist.rgb` 都在）。

### 5.3 量化肘/肩饱和 + gripper 摆动（可选，想精确判定坏集时用）

`verify` 的第 [3] 项只查「越界」，**不查「顶在限位边缘」**。要精确抓旧数据那两类坏（肘顶 100 /
shoulder_lift 顶 -99 / 夹爪几乎不动），用 `~/so101_walloss/audit_scripts/` 里的脚本（口径来自诊断）：

```bash
# 肘/肩限位饱和：把脚本顶部 DATA = "..." 改成 ~/so101_walloss/so101_real_round3 再跑
python ~/so101_walloss/audit_scripts/audit_round2d.py
# 看：per-episode elbow max / frac>=95（应接近 0），shoulder_lift min / frac<=-90（应接近 0）

# gripper 摆动：同样先改 DATA= 再跑
python ~/so101_walloss/audit_scripts/audit_round2.py
# 看：per-episode 的 min/max/range（range=max-min 大 = 开合幅度大）与 n_close/n_open（有开合帧）；
# 判定：max 高 ≈ 张开过、min 低 ≈ 闭合过、range 大；range<20 判坏（对应 §6.1 #1）
```

> 注意：这两个脚本硬编码了旧目录 `so101_real_round2_rgb` 且只 glob `data/chunk-000/*.parquet`。
> 对 50 集新数据（都在 chunk-000）改 `DATA=` 后即可复用；它们只是分析脚本，不会改数据。

---

## 6. 坏集剔除标准（哪些算坏集 + 当场怎么处理）

### 6.1 坏集判定表

| # | 坏集类型 | 判定阈值/现象 | 出处 |
|---|----------|---------------|------|
| 1 | 夹爪全程几乎没动 | 每集 gripper action 的 swing=max-min 太小（< ~20），或没有「开(>55)→闭(<30)→开」完整循环 | 诊断「11/31 集 gripper 几乎不动」+ audit_scripts 口径 |
| 2 | 肘/肩顶限位 | elbow_flex 有帧 ≥95（接近 +100），或 shoulder_lift 有帧 ≤-90（接近 -99） | 诊断「elbow 27/31 顶 100、shoulder_lift 28/31 顶 -99」+ audit_round2d.py / monitor_elbow.py 削顶 |
| 3 | 红块没拿起来 / 中途掉落 / 没放稳 | 目视：抬起时红块不在夹爪里、搬运中掉落、放下时没进目标位 | round2 SOP §8 |
| 4 | 动作乱序 / 序列不完整 | 没走全 §3 的 6 步（少了张→闭、或直接拖拽没夹） | 本次统一序列要求 |
| 5 | 时长太短 | 单集 < 15s（旧数据 9s 就是这个坑） | 诊断「每集 9s」 |
| 6 | 瞬移/大幅突跳 | verify [4] 报大量瞬移帧（单帧 >0.3 rad） | 手册 §6 + verify check[4] |
| 7 | 相机异常 | 某相机黑屏/丢帧/手入镜 | round2 SOP §5/§8 |
| 8 | gripper 记录坏 | gripper 自检不过（state.gripper 与 action.gripper 不对应） | 诊断「gripper 是坏信号」；判定靠 §1.4 的 recalibrate + verify_gripper_sync |

### 6.2 当场怎么处理

1. **录的过程中发现坏 → 立即按 ENTER 结束这一集**（别把坏动作继续录进去）。
2. **优先试 LEFT 重录本条**：本机 record.py 里按 LEFT 会 `clear_episode_buffer()`（这一条**不落盘**），
   回到 `Press ENTER to start recording episode...` 重新录这一集。
   ⚠️ **pynput 方向键在本机不可靠**（`录制执行手册.md` §2.1）。**录第一集时先试一下 LEFT 是否生效**：
   录到一半按 LEFT，看终端是否回到 `Press ENTER to start...`。生效就一直用它；不生效走第 3 步。
3. **LEFT 不生效的兜底**：这一条已保存成坏集 → 在下面的《坏集记录表》记下 episode 序号 + 原因 →
   后续批次用 `--resume` **多录一集补回来**。最终目标是 **≥50 集全部通过 §5 自检**（坏集不计入 50）。
4. 每 10 集批量 audit（§5）后，把坏集清单拉出来，下一批优先补录。

> 诚实说明：目前**没有现成的「按集删除」脚本**，所以最稳的路线是「当集抓坏、当集重录」（LEFT），
> 别把坏集拖到最后再想删。LEFT 也不可靠时，就靠「多录替补集」把好集凑到 ≥50。

### 6.3 坏集记录表（录的时候旁边放一张，逐条记）

| 批次 | 提示里的 episode 号 | 坏因（对照 §6.1 编号） | 已补录？ | 替补集 episode 号 |
|------|---------------------|------------------------|----------|-------------------|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |
| 5 | | | | |

---

## 7. 录制完的后处理（录满 ≥50 集后，按顺序做）

```bash
source ~/miniconda3/bin/activate lerobot

# ① 全量 verify（录完后再跑一次，确认 [2][3] PASS、body 无越界；check[5] 双相机误报忽略）
python ~/so101_walloss/verify_real_dataset.py --src ~/so101_walloss/so101_real_round3

# ② 相机 key 统一：global / wrist 都加 .rgb，输出到新目录（源数据不动，推荐）
python ~/so101_walloss/unify_camera_keys_round2.py \
  --src ~/so101_walloss/so101_real_round3 \
  --dst ~/so101_walloss/so101_real_round3_rgb \
  --verify
#    --verify 末尾必须打印：meta.camera_keys = ['observation.images.global.rgb',
#    'observation.images.wrist.rgb']，且 total_episodes >= 50。
#    备选（就地改、先自动备份）：--src .../so101_real_round3 --backup --verify

# ③ 重算归一化统计（只读 6 维关节，与相机无关；输出到 round3 专属文件，别覆盖 round2 的）
python ~/so101_walloss/compute_norm_stats_round2.py \
  --data_root ~/so101_walloss/so101_real_round3_rgb \
  --output_path ~/so101_walloss/norm_stats_round3.json
```

- ② 的 `--verify` 是双相机 camera key 的**权威校验**（改完必须同时有 global.rgb + wrist.rgb，
  缺一路会直接 SystemExit 报错）——verify_real_dataset.py 的 check[5] 对双相机不可信，以这里为准。
- ③ 之后把训练 `config.yml` 的 `root` / `norm_stats_path` 指到 `so101_real_round3_rgb` /
  `norm_stats_round3.json`（参考 `录制执行手册.md` §5；那一步属于训练准备，不在本录制 SOP 范围内）。

---

## 附：本次补录修复的 4 层问题对照（自我核对用）

| 问题（诊断） | 本 SOP 对应动作 |
|--------------|-----------------|
| 数据量 31 集/9s 集 | §4 num_episodes=50、episode_time_s=30 → 每集 20-30s |
| 红块位置固定 | §2 九宫格随机化 + 近/远/左/右全覆盖 |
| 动作顺序不统一 / 夹爪不动 | §3 强制 6 步 + 每集「开→闭→开」循环 + §6 剔除 |
| gripper 记录坏信号 | §1.4 录前 recalibrate 自检 + 录后 verify_gripper_sync.py 同步门禁 |
| 帧率 12fps（成功案例 30fps） | **本次沿用 12fps 链路（round2 现状）**；30fps 升级是另一条独立工作，不在本 SOP 范围 |
