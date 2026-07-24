# 遥操作数据采集、对齐与转换完整指南

## 目录

1. [系统概述](#1-系统概述)
2. [前置检查](#2-前置检查)
3. [启动遥操作](#3-启动遥操作)
4. [启动数据采集](#4-启动数据采集)
5. [键盘控制采集](#5-键盘控制采集)
6. [采集流程](#6-采集流程)
7. [产出数据格式](#7-产出数据格式)
8. [后处理：时间对齐](#8-后处理时间对齐)
9. [后处理：LeRobot 格式转换](#9-后处理lerobot-格式转换)
10. [回放验证](#10-回放验证)
11. [常见问题](#11-常见问题)

---

## 1. 系统概述

```
┌─────────────────────────────────────────────────────┐
│  Quest3 VR → UDP(9000) → quest3_udp_mocap           │
│    ├─ 腕部位姿 → nero_teleop_node → IK → CAN → 臂    │
│    └─ 手部landmark → xhand_retargeting → USB → 手    │
│                                                       │
│  [T2 终端] data_collect_node                            │
│    订阅: arm joint_states + hand xhand_state            │
│    相机: 独立线程采集 3× Gemini (335+305×2)              │
│    → robot_data.h5 + camera_data.h5 + metrics.h5       │
│                                                       │
│  [T3 终端] collect_control.py                           │
│    键盘 → TCP socket → /tmp/collect_cmd → T2            │
└─────────────────────────────────────────────────────┘
```

## 2. 前置检查

**每次启动前执行**：

```bash
# 1. 手部串口
ls /dev/ttyUSB* && sudo chmod 777 /dev/ttyUSB*

# 2. CAN 接口
bash find_all_can_port.sh && bash can_muti_activate.sh

# 3. 相机权限（如果没装 udev 规则）
for dev in /dev/bus/usb/*/*; do sudo chmod 777 $dev 2>/dev/null; done

# 4. 相机 udev 规则（一次性安装，以后不用每次跑）
sudo bash ~/.local/lib/python3.10/site-packages/pyorbbecsdk/shared/install_udev_rules.sh
# 安装后重新插拔相机 USB

# 5. Quest3 TCP 有线连接（推荐，延迟最低）
#   a. Quest3 USB 连 PC，戴上 Quest3，在弹出的"允许 USB 调试"对话框中点击允许
adb devices
#     应输出：List of devices attached
#             XXXXXXXXXX    device
#     （如果显示 unauthorized，说明 Quest3 端尚未允许）
#   b. 端口映射（Quest3 8000 → PC 8000）
adb reverse tcp:8000 tcp:8000
#     第一次执行输出：8000
#     再次执行无任何输出 → 端口已映射成功（"没有下文即可"）
#     如果反复输出 8000，说明之前映射已失效，重新执行直到无输出
#   c. 关闭防火墙（否则 TCP 连接可能被拦截）
sudo ufw disable
#   d. Quest3 HTS 应用选 TCP，地址 localhost，端口 8000
```

## 3. 启动遥操作

### 3.1 双臂双手（完整模式）

```bash
source install/setup.bash
# TCP 有线模式（推荐）
ros2 launch nero_quest_teleop teleop_full_dual.launch.py protocol:=tcp_wired
```

### 3.2 左臂左手

```bash
ros2 launch nero_quest_teleop teleop_full_left.launch.py protocol:=tcp_wired
```

### 3.3 右臂右手

```bash
ros2 launch nero_quest_teleop teleop_full_right.launch.py protocol:=tcp_wired
```

### 3.4 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `protocol` | `tcp_wired` | Quest3 传输方式：`tcp_wired`(推荐) / `udp` / `tcp_wireless` |
| `require_clench_to_start` | true | `false`=直接跟手, `true`=握拳触发 |
| `control_rate` | 80.0 | 控制频率 (Hz) |
| `motion_scale` | 0.65 | VR→臂运动缩放 |
| `data_timeout` | 5.0 | VR 超时保护 (s) |

## 4. 启动数据采集

### 4.1 双臂双手 + 三相机

```bash
source install/setup.bash
ros2 launch nero_dual_data_collect data_collect.launch.py task_name:=my_task
```

### 4.2 仅左臂左手 + overhead + 左腕相机

```bash
ros2 launch nero_dual_data_collect data_collect.launch.py \
    task_name:=my_task collect_right:=false use_cam_2:=false
```

### 4.3 仅右臂右手

```bash
ros2 launch nero_dual_data_collect data_collect.launch.py \
    task_name:=my_task collect_left:=false use_cam_1:=false
```

### 4.4 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `task_name` | default_task | 任务名（用作数据子目录） |
| `collect_left` | true | 采集左臂左手 |
| `collect_right` | true | 采集右臂右手 |
| `use_cam_0` | true | 采集 overhead Gemini 335 |
| `use_cam_1` | true | 采集左腕 Gemini 305 |
| `use_cam_2` | true | 采集右腕 Gemini 305 |

## 5. 键盘控制采集

T3 终端启动键盘控制器（不需要 source ROS2）：

```bash
python3 src/nero_dual_data_collect/scripts/collect_control.py
```

按键映射：

| 键 | 命令 | 作用 |
|---|------|------|
| `s` | start | 开始录制 |
| `p` | pause | 暂停/恢复 |
| `q` | stop | 停止并保存（臂回 init_pose，手回 HOME） |
| `d` | discard | 丢弃当前 episode |
| `n` | next | 保存当前并开始下一个 episode |
| `ESC` | — | 退出控制器 |

> 键盘通过 TCP socket (localhost:19999) 与 T2 通信，也可用 `echo start | nc 127.0.0.1 19999` 直接发命令。

## 6. 采集流程

### 6.1 握拳模式（默认）

```
T1 启动 → 臂走 init_pose → 相机预热 → Episode 0 ready
    ↓
戴上 Quest3 → 双手握拳 0.5 秒 → *** TELEOP ARMED ***
    ↓
T3 按 s → [RECORDING] → 遥操作业 → T3 按 q
    ↓
[SAVED] → /teleop/disarm → 臂回 init_pose → 手回 HOME
    ↓
重新握拳 → ARMED → 按 s → Episode 1 ...
```

### 6.2 直接模式

启动时加 `require_clench_to_start:=false`，戴上 Quest3 直接跟手，跳过握拳步骤。

### 6.3 多 Episode 采集

```
握拳 → s → 录 → q（保存）
握拳 → s → 录 → q（保存）
握拳 → s → 录 → d（做错了，丢弃）
握拳 → s → 录 → n（保存并自动跳到下一个）
...
Ctrl+C 退出（臂归零）
```

## 7. 产出数据格式

数据保存在 `~/xnero_data/Data/<task_name>/episode<N>/`：

```
episode0/
├── robot_data.h5      ← 机器人本体数据
│   ├── left_arm/joints        (N, 7)  float32  左臂 7 关节角 (rad)
│   ├── right_arm/joints       (N, 7)  float32  右臂 7 关节角
│   ├── left_hand/joints       (N,12)  float32  左手 12 关节角
│   ├── right_hand/joints      (N,12)  float32  右手 12 关节角
│   └── timestamps             (N,)    float64
├── camera_data.h5     ← 相机图像数据
│   ├── cam_0/images           (M,)    vlen uint8 JPEG
│   ├── cam_0/timestamps        (M,)    float64
│   ├── cam_1/...
│   └── cam_2/...
└── metrics.h5          ← 延时/精度指标（非必需）
```

- robot 数据 ~20Hz（独立线程读 CAN）
- camera 数据 ~30fps（独立线程采集）
- 两者时间戳独立，需后续对齐

## 8. 后处理：时间对齐

### 8.1 运行对齐

```bash
source install/setup.bash
python3 -m nero_dual_data_collect.align_data \
    --data_dir ~/xnero_data/Data/my_task
```

### 8.2 可选参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--resize_width` | 640 | 输出图像宽度 |
| `--resize_height` | 360 | 输出图像高度 |
| `--downsample` | 1 | 降采样因子 (2=隔帧保留一半) |
| `--start_episode` | 0 | 从第几个 episode 开始 |

### 8.3 对齐算法

```
cam_0 时间戳 (30Hz) → 主时钟
    │
    ├─ find_nearest_idx(robot_ts, cam_ts) → state(t)
    ├─ find_nearest_idx(robot_ts, next_cam_ts) → action(t) = state(t+1)
    └─ 提取对应相机帧 → frames/cam{N}/{frame_idx:06d}.jpg

末尾追加 20 帧 hold (静止姿态)
最后一帧丢弃 (无 t+1 action)
```

### 8.4 对齐后产出

```
episode0/
├── aligned_data.h5
│   ├── observation/left_arm_joints   (F, 7)
│   ├── observation/right_arm_joints  (F, 7)
│   ├── observation/left_hand_joints  (F,12)
│   ├── observation/right_hand_joints (F,12)
│   ├── action/left_arm_joints        (F, 7)
│   ├── action/right_arm_joints       (F, 7)
│   ├── action/left_hand_joints       (F,12)
│   ├── action/right_hand_joints      (F,12)
│   └── timestamps                    (F,)
└── frames/
    ├── cam0/{000000..}.jpg    ← overhead Gemini 335
    ├── cam1/{000000..}.jpg    ← 左腕 Gemini 305
    └── cam2/{000000..}.jpg    ← 右腕 Gemini 305
```

## 9. 后处理：LeRobot 格式转换

### 9.1 使用 conda 环境（推荐）

```bash
conda activate lerobot

cd ~/loopkok/xnero_ws

PYTHONNOUSERSITE=1 python3 \
    src/nero_dual_data_collect/nero_dual_data_collect/convert_to_lerobot.py \
    --data_dir ~/xnero_data/Data/my_task \
    --repo_id my_task_lerobot \
    --task_description "dual arm pick and place"
```

> `PYTHONNOUSERSITE=1` 防止系统 pip 包覆盖 conda 包。

### 9.2 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--data_dir` | 必填 | 对齐后的 episode 目录 |
| `--repo_id` | 必填 | LeRobot 数据集名称 |
| `--task_description` | dual arm manipulation | 任务描述 |
| `--fps` | 30 | 数据集帧率 |
| `--robot_type` | nero_xhand | 机器人类型 |
| `--output_dir` | data_dir 父目录 | 输出根目录 |

### 9.3 产出 LeRobot 格式

```
{output_dir}/{repo_id}/
├── data/chunk-000/file-000.parquet   ← 所有帧数据 (图像内嵌 PNG)
├── meta/
│   ├── info.json          ← 数据集元信息
│   ├── stats.json         ← 统计信息
│   ├── tasks.parquet      ← 任务描述
│   └── episodes/          ← episode 索引
└── images/                ← (空，新版本图像存 parquet 内)
```

### 9.4 数据集 features

```python
{
    "observation.image":               overhead 相机 (Gemini 335)
    "observation.wrist_image_left":    左腕相机 (Gemini 305)
    "observation.wrist_image_right":   右腕相机 (Gemini 305)
    "observation.state":               float32[38]   # 7+7+12+12
    "action":                          float32[38]
    "task":                            str
}
```

## 10. 回放验证

```bash
# 先停掉遥操作（释放 CAN 总线）
python3 -m nero_dual_data_collect.replay_episode \
    --episode_dir ~/xnero_data/Data/my_task/episode0

# 对齐后的数据
python3 -m nero_dual_data_collect.replay_episode \
    --episode_dir ~/xnero_data/Data/my_task/episode0 --aligned
```

## 11. 常见问题

| 问题 | 解决 |
|------|------|
| Quest3 无数据 | ① `adb devices` 确认设备和授权 ② `adb reverse tcp:8000 tcp:8000` 确认端口映射（无输出即成功）③ `sudo ufw disable` 关防火墙 ④ 确认 HTS 应用选 TCP，localhost:8000 |
| 手不动 | `ls /dev/ttyUSB*` 检查串口, `sudo chmod 777 /dev/ttyUSB*` |
| 臂不动 | `bash find_all_can_port.sh && bash can_muti_activate.sh` |
| 相机打不开 | `ls /dev/bus/usb/*/*` 检查设备, 装 udev 规则 |
| 握拳不触发 | 降低 `clench_angle_threshold` 到 30°, 手指需持续握紧 |
| T3 按键无响应 | 检查 T2 是否打印 "TCP server: localhost:19999" |
| robot_data 0 帧 | 检查 T1 是否打印 "Feedback thread started @ 20Hz" |
| 转换报错 | 确认 `conda activate lerobot` + `PYTHONNOUSERSITE=1` |
