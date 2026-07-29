# Quest3 → Nero 双臂 + XHand 双手 遥操作系统

基于 Meta Quest3 VR 头显手部追踪，控制两台 Nero 7-DOF 机械臂和两只 XHand 12-DOF 灵巧手。包含数据采集、延时量化、回放等完整工具链。

---

## 1. 系统架构

```
Quest3 VR (UDP:9000 / TCP:8000)
  │
  ├─ 手腕位姿 (PoseStamped) ──→ nero_teleop_node (解析DH IK + 安全滤波)
  │                                   │ CAN (pyAgxArm)
  │                                   ▼ Nero 机械臂 x 2
  │
  ├─ 头部 HMD (PoseStamped) ──→ /quest3/head_pose
  │
  └─ 手部 landmark (63 floats) ──→ quest3_udp_mocap ──→ xhand_dex_retargeting
                                         │                    │ DexPilot + URDF
                                         │                    ▼ XHandCommand
                                         │             xhand_control_ros2
                                         │                │ USB Serial
                                         │                ▼ XHand 灵巧手 x 2
                                         │
                                    [数据采集] nero_dual_data_collect
                                         │
                                    robot_data.h5 + camera_data.h5 + metrics.h5
                                         │
                                    [后处理] align → convert → replay
```

### 子包说明

| 包 | 职责 |
|----|------|
| `quest3_hand_mocap` | Quest3 数据接收（UDP/TCP有线/TCP无线），手部 landmark 解析 + 腕部位姿 + 头部追踪发布 |
| `xhand_retargeting` | DexPilot 重定向：landmark → XHand 12 关节角 |
| `xhand_control_ros2` | XHand C++ 硬件驱动（USB 串口） |
| `nero_quest_teleop` | Nero 机械臂遥操：解析 DH IK + 安全滤波 + CAN 控制 |
| `pyAgxArm` | AgileX 机械臂 Python SDK（CAN 通信层） |
| `nero_dual_data_collect` | 数据采集 / 对齐 / 转换 / 回放 / 延时量化 |

---

## 2. 文件结构

```
src/
├── quest3_hand_mocap/              # Quest3 UDP 手部数据接收
├── xhand_retargeting/              # 手部重定向 (DexPilot)
├── xhand_control_ros2/             # XHand C++ 硬件驱动
├── nero_quest_teleop/              # Nero 机械臂遥操
│   ├── nero_quest_teleop/
│   │   ├── nero_teleop_node.py     # 核心遥操节点
│   │   ├── ik_solver.py            # 解析DH几何IK
│   │   ├── pose_processor.py       # VR位姿预处理
│   │   ├── safety_filter.py        # 多层安全过滤
│   │   └── latency_tracker.py      # 延时/精度量化
│   ├── config/                     # 左/右臂配置
│   └── launch/                     # 单臂/双臂/全系统 launch
├── nero_dual_data_collect/         # 数据采集与处理
│   ├── nero_dual_data_collect/
│   │   ├── data_collect_node.py    # 采集主节点
│   │   ├── camera_manager.py       # 多相机管理 (Gemini 335/305)
│   │   ├── data_writer.py          # HDF5 写入
│   │   ├── align_data.py           # 后处理：时间对齐
│   │   ├── convert_to_lerobot.py   # 转 LeRobotDataset
│   │   └── replay_episode.py       # CAN 直驱回放
│   ├── config/                     # 相机/路径配置
│   └── launch/
└── pyAgxArm/                       # AgileX 机械臂 CAN SDK
```

---

## 3. 环境配置

### 3.1 依赖

```bash
# Python 依赖
pip install numpy scipy opencv-python h5py
pip install dex-retargeting

# pyAgxArm (CAN SDK)
cd src/pyAgxArm && pip install -e . && cd ../..

# lerobot (转换用，可选)
pip install lerobot
```

### 3.2 构建

```bash
cd ~/loopkok/xnero_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 4. 启动方式

### 4.0.0 Quest3 初始化设置（首次配置，只需做一次）

**前置条件**：Quest3 需进入**开发者模式**，建议通过 **SideQuest** 安装 HTS（Hand Tracking Streamer）软件。

**Quest3 端设置**：

1. **设置 → 开发者** → 将「覆盖手势追踪频率」开到**高**
2. **设置 → Link** → 将「USB 连接」**打开**
3. Quest3 用 **USB 有线连接** PC

**首次遥操前的数据验证**：

> 第一次遥操前，必须先用 UDP 模式验证 Quest3 坐标轴与实际运动的一致性，否则可能出现手往右移但数据往前跑的边界错乱问题。

1. Quest3 端进入 HTS 软件，选择 **UDP** 模式
2. 查看 HTS 的**数据面板**，分别沿三个轴移动手：
   | 移动方向 | 对应轴 | 颜色 |
   |---------|--------|------|
   | 前后移动 | X 轴 | 红色 |
   | 左右移动 | Y 轴 | 绿色 |
   | 上下移动 | Z 轴 | 蓝色 |
3. **如果移动方向与数据变化不一致**（例如手左右移但 X 轴数据变了），说明**边界出错**，需清除：
   - Quest3 → **设置 → 环境设置 → 边界** → **清除边界记录**
   - Quest3 → **设置 → 隐私与安全 → 设备权限 → 空间数据** → **清除实际空间记录**
   - 回到 HTS 数据面板，再次验证三个轴是否正确
4. 数据验证通过后，切换到 **TCP** 模式，地址 `localhost`，端口 `8000`

### 4.0 启动前检查（每次必做）

```bash
# 1. 检查手部串口
ls /dev/ttyUSB*
sudo chmod 777 /dev/ttyUSB*

# 2. 检查并激活 CAN 接口
bash find_all_can_port.sh
bash can_muti_activate.sh

# 3. Quest3 TCP 有线连接（推荐，延迟最低）
#   a. Quest3 USB 连 PC，戴上 Quest3，在弹出的"允许 USB 调试"对话框中点击允许
adb devices
#     应输出：List of devices attached
#             XXXXXXXXXX    device
#     （如果显示 unauthorized，说明 Quest3 端尚未允许）
#   b. 端口映射（Quest3 8000 → PC 8000）
adb reverse tcp:8000 tcp:8000
#     第一次输出：8000
#     再次执行无输出 → 成功（"没有下文即可"）
#     如果反复输出 8000，重新执行直到无输出
#   c. 关闭防火墙
sudo ufw disable
#   d. Quest3 HTS 应用选 TCP，地址 localhost，端口 8000
```

### 4.1 全系统遥操（双臂 + 双手）

```bash
# TCP 有线模式（推荐）
ros2 launch nero_quest_teleop teleop_full_dual.launch.py protocol:=tcp_wired

# UDP / TCP 无线模式
ros2 launch nero_quest_teleop teleop_full_dual.launch.py
```

### 4.2 单臂遥操

```bash
ros2 launch nero_quest_teleop teleop_single_nero.launch.py arm_side:=left
```

### 4.3 仅手部遥操

```bash
ros2 launch xhand_retargeting quest3_xhand_teleop.launch.py
```

### 4.4 遥操 + 数据采集

```bash
# 终端1：遥操（TCP 有线）
ros2 launch nero_quest_teleop teleop_full_dual.launch.py protocol:=tcp_wired

# 终端2：数据采集
ros2 launch nero_dual_data_collect data_collect.launch.py task_name:=my_task
```

### 4.5 分节点启动（调试用）

```bash
# 终端1：VR数据接收
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args -p arm_side:=both

# 终端2：手部重定向
ros2 run xhand_retargeting xhand_dex_retargeting_node

# 终端3：右手硬件
ros2 run xhand_control_ros2 xhand_control_ros2_node --ros-args \
  -r __ns:=/right_hand -p port_name:=/dev/ttyUSB1

# 终端4：左手硬件
ros2 run xhand_control_ros2 xhand_control_ros2_node --ros-args \
  -r __ns:=/left_hand -p port_name:=/dev/ttyUSB0
```

### 4.6 数据后处理

```bash
# 时间对齐
python3 -m nero_dual_data_collect.align_data --data_dir ~/xnero_data/Data/my_task

# 转 LeRobot
python3 -m nero_dual_data_collect.convert_to_lerobot \
    --data_dir ~/xnero_data/Data/my_task --repo_id my_task_lerobot

# CAN 直驱回放
python3 -m nero_dual_data_collect.replay_episode \
    --episode_dir ~/xnero_data/Data/my_task/episode0
```

### 4.7 测试（无需 VR）

```bash
# 回零
ros2 service call /xhand_dex_retargeting_node/home_right std_srvs/srv/Trigger
ros2 service call /xhand_dex_retargeting_node/home_left std_srvs/srv/Trigger

# 直接关节控制（验证硬件）
ros2 run xhand_retargeting test_direct_control --ros-args -p hand_side:=right
```

---

## 5. 手部数据管道（详细）

```
UDP 原始数据 (63 floats = 21x3)
    │
    ▼
quest3_udp_mocap:
    ├── ROS坐标系转换 (X轴翻转)
    ├── process_landmarks:
    │   ├── x1.05 缩放
    │   ├── 以手腕为原点居中
    │   ├── estimate_frame_from_hand_points (SVD)
    │   ├── @ operator2mano (MANO坐标系)
    │   └── adaptive_retargeting_xhand (pinky缩放)
    ├── EMA平滑 (ema_alpha=0.7)
    └── 发布 PoseArray on hand_landmarks/{left,right}

    │
    ▼
xhand_dex_retargeting_node:
    ├── 接收 PoseArray (21个关键点)
    ├── DexPilot retargeting (参考向量 -> 关节角度)
    │   . 使用 XHand URDF 运动学模型
    │   . low_pass_alpha=0.6 内置滤波
    ├── URDF关节名 -> XHand控制名 映射
    ├── index_bend 关节取反 (右手)
    ├── 拇指后处理 (enable_thumb_fix)
    ├── 关节级平滑 (smoothing_alpha=0.7)
    └── 发布 XHandCommand on /{left,right}_hand/xhand_command

    │
    ▼
xhand_control_ros2:
    └── 串口发送角度命令 -> XHand 电机
```

---

## 6. 参数说明

### 6.1 Nero 遥操参数

详见 [nero_quest_teleop/README.md](nero_quest_teleop/README.md)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `control_rate: 100.0 Hz) |
| `motion_scale` | 0.65 | VR到机械臂运动缩放 |
| `pos_smoothing` | 0.5 | 位置 EMA 滤波 |
| `max_joint_vel` | 0.05 | 单步最大关节角变化 |
| `workspace_radius` | 0.58 | 末端工作空间球半径 (m) |
| `solver_type` | analytic_dh | IK方式 |

### 6.2 手部参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `udp_port` | 9000 | Quest3 UDP 端口 |
| `ema_alpha` | 0.7 | EMA 平滑系数 |
| `smoothing_alpha` | 0.7 | 关节级平滑 |
| `enable_thumb_fix` | true | 拇指后处理修正 |

---

## 7. 关键算法

### 7.1 Nero IK (解析DH几何法)

S-W-E 臂角参数化 + 连续性分支选择 + 1D QP 精修。详见 `nero_quest_teleop/ik_solver.py`。

### 7.2 SVD 坐标帧估计

对 wrist/index_mcp/middle_mcp 三个 3D 点做 SVD 拟合平面，得到正交手腕局部坐标系。比传统 cross-product 方法更稳定。

### 7.3 MANO 坐标系

将 SVD 估计的局部帧旋转到学术界标准 MANO 坐标系，作为 DexPilot 重定向的统一输入。

### 7.4 DexPilot 重定向

基于手指间 Task-Origin 参考向量的优化重定向，在 XHand URDF 运动学空间内用梯度下降求最优关节角。

### 7.5 拇指后处理

检测拇指伸直度，当伸直度 > 0.85 时线性抑制 thumb_rota_joint2（最多 30%），补偿 DexPilot 解不唯一性。

---

## 8. 延时量化

遥操过程中实时打印各阶段延时和运动精度：

```
[LEFT ARM] -- Latency & Accuracy (window=100) --
  Software pipeline:
    VR->Cmd:  avg=  6.5  max= 12.0 ms
    IK:       avg=  2.8  max=  5.3 ms
    Safety:   avg=  0.1  max=  0.3 ms
  Physical control (CAN+Motor):
    === Control: avg=  5.1  max=  9.8 ms ===
  === E2E total:  avg= 18.3  max= 25.1 ms ===
  Accuracy:
    Pos err: avg= 12.3  max= 35.1 mm
    Ori err: avg=  2.15 max=  5.80 deg
```

详见 `nero_quest_teleop/latency_tracker.py`。

---

## 9. 已知问题

| 问题 | 状态 | 说明 |
|------|------|------|
| 拇指跟随精度 | 已缓解 | DexPilot 三关节解不唯一，`enable_thumb_fix` 可缓解 |
| 物理到位检测 | 未实现 | Control 延时 = 指令发出到编码器回读，非真正到位 |
| 左手 retargeting | 已可用 | 左右手用不同 `operator2mano` 矩阵 |

---

## 10. 调试

```bash
# 查看话题
ros2 topic list
ros2 topic echo /right_hand/xhand_command
ros2 topic echo /nero_teleop_left/joint_states

# 查看频率
ros2 topic hz /hand_landmarks/right

# 节点图
rqt_graph
```

## 各包详细文档

- [nero_quest_teleop](nero_quest_teleop/README.md) — 遥操系统完整文档
- [nero_dual_data_collect](nero_dual_data_collect/README.md) — 数据采集系统文档
- [pyAgxArm](pyAgxArm/README.md) — 机械臂 CAN SDK
- [xhand_control_ros2](xhand_control_ros2/README.md) — 灵巧手硬件驱动
