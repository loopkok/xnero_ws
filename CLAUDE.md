# CLAUDE.md — Nero 双臂 + XHand 双手 VR 遥操作系统

## 项目概述

基于 **Meta Quest 3 + ROS 2 Humble** 的双臂双灵巧手 VR 遥操作系统（Teleoperation）。
戴上 Quest 3 VR 头显，手部动作通过 UDP/TCP 串流到 ROS 2，经手部重定向和机械臂 IK 逆解后，实时控制：
- **2× Nero 7-DOF 机械臂** — CAN 总线（pyAgxArm SDK），`move_js` MIT 透传模式
- **2× XHand 12-DOF 灵巧手** — USB 串口（C++ 驱动），3M baud

附完整工具链：数据采集 → 时间对齐 → LeRobot 格式转换 → CAN 直驱回放 → 延迟/精度量化。

**工作空间路径**：`/home/loopkok/loopkok/xnero_ws/`（ROS 2 workspace，colcon 构建）

---

## 构建系统

### 构建工具

使用 **colcon** 构建 ROS 2 workspace，`--symlink-install` 方式安装（Python 文件修改即生效，无需重新构建）。

```bash
cd /home/loopkok/loopkok/xnero_ws
colcon build --symlink-install
source install/setup.bash
```

### 构建产物

- `build/` — 构建中间文件（每个包一个子目录）
- `install/` — 安装产物（含 `setup.bash`、`local_setup.bash` 等）
- `log/` — colcon 构建日志（按时间戳命名）

### 包依赖关系

```
pyAgxArm (独立 Python SDK，无 ROS 依赖)
    ↑
nero_quest_teleop ──→ quest3_hand_mocap
    ↑                       ↑
nero_dual_data_collect ─────┤
    ↑
xhand_control_ros2 (C++) ←── xhand_retargeting
                                  ↑
                            quest3_hand_mocap
```

---

## 仓库结构

```
xnero_ws/
├── src/                              ← 所有源码包
│   ├── README.md                     ← 项目主文档
│   ├── DESIGN.md                     ← 软件设计与算法详解
│   ├── quest3_hand_mocap/            ← Quest3 UDP/TCP 手部数据接收
│   ├── xhand_retargeting/            ← DexPilot 手部重定向 (landmark→关节角)
│   ├── xhand_control_ros2/           ← XHand C++ 硬件驱动 (USB 串口)
│   ├── xhand_nero_description/       ← URDF 模型描述文件
│   ├── nero_quest_teleop/            ← Nero 机械臂遥操作核心
│   ├── nero_dual_data_collect/       ← 数据采集 / 对齐 / 转换 / 回放
│   ├── pyAgxArm/                     ← AgileX 机械臂 CAN SDK
│   └── 参考文件/                     ← 参考资料
├── docs/                             ← 文档
│   └── data_collection_guide.md      ← 数据采集完整指南
├── can_muti_activate.sh              ← CAN 接口批量激活脚本
├── find_all_can_port.sh              ← CAN 端口查找脚本
├── build/                            ← colcon 构建目录
├── install/                          ← colcon 安装目录
└── log/                              ← 构建日志
```

---

## 架构与数据流

### 完整数据流图

```
Meta Quest 3 VR (Unity App)
  │ UDP :9000 or TCP :8000, ~60 Hz
  ▼
┌─────────────────────────────────────────────────┐
│ quest3_udp_mocap (Python, 单节点)                 │
│   ├─ 解析 raw data: wrist Pose + hand 63 floats │
│   ├─ 坐标转换: X反转 → ×1.05 → 居中 → SVD帧     │
│   ├─ → MANO 坐标系 @ operator2mano               │
│   ├─ EMA 平滑 (α=0.7)                            │
│   ├─ 发布: /quest3/{left,right}_wrist_pose        │
│   └─ 发布: /hand_landmarks/{left,right}           │
└─────────────────────────────────────────────────┘
        │                              │
        ▼ (PoseStamped)               ▼ (PoseArray, 21×3)
┌──────────────────────┐    ┌──────────────────────────────┐
│ nero_teleop_node ×2  │    │ xhand_dex_retargeting_node   │
│ (Python, 125Hz配置    │    │ (Python, event-driven)       │
│  实际50Hz timer)      │    │                              │
│                      │    │ DexPilot + URDF 优化          │
│ 1. PoseProcessor     │    │ → 12 关节角                   │
│    VR→臂坐标映射      │    │ → 拇指后处理修正              │
│    EMA/SLERP 平滑     │    │ → EMA 关节平滑 (α=0.7)       │
│ 2. TCP offset 补偿    │    │ → XHandCommand               │
│ 3. IKSolver (解析DH)  │    └──────────┬───────────────────┘
│    臂角参数化+连续性   │               │ XHandCommand
│    分支选择+1D QP     │               ▼
│ 4. SafetyFilter       │    ┌──────────────────────────────┐
│    限位+速度+工作空间  │    │ xhand_control_ros2 ×2 (C++)  │
│ 5. move_js CAN 下发   │    │ USB Serial 3M baud → XHand   │
└──────────┬───────────┘    └──────────────────────────────┘
           │ CAN (socketcan)
           ▼
    Nero 机械臂 ×2
```

### 数据采集管道

```
遥操作运行中:
  /nero_teleop_{left,right}/joint_states ──┐
  /{left,right}_hand/xhand_state ──────────┤
                                            ├→ data_collect_node (50Hz)
  Gemini 335 (overhead) ───┐               │    ├─ robot_data.h5
  Gemini 305 (左腕) ───────┤               │    ├─ camera_data.h5
  Gemini 305 (右腕) ───────┘               │    └─ metrics.h5
    (独立 multiprocessing)                 │

后处理:
  robot_data.h5 + camera_data.h5
    → align_data.py (最近邻时间对齐)
    → aligned_data.h5 + frames/*.jpg
    → convert_to_lerobot.py
    → LeRobotDataset (HuggingFace 格式)
```

---

## 各包详解

### 1. `quest3_hand_mocap` — VR 数据接收

**职责**：从 Quest 3 接收原始 UDP/TCP 数据，解析手部 landmark + 腕部位姿 + HMD 头部位姿，做预处理后发布 ROS 2 话题。

**入口点**：`quest3_udp_mocap`（`quest3_udp_mocap.py` 中的 `Quest3UDPMocap` 类）

**三种传输协议**（通过 `protocol` 参数切换）：

| 模式 | 参数值 | Quest3 → PC | 前置操作 | 延迟 |
|------|--------|------------|----------|------|
| UDP（默认） | `udp` | UDP → PC_IP:9000 | 无 | 低 |
| TCP 有线 | `tcp_wired` | TCP → localhost:8000 | `adb reverse tcp:8000 tcp:8000` | 最低 |
| TCP 无线 | `tcp_wireless` | TCP → PC_IP:8000 | 无 | 中 |

**预处理管道**（`process_landmarks()`）：
1. X 轴翻转（Quest3 左手系 → ROS 右手系）
2. ×1.05 缩放（匹配 MediaPipe 训练尺度）
3. 以手腕为原点居中
4. SVD 坐标帧估计（`estimate_frame_from_hand_points()`）— 对 wrist/index_mcp/middle_mcp 三点做 SVD 得到正交手腕局部坐标系。**为什么用 SVD 而非 cross-product？** 因为真实人手三点不严格共面，叉积会放大非正交偏差导致抖动；SVD 直接拟合最优平面，结果更稳定
5. `@ operator2mano` 旋转到 MANO 学术标准坐标系
6. `adaptive_retargeting_xhand()` 小指段长自适应缩放（弯曲时 1.2×，伸展时 2.2×）
7. EMA 平滑（α=0.7）

**发布话题**：
- `hand_landmarks/left`、`hand_landmarks/right`（PoseArray，21 点）
- `quest3/left_wrist_pose`、`quest3/right_wrist_pose`（PoseStamped）
- `quest3/head_pose`（PoseStamped）
- `quest3/left_hand_markers`、`quest3/right_hand_markers`（MarkerArray，RViz 可视化）

**MANO 坐标系约定**：
- 原点：手腕
- X：拇指方向
- Y：掌面法向量（掌心→手背）
- Z：手指伸展方向（指向指尖）

**operator2mano 矩阵**（左右手不同）：
```python
OPERATOR2MANO_RIGHT = [[0, 0,-1], [-1, 0, 0], [0, 1, 0]]
OPERATOR2MANO_LEFT  = [[0, 0,-1], [ 1, 0, 0], [0,-1, 0]]
```

---

### 2. `nero_quest_teleop` — 机械臂遥操作核心

**职责**：Nero 7-DOF 机械臂 VR 遥操作的完整控制管道。VR 位姿 → 坐标映射 → IK 逆解 → 安全过滤 → CAN 控制。

**核心文件**：

| 文件 | 类/功能 | 说明 |
|------|---------|------|
| `nero_teleop_node.py` | `NeroTeleopNode` | 核心遥操作 ROS 2 节点，50Hz 控制循环（777 行） |
| `ik_solver.py` | `IKSolver`, `NeroParams`, `ContinuityParams` | 解析 DH 几何 IK 求解器（纯 numpy+scipy，~500 行） |
| `pose_processor.py` | `PoseProcessor` | VR 位姿坐标映射 + EMA/SLERP 平滑 |
| `safety_filter.py` | `SafetyFilter` | 多层安全过滤（限位+速度+工作空间） |
| `latency_tracker.py` | `LatencyTracker` | 延迟/精度量化追踪 |
| `ik_solver_node.py` | 独立 IK 节点 | 可选的独立 IK 求解 ROS 2 节点（CPU 隔离用） |
| `urdf_ik_solver.py` | `URDFIKSolver` | URDF 数值 IK 求解器（IPOPT，备选方案） |
| `keyboard_vr_sim.py` | 键盘模拟器 | 用 WASD+IJKL 模拟 VR 手部数据，无 Quest3 时开发测试用 |
| `fps_counter.py` | `FPSCounter` | 滑动窗口 FPS 计数器 |
| `test_ik_solver.py` | — | IK 数学正确性测试（FK↔IK 往返、轨迹连续性） |
| `test_vr_mapping.py` | — | VR→机械臂坐标映射矩阵验证 |
| `test_dataflow.py` | — | 端到端数据流测试（无需机械臂） |
| `test_safe_teleop.py` | — | 安全实机遥操作测试（保守参数，支持 dry_run） |

**控制循环**（`NeroTeleopNode._control_loop()`，50Hz）：
```
1. PoseProcessor.process()
     VR delta (VR坐标系) → R_vr_to_arm 旋转映射 (臂坐标系)
     → EMA 位置平滑 (α=0.8) + SLERP 姿态平滑 (α=0.8)
     → motion_scale 缩放 (0.65)

2. PoseProcessor.compute_target_pose()
     T_gripper = robot_init @ delta_arm
     T_flange = T_gripper @ inv(T_flange_to_tcp)   ← TCP offset 补偿

3. SafetyFilter.check_workspace(T_flange[:3,3])
     球半径 (0.58m) + XYZ 盒式边界 clamp

4. IKSolver.solve(T_flange)
     → 7 关节角 (rad)

5. SafetyFilter.filter(sol_q, dt)
     关节限位 clamp + 角速度限幅 (max_joint_vel=0.065)

6. robot.move_js(cmd)  ──CAN──→ Nero 马达
```

**独立 feedback 线程**（20Hz）：与控制循环解耦，读 CAN 反馈，发布 joint_states / tcp_pose / metrics。

**IK 求解器**（`ik_solver.py`）：
- **方法**：Modified DH + S-W-E 臂角参数化
- **DH 参数**：7 个关节的 a, alpha, d, theta_offset（见 [DESIGN.md](src/DESIGN.md) 第 3.2 节）
- **S-W-E 三角形**：由臂角 θ₀ 参数化 elbow 位置，三角形几何解 theta4
- **连续性**：局部 θ₀ 窗口 (±0.15 rad) + 滞后锁定 (margin=0.03)，避免分支跳变
- **后优化**：1D QP 逐关节微调（scipy minimize_scalar, bounded），位姿误差 <1mm
- **性能**：连续模式 ~2-5ms，全局扫描 ~2-5ms
- **依赖**：仅 numpy + scipy，无 Pinocchio / CasADI

**Modified DH 参数**：
| 关节 i | a_{i-1} | alpha_{i-1} (deg) | d_i (m) | theta_offset (deg) |
|--------|---------|-------------------|---------|--------------------|
| 1 | 0 | 0 | 0.138 | 0 |
| 2 | 0 | 90 | 0 | -180 |
| 3 | 0 | 90 | 0.31 | -180 |
| 4 | 0 | 90 | 0 | -180 |
| 5 | 0 | 90 | 0.27 | 90 |
| 6 | 0 | 90 | 0 | 90 |
| 7 | 0 | 90 | 0.0235 | 0 |

**关节限位**：

| J1 | J2 | J3 | J4 | J5 | J6 | J7 |
|----|----|----|----|----|----|----|
| ±155° | ±100° | ±158° | -58°/+123° | ±158° | -42°/+55° | ±90° |

**安全机制**：
1. 关节限位 clamp（IK 内建 + 安全过滤器双重保护）
2. 角速度限幅（单步 max_joint_vel=0.065 rad）
3. 笛卡尔工作空间（球半径 0.58m + 可选 XYZ 盒式边界）
4. VR 数据超时保护（data_timeout=5s 后保持位置）
5. 四元数有效性检查（|norm-1| > 0.1 则丢弃）
6. 位置突变自动重校准（>2m 跳变）
7. `emergency_stop()` → `robot.electronic_emergency_stop()`

**关闭流程**（`destroy_node()`）：
1. 手部发布 HOME 位置命令
2. 三段归零：J2→90°, J4→0 → J1→0 → all→0 (J2=90°)
3. disconnect CAN

**握拳触发**（Clench-to-Start）：
- 订阅 `/teleop/armed` Bool topic（由 retargeting 节点发布）
- `require_clench_to_start=true` 时必须握拳 0.5s 才能开始遥操作
- `/teleop/disarm` 解除武装，臂回 init_pose

**VR→机械臂坐标映射矩阵**（参见配置文件 `vr_to_arm_rot`）：

| | 左臂 | 右臂 |
|---|---|---|
| VR X+ (前) → | Arm Y+ | Arm Y-（右臂 Y+ 朝后） |
| VR Y+ (外) → | Arm Z+ | Arm Z+ |
| VR Z+ (下) → | Arm X- | Arm X+ |

---

### 3. `xhand_retargeting` — 手部重定向

**职责**：将 MANO 坐标系 21 点手部 landmark 重定向为 XHand 12 关节角。

**入口点**：`xhand_dex_retargeting_node`（`XHandDexRetargetingNode` 类）

**核心算法**：DexPilot — 基于手指间 Task-Origin 参考向量的梯度下降优化，在 XHand URDF 运动学空间内最小化 `|参考向量_人手 - 参考向量_机器人|`。

**管道**：
```
PoseArray (21×3, MANO 坐标系)
  │
  ▼
DexPilot retargeting (XHand URDF 运动学空间)
  → 12 关节角 (URDF 坐标系)
  │
  ▼
URDF 关节名 → XHand 控制名 映射
  │
  ▼
右手 index_bend 取反 (URDF 轴 vs 硬件固件定义相反)
  │
  ▼
拇指后处理 (enable_thumb_fix=true):
  伸直度 = |MCP→TIP| / (|MCP→IP| + |IP→TIP|)
  straightness > 0.85 → 抑制 thumb_rota_joint2 (最多 30%)
  │
  ▼
关节级 EMA 平滑 (smoothing_alpha=0.7)
  │
  ▼
XHandCommand (12 joint angles + PID gains + effort limits)
  → /{left,right}_hand/xhand_command
```

**XHand 12 关节名**（按顺序）：
```
thumb_bend_joint, thumb_rota_joint1, thumb_rota_joint2,
index_bend_joint, index_joint1, index_joint2,
mid_joint1, mid_joint2,
ring_joint1, ring_joint2,
pinky_joint1, pinky_joint2
```

**控制参数**：KP=80.0, KI=0.0, KD=0.0, effort_limit=400mA, mode=3

**HOME 位置**（角度制）：
```
thumb: 0°, 80.66°, 33.2°
index: 0°, 5.11°, 5.0°
mid: 6.53°, 5.0°
ring: 6.76°, 5.0°
pinky: 10.13°, 5.0°
```

**握拳检测**：手指弯曲角 > clench_threshold (40°)，持续 clench_debounce (30 帧 ≈ 0.5s) → 发布 `/teleop/armed=True`

**HOME 服务**：`~/home_right`、`~/home_left`（Trigger srv）

**配置文件**：
- `config/retargeting_params.yaml` — 运行时参数
- `config/xhand_left_dexpilot.yml` — 左手 DexPilot 配置
- `config/xhand_right_dexpilot.yml` — 右手 DexPilot 配置

**URDF 模型**：`urdf/xhand_left.urdf`、`urdf/xhand_right.urdf`（含 GLB/STL mesh 文件）

---

### 4. `xhand_control_ros2` — XHand C++ 硬件驱动

**职责**：接收 XHandCommand → USB serial → XHand 马达，回读 XHandState。

**注意**：这是 C++ 包，不是 Python。通过 USB 串口 (3M baud) 与 XHand 硬件通讯。

**命名空间**：`/left_hand`、`/right_hand`

**串口设备**：`/dev/ttyUSB0`（左手）、`/dev/ttyUSB1`（右手）

**发布话题**：`/{left,right}_hand/xhand_state`（XHandStateArray）

---

### 5. `pyAgxArm` — 机械臂 CAN SDK

**职责**：AgileX（松灵）机械臂 Python SDK。CAN 总线通讯、状态读取、运动控制。

**核心 API 用法**（在 teleop node 中的使用模式）：
```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

# 1. 建立配置
cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.DEFAULT,  # or NeroFW.V111
    channel="can_nero_left",
    interface="socketcan",
)

# 2. 创建并连接
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
robot.enable()

# 3. MIT 透传模式（低延迟，无轨迹规划）
robot.set_motion_mode("js")          # joint streaming
robot.set_auto_set_motion_mode_enabled(False)

# 4. 控制
robot.move_js([q1..q7])             # 关节角命令
robot.get_joint_angles()            # 读取当前关节角
robot.get_flange_pose()             # 读取法兰位姿
robot.set_speed_percent(25)         # 速度百分比
robot.electronic_emergency_stop()   # 电子急停
robot.set_normal_mode()             # 恢复正常模式
robot.disconnect()                  # 断开连接
```

**运动模式对比**：
- `"j"` — 关节空间轨迹规划（带加减速，较慢，用于归零/初始化）
- `"js"` — 关节串流 MIT 透传（无轨迹规划，低延迟，用于遥操作）

---

### 6. `nero_dual_data_collect` — 数据采集与后处理

**职责**：遥操作数据的实时采集、时间对齐、LeRobot 转换、CAN 回放。

| 文件 | 功能 |
|------|------|
| `data_collect_node.py` | 采集主节点（`DataCollectNode`），50Hz tick |
| `data_writer.py` | HDF5 写入（robot_data.h5） |
| `camera_manager.py` | 多相机管理（独立 multiprocessing.Process） |
| `align_data.py` | 后处理：cam_0 主时钟 + 最近邻对齐 |
| `convert_to_lerobot.py` | 转 LeRobotDataset（HuggingFace 格式） |
| `replay_episode.py` | CAN 直驱回放 |
| `keyboard_controller.py` | 终端键盘显示 + 颜色输出 |
| `scripts/collect_control.py` | T3 终端键盘控制器（TCP socket → T2） |

**相机配置**（`config/data_collect.yaml`）：

| 相机 | 型号 | 分辨率 | FPS | 位置 |
|------|------|--------|-----|------|
| cam_0 | Gemini 335 | 1280×720 | 30 | overhead（俯瞰） |
| cam_1 | Gemini 305 | 640×480 | 30 | 左腕 |
| cam_2 | Gemini 305 | 640×480 | 30 | 右腕 |

**采集状态机**：`idle → recording → paused → idle`

**键盘控制**：[s]开始 [p]暂停 [q]保存 [d]丢弃 [n]下一集

**时间对齐算法**：cam_0 时间戳 (30Hz) 作为主时钟，对每个相机帧找最近的 robot 状态（observation），下一帧找最近的 robot 状态（action），末尾追加 20 帧 hold。

**LeRobot 输出格式**：38 维 state = 7(左臂)+7(右臂)+12(左手)+12(右手)，3 路相机图像嵌入 parquet。

---

### 7. `xhand_nero_description` — URDF 模型描述

**职责**：XHand + Nero 的 URDF 模型文件。供 RViz 可视化和 URDF IK 求解器使用。

---

## 启动流程

### 前置检查（每次必做）

```bash
# 1. 手部串口
ls /dev/ttyUSB*
sudo chmod 777 /dev/ttyUSB*

# 2. CAN 接口
bash find_all_can_port.sh
bash can_muti_activate.sh

# 3. 相机 udev（一次性安装）
sudo bash ~/.local/lib/python3.10/site-packages/pyorbbecsdk/shared/install_udev_rules.sh

# 4. Quest3 TCP 有线连接（推荐，延迟最低）
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

### 全系统启动

```bash
# 终端 1：全系统遥操作（双臂 + 双手，TCP 有线模式）
ros2 launch nero_quest_teleop teleop_full_dual.launch.py protocol:=tcp_wired

# 终端 2：数据采集
ros2 launch nero_dual_data_collect data_collect.launch.py task_name:=my_task

# 终端 3：键盘控制器
python3 src/nero_dual_data_collect/scripts/collect_control.py
```

### 分节点启动（调试）

```bash
# 仅 VR 数据接收
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args -p arm_side:=both

# 仅手部重定向
ros2 run xhand_retargeting xhand_dex_retargeting_node

# 仅左臂遥操作
ros2 launch nero_quest_teleop teleop_single_nero.launch.py arm_side:=left

# 仅右臂遥操作
ros2 launch nero_quest_teleop teleop_single_nero.launch.py arm_side:=right
```

### 单臂启动

```bash
ros2 launch nero_quest_teleop teleop_single_nero.launch.py \
    arm_side:=left \
    can_channel:=can_nero_left
```

### 后处理

```bash
# 时间对齐
python3 -m nero_dual_data_collect.align_data --data_dir ~/xnero_data/Data/my_task

# 转 LeRobot
conda activate lerobot
PYTHONNOUSERSITE=1 python3 \
    src/nero_dual_data_collect/nero_dual_data_collect/convert_to_lerobot.py \
    --data_dir ~/xnero_data/Data/my_task --repo_id my_task

# CAN 回放
python3 -m nero_dual_data_collect.replay_episode \
    --episode_dir ~/xnero_data/Data/my_task/episode0
```

---

## ROS 2 话题速查

| 话题 | 类型 | 发布者 | 频率 |
|------|------|--------|------|
| `/hand_landmarks/left` | PoseArray | quest3_udp_mocap | ~60Hz |
| `/hand_landmarks/right` | PoseArray | quest3_udp_mocap | ~60Hz |
| `/quest3/left_wrist_pose` | PoseStamped | quest3_udp_mocap | ~60Hz |
| `/quest3/right_wrist_pose` | PoseStamped | quest3_udp_mocap | ~60Hz |
| `/quest3/head_pose` | PoseStamped | quest3_udp_mocap | ~60Hz |
| `/left_hand/xhand_command` | XHandCommand | xhand_dex_retargeting | event |
| `/right_hand/xhand_command` | XHandCommand | xhand_dex_retargeting | event |
| `/left_hand/xhand_state` | XHandStateArray | xhand_control_ros2 | ~100Hz |
| `/right_hand/xhand_state` | XHandStateArray | xhand_control_ros2 | ~100Hz |
| `/nero_teleop_left/joint_states` | JointState | nero_teleop_node | 20Hz |
| `/nero_teleop_right/joint_states` | JointState | nero_teleop_node | 20Hz |
| `/nero_teleop_left/tcp_pose` | PoseStamped | nero_teleop_node | 20Hz |
| `/nero_teleop_right/tcp_pose` | PoseStamped | nero_teleop_node | 20Hz |
| `/nero_teleop_left/metrics` | Float64MultiArray | nero_teleop_node | 20Hz |
| `/nero_teleop_right/metrics` | Float64MultiArray | nero_teleop_node | 20Hz |
| `/teleop/armed` | Bool | xhand_dex_retargeting | event |
| `/teleop/disarm` | Bool | data_collect_node | event |
| `/nero_teleop/left_target_ee` | PoseStamped | nero_teleop_node | 50Hz |
| `/nero_teleop/right_target_ee` | PoseStamped | nero_teleop_node | 50Hz |

---

## 配置参数速查

### 臂参数（`config/nero_teleop_{left,right}.yaml`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm_side` | `left` / `right` | 控制哪条臂 |
| `control_rate` | 125.0 Hz | 控制循环频率（代码中实际用 50Hz） |
| `solver_type` | `analytic_dh` | IK 方式：`analytic_dh` 或 `urdf_numerical` |
| `motion_scale` | 0.65 | VR→机械臂运动缩放 |
| `pos_smoothing` | 0.8 | 位置 EMA 系数（越大越跟手但越抖） |
| `rot_smoothing` | 0.8 | 姿态 SLERP 系数 |
| `max_joint_vel` | 0.065 | 单步最大关节角变化 (rad/step) |
| `workspace_radius` | 0.58 | 末端工作空间球半径 (m) |
| `data_timeout` | 5.0 | VR 数据超时秒数 |
| `init_pose` | [7 floats] | 启动时移动到的初始关节角 (rad) |
| `init_speed_percent` | 10 | 初始逼近速度 % |
| `tcp_offset` | [0,0,0,0,0,0] | 法兰到夹爪的变换 [x,y,z,roll,pitch,yaw] |
| `dry_run` | false | true=不实际移动机械臂 |
| `require_clench_to_start` | true | true=需握拳触发后才开始遥操作 |

**左右臂 init_pose 差异**（左右镜像）：
- 左臂：`[-0.405, 1.281, -0.957, 1.311, 2.682, -0.314, -0.163]`
- 右臂：`[0.405, 1.281, 0.957, 1.311, -2.682, 0.314, -0.163]`

**左右臂 vr_to_arm_rot 差异**：
- 左臂：`[0,1,0, 0,0,1, -1,0,0]`（VR X+→Arm Y+, VR Y+→Arm Z+, VR Z+→Arm X-）
- 右臂：`[0,1,0, 0,0,-1, 1,0,0]`（VR X+→Arm Y-, VR Y+→Arm Z+, VR Z+→Arm X+）

### 手部参数（`config/retargeting_params.yaml`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ema_alpha` | 0.7 | VR landmark EMA 平滑系数 |
| `smoothing_alpha` | 0.7 | 关节角 EMA 平滑系数 |
| `enable_thumb_fix` | false | 拇指后处理修正 |
| `clench_angle_threshold` | 40.0 | 握拳检测角度阈值 (deg) |
| `clench_debounce_frames` | 30 | 握拳去抖帧数 (~0.5s) |
| `protocol` | `tcp_wired` | Quest3 传输协议 |
| `udp_port` | 9000 | UDP 端口 |
| `tcp_port` | 8000 | TCP 端口 |

---

## 测试顺序

强烈建议按以下顺序逐步测试（参见 [nero_quest_teleop/README.md](src/nero_quest_teleop/README.md)）：

1. **`test_ik_solver.py`** — IK 数学正确性（无需硬件）。测试 FK↔IK 往返一致性（位置误差 < 10mm）、轨迹平滑性、安全过滤器
   ```bash
   ros2 run nero_quest_teleop test_ik_solver
   ```
2. **`test_vr_mapping.py`** — VR→臂坐标映射矩阵验证（无需硬件）。测试 6 方向运动映射、TCP offset 往返、端到端
   ```bash
   ros2 run nero_quest_teleop test_vr_mapping
   ```
3. **`test_dataflow.py`** — 完整数据流测试（需要 Quest3 或键盘模拟器）
   ```bash
   ros2 run nero_quest_teleop keyboard_vr_sim --ros-args -p arm_side:=left
   ros2 run nero_quest_teleop test_dataflow --ros-args -p arm_side:=left
   ```
4. **`test_safe_teleop.py`** — 安全实机测试（保守参数，支持 dry_run）
   ```bash
   ros2 run nero_quest_teleop test_safe_teleop --ros-args -p arm_side:=left -p dry_run:=false
   ```

---

## 重要惯例与注意事项

### Python 版本

- 使用 Python 3.10（系统环境）
- ROS 2 节点必须在 `setup.py` / `setup.cfg` 中注册 entry point

### 坐标系

- **VR 手部**：Quest3 左手系（X 前, Y 左外, Z 下）
- **ROS**：右手系
- **VR→ROS 转换**：X 轴翻转
- **MANO**：X 拇指方向, Y 掌面法向量, Z 指向指尖
- **Nero 左臂**：X 上, Y 前, Z 左外
- **Nero 右臂**：X 上, Y 后, Z 右外

### CAN 总线

- 接口类型：`socketcan` (Linux)
- 通道名：`can_nero_left`、`can_nero_right`
- 波特率：1 Mbps（在 `can_muti_activate.sh` 中设定）

### XHand 串口

- 左手：`/dev/ttyUSB0`
- 右手：`/dev/ttyUSB1`
- 波特率：3M baud
- 每次启动需要 `sudo chmod 777`

### 固件版本

- `default`：≤ 1.10
- `v111`：≥ 1.11
- 会影响 CAN 通讯协议

### 右手特殊处理

- `index_bend_joint` 取反（URDF 轴与硬件固件定义相反）
- 左手不需要此处理

### ROS 2 namespace

- 手部驱动：`/left_hand`、`/right_hand`（命名空间隔离）
- 臂节点：使用 topic 名称前缀而非 namespace（`/nero_teleop_left/...`）

### 数据路径

- 原始采集数据：`~/xnero_data/Data/<task_name>/episode<N>/`
- 对齐后数据：同上 + `aligned_data.h5` + `frames/`
- LeRobot 输出：`~/xnero_data/<repo_id>/`

---

## 第三方依赖

| 依赖 | 用途 | 安装方式 |
|------|------|---------|
| `dex-retargeting` | DexPilot 手部重定向算法 | `pip install dex-retargeting` |
| `pyAgxArm` | Nero CAN SDK | `pip install -e src/pyAgxArm` |
| `lerobot` | LeRobot 数据集格式 | `pip install lerobot`（conda 环境） |
| `pyorbbecsdk` | Gemini 相机 SDK | 系统 pip 安装 |
| `numpy` | 矩阵运算 | 系统依赖 |
| `scipy` | 数值优化 (1D QP) | 系统依赖 |
| `opencv-python` | 图像处理 | `pip install opencv-python` |
| `h5py` | HDF5 文件读写 | `pip install h5py` |

---

## 已知问题

| 问题 | 状态 | 说明 |
|------|------|------|
| 拇指跟随精度 | 已缓解 | DexPilot 三关节解不唯一，`enable_thumb_fix` 可缓解 |
| 物理到位检测 | 未实现 | Control 延迟 = 指令发出到编码器回读，非真正到位 |
| 左手 retargeting | 已可用 | 左右手使用不同的 `operator2mano` 矩阵 |
| DH 模型 vs 物理臂不一致 | 已处理 | 用 1D QP 后优化补偿（位姿误差 <1mm） |
| TCP offset 默认为零 | 待配置 | 实际使用时可能需要设置夹爪偏移 |

---

## 调试命令

```bash
# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /hand_landmarks/right
ros2 topic hz /nero_teleop_left/joint_states

# 查看话题内容
ros2 topic echo /right_hand/xhand_command
ros2 topic echo /nero_teleop_left/joint_states

# 节点图
rqt_graph

# 手部回零
ros2 service call /xhand_dex_retargeting_node/home_right std_srvs/srv/Trigger
ros2 service call /xhand_dex_retargeting_node/home_left std_srvs/srv/Trigger

# 直接关节控制（验证硬件）
ros2 run xhand_retargeting test_direct_control --ros-args -p hand_side:=right

# 检查 CAN 接口
bash find_all_can_port.sh
ip link show can_nero_left
ip link show can_nero_right
```
