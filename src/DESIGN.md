# Nero 双臂 + XHand 双手 VR 遥操作系统 — 软件设计与算法文档

---

## 目录

1. [系统总览](#1-系统总览)
2. [数据流全景](#2-数据流全景)
3. [机械臂逆解算 (IK)](#3-机械臂逆解算)
4. [手部重定向 (Retargeting)](#4-手部重定向)
5. [数据采集与后处理](#5-数据采集与后处理)
6. [延时与精度量化](#6-延时与精度量化)

---

## 1. 系统总览

### 1.1 硬件组成

| 硬件 | 数量 | 接口 | 自由度 |
|------|------|------|--------|
| Meta Quest 3 VR 头显 | 1 | UDP (WiFi) | 双手 21 点 landmark + 腕部位姿 |
| Nero 机械臂 | 2 | CAN (1 Mbps) | 7 DOF × 2 |
| XHand 灵巧手 | 2 | USB 串口 (3M baud) | 12 DOF × 2 |
| Gemini 335 相机 | 1 | USB | 固定 overhead RGB-D |
| Gemini 305 相机 | 2 | USB | 腕部 RGB-D |

### 1.2 软件包

```
src/
├── quest3_hand_mocap/         UDP 接收 + 手部预处理
├── xhand_retargeting/         手部重定向 (DexPilot)
├── xhand_control_ros2/        XHand C++ 硬件驱动
├── nero_quest_teleop/         Nero 机械臂遥操 (IK + 安全滤波)
├── nero_dual_data_collect/    数据采集 / 对齐 / 转换 / 回放 / 延时量化
└── pyAgxArm/                  AgileX 机械臂 CAN SDK
```

### 1.3 进程拓扑

```
 Quest3 VR App (Unity)
   │ UDP :9000, ~60 Hz
   ▼
 quest3_udp_mocap (Python, 单进程单线程)
   │ PoseStamped           PoseArray (21×3)
   ├──────────────────┐    ┌──────────────────
   ▼                  ▼    ▼
 nero_teleop_left    nero_teleop_right    xhand_dex_retargeting (Python)
 (Python, 50Hz)      (Python, 50Hz)       (Python, event-driven)
   │ CAN                │ CAN              │ XHandCommand
   ▼                    ▼                  ▼
 Nero 左臂             Nero 右臂           xhand_control_left (C++)
                                            xhand_control_right (C++)
                                              │ USB serial
                                              ▼
                                            XHand 左手 / 右手
```

---

## 2. 数据流全景

### 2.1 手臂数据管道

```
Quest3 腕部位姿 (UDP: x,y,z,qx,qy,qz,qw)
  │
  ▼
quest3_udp_mocap.process_line()
  │ 记录 UDP 到达时间 → header.stamp
  │ 发布 /quest3/{left,right}_wrist_pose (PoseStamped)
  │
  ▼ [ROS2 pub/sub, ~1-3ms]
nero_teleop_node._vr_callback()
  │ 校验四元数范数、位置突变检测
  │ PoseProcessor.update_vr_pose(pos, rot)
  │
  ▼ [50Hz Timer]
nero_teleop_node._control_loop():
  │
  ├── 1. PoseProcessor.process()
  │        delta = vr_current - vr_zero  (VR坐标系)
  │        delta_arm = R_vr_to_arm @ delta  (旋转映射)
  │        EMA位置平滑 + SLERP姿态平滑
  │        motion_scale 缩放
  │
  ├── 2. PoseProcessor.compute_target_pose()
  │        T_gripper = robot_init @ delta_arm
  │        T_flange = T_gripper @ inv(T_flange_to_tcp)
  │
  ├── 3. SafetyFilter.check_workspace(T_flange[:3,3])
  │        球半径 + XYZ盒式边界 clamp
  │
  ├── 4. IKSolver.solve(T_flange)          ← 核心算法，见第3节
  │        → 7 关节角 (rad)
  │
  ├── 5. SafetyFilter.filter(sol_q, dt)
  │        关节限位 clamp
  │        角速度限幅 (max_joint_vel/step)
  │
  └── 6. robot.move_js(cmd)  ──CAN──→ Nero 电机
           robot.get_joint_angles()  ←──CAN── 编码器反馈
```

### 2.2 手部数据管道

```
Quest3 手部 landmark (UDP: 63 floats = 21点 × 3维)
  │
  ▼
quest3_udp_mocap.process_line()
  │ 解析 63 floats → (21,3) 数组
  │
  ├── ROS坐标转换: X轴翻转 (Quest3左手系→ROS右手系)
  │
  ├── process_landmarks():                 ← 核心预处理，见第4.1节
  │     ×1.05 缩放 (匹配MediaPipe尺度)
  │     以手腕为原点居中
  │     SVD 手腕坐标帧估计
  │     @ operator2mano (MANO坐标系对齐)
  │     adaptive_retargeting_xhand (小指段长缩放)
  │
  ├── EMA平滑 (ema_alpha=0.7)
  │
  └── 发布 /hand_landmarks/{left,right} (PoseArray, 21点)
  │
  ▼ [ROS2 pub/sub]
xhand_dex_retargeting_node._process_pose()
  │
  ├── DexPilot retargeting:               ← 核心算法，见第4.2节
  │     构建手指间 Task-Origin 参考向量
  │     XHand URDF 运动学空间优化
  │     → 12关节角 (URDF坐标系)
  │
  ├── URDF关节名 → XHand控制名 映射
  │
  ├── 右手 index_bend 取反
  │
  ├── 拇指后处理 (enable_thumb_fix):       ← 见第4.3节
  │     检测拇指伸直度
  │     抑制不合理的 thumb_rota_joint2
  │
  ├── 关节级 EMA 平滑 (smoothing_alpha=0.7)
  │
  └── 发布 /{left,right}_hand/xhand_command (XHandCommand)
  │
  ▼ [ROS2 pub/sub]
xhand_control_ros2 (C++)
  │ 接收 XHandCommand
  │ USB 串口 → XHand 电机
  │ 回读状态 → /{left,right}_hand/xhand_state
```

### 2.3 数据采集管道

```
遥操过程中:

  /nero_teleop_left/joint_states ──┐
  /nero_teleop_right/joint_states ─┤
  /left_hand/xhand_state ──────────┼──→ data_collect_node (50Hz tick)
  /right_hand/xhand_state ─────────┤       │
                                    │       ▼
  Gemini相机 ×3 ──→ CameraDataProcess ──→ robot_data.h5
  (独立multiprocessing)                  camera_data.h5
                                         metrics.h5

后处理:

  robot_data.h5 + camera_data.h5
       │ align_data.py (最近邻时间对齐)
       ▼
  aligned_data.h5 + frames/*.jpg
       │ convert_to_lerobot.py
       ▼
  LeRobotDataset (HuggingFace格式)
```

---

## 3. 机械臂逆解算 (IK)

### 3.1 算法概述

Nero 是 7-DOF 冗余机械臂。IK 采用**解析 DH 几何法 + 臂角参数化**：

1. **Modified DH 前向运动学**：建立 7 个关节的 DH 参数表
2. **S-W-E 三角形几何**：将 7 自由度冗余解耦为臂角（theta0）参数化
3. **可行域扫描**：在 [-pi, pi] 扫描 theta0，过滤不可行解
4. **连续性分支选择**：基于关节限位加权代价 + 滞后锁定，避免分支跳变
5. **1D QP 后优化**：逐关节微调，减少位姿残差

纯 numpy + scipy，无 Pinocchio/CasADI 依赖。

### 3.2 Modified DH 参数

| 关节 i | a_{i-1} | alpha_{i-1} (deg) | d_i (m) | theta_offset (deg) |
|--------|---------|-------------------|---------|--------------------|
| 1 | 0 | 0 | 0.138 | 0 |
| 2 | 0 | 90 | 0 | -180 |
| 3 | 0 | 90 | 0.31 | -180 |
| 4 | 0 | 90 | 0 | -180 |
| 5 | 0 | 90 | 0.27 | 90 |
| 6 | 0 | 90 | 0 | 90 |
| 7 | 0 | 90 | 0.0235 | 0 |

```
Modified DH 齐次变换:
  T_i^{i-1} = RotX(alpha_{i-1}) · TransX(a_{i-1}) · RotZ(theta_i) · TransZ(d_i)
```

### 3.3 关节限位

| 关节 | min (rad) | max (rad) | min (deg) | max (deg) |
|------|-----------|-----------|-----------|-----------|
| 1 | -2.705 | +2.705 | -155 | +155 |
| 2 | -1.745 | +1.745 | -100 | +100 |
| 3 | -2.758 | +2.758 | -158 | +158 |
| 4 | -1.012 | +2.147 | -58 | +123 |
| 5 | -2.758 | +2.758 | -158 | +158 |
| 6 | -0.733 | +0.960 | -42 | +55 |
| 7 | -1.571 | +1.571 | -90 | +90 |

### 3.4 S-W-E 三角形几何

```
           Shoulder (S)
              │
              │ d2 = 0.31m (上臂长)
              │
              ▼
           Elbow (E)  ←── 由 arm-angle theta0 参数化
              │
              │ d4 = 0.27m (前臂长)
              │
              ▼
           Wrist (W)   ←── 由目标位姿反算
              │
              │ d6 = 0.0235m
              ▼
           Flange (O7)
```

**步骤：**

1. **从目标位姿 T07 计算 Wrist 位置 W**：
   ```
   O7 = p_target - post_transform_d8 · z7
   W  = O7 - d6 · z7
   ```
   其中 z7 是目标法兰的 Z 轴。

2. **由三角形 S-W-E 解 theta4（肘关节）**：
   ```
   cos(theta4) = (|SW|² - d2² - d4²) / (2 · d2 · d4)
   theta4 = ±acos(cos_theta4)
   ```

3. **由 arm-angle theta0 参数化 Elbow 位置 E**：
   ```
   C = S + x · u_sw          (SW线段上距离S为x的点)
   x = (d2² - d4² + |SW|²) / (2·|SW|)
   r = sqrt(d2² - x²)         (肘圆半径)
   E = C + r·cos(theta0)·e1 + r·sin(theta0)·e2
   ```
   其中 e1, e2 是垂直于 SW 的平面基向量。

4. **由 S, E, W, theta4 解析解 q1, q2, q3**：
   - q2 由 E 的高度约束直接解：cos(q2) = (Ez - d0) / d2
   - q1 由 E 在 XY 平面的投影解
   - q3 由腕部指向约束解（含 q4 相关项）

5. **构建 T04 = FK(q1..q4)，求 T47 = inv(T04)·T07**：
   从 T47 的旋转子矩阵解 q5, q6, q7。

### 3.5 连续性分支选择

7-DOF 冗余臂同一末端位姿可对应无穷多解（由 theta0 参数化）。遥操需要帧间连续的关节轨迹。

**策略：局部 theta0 窗口 + 滞后锁定**

```
1. 以上一帧选中的 theta0 为中心, ±0.15 rad 窗口采样 9 个点
2. 对每个采样点求解, 计算综合评分:
     score = w_vel·|dq| + w_acc·|ddq| + w_pose·pose_err + w_theta0·|dtheta0|
3. 选评分最低的解
4. 滞后锁定: 如果当前锁定分支的解评分 ≤ best_score + 0.03,
   则保持锁定分支不变 (避免微小扰动导致分支抖动)
5. 如果局部窗口无解, 回退到全局扫描
```

### 3.6 1D QP 后优化

解析解有微量位姿残差（DH 参数与物理臂不完全一致）。对 7 个关节**逐个**做 1D 数值优化：

```
for i in 0..6:
    q_new[i] = q[i] + dq
    minimize: w_joint·dq² + w_pose·pose_error(fk(q_new), T_target)²
    subject to: -0.05 ≤ dq ≤ 0.05,  joint_limits
    method: scipy.minimize_scalar(method="bounded")
```

每个关节独立优化，每次微调 ≤ 0.05 rad。位姿误差通常降至 < 1mm。

### 3.7 性能

| 阶段 | 耗时 (ms) | 说明 |
|------|-----------|------|
| S-W-E 几何 | ~0.05 | 解析公式，常数时间 |
| theta0 全局扫描 (181点) | ~2-5 | 首次求解 / 全局回退 |
| 局部窗口 (9点) | ~0.2-0.5 | 连续跟踪模式 |
| 1D QP | ~0.5-1 | 7 关节 × scipy bounded |
| **总计 (连续模式)** | **~2-5 ms** | 50Hz 控制充裕 |

---

## 4. 手部重定向 (Retargeting)

手部重定向分为两阶段：

- **阶段一（quest3_udp_mocap）**：VR 原始数据 → MANO 坐标系 21 点 landmark
- **阶段二（xhand_retargeting）**：MANO landmark → XHand 12 关节角

### 4.1 阶段一：VR 数据预处理

#### 4.1.1 坐标转换链

```
输入: Quest3 右手坐标系, 21个landmark绝对3D坐标 (米)

Step 1 — X轴翻转:
  ros[i] = (-VR_x, VR_y, VR_z)
  原因: Quest3 左手坐标系 → ROS 右手坐标系

Step 2 — ×1.05 缩放:
  keypoint *= 1.05
  原因: 匹配 MediaPipe 训练的 landmark 尺度

Step 3 — 以手腕为原点居中:
  keypoint = keypoint - keypoint[0]

Step 4 — SVD 手腕坐标帧估计:
  取 wrist(0), index_mcp(5), middle_mcp(9) 三个点做 SVD
  → 得到手腕局部正交坐标系 (3×3 旋转矩阵)

Step 5 — @ operator2mano:
  旋转到 MANO 标准坐标系

Step 6 — adaptive_retargeting_xhand:
  小指段长自适应缩放
```

#### 4.1.2 SVD 坐标帧估计

**为什么用 SVD 而非 cross-product？**

cross-product 方法（用两点连线 + 叉积构造坐标轴）假设三点严格共面。
但真实人手的三点不在完美平面上，叉积会放大非正交偏差，导致抖动。

SVD 直接拟合最优平面：

```
1. 取 wrist(0), index_mcp(5), middle_mcp(9) 三个点
2. 去均值，对 (3×3) 矩阵做 SVD
3. v[2] = 最小奇异值对应的右奇异向量 = 平面法向量（Normal）
4. 构造 X 轴: wrist→middle_mcp 减去法向量分量 (Gram-Schmidt 正交化)
5. 构造 Z 轴: X × Normal (cross product)
6. Frame = [X, Normal, Z]  → 3×3 严格正交矩阵
```

结果比 cross-product 法更稳定、更不易抖动。

#### 4.1.3 MANO 坐标系

MANO 是学术界标准手部参数化模型。DexPilot 优化器硬编码了 MANO 坐标系下的 landmark 位置假设。

```
MANO 约定:
  原点: 手腕
  X 轴: 拇指方向
  Y 轴: 手掌法向量（掌心→手背）
  Z 轴: 手指伸展方向（指向指尖）
```

**operator2mano 矩阵** 将 SVD 估计的局部帧旋转到 MANO 约定：

```python
OPERATOR2MANO_RIGHT = [[ 0, 0,-1],   # VR_X → MANO_-Y
                       [-1, 0, 0],   # VR_Y → MANO_-X
                       [ 0, 1, 0]]   # VR_Z → MANO_Z

OPERATOR2MANO_LEFT  = [[ 0, 0,-1],   # 左手镜像
                       [ 1, 0, 0],
                       [ 0,-1, 0]]
```

#### 4.1.4 小指自适应缩放

人手小指与 XHand 小指的段长比例差异大。根据小指伸展程度做自适应缩放：

```python
extension_ratio = clip((|MCP→TIP| - 0.03) / (0.10 - 0.03), 0, 1)
scale = 1.2 + 1.0 × extension_ratio   # 弯曲时1.2倍, 伸展时2.2倍
# 对 PIP, DIP, TIP 段分别施加缩放
```

### 4.2 阶段二：DexPilot 重定向

#### 4.2.1 算法原理

DexPilot 是基于**手指间参考向量**的优化重定向方法。

**输入**：MANO 坐标系下 21 个 landmark 3D 位置
**输出**：XHand URDF 模型对应的 12 个关节角度

```
1. 从 21 个 landmark 提取手指间 Task-Origin 参考向量:
   10 个手指间向量 (食指→中指, 中指→无名指等)
   5 个手指到掌心向量

2. 在 XHand URDF 运动学空间内:
   给定一组关节角 → FK 计算指尖位置 → 构造相同的参考向量
   用梯度下降最小化 |参考向量_人手 - 参考向量_机器人|

3. 内置机制:
   low_pass_alpha=0.6: 优化变量低通滤波
   project_dist=0.03: 手指穿透投影检测
   escape_dist=0.03: 手指脱离投影检测
```

#### 4.2.2 URDF 关节名映射

DexPilot 输出的是 URDF 模型中的关节名（带 `left_hand_`/`right_hand_` 前缀），需要映射到 XHand 硬件控制名：

```python
XHAND_JOINT_NAMES = [
    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
    "index_bend_joint", "index_joint1", "index_joint2",
    "mid_joint1", "mid_joint2",
    "ring_joint1", "ring_joint2",
    "pinky_joint1", "pinky_joint2",
]
```

#### 4.2.3 右手 index_bend 取反

XHand 右手 URDF 的 `index_bend_joint` 轴方向与硬件固件定义相反，需要对 retargeting 输出的该关节值取反。左手不需要。

### 4.3 拇指后处理

DexPilot 对拇指三关节（bend, rota1, rota2）的映射质量不稳定——rota_joint2 常常不必要地卷曲。

**检测拇指伸直度**：
```
straightness = |MCP → TIP| / (|MCP → IP| + |IP → TIP|)
```

当 straightness > 0.85 时，线性抑制 rota_joint2（最多抑制 30%）：
```
factor = (straightness - 0.85) / 0.15 × 0.3
rota_joint2 *= (1.0 - factor)
```

### 4.4 EMA 平滑

两级平滑：

| 位置 | 系数 | 作用 |
|------|------|------|
| quest3_udp_mocap | ema_alpha=0.7 | landmark 位置平滑 |
| xhand_retargeting | smoothing_alpha=0.7 | 关节角平滑 |

```
smoothed = alpha × current + (1 - alpha) × previous
```

alpha 越大 = 越跟手（延迟小但抖动大），alpha 越小 = 越平滑（延迟大但稳定）。0.7 是实验最佳平衡点。

---

## 5. 数据采集与后处理

### 5.1 实时采集

```
data_collect_node (50Hz ROS2 timer)
  │
  ├── 订阅:
  │     /nero_teleop_{left,right}/joint_states  (臂关节, 7维×2)
  │     /{left,right}_hand/xhand_state          (手关节, 12维×2)
  │     指标 topics (臂延时/精度 + 手延时)
  │
  ├── CameraDataProcess (独立 multiprocessing.Process)
  │     3 路 Gemini 相机, JPEG 压缩 → camera_data.h5
  │
  └── 写入 robot_data.h5 + metrics.h5

键盘控制: [s]开始 [p]暂停 [q]保存 [d]丢弃 [n]下一集
```

### 5.2 时间对齐

```
cam_0 时间戳 (Gemini335, 30Hz) → 主时钟
  │
  ├─ find_nearest_idx(robot_ts, cam_ts) → state(t)
  ├─ find_nearest_idx(robot_ts, next_cam_ts) → action(t) = state(t+1)
  └─ 提取对应相机帧 → frames/cam{N}/{frame_idx:06d}.jpg

末尾追加 20 帧 hold (停止姿态)
```

### 5.3 数据格式

**aligned_data.h5**：
```
/observation/left_arm_joints   (F, 7)   state
/observation/right_arm_joints  (F, 7)
/observation/left_hand_joints  (F,12)
/observation/right_hand_joints (F,12)
/action/left_arm_joints        (F, 7)   next state
/action/right_arm_joints       (F, 7)
/action/left_hand_joints       (F,12)
/action/right_hand_joints      (F,12)
/timestamps                    (F,)
```

**LeRobot 转换后**：38维 state = 7(左臂)+7(右臂)+12(左手)+12(右手)，3 路相机图像。

---

## 6. 延时与精度量化

### 6.1 延时链路

```
手臂:
  UDP到达  →  VR→Cmd(软件管线)  →  move_js发出
                                      │
                              Control(物理控制) = CAN+电机执行
                                      │
                                  编码器反馈 ← E2E ←─┘

手部:
  UDP到达  →  LM→Cmd(软件管线)  →  XHandCommand发布
                                      │
                              Control(物理控制) = ROS2+USB串口+手执行
                                      │
                                  XHandState接收 ← E2E ←─┘
```

### 6.2 实时打印（每 2 秒）

```
[LEFT ARM] -- Latency & Accuracy (window=100) --
  Software pipeline:
    VR→Cmd:  avg=  6.5  max= 12.0 ms
    IK:      avg=  2.8  max=  5.3 ms
    Safety:  avg=  0.1  max=  0.3 ms
  Physical control (CAN+Motor):
    === Control: avg=  5.1  max=  9.8 ms ===
  === E2E total:  avg= 18.3  max= 25.1 ms ===
  Accuracy:
    Pos err: avg= 12.3  max= 35.1 mm
    Ori err: avg=  2.15 max=  5.80 deg

[RIGHT HAND] -- Latency (window=100) --
  Software pipeline:
    LM→Cmd:   avg=  2.1  max=  5.3 ms
    Retarget: avg=  1.5  max=  3.2 ms
    PostProc: avg=  0.2  max=  0.5 ms
  Physical control (ROS+USB+Motor):
    === Control: avg=  8.5  max= 15.2 ms ===
  === E2E total:  avg= 11.8  max= 20.1 ms ===
```

### 6.3 存档

所有指标帧级数据存入 `metrics.h5`，含 summary 统计（mean/std/max），供离线分析。

---

## 附录 A：ROS2 话题速查

| 话题 | 类型 | 发布者 |
|------|------|--------|
| `/quest3/{left,right}_wrist_pose` | PoseStamped | quest3_udp_mocap |
| `/hand_landmarks/{left,right}` | PoseArray | quest3_udp_mocap |
| `/{left,right}_hand/xhand_command` | XHandCommand | xhand_dex_retargeting |
| `/{left,right}_hand/xhand_state` | XHandStateArray | xhand_control_ros2 |
| `/nero_teleop_{left,right}/joint_states` | JointState | nero_teleop_node |
| `/nero_teleop_{left,right}/metrics` | Float64MultiArray | nero_teleop_node |
| `/xhand_dex_retargeting/metrics/{left,right}_hand` | Float64MultiArray | xhand_dex_retargeting |

## 附录 B：控制参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `control_rate` | 50 Hz | 臂控制循环频率 |
| `motion_scale` | 0.65 | VR→机器人运动缩放 |
| `pos_smoothing` | 0.5 | 位置 EMA 系数 |
| `rot_smoothing` | 0.8 | 姿态 SLERP 系数 |
| `max_joint_vel` | 0.05 rad/step | 单步速度限幅 |
| `workspace_radius` | 0.58 m | 工作空间球半径 |
| `ema_alpha` | 0.7 | 手部 landmark 平滑 |
| `smoothing_alpha` | 0.7 | 关节角平滑 |
| `enable_thumb_fix` | true | 拇指后处理 |
