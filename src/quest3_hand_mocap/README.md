# quest3_hand_mocap

Quest3 VR 手部追踪 + 头部位姿数据接收节点。支持 UDP / TCP 有线 / TCP 无线三种传输方式。

---

## 功能

接收 Quest3 HTS 应用发送的手部 landmark（21 点 x 3 维）、腕部位姿、头部（HMD）位姿。经坐标转换、SVD 帧估计、MANO 对齐、EMA 平滑后，发布 ROS2 话题供下游节点使用。

## 数据传输方式

| 模式 | 参数值 | Quest3 端配置 | PC 端 | 延迟 | 需要 WiFi 同网 |
|------|--------|-------------|-------|------|:---:|
| UDP | `udp` (默认) | UDP, 端口 9000, PC_IP | 无额外操作 | 低 | 是 |
| TCP 有线 | `tcp_wired` | TCP, localhost:8000 | `adb reverse tcp:8000 tcp:8000` | 最低 | 否 |
| TCP 无线 | `tcp_wireless` | TCP, PC_IP:8000 | 无额外操作 | 中 | 是 |

### TCP 有线模式设置（推荐，延迟最低）

使用 Quest3 通过 USB 有线连接 PC，延迟最低且不受 WiFi 干扰。

**步骤：**

1. **Quest3 用 USB 线连接到 PC**
2. **戴上 Quest3**，在弹出的"允许 USB 调试"对话框中点击**允许**
3. **PC 终端检查设备是否识别**：
   ```bash
   adb devices
   # 应看到类似输出：
   # List of devices attached
   # 1A2B3C4D    device
   ```
   如果显示 `unauthorized`，说明 Quest3 端尚未允许 USB 调试，回到步骤 2。
4. **端口映射**（将 Quest3 的 8000 端口映射到 PC 的 8000 端口）：
   ```bash
   adb reverse tcp:8000 tcp:8000
   # 第一次执行输出：8000
   # 再次执行无任何输出 → 表示端口已映射成功，无需重复操作
   ```
   **核心判断**：执行后不再打印 `8000`，没有下文即为成功。如果反复输出 `8000`，说明之前的映射已失效，需要重新执行直到无输出为止。
5. **关闭 PC 防火墙**（否则 Quest3 TCP 连接可能被拦截）：
   ```bash
   sudo ufw disable
   ```
6. **Quest3 HTS 应用中选择 TCP**，地址填写 `localhost`，端口 `8000`
7. **PC 端启动时使用 `protocol:=tcp_wired` 参数**：
   ```bash
   ros2 launch nero_quest_teleop teleop_full_dual.launch.py protocol:=tcp_wired
   ```

## 数据流

```
Quest3 (UDP/TCP/TCP有线) → process_line() → process_landmarks()
  ├── ROS 坐标转换
  ├── SVD 手腕帧估计
  ├── @ operator2mano (MANO 对齐)
  ├── adaptive_retargeting_xhand (小指缩放)
  └── EMA 平滑
       │
       ├── /hand_landmarks/{left,right}  (PoseArray, 21点)
       ├── /quest3/{left,right}_wrist_pose  (PoseStamped)
       └── /quest3/head_pose  (PoseStamped, 头部 6-DOF)
```

## 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `hand_landmarks/left` | PoseArray | 左手 21 个关键点 |
| `hand_landmarks/right` | PoseArray | 右手 21 个关键点 |
| `quest3/left_wrist_pose` | PoseStamped | 左手腕部位姿 |
| `quest3/right_wrist_pose` | PoseStamped | 右手腕部位姿 |
| `quest3/head_pose` | PoseStamped | 头部 (HMD) 6-DOF 位姿 |
| `quest3/{side}_hand_markers` | MarkerArray | RViz 可视化（可选） |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `protocol` | udp | 传输方式：`udp` / `tcp_wired` / `tcp_wireless` |
| `udp_port` | 9000 | UDP 监听端口 |
| `tcp_port` | 8000 | TCP 监听端口 |
| `arm_side` | left | 接收手：left / right / both |
| `ema_alpha` | 0.7 | EMA 平滑系数 |
| `viz` | true | 是否发布 RViz 可视化 |

## 启动

```bash
# UDP（默认）
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args -p arm_side:=both

# TCP 有线
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args -p arm_side:=both -p protocol:=tcp_wired

# TCP 无线
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args -p arm_side:=both -p protocol:=tcp_wireless
```

## 在系统中的位置

```
quest3_udp_mocap
    │
    ├── /hand_landmarks/* → xhand_retargeting → xhand_control_ros2 → XHand
    ├── /quest3/*_wrist_pose → nero_quest_teleop → pyAgxArm → Nero
    └── /quest3/head_pose → 头部追踪（可录制/可视化）
```

详见顶层 [README](../README.md)。
