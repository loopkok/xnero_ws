# 环境依赖清单

## 系统环境

| 依赖 | 版本要求 | 说明 |
|------|---------|------|
| Ubuntu | 22.04 / 24.04 | 操作系统 |
| ROS 2 | Humble | 机器人中间件 |
| Python | 3.10 | 系统自带 |
| CAN 适配器 | socketcan | Linux CAN 总线支持 |
| Meta Quest 3 | — | VR 头显 |
| SideQuest | — | Quest3 侧载 HTS 软件 |

---

## 安装步骤

### 1. ROS 2 Humble

```bash
# 依照官方文档安装 ros-humble-desktop
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 可视化相关包
sudo apt install -y \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-urdf \
    ros-humble-xacro
```

### 2. Python 系统包

```bash
sudo apt install -y python3-numpy python3-scipy python3-opencv python3-h5py python3-pip
```

### 3. pip 核心依赖

```bash
# 手部重定向
pip install dex-retargeting

# CAN 总线通讯
pip install "python-can>=3.3.4" typing-extensions

# 相机 SDK
pip install pyorbbecsdk
```

### 4. pyAgxArm SDK

```bash
cd src/pyAgxArm && pip install -e . && cd ../..
```

### 5. 相机 udev 规则

```bash
sudo bash ~/.local/lib/python3.10/site-packages/pyorbbecsdk/shared/install_udev_rules.sh
# 安装后重新插拔相机 USB
```

### 6. 可选依赖

```bash
# LeRobot 数据转换
pip install lerobot tqdm

# URDF 数值 IK（备选方案）
sudo apt install ros-humble-pinocchio  # 或 pip install pin

# WebVR 监控面板
pip install torch websockets pyyaml pynput
```

---

## 依赖汇总表

| 类别 | 包名 | 方式 | 用途 |
|------|------|------|------|
| **ROS** | rclpy | apt | Python ROS2 客户端 |
| **ROS** | geometry_msgs | apt | 位姿话题定义 |
| **ROS** | sensor_msgs | apt | 关节状态话题定义 |
| **ROS** | std_msgs, std_srvs | apt | 标准消息/服务 |
| **ROS** | visualization_msgs | apt | RViz 可视化消息 |
| **ROS** | rosidl_default_runtime | apt | 自定义消息接口 |
| **ROS** | joint_state_publisher* | apt | 关节状态发布 |
| **ROS** | robot_state_publisher | apt | URDF 状态发布 |
| **ROS** | rviz2 | apt | 3D 可视化 |
| **ROS** | urdf, xacro | apt | URDF 模型解析 |
| **ROS** | ament_index_python | apt | 包路径查找 |
| **数学** | numpy | apt | 矩阵运算 |
| **优化** | scipy | apt | SVD / 1D QP / IK 优化 |
| **图像** | opencv-python | apt | 相机图像处理 |
| **存储** | h5py | apt | HDF5 数据读写 |
| **重定向** | dex-retargeting | pip | DexPilot 手部重定向 |
| **CAN** | python-can ≥ 3.3.4 | pip | CAN 总线通讯 |
| **类型** | typing-extensions | pip | Python 类型扩展 |
| **相机** | pyorbbecsdk | pip | Gemini 相机 SDK |
| **SDK** | pyAgxArm | `pip install -e .` | Nero 机械臂 CAN SDK |
| **可选** | lerobot, tqdm | pip | LeRobot 数据集转换 |
| **可选** | pin / pinocchio | apt/pip | URDF 数值 IK |
| **可选** | torch, websockets, pyyaml, pynput | pip | WebVR 监控面板 |

---

## 一键安装脚本

```bash
#!/bin/bash
set -e

# 系统包
sudo apt install -y \
    python3-numpy python3-scipy python3-opencv python3-h5py python3-pip

# pip 核心
pip install dex-retargeting "python-can>=3.3.4" typing-extensions pyorbbecsdk

# pyAgxArm SDK
cd $(dirname "$0")/src/pyAgxArm && pip install -e . && cd -

# 相机 udev
sudo bash ~/.local/lib/python3.10/site-packages/pyorbbecsdk/shared/install_udev_rules.sh

echo "依赖安装完成。"
echo "请执行: colcon build --symlink-install && source install/setup.bash"
```
