#!/usr/bin/env python3
"""xnero 仿真全链路（不需要真机）。

与 `nero_quest_teleop/launch/teleop_full_dual.launch.py` 的差别只有两处：
  1. **不起 xhand_control_ros2**（两个 XHand 串口驱动）—— 由 MuJoCo 仿真顶替；
  2. 臂遥操节点加 `sim_mode:=true` —— 不连 CAN，命令发 ~/joint_commands。

链路：
  quest3_udp_mocap → xhand_dex_retargeting ─→ /{L,R}_hand/xhand_command ─┐
                   → nero_teleop_{L,R}(sim) ─→ /nero_teleop_{L,R}/joint_commands ─┤
                                                                            ▼
                                            xnero_mujoco_sim_node（手物理 + 臂运动学）
                                              ├→ /{L,R}_hand/xhand_state（含触觉）
                                              └→ /nero_teleop_{L,R}/joint_states
                                                                            ▼
                                                    nero_dual_data_collect（零改动可用）

用法：
    ros2 launch xnero_mujoco_sim xnero_sim_pipeline.launch.py
    ros2 launch xnero_mujoco_sim xnero_sim_pipeline.launch.py protocol:=udp enable_viewer:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_dir = get_package_share_directory("nero_quest_teleop")
    retarget_dir = get_package_share_directory("xhand_retargeting")
    cfg_left = os.path.join(teleop_dir, "config", "nero_teleop_left.yaml")
    cfg_right = os.path.join(teleop_dir, "config", "nero_teleop_right.yaml")
    cfg_retarget = os.path.join(retarget_dir, "config", "retargeting_params.yaml")

    args = [
        DeclareLaunchArgument("protocol", default_value="tcp_wired"),
        DeclareLaunchArgument("udp_port", default_value="9000"),
        DeclareLaunchArgument("enable_viewer", default_value="true"),
        DeclareLaunchArgument("publish_tactile", default_value="true"),
        DeclareLaunchArgument("state_rate", default_value="50.0"),
        DeclareLaunchArgument("arm_side", default_value="both",
                              description="both | left | right"),
        DeclareLaunchArgument("require_clench_to_start", default_value="true"),
        DeclareLaunchArgument("enable_retargeting", default_value="true"),
    ]

    quest3 = Node(
        package="quest3_hand_mocap", executable="quest3_udp_mocap",
        name="quest3_udp_mocap", output="screen",
        parameters=[cfg_retarget],
    )

    retargeting = Node(
        package="xhand_retargeting", executable="xhand_dex_retargeting_node",
        name="xhand_dex_retargeting", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_retargeting")),
        parameters=[cfg_retarget,
                    {"protocol": LaunchConfiguration("protocol"),
                     "udp_port": LaunchConfiguration("udp_port")}],
    )

    # 臂遥操：sim_mode=true 是关键 —— 不连 CAN，命令走 ROS
    teleop_left = Node(
        package="nero_quest_teleop", executable="nero_teleop_node",
        name="nero_teleop_left", output="screen",
        parameters=[cfg_left,
                    {"sim_mode": True,
                     "require_clench_to_start": LaunchConfiguration("require_clench_to_start")}],
    )
    teleop_right = Node(
        package="nero_quest_teleop", executable="nero_teleop_node",
        name="nero_teleop_right", output="screen",
        parameters=[cfg_right,
                    {"sim_mode": True,
                     "require_clench_to_start": LaunchConfiguration("require_clench_to_start")}],
    )

    sim = Node(
        package="xnero_mujoco_sim", executable="xnero_mujoco_sim_node",
        name="xnero_mujoco_sim_node", output="screen",
        parameters=[{
            "enable_viewer": LaunchConfiguration("enable_viewer"),
            "publish_tactile": LaunchConfiguration("publish_tactile"),
            "state_rate": LaunchConfiguration("state_rate"),
        }],
    )

    banner = LogInfo(msg=(
        "\n\033[1;36m"
        "╔══════════════════════════════════════════════════════════╗\n"
        "║  xnero 仿真链路（无真机）                                  ║\n"
        "║  · XHand 串口驱动已被 MuJoCo 顶替                          ║\n"
        "║  · 臂 teleop 以 sim_mode 运行，不碰 CAN                    ║\n"
        "║  · 触觉力在 /{left,right}_hand/xhand_state.sensor_states   ║\n"
        "╚══════════════════════════════════════════════════════════╝\033[0m"))

    return LaunchDescription(args + [banner, quest3, retargeting,
                                     teleop_left, teleop_right, sim])
