#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sim_mode 下的 Nero 臂替身 —— 不碰 CAN，把命令发到 ROS 话题。

为什么做成"替身对象"而不是在 nero_teleop_node 里到处加 `if sim_mode`：
  teleop 里有 28 处 `self.robot.xxx()` 调用（连接/使能/模式/下发/回读/急停/归零/
  断开）。逐处加守卫既脏又容易漏。这里把 pyAgxArm 的接口**全部实现成 no-op**，
  只在 `move_js` / `move_j` 上把命令发布出去 —— 于是 teleop 的既有逻辑一行都不用改，
  仅需把 `AgxArmFactory.create_arm(cfg)` 换成 `SimArmStub(node)`。

行为约定：
  - `move_js(q)` / `move_j(q)`：记下命令 + 发 `<node_ns>/joint_commands`（JointState）
  - `get_joint_angles()`：返回**上一条命令**（把仿真臂当作理想跟随器）。
    这样 `_move_to_init_pose()`、`destroy_node()` 的归零流程会立刻收敛，
    而不是空等 15s 超时。
  - `get_flange_pose()`：返回 None（拿不到 FK）。副作用：teleop 的反馈线程不会发
    `/nero_teleop_*/joint_states` —— 臂状态改由 MuJoCo 仿真节点发布（见
    xnero_mujoco_sim 的说明）。
  - 其余（connect/enable/set_motion_mode/set_speed_percent/急停/断开）全为 no-op；
    `enable()` 返回 True 让 `while not robot.enable()` 直接通过。
"""

from __future__ import annotations

from typing import List, Optional

from sensor_msgs.msg import JointState

# 与真机 teleop 的反馈线程保持一致的关节名（它就是 joint1..joint7）
ARM_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]


class _JointAngles:
    """模仿 pyAgxArm 返回值的容器（有 .msg 列表字段）。"""

    __slots__ = ("msg",)

    def __init__(self, values: List[float]) -> None:
        self.msg = list(values)


class SimArmStub:
    def __init__(self, node, topic: str = "~/joint_commands") -> None:
        self._node = node
        self._pub = node.create_publisher(JointState, topic, 10)
        self._last: Optional[List[float]] = None
        self._sent = 0

    # ---- 下发：记下来 + 发出去 ----
    def _send(self, values) -> None:
        q = [float(v) for v in values][:7]
        if len(q) < 7:
            return
        self._last = q
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = q
        self._pub.publish(msg)
        self._sent += 1

    def move_js(self, values) -> None:
        self._send(values)

    def move_j(self, values) -> None:
        self._send(values)

    # ---- 回读 ----
    def get_joint_angles(self) -> Optional[_JointAngles]:
        return _JointAngles(self._last) if self._last is not None else None

    def get_flange_pose(self):
        return None          # 没有 FK；反馈线程据此不发 joint_states

    # ---- 其余全部 no-op ----
    def connect(self) -> None:
        self._node.get_logger().warn(
            "[SIM] sim_mode=true：不连接 CAN，臂命令发到 ~/joint_commands")

    def enable(self) -> bool:
        return True

    def set_motion_mode(self, *_a, **_k) -> None: ...
    def set_auto_set_motion_mode_enabled(self, *_a, **_k) -> None: ...
    def set_speed_percent(self, *_a, **_k) -> None: ...
    def set_tcp_offset(self, *_a, **_k) -> None: ...
    def electronic_emergency_stop(self, *_a, **_k) -> None:
        self._node.get_logger().warn("[SIM] 急停（仿真：仅记录，不动真机）")

    def set_normal_mode(self, *_a, **_k) -> None: ...
    def disconnect(self, *_a, **_k) -> None: ...

    @property
    def sent_count(self) -> int:
        return self._sent
