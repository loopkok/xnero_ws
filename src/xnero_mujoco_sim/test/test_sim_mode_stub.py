"""teleop 的 sim_mode：默认关、真机路径未被替换、替身行为正确。

这是防回归的关键闸门 —— teleop 是驱动真机的节点，改动必须可审计。
"""
import inspect
import re
from pathlib import Path

import pytest

pytest.importorskip("rclpy")

NODE_SRC = (Path(__file__).resolve().parents[2]
            / "nero_quest_teleop" / "nero_quest_teleop" / "nero_teleop_node.py")


@pytest.fixture(scope="module")
def src():
    assert NODE_SRC.is_file(), f"找不到 {NODE_SRC}"
    return NODE_SRC.read_text(encoding="utf-8")


def test_sim_mode_defaults_false(src):
    assert re.search(r'declare_parameter\(\s*"sim_mode"\s*,\s*False\s*\)', src), \
        "sim_mode 必须默认 False，否则真机行为会变"


def test_real_path_untouched(src):
    """真机分支必须仍然是 AgxArmFactory.create_arm + move_js。"""
    assert "self.robot = AgxArmFactory.create_arm(cfg)" in src
    assert "self.robot.move_js(cmd.tolist())" in src, "真机的 move_js 下发被动了"


def test_stub_switch_present(src):
    assert "SimArmStub(self)" in src
    # 必须二选一，不能把 create_arm 直接删掉
    i_stub = src.index("SimArmStub(self)")
    i_factory = src.index("self.robot = AgxArmFactory.create_arm(cfg)")
    assert abs(i_stub - i_factory) < 400, "替身切换点离工厂调用太远，可能改错了地方"


def test_stub_behaviour():
    """替身：move_js 发话题、get_joint_angles 回上一条命令、get_flange_pose 为 None。"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from nero_quest_teleop.sim_arm_stub import ARM_JOINT_NAMES, SimArmStub

    host = Node("stub_host")
    stub = SimArmStub(host)
    got = {}
    host.create_subscription(JointState, "/stub_host/joint_commands",
                             lambda m: got.__setitem__("m", m), 10)
    try:
        assert stub.enable() is True
        assert stub.get_flange_pose() is None
        assert stub.get_joint_angles() is None, "还没下发的应返回 None"

        cmd = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        stub.move_js(cmd)
        # 把消息抽出来（publisher 与 subscriber 都在 host 上）
        for _ in range(20):
            rclpy.spin_once(host, timeout_sec=0.02)
            if "m" in got:
                break
        assert "m" in got, "move_js 没把命令发出来"
        assert got["m"].name == ARM_JOINT_NAMES
        assert got["m"].position == pytest.approx(cmd)

        ja = stub.get_joint_angles()
        assert ja is not None and ja.msg == pytest.approx(cmd), \
            "get_joint_angles 应回上一条命令（理想跟随器），否则归零流程会空等超时"
    finally:
        host.destroy_node()
