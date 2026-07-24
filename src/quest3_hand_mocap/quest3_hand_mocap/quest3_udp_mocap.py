#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time as RosTime

import socket
import select
import threading
import time
import numpy as np
import re


class FPSCounter:
    """Simple sliding-window FPS counter."""
    def __init__(self, window=100, print_interval=5.0):
        self._window = window
        self._print_interval = print_interval
        self._times = []
        self._last_print = time.time()
    def tick(self):
        now = time.time()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times.pop(0)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / max(1e-9, self._times[-1] - self._times[0])
    def should_print(self):
        now = time.time()
        if now - self._last_print >= self._print_interval:
            self._last_print = now
            return True
        return False


OPERATOR2MANO_RIGHT = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
])

OPERATOR2MANO_LEFT = np.array([
    [0, 0, -1],
    [1, 0, 0],
    [0, -1, 0],
])


def adaptive_retargeting_xhand(landmarks):
    """
    Apply adaptive pinky retargeting specifically for XHand robot.
    Compensates for human-to-robot finger length differences by using
    adaptive scaling based on finger extension state.
    """
    landmarks = landmarks.copy()

    pinky_mcp = 17
    pinky_pip = 18
    pinky_dip = 19
    pinky_tip = 20

    pinky_extension = np.linalg.norm(landmarks[pinky_tip] - landmarks[pinky_mcp])

    max_extension = 0.10
    min_extension = 0.03
    extension_ratio = np.clip((pinky_extension - min_extension) / (max_extension - min_extension), 0.0, 1.0)

    base_scale = 1.2
    max_scale = 2.2
    adaptive_scale = base_scale + (max_scale - base_scale) * extension_ratio

    mcp_to_pip_vector = landmarks[pinky_pip] - landmarks[pinky_mcp]
    landmarks[pinky_pip] = landmarks[pinky_mcp] + mcp_to_pip_vector * adaptive_scale

    pip_to_dip_vector = landmarks[pinky_dip] - landmarks[pinky_pip]
    landmarks[pinky_dip] = landmarks[pinky_pip] + pip_to_dip_vector * adaptive_scale

    dip_to_tip_vector = landmarks[pinky_tip] - landmarks[pinky_dip]
    landmarks[pinky_tip] = landmarks[pinky_dip] + dip_to_tip_vector * adaptive_scale

    return landmarks


class Quest3UDPMocap(Node):
    def __init__(self):
        super().__init__("quest3_udp_mocap")
        self.get_logger().info("Quest 3 Hand Mocap Node Initialized")

        # 参数配置
        self.declare_parameter("protocol", "tcp_wired")  # "udp" | "tcp_wired" | "tcp_wireless"
        self.declare_parameter("udp_port", 9000)
        self.declare_parameter("tcp_port", 8000)
        self.declare_parameter("ema_alpha", 0.7)
        self.declare_parameter("viz", True)
        self.declare_parameter("arm_side", "left")

        self.protocol = self.get_parameter("protocol").value.lower()
        self.udp_port = self.get_parameter("udp_port").value
        self.tcp_port = self.get_parameter("tcp_port").value
        self.ema_alpha = self.get_parameter("ema_alpha").value
        self.viz = self.get_parameter("viz").value
        self.arm_side = self.get_parameter("arm_side").value.lower()
        self.both = self.arm_side == "both"

        self.get_logger().info(
            f"Quest 3 Hand Mocap: side=[{self.arm_side.upper()}], "
            f"protocol={self.protocol}"
        )

        # 缓存用于 EMA 平滑滤波
        self.landmark_cache_right = np.zeros((21, 3))
        self.landmark_cache_left = np.zeros((21, 3))

        # 发布者：始终按左右分 topic
        self.mocap_pub_right = self.create_publisher(PoseArray, "hand_landmarks/right", 10)
        self.mocap_pub_left = self.create_publisher(PoseArray, "hand_landmarks/left", 10)
        self.wrist_pub_right = self.create_publisher(PoseStamped, "quest3/right_wrist_pose", 10)
        self.wrist_pub_left = self.create_publisher(PoseStamped, "quest3/left_wrist_pose", 10)
        self.head_pub = self.create_publisher(PoseStamped, "quest3/head_pose", 10)
        
        # 新增：RViz 可视化发布者
        if self.viz:
            self.right_marker_pub = self.create_publisher(MarkerArray, "quest3/right_hand_markers", 10)
            self.left_marker_pub = self.create_publisher(MarkerArray, "quest3/left_hand_markers", 10)
            self.marker_pub = self.create_publisher(MarkerArray, f"quest3/{self.arm_side}_hand_markers", 10)

        # 腕部数据的正则解析器
        self.wrist_pattern = re.compile(r'wrist:,?\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)')

        self.is_running = True

        # ---- Network listener (UDP / TCP wired / TCP wireless) ----
        if self.protocol == "udp":
            self._start_udp_listener()
        elif self.protocol in ("tcp_wired", "tcp_wireless"):
            self._start_tcp_listener()
        else:
            self.get_logger().error(f"Unknown protocol: {self.protocol}")
            raise ValueError(f"Unknown protocol: {self.protocol}")

    def _start_udp_listener(self):
        """Start UDP listener thread."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('0.0.0.0', self.udp_port))
        self.socket.settimeout(0.5)
        self.get_logger().info(f"Listening for UDP data on port {self.udp_port}...")
        self.thread = threading.Thread(target=self.udp_listener, daemon=True)
        self.thread.start()

    def _start_tcp_listener(self):
        """Start TCP server listener thread."""
        host = "localhost" if self.protocol == "tcp_wired" else "0.0.0.0"
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_server.bind((host, self.tcp_port))
        self.tcp_server.listen(3)
        self.tcp_server.settimeout(1.0)
        mode = "wired (adb reverse)" if self.protocol == "tcp_wired" else "wireless (WiFi)"
        self.get_logger().info(
            f"TCP server listening on {host}:{self.tcp_port} ({mode})"
        )
        if self.protocol == "tcp_wired":
            self.get_logger().info(
                "  On Quest3: adb reverse tcp:{0} tcp:{0}".format(self.tcp_port)
            )
        self.thread = threading.Thread(target=self.tcp_listener, daemon=True)
        self.thread.start()

    @staticmethod
    def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
        """
        Compute the 3D coordinate frame (orientation only) from detected 3d key points.
        :param keypoint_3d_array: (21, 3) keypoints in the order: [wrist, index_mcp, middle_mcp, ...]
        :return: the coordinate frame of wrist in MANO convention (3x3 rotation matrix)
        """
        assert keypoint_3d_array.shape == (21, 3)
        points = keypoint_3d_array[[0, 5, 9], :]

        x_vector = points[0] - points[2]
        points = points - np.mean(points, axis=0, keepdims=True)
        _, _, v = np.linalg.svd(points)
        normal = v[2, :]

        x = x_vector - np.sum(x_vector * normal) * normal
        x = x / np.linalg.norm(x)
        z = np.cross(x, normal)

        if np.sum(z * (points[1] - points[2])) < 0:
            normal *= -1
            z *= -1
        frame = np.stack([x, normal, z], axis=1)
        return frame

    def process_landmarks(self, landmarks, hand_label):
        operator2mano = OPERATOR2MANO_RIGHT if hand_label == "right" else OPERATOR2MANO_LEFT

        keypoint_3d_array = landmarks.copy()
        keypoint_3d_array *= 1.05
        keypoint_3d_array = keypoint_3d_array - keypoint_3d_array[0:1, :]

        wrist_rot = self.estimate_frame_from_hand_points(keypoint_3d_array)

        joint_pos = keypoint_3d_array @ wrist_rot @ operator2mano

        joint_pos = adaptive_retargeting_xhand(joint_pos)

        return joint_pos

    def udp_listener(self):
        """UDP listener thread."""
        while self.is_running and rclpy.ok():
            try:
                data, _ = self.socket.recvfrom(65536)
                arrival_time = time.time()
                message = data.decode('utf-8')
                for line in message.splitlines():
                    if not line: continue
                    self.process_line(line, arrival_time)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP Listener Error: {e}")

    def tcp_listener(self):
        """TCP server thread — accept connections and handle each."""
        while self.is_running and rclpy.ok():
            try:
                conn, addr = self.tcp_server.accept()
                self.get_logger().info(f"TCP connection from {addr}")
                t = threading.Thread(
                    target=self._tcp_connection_handler,
                    args=(conn, addr), daemon=True,
                )
                t.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f"TCP accept error: {e}")

    def _tcp_connection_handler(self, conn, addr):
        """Handle a single TCP connection — read lines and process."""
        buf = ""
        with conn:
            while self.is_running and rclpy.ok():
                try:
                    data = conn.recv(65536)
                    if not data:
                        self.get_logger().info(f"TCP {addr} disconnected")
                        break
                    arrival_time = time.time()
                    buf += data.decode('utf-8')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        if line.strip():
                            self.process_line(line.strip(), arrival_time)
                except Exception as e:
                    self.get_logger().error(f"TCP read error from {addr}: {e}")
                    break

    def _float_to_ros_time(self, t: float) -> RosTime:
        """Convert wall-clock float seconds to ROS Time message."""
        stamp = RosTime()
        stamp.sec = int(t)
        stamp.nanosec = int((t - int(t)) * 1e9)
        return stamp

    def process_line(self, line, arrival_time: float = 0.0):
        # FPS tracking
        if not hasattr(self, '_fps'):
            self._fps = FPSCounter(window=100, print_interval=5.0)
        fps = self._fps.tick()
        if self._fps.should_print():
            self.get_logger().info(f"[VR Data Rate] {fps:.0f} Hz")

        line_lower = line.lower()

        # ===== 头部解析（无左右区分，全局）=====
        if "head" in line_lower or "hmd" in line_lower:
            # Format: head, x, y, z, qx, qy, qz, qw  (or colon-separated)
            line_clean = line_lower.replace(":", ",")
            parts = line_clean.split(",")
            if len(parts) >= 7:
                try:
                    vals = [float(p.strip()) for p in parts[-7:]]
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self._float_to_ros_time(arrival_time)
                    pose_msg.header.frame_id = "vr_world"
                    pose_msg.pose.position.x = vals[0]
                    pose_msg.pose.position.y = vals[1]
                    pose_msg.pose.position.z = vals[2]
                    pose_msg.pose.orientation.x = vals[3]
                    pose_msg.pose.orientation.y = vals[4]
                    pose_msg.pose.orientation.z = vals[5]
                    pose_msg.pose.orientation.w = vals[6]
                    self.head_pub.publish(pose_msg)
                except Exception:
                    pass
            return  # 头部行不继续处理

        side = "right" if "right" in line_lower else "left"

        # 过滤：非 both 模式下只处理指定手
        if not self.both and side != self.arm_side:
            return

        # 1. 腕部解析
        if "wrist" in line.lower():
            parts = line.split(",")
            if len(parts) >= 7:
                try:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self._float_to_ros_time(arrival_time)
                    pose_msg.header.frame_id = "vr_world"
                    vals = [float(p.strip()) for p in parts[-7:]]
                    pose_msg.pose.position.x = vals[0]
                    pose_msg.pose.position.y = vals[1]
                    pose_msg.pose.position.z = vals[2]
                    pose_msg.pose.orientation.x = vals[3]
                    pose_msg.pose.orientation.y = vals[4]
                    pose_msg.pose.orientation.z = vals[5]
                    pose_msg.pose.orientation.w = vals[6]
                    (self.wrist_pub_right if side == "right" else self.wrist_pub_left).publish(pose_msg)
                except Exception as e:
                    self.get_logger().error(f"解析浮点数失败: {e}")
        # 2. 手指解析
        if "landmarks" in line.lower():
            parts = line.split(":")
            if len(parts) < 2: return
            values_str = parts[1].split(",")
            values = [float(v.strip()) for v in values_str if v.strip()]

            if len(values) == 63:
                landmarks = np.array(values).reshape(21, 3)
                ros_landmarks = np.zeros_like(landmarks)
                ros_landmarks[:, 0] = -landmarks[:, 0]
                ros_landmarks[:, 1] = landmarks[:, 1]
                ros_landmarks[:, 2] = landmarks[:, 2]

                processed_landmarks = self.process_landmarks(ros_landmarks, side)
                self.publish_mocap_data(processed_landmarks, side, arrival_time)
    def publish_hand_markers(self, hand_data, hand):
        """新增：将关节点转化为 RViz 的球体列表"""
        m = Marker()
        m.header.frame_id = 'world'  # RViz 的固定坐标系
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f"hand_{hand}"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.015; m.scale.y = 0.015; m.scale.z = 0.015
        m.color.a = 1.0
        
        if hand == "right":
            m.color.r = 0.9; m.color.g = 0.2; m.color.b = 0.2 # 右手红色
            publisher = self.right_marker_pub
        else:
            m.color.r = 0.2; m.color.g = 0.6; m.color.b = 0.9 # 左手蓝色
            publisher = self.left_marker_pub
            
        for i in range(hand_data.shape[0]):
            point = Point(x=float(hand_data[i, 0]), y=float(hand_data[i, 1]), z=float(hand_data[i, 2]))
            m.points.append(point)
            
        ma = MarkerArray()
        ma.markers.append(m)
        publisher.publish(ma)

    def publish_mocap_data(self, landmarks, side, arrival_time: float = 0.0):
        if side == 'right':
            self.landmark_cache_right = self.ema_alpha * landmarks + (1 - self.ema_alpha) * self.landmark_cache_right
            landmarks = self.landmark_cache_right
        elif side == 'left':
            self.landmark_cache_left = self.ema_alpha * landmarks + (1 - self.ema_alpha) * self.landmark_cache_left
            landmarks = self.landmark_cache_left
        else:
            return

        msg = PoseArray()
        msg.header.stamp = self._float_to_ros_time(arrival_time)
        msg.header.frame_id = f"hand_{side}"
        msg.poses = [
            Pose(
                position=Point(x=float(lm[0]), y=float(lm[1]), z=float(lm[2])),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ) for lm in landmarks
        ]

        if side == "right":
            self.mocap_pub_right.publish(msg)
        else:
            self.mocap_pub_left.publish(msg)

        if self.viz:
            self.publish_hand_markers(landmarks, side)

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'socket') and self.socket:
            self.socket.close()
        if hasattr(self, 'tcp_server') and self.tcp_server:
            self.tcp_server.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Quest3UDPMocap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()