#!/usr/bin/env python3
"""ROS2视觉按钮操作整合器

该节点订阅视觉检测到的按钮位置/类型, 根据 `button_actions.py` 提供的动作流程
驱动 Piper 机械臂执行 Toggle / Plug-in / Push / Knob 四类操作。
"""
from __future__ import annotations

import math
import traceback
from typing import Callable, Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from piper_sdk import C_PiperInterface_V2
from piper_arm import PiperArm

from utils.utils_math import quaternion_to_rotation_matrix
from utils.utils_piper import enable_fun

import button_actions
from button_actions import PI, action_knob, action_plugin, action_push, action_toggle


class VisionButtonActionNode(Node):
    """ROS2 节点: 监听视觉输入, 驱动 button_actions."""

    def __init__(self) -> None:
        super().__init__("vision_button_action_ros2")

        # ---- 参数 & Qos ----
        self.declare_parameter("object_point_topic", "/object_point")
        self.declare_parameter("button_type_topic", "/button_type")
        self.declare_parameter("target_marker_topic", "/target_button_base")
        self.declare_parameter("tcp_offset_local", [-0.018, 0.007, 0.063])
        self.declare_parameter("process_rate", 10.0)

        self.object_topic = self.get_parameter("object_point_topic").get_parameter_value().string_value
        self.button_type_topic = self.get_parameter("button_type_topic").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("target_marker_topic").get_parameter_value().string_value
        tcp_offset_param = self.get_parameter("tcp_offset_local").get_parameter_value().double_array_value
        self.tcp_offset_local = np.array(tcp_offset_param if tcp_offset_param else [-0.018, 0.007, 0.063])
        self.process_period = 1.0 / max(self.get_parameter("process_rate").value, 1.0)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # ---- 运行时状态 ----
        self.button_center: Optional[np.ndarray] = None
        self.button_type: Optional[str] = None
        self.last_point_stamp: Optional[float] = None
        self.action_map = {
            "toggle": action_toggle,
            "plugin": action_plugin,
            "push": action_push,
            "knob": action_knob,
        }

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, qos)
        self.create_subscription(PointStamped, self.object_topic, self._object_point_callback, qos)
        self.create_subscription(String, self.button_type_topic, self._button_type_callback, qos)

        self._hardware_ready = self._initialize_hardware()
        if not self._hardware_ready:
            self.get_logger().fatal("硬件初始化失败, 节点将退出")
            raise RuntimeError("hardware init failed")

        self.timer = self.create_timer(self.process_period, self._process_if_ready)
        self.get_logger().info(
            f"vision_button_action_ros2 已启动, 订阅 {self.object_topic} / {self.button_type_topic}"
        )

    # ------------------------------------------------------------------
    # ROS 回调
    # ------------------------------------------------------------------
    def _object_point_callback(self, msg: PointStamped) -> None:
        if any(math.isnan(val) for val in (msg.point.x, msg.point.y, msg.point.z)):
            self.get_logger().warn("忽略包含 NaN 的按钮位置")
            return
        self.button_center = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.last_point_stamp = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"收到按钮位置: ({msg.point.x:.4f}, {msg.point.y:.4f}, {msg.point.z:.4f})"
        )

    def _button_type_callback(self, msg: String) -> None:
        button_type = msg.data.strip().lower()
        if button_type not in self.action_map:
            self.get_logger().warn(f"未知按钮类型 '{button_type}'")
            return
        self.button_type = button_type
        self.get_logger().info(f"收到按钮类型: {self.button_type}")

    # ------------------------------------------------------------------
    # 主处理逻辑
    # ------------------------------------------------------------------
    def _process_if_ready(self) -> None:
        if not self._hardware_ready:
            return
        if self.button_center is None or self.button_type is None:
            return

        try:
            self.get_logger().info(f"开始处理按钮: type={self.button_type}")
            success = self._execute_button_action(self.button_center, self.button_type)
            if success:
                self.get_logger().info("按钮操作已完成")
            else:
                self.get_logger().error("按钮操作失败, 请检查日志")
        except Exception as exc:  # 鼓励鲁棒性
            self.get_logger().error(f"执行按钮操作时异常: {exc}")
            self.get_logger().debug(traceback.format_exc())
        finally:
            self.button_center = None
            self.button_type = None

    # ------------------------------------------------------------------
    # 具体执行步骤
    # ------------------------------------------------------------------
    def _execute_button_action(self, button_center_camera: np.ndarray, button_type: str) -> bool:
        piper = button_actions.piper
        if piper is None:
            self.get_logger().error("button_actions.piper 未初始化")
            return False

        current_joints = button_actions.get_current_joints()
        self.get_logger().info(
            f"当前关节角: {np.array(current_joints) * 180.0 / PI}"
        )

        button_base = self._transform_camera_to_base(button_center_camera, current_joints)
        target_base = self._apply_tcp_offset(button_base, current_joints, self.tcp_offset_local)
        self._publish_target_marker(target_base[:3])

        button_actions.TARGET_X = float(target_base[0])
        button_actions.TARGET_Y = float(target_base[1])
        button_actions.TARGET_Z = float(target_base[2])

        self.get_logger().info(
            f"已更新 target XYZ = ({button_actions.TARGET_X:.4f}, "
            f"{button_actions.TARGET_Y:.4f}, {button_actions.TARGET_Z:.4f})"
        )

        action_fn = self.action_map.get(button_type)
        if action_fn is None:
            self.get_logger().error(f"按钮类型 {button_type} 未注册")
            return False

        self.get_logger().info(f"执行动作: {button_type}")
        success = action_fn()
        return bool(success)

    # ------------------------------------------------------------------
    # 工具函数
    # ------------------------------------------------------------------
    def _initialize_hardware(self) -> bool:
        """初始化 Piper SDK / PiperArm, 同步至 button_actions."""
        try:
            self.get_logger().info("初始化 Piper SDK ...")
            piper = C_PiperInterface_V2("can0")
            piper.ConnectPort()
            piper.EnableArm(7)
            enable_fun(piper=piper)
            self.get_logger().info("Piper SDK 初始化成功")
        except Exception as exc:
            self.get_logger().error(f"Piper SDK 初始化失败: {exc}")
            self.get_logger().debug(traceback.format_exc())
            return False

        try:
            piper_arm = PiperArm()
            self.get_logger().info("PiperArm 初始化成功")
        except Exception as exc:
            self.get_logger().error(f"PiperArm 初始化失败: {exc}")
            return False

        button_actions.piper = piper
        button_actions.piper_arm = piper_arm
        self.piper = piper
        self.piper_arm = piper_arm
        return True

    def _transform_camera_to_base(self, button_center_camera: np.ndarray, current_joints) -> np.ndarray:
        # 获取基座到link6的变换矩阵
        base_T_link6 = self.piper_arm.forward_kinematics(current_joints)
        
        # 构建link6到相机的变换矩阵（手眼标定结果）
        link6_T_cam = np.eye(4)
        link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(self.piper_arm.link6_q_camera)
        link6_T_cam[:3, 3] = self.piper_arm.link6_t_camera
        
        # 按钮在相机坐标系的齐次坐标
        button_cam_h = np.array([button_center_camera[0], button_center_camera[1], button_center_camera[2], 1.0])
        
        # ===== 调试输出 =====
        self.get_logger().info("【坐标转换调试】")
        self.get_logger().info(f"  输入: 按钮相机坐标 (optical) = ({button_center_camera[0]:.4f}, {button_center_camera[1]:.4f}, {button_center_camera[2]:.4f})")
        self.get_logger().info(f"  当前关节角 (度) = {np.array(current_joints) * 180 / PI}")
        self.get_logger().info(f"  link6_t_camera = {self.piper_arm.link6_t_camera}")
        self.get_logger().info(f"  link6_q_camera = {self.piper_arm.link6_q_camera}")
        self.get_logger().info(f"  base_T_link6 末端位置 = ({base_T_link6[0, 3]:.4f}, {base_T_link6[1, 3]:.4f}, {base_T_link6[2, 3]:.4f})")
        
        # 方案1: 直接转换 (假设标定使用optical frame)
        button_base_v1 = base_T_link6 @ link6_T_cam @ button_cam_h
        self.get_logger().info(f"  方案1 (optical直接): ({button_base_v1[0]:.4f}, {button_base_v1[1]:.4f}, {button_base_v1[2]:.4f})")
        
        # 方案2: 光学坐标系 → 标准ROS坐标系转换
        # optical: x右 y下 z前  →  standard: x前 y左 z上
        optical_T_standard = np.array([[0, 0, 1, 0],
                                       [-1, 0, 0, 0],
                                       [0, -1, 0, 0],
                                       [0, 0, 0, 1]])
        button_standard_h = optical_T_standard @ button_cam_h
        button_base_v2 = base_T_link6 @ link6_T_cam @ button_standard_h
        self.get_logger().info(f"  方案2 (optical→std): ({button_base_v2[0]:.4f}, {button_base_v2[1]:.4f}, {button_base_v2[2]:.4f})")
        
        # 方案3: 交换轴向 (常见的debug方法)
        button_cam_swapped = np.array([button_center_camera[2], -button_center_camera[0], -button_center_camera[1], 1.0])
        button_base_v3 = base_T_link6 @ link6_T_cam @ button_cam_swapped
        self.get_logger().info(f"  方案3 (轴向交换): ({button_base_v3[0]:.4f}, {button_base_v3[1]:.4f}, {button_base_v3[2]:.4f})")
        
        # 默认使用方案1，请根据实际结果调整
        button_base = button_base_v1
        self.get_logger().warn("⚠️  当前使用方案1，若误差>10cm请根据调试日志切换方案2或3")
        
        return button_base

    def _apply_tcp_offset(self, button_base: np.ndarray, current_joints, tcp_offset_local: np.ndarray) -> np.ndarray:
        # 获取末端姿态
        base_T_link6 = self.piper_arm.forward_kinematics(current_joints)
        R_base_link6 = base_T_link6[:3, :3]
        
        # 将夹爪坐标系的偏移转换到基座坐标系
        offset_base = R_base_link6 @ np.array(tcp_offset_local)
        
        # 应用偏移：目标 = 按钮 - 偏移
        target_base = button_base.copy()
        target_base[:3] = button_base[:3] - offset_base
        
        # ===== 调试输出 =====
        self.get_logger().info("【TCP偏移补偿】")
        self.get_logger().info(f"  按钮位置 (基座系): ({button_base[0]:.4f}, {button_base[1]:.4f}, {button_base[2]:.4f})")
        self.get_logger().info(f"  TCP偏移 (夹爪系): ({tcp_offset_local[0]:.4f}, {tcp_offset_local[1]:.4f}, {tcp_offset_local[2]:.4f})")
        self.get_logger().info(f"  偏移量 (基座系): ({offset_base[0]:.4f}, {offset_base[1]:.4f}, {offset_base[2]:.4f})")
        self.get_logger().info(f"  目标位置 (基座系): ({target_base[0]:.4f}, {target_base[1]:.4f}, {target_base[2]:.4f})")
        
        return target_base

    def _publish_target_marker(self, target_xyz) -> None:
        marker = Marker()
        marker.header.frame_id = "arm_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vision_button_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(target_xyz[0])
        marker.pose.position.y = float(target_xyz[1])
        marker.pose.position.z = float(target_xyz[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        self.marker_pub.publish(marker)


def main() -> None:
    rclpy.init()
    node: Optional[VisionButtonActionNode] = None
    try:
        node = VisionButtonActionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"节点异常退出: {exc}")
        else:
            print(f"vision_button_action_ros2 初始化失败: {exc}")
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
