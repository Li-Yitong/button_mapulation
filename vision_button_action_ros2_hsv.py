#!/usr/bin/env python3
"""ROS2视觉按钮操作整合器

该节点订阅视觉检测到的按钮位置/类型, 根据 `button_actions.py` 提供的动作流程
驱动 Piper 机械臂执行 Toggle / Plug-in / Push / Knob 四类操作。
"""

from __future__ import annotations

import math
import time
import threading
import traceback
from typing import Callable, Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3
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
        self.declare_parameter("button_normal_topic", "/button_normal")
        self.declare_parameter("target_marker_topic", "/target_button_base")
        self.declare_parameter("tcp_offset_local", [-0.051, 0.007, 0.080])
        self.declare_parameter("process_rate", 10.0)
        self.declare_parameter("normal_timeout", 5.0)  # 法向量超时（秒）

        self.object_topic = self.get_parameter("object_point_topic").get_parameter_value().string_value
        self.button_type_topic = self.get_parameter("button_type_topic").get_parameter_value().string_value
        self.normal_topic = self.get_parameter("button_normal_topic").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("target_marker_topic").get_parameter_value().string_value
        tcp_offset_param = self.get_parameter("tcp_offset_local").get_parameter_value().double_array_value
        self.tcp_offset_local = np.array(tcp_offset_param if tcp_offset_param else [-0.018, 0.007, 0.063])
        process_rate = self.get_parameter("process_rate").get_parameter_value().double_value
        self.process_period = 1.0 / max(float(process_rate) if process_rate else 10.0, 1.0)
        self.normal_timeout = self.get_parameter("normal_timeout").get_parameter_value().double_value
        self.home_joints = getattr(button_actions, "HOME_JOINTS", None)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # ---- 运行时状态 ----
        self.button_center: Optional[np.ndarray] = None
        self.button_type: Optional[str] = None
        self.button_normal: Optional[np.ndarray] = None  # 面板法向量（相机坐标系）
        self.last_point_stamp: Optional[float] = None
        self.last_type_stamp: Optional[float] = None
        self.last_normal_stamp: Optional[float] = None
        self.action_map: Dict[str, Callable[[], bool]] = {
            "toggle": action_toggle,
            "plugin": action_plugin,
            "push": action_push,
            "knob": action_knob,
        }

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, qos)
        self.create_subscription(PointStamped, self.object_topic, self._object_point_callback, qos)
        self.create_subscription(String, self.button_type_topic, self._button_type_callback, qos)
        self.create_subscription(Vector3, self.normal_topic, self._button_normal_callback, qos)

        # 动作执行线程状态
        self._action_thread: Optional[threading.Thread] = None
        self._action_lock = threading.Lock()

        self._hardware_ready = self._initialize_hardware()
        if not self._hardware_ready:
            self.get_logger().fatal("硬件初始化失败, 节点将退出")
            raise RuntimeError("hardware init failed")

        if (
            self._hardware_ready
            and self.home_joints is not None
            and getattr(button_actions, "USE_HOME_POSITION", False)
        ):
            self._move_to_home_position()

        self.timer = self.create_timer(self.process_period, self._process_if_ready)
        
        # 启动日志
        self.get_logger().info("="*70)
        self.get_logger().info("vision_button_action_ros2 已启动")
        self.get_logger().info("="*70)
        self.get_logger().info(f"  订阅话题:")
        self.get_logger().info(f"    - 按钮位置: {self.object_topic}")
        self.get_logger().info(f"    - 按钮类型: {self.button_type_topic}")
        self.get_logger().info(f"    - 面板法向: {self.normal_topic}")
        self.get_logger().info(f"  配置参数:")
        self.get_logger().info(f"    - TCP偏移 (夹爪系): {self.tcp_offset_local}")
        self.get_logger().info(f"    - 法向量超时: {self.normal_timeout}秒")
        self.get_logger().info(f"    - 处理频率: {1.0/self.process_period:.1f}Hz")
        self.get_logger().info("="*70)

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
        self.last_type_stamp = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"收到按钮类型: {self.button_type}")

    def _button_normal_callback(self, msg: Vector3) -> None:
        """接收面板法向量（假设为相机坐标系）"""
        if any(math.isnan(val) for val in (msg.x, msg.y, msg.z)):
            self.get_logger().warn("忽略包含 NaN 的法向量")
            return
        self.button_normal = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.last_normal_stamp = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"收到面板法向量 (相机系): ({msg.x:.4f}, {msg.y:.4f}, {msg.z:.4f})"
        )

    # ------------------------------------------------------------------
    # 主处理逻辑
    # ------------------------------------------------------------------
    def _process_if_ready(self) -> None:
        if not self._hardware_ready:
            return
        if self.button_center is None or self.button_type is None:
            return

        # 避免重复启动
        if self._action_thread is not None and self._action_thread.is_alive():
            self.get_logger().warn("已有动作在执行，忽略新的按钮请求")
            return

        # 时间戳校验：确保位置和类型来自同一检测周期
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_point_stamp is not None and self.last_type_stamp is not None:
            time_diff = abs(self.last_point_stamp - self.last_type_stamp)
            if time_diff > 1.0:  # 超过1秒认为不匹配
                self.get_logger().warn(
                    f"位置与类型时间戳不匹配 (差值={time_diff:.2f}s)，忽略本次请求"
                )
                self.button_center = None
                self.button_type = None
                return

        button_center = np.copy(self.button_center)
        button_type = str(self.button_type)
        button_normal = np.copy(self.button_normal) if self.button_normal is not None else None
        normal_stamp = self.last_normal_stamp
        
        self.button_center = None
        self.button_type = None

        def worker():
            try:
                self.get_logger().info(f"开始处理按钮: type={button_type}")
                
                # 检查法向量是否可用
                if button_normal is not None and normal_stamp is not None:
                    normal_age = current_time - normal_stamp
                    if normal_age > self.normal_timeout:
                        self.get_logger().warn(
                            f"⚠️  法向量数据过期 ({normal_age:.1f}秒)，将使用默认姿态"
                        )
                        button_normal_valid = None
                    else:
                        button_normal_valid = button_normal
                        self.get_logger().info(
                            f"✓ 法向量有效 (age={normal_age:.1f}s): "
                            f"({button_normal[0]:.4f}, {button_normal[1]:.4f}, {button_normal[2]:.4f})"
                        )
                else:
                    self.get_logger().warn("⚠️  未收到法向量数据，将使用默认姿态")
                    button_normal_valid = None
                
                success = self._execute_button_action(button_center, button_type, button_normal_valid)
                if success:
                    self.get_logger().info("按钮操作已完成")
                else:
                    self.get_logger().error("按钮操作失败, 请检查日志")
            except Exception as exc:
                self.get_logger().error(f"执行按钮操作时异常: {exc}")
                self.get_logger().debug(traceback.format_exc())

        self._action_thread = threading.Thread(target=worker, daemon=True)
        self._action_thread.start()

    # ------------------------------------------------------------------
    # 具体执行步骤
    # ------------------------------------------------------------------
    def _execute_button_action(
        self, 
        button_center_camera: np.ndarray, 
        button_type: str,
        button_normal_camera: Optional[np.ndarray] = None
    ) -> bool:
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

        # 处理法向量并构建目标位姿矩阵
        if button_normal_camera is not None:
            try:
                # 将法向量从相机系转换到基座系
                normal_base = self._transform_normal_camera_to_base(
                    button_normal_camera, current_joints
                )
                self.get_logger().info(
                    f"✓ 法向量 (基座系): ({normal_base[0]:.4f}, {normal_base[1]:.4f}, {normal_base[2]:.4f})"
                )
                
                # 构建目标位姿矩阵（使法向量对齐）
                target_pose_matrix = self._build_target_pose_matrix(target_base[:3], normal_base)
                button_actions.TARGET_POSE_MATRIX = target_pose_matrix
                
                self.get_logger().info("✓ 已生成目标位姿矩阵（包含法向量对齐）")
            except Exception as exc:
                self.get_logger().warn(f"⚠️  法向量处理失败: {exc}，将使用默认姿态")
                button_actions.TARGET_POSE_MATRIX = None
        else:
            self.get_logger().warn("⚠️  无法向量，将使用默认姿态")
            button_actions.TARGET_POSE_MATRIX = None

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

        # 初始化MoveIt2（复用当前节点）
        if button_actions.USE_MOVEIT:
            self.get_logger().info("初始化 MoveIt2...")
            moveit_success = button_actions.initialize_moveit2(external_node=self)
            if moveit_success:
                self.get_logger().info("✓ MoveIt2 初始化成功")
            else:
                self.get_logger().warn("⚠️  MoveIt2 初始化失败，将使用SDK模式")

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

        # 🔧 使用方案1（optical直接）- 与参考实现保持一致
        button_base = button_base_v1
        self.get_logger().info("✓ 使用方案1 (optical直接转换)")
        
        # 验证高度合理性（按钮应该在5cm~70cm之间）
        if button_base[2] < 0.05 or button_base[2] > 0.70:
            self.get_logger().warn(
                f"⚠️  按钮高度异常: {button_base[2]*100:.1f}cm (预期范围: 5~70cm)"
            )

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

    def _transform_normal_camera_to_base(
        self, 
        normal_camera: np.ndarray, 
        current_joints
    ) -> np.ndarray:
        """
        将相机坐标系的法向量转换到基座坐标系
        
        注意：法向量只需旋转，不需要平移
        """
        # 获取基座到link6的变换矩阵
        base_T_link6 = self.piper_arm.forward_kinematics(current_joints)
        
        # 构建link6到相机的旋转矩阵
        link6_R_cam = quaternion_to_rotation_matrix(self.piper_arm.link6_q_camera)
        
        # 提取基座到link6的旋转矩阵
        base_R_link6 = base_T_link6[:3, :3]
        
        # 法向量转换：直接转换（与位置转换保持一致）
        normal_base = base_R_link6 @ link6_R_cam @ normal_camera
        
        # 归一化
        normal_base = normal_base / np.linalg.norm(normal_base)
        
        return normal_base

    def _build_target_pose_matrix(
        self, 
        target_xyz: np.ndarray, 
        normal_base: np.ndarray
    ) -> np.ndarray:
        """
        构建目标位姿矩阵，使末端Z轴对齐法向量
        
        Args:
            target_xyz: 目标位置 [x, y, z]
            normal_base: 面板法向量 (基座坐标系，已归一化)
        
        Returns:
            4x4 齐次变换矩阵
        """
        # 末端Z轴对齐法向量反向（机械臂应朝向面板）
        z_axis = -normal_base / np.linalg.norm(normal_base)
        
        # 选择一个辅助向量来构建完整的旋转矩阵
        # 通常选择世界Z轴或X轴
        if abs(z_axis[2]) < 0.9:
            # 如果末端Z轴不接近竖直，用世界Z轴叉乘
            x_axis = np.cross([0, 0, 1], z_axis)
        else:
            # 如果末端Z轴接近竖直，用世界X轴叉乘
            x_axis = np.cross([1, 0, 0], z_axis)
        
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # 构建变换矩阵
        target_matrix = np.eye(4)
        target_matrix[:3, 0] = x_axis
        target_matrix[:3, 1] = y_axis
        target_matrix[:3, 2] = z_axis
        target_matrix[:3, 3] = target_xyz
        
        return target_matrix

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

    def _move_to_home_position(self, speed: int = 50, settle: float = 2.0) -> None:
        control_fn = getattr(button_actions, "control_arm_sdk", None)
        if control_fn is None:
            self.get_logger().warn("button_actions.control_arm_sdk 不可用，跳过HOME定位")
            return

        self.get_logger().info("移动到HOME位姿，等待视觉输入...")
        try:
            control_fn(self.home_joints, speed=speed)
            if settle > 0:
                time.sleep(settle)
            self.get_logger().info("✓ 已到达HOME位姿")
        except Exception as exc:
            self.get_logger().error(f"移动到HOME位姿失败: {exc}")


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
