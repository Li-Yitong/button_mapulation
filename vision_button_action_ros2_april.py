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
        self.declare_parameter("object_point_topic", "/object_point_base")  # 🔧 改为基座系
        self.declare_parameter("button_type_topic", "/button_type")
        self.declare_parameter("button_normal_topic", "/button_normal_base")  # 🔧 改为基座系
        self.declare_parameter("target_marker_topic", "/target_button_base")
        self.declare_parameter("tcp_offset_local", [-0.051, 0.007, 0.080])
        self.declare_parameter("process_rate", 10.0)
        self.declare_parameter("rpy_sample_count", 10)  # AprilTag RPY采样数量

        self.object_topic = self.get_parameter("object_point_topic").get_parameter_value().string_value
        self.button_type_topic = self.get_parameter("button_type_topic").get_parameter_value().string_value
        self.normal_topic = self.get_parameter("button_normal_topic").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("target_marker_topic").get_parameter_value().string_value
        tcp_offset_param = self.get_parameter("tcp_offset_local").get_parameter_value().double_array_value
        self.tcp_offset_local = np.array(tcp_offset_param if tcp_offset_param else [-0.018, 0.007, 0.063])
        process_rate = self.get_parameter("process_rate").get_parameter_value().double_value
        self.process_period = 1.0 / max(float(process_rate) if process_rate else 10.0, 1.0)
        self.rpy_sample_count = int(self.get_parameter("rpy_sample_count").get_parameter_value().integer_value)
        self.home_joints = getattr(button_actions, "HOME_JOINTS", None)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # ---- 运行时状态 ----
        self.button_center: Optional[np.ndarray] = None
        self.button_type: Optional[str] = None
        self.last_point_stamp: Optional[float] = None
        self.last_type_stamp: Optional[float] = None
        self.reference_pose_set: bool = False  # 标记是否已设置参考姿态
        
        # AprilTag RPY采样相关
        self.rpy_samples: list = []  # RPY采样缓冲区
        self.normal_subscription = None  # 订阅器引用（延迟创建）
        self.action_map: Dict[str, Callable[[], bool]] = {
            "toggle": action_toggle,
            "plugin": action_plugin,
            "push": action_push,
            "knob": action_knob,
        }

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, qos)
        self.create_subscription(PointStamped, self.object_topic, self._object_point_callback, qos)
        self.create_subscription(String, self.button_type_topic, self._button_type_callback, qos)
        # 注意：不在初始化时订阅RPY，等到HOME位姿后再订阅

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
            # 延迟移动到HOME，等待ROS2通信建立
            self.get_logger().info("⏳ 等待2秒让ROS2通信稳定...")
            time.sleep(2.0)
            self._move_to_home_position()
            self.get_logger().info("✓ 已到达HOME位姿，开始AprilTag姿态采样...")
            # 启动RPY订阅（采样模式）
            self._start_normal_subscription()

        self.timer = self.create_timer(self.process_period, self._process_if_ready)
        
        # 启动日志
        self.get_logger().info("="*70)
        self.get_logger().info("vision_button_action_ros2 已启动")
        self.get_logger().info("="*70)
        self.get_logger().info(f"  订阅话题:")
        self.get_logger().info(f"    - 按钮位置: {self.object_topic}")
        self.get_logger().info(f"    - 按钮类型: {self.button_type_topic}")
        self.get_logger().info(f"    - 面板法向: {self.normal_topic} (HOME位姿后启动)")
        self.get_logger().info(f"  配置参数:")
        self.get_logger().info(f"    - TCP偏移 (夹爪系): {self.tcp_offset_local}")
        self.get_logger().info(f"    - AprilTag采样数: {self.rpy_sample_count}")
        self.get_logger().info(f"    - 处理频率: {1.0/self.process_period:.1f}Hz")
        self.get_logger().info("="*70)

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------
    def _start_normal_subscription(self):
        """在HOME位姿后启动AprilTag姿态订阅（采样模式）"""
        if self.normal_subscription is None:
            self.get_logger().info("="*70)
            self.get_logger().info("📡 启动AprilTag姿态采样...")
            self.get_logger().info(f"  目标采样数: {self.rpy_sample_count}")
            self.get_logger().info(f"  订阅话题: {self.normal_topic}")
            self.get_logger().info("="*70)
            
            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            qos.history = HistoryPolicy.KEEP_LAST
            
            self.normal_subscription = self.create_subscription(
                Vector3, 
                self.normal_topic, 
                self._button_normal_callback, 
                qos
            )

    # ------------------------------------------------------------------
    # ROS 回调
    # ------------------------------------------------------------------
    def _object_point_callback(self, msg: PointStamped) -> None:
        """接收按钮位置（基座坐标系）"""
        if any(math.isnan(val) for val in (msg.point.x, msg.point.y, msg.point.z)):
            self.get_logger().warn("忽略包含 NaN 的按钮位置")
            return
        self.button_center = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.last_point_stamp = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"收到按钮位置 (基座系): ({msg.point.x:.4f}, {msg.point.y:.4f}, {msg.point.z:.4f})"
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
        """
        接收AprilTag基座系RPY（度）- 采样平均模式
        在HOME位姿采集多个样本，计算平均值后记录
        """
        if any(math.isnan(val) for val in (msg.x, msg.y, msg.z)):
            self.get_logger().warn("忽略包含 NaN 的RPY")
            return
        
        # 如果已经记录完成，忽略后续数据
        if self.reference_pose_set:
            return
        
        # 收集样本
        base_rpy_deg = np.array([msg.x, msg.y, msg.z])
        self.rpy_samples.append(base_rpy_deg)
        
        self.get_logger().info(
            f"📊 采样 {len(self.rpy_samples)}/{self.rpy_sample_count}: "
            f"R={msg.x:.2f}°, P={msg.y:.2f}°, Y={msg.z:.2f}°"
        )
        
        # 达到采样数量后计算平均值
        if len(self.rpy_samples) >= self.rpy_sample_count:
            # 计算平均值
            rpy_array = np.array(self.rpy_samples)
            avg_rpy_deg = np.mean(rpy_array, axis=0)
            std_rpy_deg = np.std(rpy_array, axis=0)
            
            self.get_logger().info("="*70)
            self.get_logger().info("📊 采样完成，计算平均姿态：")
            self.get_logger().info(f"  平均值: R={avg_rpy_deg[0]:.2f}°, P={avg_rpy_deg[1]:.2f}°, Y={avg_rpy_deg[2]:.2f}°")
            self.get_logger().info(f"  标准差: R={std_rpy_deg[0]:.2f}°, P={std_rpy_deg[1]:.2f}°, Y={std_rpy_deg[2]:.2f}°")
            
            # 检查标准差（如果过大，说明检测不稳定）
            max_std = np.max(std_rpy_deg)
            if max_std > 5.0:  # 标准差超过5度
                self.get_logger().warn(f"⚠️  姿态检测不稳定（最大标准差={max_std:.2f}°），但仍将使用平均值")
            
            # 转换为弧度并记录
            avg_rpy_rad = np.radians(avg_rpy_deg)
            success = button_actions.set_apriltag_reference_from_base_rpy(avg_rpy_rad)
            
            if success:
                self.reference_pose_set = True
                self.get_logger().info("✓ 已记录AprilTag参考姿态（基于平均值）")
                self.get_logger().info(f"  Tag RPY (基座系): R={avg_rpy_deg[0]:.1f}°, P={avg_rpy_deg[1]:.1f}°, Y={avg_rpy_deg[2]:.1f}°")
                self.get_logger().info(f"  夹爪垂直按压时: R={-avg_rpy_deg[0]:.1f}°, P={avg_rpy_deg[1]:.1f}°, Y={avg_rpy_deg[2]:.1f}°")
                self.get_logger().info("="*70)
                
                # 停止订阅，节省资源
                if self.normal_subscription is not None:
                    self.destroy_subscription(self.normal_subscription)
                    self.normal_subscription = None
                    self.get_logger().info("✓ 已停止AprilTag姿态订阅")
            else:
                self.get_logger().error("✗ 记录参考姿态失败")

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
        
        self.button_center = None
        self.button_type = None

        def worker():
            try:
                self.get_logger().info(f"开始处理按钮: type={button_type}")
                success = self._execute_button_action(button_center, button_type)
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
        button_center_base: np.ndarray,
        button_type: str
    ) -> bool:
        """
        执行按钮动作
        
        参数：
        - button_center_base: 按钮在基座系的XYZ坐标
        - button_type: 按钮类型 ('toggle'/'plugin'/'push'/'knob')
        
        姿态约束：由 APRILTAG_REFERENCE_POSE_BASE 全局变量提供（在HOME位姿采样记录）
        
        检查项：
        1. 硬件初始化
        2. AprilTag参考姿态已设置
        3. 设置目标位置（XYZ）
        """
        piper = button_actions.piper
        if piper is None:
            self.get_logger().error("button_actions.piper 未初始化")
            return False

        # 检查AprilTag参考姿态
        if not self.reference_pose_set:
            self.get_logger().error("="*70)
            self.get_logger().error("✗✗✗ 无法执行：面板参考姿态未设置")
            self.get_logger().error("  原因：未完成AprilTag姿态采样")
            self.get_logger().error("  解决方案：")
            self.get_logger().error("    1. 确保机械臂在HOME位姿")
            self.get_logger().error("    2. 确保AprilTag在相机视野内")
            self.get_logger().error(f"    3. 等待系统采样{self.rpy_sample_count}次RPY数据")
            self.get_logger().error("="*70)
            return False

        current_joints = button_actions.get_current_joints()
        self.get_logger().info(
            f"当前关节角: {np.array(current_joints) * 180.0 / PI}"
        )

        # 🔧 直接使用基座坐标系数据，只需应用TCP偏移
        button_base_h = np.array([button_center_base[0], button_center_base[1], button_center_base[2], 1.0])
        target_base = self._apply_tcp_offset(button_base_h, current_joints, self.tcp_offset_local)
        self._publish_target_marker(target_base[:3])

        button_actions.TARGET_X = float(target_base[0])
        button_actions.TARGET_Y = float(target_base[1])
        button_actions.TARGET_Z = float(target_base[2])

        self.get_logger().info(
            f"已更新 target XYZ (基座系) = ({button_actions.TARGET_X:.4f}, "
            f"{button_actions.TARGET_Y:.4f}, {button_actions.TARGET_Z:.4f})"
        )

        # 姿态约束由APRILTAG_REFERENCE_POSE_BASE全局变量提供
        # button_actions.py 会自动使用该变量计算夹爪姿态

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
