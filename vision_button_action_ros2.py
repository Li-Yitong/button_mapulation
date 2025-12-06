#!/usr/bin/env python3
"""ROS2视觉按钮操作整合器

该节点订阅视觉检测到的按钮位置/类型, 根据 `button_actions.py` 提供的动作流程
驱动 Piper 机械臂执行 Toggle / Plug-in / Push / Knob 四类操作。
"""
from __future__ import annotations

import math
import time
import traceback
from typing import Callable, Dict, Optional
import threading

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
from utils.utils_plane import compute_approach_pose

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
        self.declare_parameter("tcp_offset_local", [-0.051, 0.007, 0.080])
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
        # ✅ 简化工作流：两阶段状态机
        self.workflow_state = "HOME"  # HOME → BUTTON_ACTION（删除了PANEL_ALIGN阶段）
        
        # 数据存储
        self.button_center: Optional[np.ndarray] = None
        self.button_type: Optional[str] = None
        self.button_normal: Optional[np.ndarray] = None  # 面板法向量（基座系），用于按压时调整朝向
        self.last_point_stamp: Optional[float] = None
        
        # 面板区域估计（用于计算面板中心）
        self.detected_buttons: list = []  # 存储检测到的所有按钮位置（基座系）
        self.panel_center_base: Optional[np.ndarray] = None  # 估计的面板中心（基座系）
        
        self.action_map = {
            "toggle": action_toggle,
            "plugin": action_plugin,
            "push": action_push,
            "knob": action_knob,
        }

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, qos)
        self.create_subscription(PointStamped, self.object_topic, self._object_point_callback, qos)
        self.create_subscription(String, self.button_type_topic, self._button_type_callback, qos)
        self.create_subscription(Vector3, "/button_normal", self._button_normal_callback, qos)

        # 动作执行线程状态
        self._action_thread: Optional[threading.Thread] = None
        self._action_lock = threading.Lock()
        
        # 用户交互线程
        self._user_input_thread: Optional[threading.Thread] = None

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
        
        # ✅ 简化：只记录按钮位置，不再收集（法向量由视觉系统直接从蓝色区域计算）
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
    
    def _button_normal_callback(self, msg: Vector3) -> None:
        """接收面板法向量（基座坐标系）"""
        if any(math.isnan(val) for val in (msg.x, msg.y, msg.z)):
            self.get_logger().warn("忽略包含 NaN 的法向量")
            return
        
        normal_raw = np.array([msg.x, msg.y, msg.z], dtype=float)
        
        # 验证法向量方向：应该指向外侧（远离机械臂基座）
        # 方法：计算当前末端位置，法向量应该从面板指向末端方向
        current_joints = button_actions.get_current_joints()
        base_T_link6 = self.piper_arm.forward_kinematics(current_joints)
        camera_position = base_T_link6[:3, 3]
        
        # 假设面板在相机前方，法向量应该指向相机方向
        # 如果法向量与"相机-面板"方向相反，则翻转
        # 简化判断：法向量的Z分量应该为正（指向上方/外侧）或X分量为正（指向前方）
        # 更精确的方法：检查法向量是否指向相机
        
        # 使用相机位置作为参考：法向量应该大致指向相机方向
        # 计算从面板（假设在视野中心）到相机的向量
        # 由于没有面板位置，使用简化规则：
        # 在机械臂前侧操作时，法向量X分量通常为正（指向机械臂）
        # 或者Z分量为正（倾斜面板时）
        
        # 🔧 关键判断：法向量应该指向"外侧"（远离面板表面）
        # 对于前置相机：法向量的Z分量应该>0（指向上方）或X分量>0（指向前方）
        # 如果两者都是负数，说明指向内侧，需要翻转
        
        normal = normal_raw.copy()
        flip_reason = None
        
        # 规则1：优先检查Z分量（高度方向）
        # 对于水平/倾斜面板，法向量的Z分量通常应该>0（指向上方外侧）
        if abs(normal[2]) > 0.5:  # Z分量占主导
            if normal[2] < 0:  # 指向下方，错误
                normal = -normal
                flip_reason = "Z分量<0 (指向下方，已翻转)"
        
        # 规则2：检查X分量（前后方向）
        # 对于竖直面板，法向量的X分量应该>0（指向机械臂前方）
        elif abs(normal[0]) > 0.5:  # X分量占主导
            if normal[0] < 0:  # 指向后方，错误
                normal = -normal
                flip_reason = "X分量<0 (指向后方，已翻转)"
        
        # 规则3：综合判断（X和Z都不占主导时）
        else:
            # 法向量应该指向机械臂工作空间的外侧
            # 对于前方的面板：X>0 或 Z>0
            if normal[0] < 0 and normal[2] < 0:
                normal = -normal
                flip_reason = "X,Z均<0 (指向内侧，已翻转)"
        
        self.button_normal = normal
        
        if flip_reason:
            self.get_logger().warn(f"⚠️  法向量方向异常，自动修正: {flip_reason}")
            self.get_logger().info(
                f"  原始法向量: ({normal_raw[0]:.4f}, {normal_raw[1]:.4f}, {normal_raw[2]:.4f})"
            )
            self.get_logger().info(
                f"  修正后法向量: ({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})"
            )
        else:
            self.get_logger().info(
                f"收到面板法向量(基座系): ({msg.x:.4f}, {msg.y:.4f}, {msg.z:.4f}) ✓方向正确"
            )

    # ------------------------------------------------------------------
    # 用户交互循环
    # ------------------------------------------------------------------
    def _user_interaction_loop(self) -> None:
        """用户交互主循环 - ✅ 简化为两阶段工作流"""
        try:
            while True:
                if self.workflow_state == "HOME":
                    self._handle_home_state()
                elif self.workflow_state == "BUTTON_ACTION":
                    self._handle_button_action_state()
                else:
                    time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"用户交互循环异常: {e}")
            import traceback
            self.get_logger().debug(traceback.format_exc())
    
    def _handle_home_state(self) -> None:
        """阶段1: HOME观察位姿"""
        # 清空之前的数据
        self.detected_buttons.clear()
        self.panel_center_base = None
        
        print("\n" + "="*70)
        print("阶段1: HOME观察位姿")
        print("="*70)
        print("机械臂已在HOME位姿，相机正在观察工作区域")
        print("请确保视觉检测节点正在运行")
        print("\n系统正在实时计算：")
        print("  - 面板法向量（基于RANSAC平面拟合）")
        print("  - 按钮位置分布（自动收集检测结果）")
        print("\n按 Enter 键进入面板对齐阶段...")
        input()
        
        self.get_logger().info("✓ 用户确认，检查法向量数据...")
        
        # ✅ 简化：直接进入按钮操作阶段（跳过面板对齐）
        if self.button_normal is not None:
            self.get_logger().info(f"✓ 已接收面板法向量: ({self.button_normal[0]:.4f}, {self.button_normal[1]:.4f}, {self.button_normal[2]:.4f})")
        else:
            self.get_logger().warn("⚠️  尚未接收到法向量，将在按压时使用默认朝向")
        
        self.workflow_state = "BUTTON_ACTION"
        self.get_logger().info("✓ 准备就绪，等待选择按钮执行操作...")
    
    def _handle_button_action_state(self) -> None:
        """阶段2: 按钮操作执行（✅ 简化版：直接从HOME到按钮）"""
        print("\n" + "="*70)
        print("阶段2: 按钮操作执行")
        print("="*70)
        print("机械臂在HOME位姿，等待选择按钮")
        print("系统将：")
        print("  1. 直接移动到按钮上方（保持HOME朝向，IK高成功率）")
        print("  2. 按压时调整朝向垂直面板（小幅度调整）")
        print("\n等待接收按钮位置和类型数据...")
        print("(提示: 在视觉检测界面点击按钮)")
        
        # 等待接收按钮数据
        while self.button_center is None or self.button_type is None:
            time.sleep(0.1)
        
        # 获取按钮数据
        button_center = np.copy(self.button_center)
        button_type = str(self.button_type)
        self.button_center = None
        self.button_type = None
        
        self.get_logger().info(f"✓ 接收到按钮: type={button_type}")
        print(f"\n✓ 检测到按钮类型: {button_type}")
        print("按 Enter 键执行按钮操作...")
        input()
        
        # 执行按钮动作
        try:
            success = self._execute_button_action(button_center, button_type, self.button_normal)
            if success:
                self.get_logger().info("✓ 按钮操作完成")
                print("\n✓ 按钮操作成功完成！")
            else:
                self.get_logger().error("✗ 按钮操作失败")
                print("\n✗ 按钮操作失败，请查看日志")
        except Exception as e:
            self.get_logger().error(f"按钮操作异常: {e}")
            print(f"\n✗ 操作异常: {e}")
        
        # 询问是否继续
        print("\n是否继续操作其他按钮？")
        print("  y - 继续")
        print("  n - 返回HOME位姿")
        choice = input("请选择 (y/n): ").strip().lower()
        
        if choice == 'y':
            # 继续操作其他按钮
            pass
        else:
            # 返回HOME
            self.get_logger().info("返回HOME位姿...")
            self._move_to_home_position()
            self.workflow_state = "HOME"

    # ------------------------------------------------------------------
    # 主处理逻辑（已废弃，由用户交互循环接管）
    # ------------------------------------------------------------------
    def _process_if_ready(self) -> None:
        """定时检查 - 已改为用户交互驱动，此函数保留但不执行动作"""
        # 仅用于保持ROS2节点活跃
        pass

    # ------------------------------------------------------------------
    # 具体执行步骤
    # ------------------------------------------------------------------
    def _execute_button_action(self, button_center_camera: np.ndarray, button_type: str, 
                               button_normal_base: Optional[np.ndarray]) -> bool:
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

        # 🔧 新增：如果有法向量，计算接近位姿并设置到 button_actions
        if button_normal_base is not None:
            self.get_logger().info(
                f"✓ 使用面板法向量计算接近位姿: ({button_normal_base[0]:.4f}, "
                f"{button_normal_base[1]:.4f}, {button_normal_base[2]:.4f})"
            )
            
            # 计算接近位姿：按钮上方30cm，Gripper Z轴 = 法向量（垂直于面板）
            approach_pose = self._compute_approach_pose_base(
                target_base[:3], 
                button_normal_base, 
                approach_distance=0.30
            )
            
            # 设置完整位姿矩阵到 button_actions
            button_actions.TARGET_POSE_MATRIX = approach_pose
            self.get_logger().info(f"  接近位置: ({approach_pose[0,3]:.4f}, {approach_pose[1,3]:.4f}, {approach_pose[2,3]:.4f})")
            self.get_logger().info(f"  Gripper Z轴（=法向量）: ({approach_pose[0,2]:.4f}, {approach_pose[1,2]:.4f}, {approach_pose[2,2]:.4f})")
            self.get_logger().info(f"  → Gripper Z轴垂直于面板，可直接沿Z轴按压")
        else:
            self.get_logger().warn("⚠️  无法向量，使用默认姿态（末端朝前）")
            button_actions.TARGET_POSE_MATRIX = None
            button_actions.TARGET_X = float(target_base[0])
            button_actions.TARGET_Y = float(target_base[1])
            button_actions.TARGET_Z = float(target_base[2])

        self.get_logger().info(
            f"目标位置: ({target_base[0]:.4f}, {target_base[1]:.4f}, {target_base[2]:.4f})"
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
    
    def _move_to_home_position(self) -> bool:
        """
        移动到HOME观察位姿
        
        返回:
            True: 成功到达
            False: 移动失败
        """
        if not hasattr(button_actions, 'HOME_JOINTS'):
            self.get_logger().error("HOME_JOINTS 未定义")
            return False
        
        home_joints = button_actions.HOME_JOINTS
        home_gripper = button_actions.HOME_GRIPPER if hasattr(button_actions, 'HOME_GRIPPER') else 0
        
        self.get_logger().info(
            f"目标关节角(度): {np.array(home_joints) * 180.0 / PI}"
        )
        
        try:
            # 使用SDK模式移动（更可靠）
            piper = button_actions.piper
            if piper is None:
                self.get_logger().error("piper SDK 未初始化")
                return False
            
            # 设置夹爪位置
            piper.GripperCtrl(home_gripper, 1000, 0x01, 0)
            time.sleep(0.3)
            
            # 转换为整数格式（SDK要求）
            factor = 1000 * 180 / PI
            joints_int = [int(home_joints[i] * factor) for i in range(6)]
            
            # 设置运动模式
            piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
            time.sleep(0.1)  # 等待运动模式切换生效
            
            # 发送关节控制指令
            piper.JointCtrl(*joints_int)
            self.get_logger().info("✓ HOME位姿运动指令已发送，等待到达...")
            
            # 估算运动时间
            current = button_actions.get_current_joints()
            max_joint_diff = max([abs(home_joints[i] - current[i]) for i in range(6)])
            estimated_time = max_joint_diff / (50 / 100.0 * 2.0) + 0.5
            estimated_time = min(estimated_time, 10.0)
            
            self.get_logger().info(f"  预计运动时间: {estimated_time:.1f}秒")
            time.sleep(estimated_time)
            
            # 验证是否到达
            current = button_actions.get_current_joints()
            error = np.array(current) - np.array(home_joints)
            max_error_deg = np.max(np.abs(error)) * 180.0 / PI
            
            if max_error_deg < 5.0:
                self.get_logger().info(f"✓ 已到达HOME位姿（误差 {max_error_deg:.2f}°）")
                return True
            else:
                self.get_logger().warn(f"⚠️  HOME位姿误差较大（{max_error_deg:.2f}°），但继续运行")
                return True
                
        except Exception as e:
            self.get_logger().error(f"移动到HOME位姿异常: {e}")
            import traceback
            self.get_logger().debug(traceback.format_exc())
            return False
    
    def _enable_arm_robust(self, piper: C_PiperInterface_V2, timeout: float = 10.0) -> bool:
        """
        健壮的机械臂使能方法（不会exit）
        
        参数:
            piper: Piper SDK接口
            timeout: 超时时间（秒）
        
        返回:
            True: 使能成功
            False: 使能失败
        """
        import time
        start_time = time.time()
        retry_count = 0
        
        while time.time() - start_time < timeout:
            retry_count += 1
            
            # 发送使能指令
            piper.EnableArm(7)
            time.sleep(0.5)  # 等待使能生效
            
            # 检查使能状态
            try:
                status = piper.GetArmLowSpdInfoMsgs()
                enable_flag = (
                    status.motor_1.foc_status.driver_enable_status and
                    status.motor_2.foc_status.driver_enable_status and
                    success = button_actions.control_arm_moveit(intermediate_joints, speed=50)
                else:
                    success = button_actions.control_arm_sdk(intermediate_joints, speed=50)
                
                if not success:
                    self.get_logger().error("✗ 移动到中间位姿失败")
                    return False
                
                time.sleep(0.5)
                actual_joints_step1 = button_actions.get_current_joints()
                actual_pose_step1 = self.piper_arm.forward_kinematics(actual_joints_step1)
                actual_pos_step1 = actual_pose_step1[:3, 3]
                pos_error = np.linalg.norm(actual_pos_step1 - target_pos)
                self.get_logger().info(f"  ✓ 已到达面板前方，位置误差: {pos_error*100:.2f}cm")
            
            # ========== 阶段2：微调姿态对齐法向量 ==========
            self.get_logger().info("阶段2: 微调姿态对齐法向量...")
            
            # 🔧 关键：使用笛卡尔插值逐步调整姿态
            # 方法：计算从当前姿态到目标姿态的旋转，分多步插值
            
            current_rot = actual_pose_step1[:3, :3]
            target_rot_final = target_rot
            
            # 计算姿态差异
            from scipy.spatial.transform import Rotation as R
            R_current = R.from_matrix(current_rot)
            R_target = R.from_matrix(target_rot_final)
            rotation_diff = (R_target.inv() * R_current).magnitude() * 180 / np.pi
            
            self.get_logger().info(f"  当前姿态与目标姿态差异: {rotation_diff:.2f}°")
            
            # 🔧 优化：如果姿态差异小，减少插值步数
            if rotation_diff < 5.0:
                self.get_logger().info("  ✓ 姿态已接近目标，跳过调整")
                return True
            elif rotation_diff < 20.0:
                num_orientation_steps = 3
                self.get_logger().info(f"  姿态差异较小，使用 {num_orientation_steps} 步调整")
            else:
                num_orientation_steps = 5
                self.get_logger().info(f"  姿态差异较大，使用 {num_orientation_steps} 步调整")
            
            # 生成多个中间姿态（SLERP插值）
            orientation_waypoints = []
            
            # 使用 scipy 的 Slerp 类进行球面线性插值
            from scipy.spatial.transform import Slerp
            key_times = [0, 1]
            key_rots = R.from_matrix([current_rot, target_rot_final])
            slerp = Slerp(key_times, key_rots)
            
            for i in range(1, num_orientation_steps + 1):
                alpha = i / num_orientation_steps
                R_interp = slerp([alpha])[0]  # Slerp返回数组，取第一个
                
                waypoint_pose = np.eye(4)
                waypoint_pose[:3, :3] = R_interp.as_matrix()
                waypoint_pose[:3, 3] = target_pos  # 保持位置不变
                orientation_waypoints.append(waypoint_pose)
            
            # 逐步执行姿态调整
            current_joints_orient = actual_joints_step1
            for step_idx, waypoint in enumerate(orientation_waypoints):
                self.get_logger().info(f"  姿态调整步骤 {step_idx+1}/{num_orientation_steps}...")
                
                # IK求解
                result = self.piper_arm.inverse_kinematics_refined(
                    waypoint,
                    initial_guess=current_joints_orient,
                    max_iterations=50,
                    tolerance=3e-3
                )
                
                if result is None or not isinstance(result, tuple):
                    self.get_logger().warn(f"    步骤 {step_idx+1} IK失败，停止姿态调整")
                    break
                
                joints_candidate, converged, error = result[:3]
                if not isinstance(joints_candidate, (list, np.ndarray)) or len(joints_candidate) != 6:
                    self.get_logger().warn(f"    步骤 {step_idx+1} IK结果无效")
                    break
                
                # 移动到该姿态
                if button_actions.USE_MOVEIT:
                    step_success = button_actions.control_arm_moveit(joints_candidate, speed=30)
                else:
                    step_success = button_actions.control_arm_sdk(joints_candidate, speed=30)
                
                if not step_success:
                    self.get_logger().warn(f"    步骤 {step_idx+1} 移动失败")
                    break
                
                time.sleep(0.3)
                current_joints_orient = button_actions.get_current_joints()
            
            # ========== 验证最终姿态 ==========
            final_joints = button_actions.get_current_joints()
            final_pose = self.piper_arm.forward_kinematics(final_joints)
            final_pos = final_pose[:3, 3]
            final_rot_actual = final_pose[:3, :3]
            
            pos_error_final = np.linalg.norm(final_pos - target_pos)
            
            # 计算姿态误差（旋转角度差）
            R_final = R.from_matrix(final_rot_actual)
            R_target_check = R.from_matrix(target_rot_final)
            rotation_error = (R_target_check.inv() * R_final).magnitude() * 180 / np.pi
            
            self.get_logger().info(f"✓ 面板对齐完成:")
            self.get_logger().info(f"  位置误差: {pos_error_final*100:.2f}cm")
            self.get_logger().info(f"  姿态误差: {rotation_error:.2f}°")
            
            if pos_error_final > 0.05 or rotation_error > 15.0:
                self.get_logger().warn(f"⚠️  对齐精度不佳，但继续执行")
            
            return True
                
        except Exception as e:
            self.get_logger().error(f"移动到面板对齐位姿异常: {e}")
            import traceback
            self.get_logger().debug(traceback.format_exc())
            return False
    
    def _move_to_home_position(self) -> bool:
        """
        移动到HOME观察位姿
        
        返回:
            True: 成功到达
            False: 移动失败
        """
        if not hasattr(button_actions, 'HOME_JOINTS'):
            self.get_logger().error("HOME_JOINTS 未定义")
            return False
        
        home_joints = button_actions.HOME_JOINTS
        home_gripper = button_actions.HOME_GRIPPER if hasattr(button_actions, 'HOME_GRIPPER') else 0
        
        self.get_logger().info(
            f"目标关节角(度): {np.array(home_joints) * 180.0 / PI}"
        )
        
        try:
            # 使用SDK模式移动（更可靠）
            piper = button_actions.piper
            if piper is None:
                self.get_logger().error("piper SDK 未初始化")
                return False
            
            # 设置夹爪位置
            piper.GripperCtrl(home_gripper, 1000, 0x01, 0)
            time.sleep(0.3)
            
            # 转换为整数格式（SDK要求）
            factor = 1000 * 180 / PI
            joints_int = [int(home_joints[i] * factor) for i in range(6)]
            
            # 设置运动模式
            piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
            time.sleep(0.1)  # 等待运动模式切换生效
            
            # 发送关节控制指令
            piper.JointCtrl(*joints_int)
            self.get_logger().info("✓ HOME位姿运动指令已发送，等待到达...")
            
            # 估算运动时间
            current = button_actions.get_current_joints()
            max_joint_diff = max([abs(home_joints[i] - current[i]) for i in range(6)])
            estimated_time = max_joint_diff / (50 / 100.0 * 2.0) + 0.5
            estimated_time = min(estimated_time, 10.0)
            
            self.get_logger().info(f"  预计运动时间: {estimated_time:.1f}秒")
            time.sleep(estimated_time)
            
            # 验证是否到达
            current = button_actions.get_current_joints()
            error = np.array(current) - np.array(home_joints)
            max_error_deg = np.max(np.abs(error)) * 180.0 / PI
            
            if max_error_deg < 5.0:
                self.get_logger().info(f"✓ 已到达HOME位姿（误差 {max_error_deg:.2f}°）")
                return True
            else:
                self.get_logger().warn(f"⚠️  HOME位姿误差较大（{max_error_deg:.2f}°），但继续运行")
                return True
                
        except Exception as e:
            self.get_logger().error(f"移动到HOME位姿异常: {e}")
            import traceback
            self.get_logger().debug(traceback.format_exc())
            return False
    
    def _enable_arm_robust(self, piper: C_PiperInterface_V2, timeout: float = 10.0) -> bool:
        """
        健壮的机械臂使能方法（不会exit）
        
        参数:
            piper: Piper SDK接口
            timeout: 超时时间（秒）
        
        返回:
            True: 使能成功
            False: 使能失败
        """
        import time
        start_time = time.time()
        retry_count = 0
        
        while time.time() - start_time < timeout:
            retry_count += 1
            
            # 发送使能指令
            piper.EnableArm(7)
            time.sleep(0.5)  # 等待使能生效
            
            # 检查使能状态
            try:
                status = piper.GetArmLowSpdInfoMsgs()
                enable_flag = (
                    status.motor_1.foc_status.driver_enable_status and
                    status.motor_2.foc_status.driver_enable_status and
                    status.motor_3.foc_status.driver_enable_status and
                    status.motor_4.foc_status.driver_enable_status and
                    status.motor_5.foc_status.driver_enable_status and
                    status.motor_6.foc_status.driver_enable_status
                )
                
                if enable_flag:
                    self.get_logger().info(f"✓ 机械臂使能成功（尝试 {retry_count} 次）")
                    # 闭合夹爪
                    piper.GripperCtrl(0, 1000, 0x01, 0)
                    time.sleep(0.3)
                    return True
                else:
                    self.get_logger().debug(f"使能状态: False (尝试 {retry_count}/{int(timeout)} 秒)")
                    
            except Exception as e:
                self.get_logger().debug(f"读取状态失败: {e}")
            
            time.sleep(0.5)
        
        self.get_logger().error(f"✗ 机械臂使能超时（{timeout}秒内尝试 {retry_count} 次）")
        return False
    
    def _initialize_hardware(self) -> bool:
        """初始化 Piper SDK / PiperArm, 同步至 button_actions."""
        try:
            self.get_logger().info("初始化 Piper SDK ...")
            piper = C_PiperInterface_V2("can0")
            piper.ConnectPort()
            
            # 使用更健壮的使能方法（不会exit）
            self.get_logger().info("正在使能机械臂（最多等待10秒）...")
            enable_success = self._enable_arm_robust(piper, timeout=10.0)
            if not enable_success:
                self.get_logger().error("机械臂使能失败！请检查：")
                self.get_logger().error("  1. 机械臂是否上电")
                self.get_logger().error("  2. 急停按钮是否释放")
                self.get_logger().error("  3. CAN0接口是否正常")
                self.get_logger().error("  4. 电源电压是否充足")
                return False
            
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
        
        # 移动到HOME观察位姿（如果启用）
        if button_actions.USE_HOME_POSITION and hasattr(button_actions, 'HOME_JOINTS'):
            self.get_logger().info("正在移动到HOME观察位姿...")
            try:
                home_success = self._move_to_home_position()
                if home_success:
                    self.get_logger().info("✓ 已到达HOME位姿")
                    self.workflow_state = "HOME"
                else:
                    self.get_logger().warn("⚠️  移动到HOME位姿失败，将从当前位置开始")
            except Exception as e:
                self.get_logger().error(f"移动到HOME位姿异常: {e}")
                return False
        else:
            self.get_logger().info("跳过HOME位姿（USE_HOME_POSITION=False）")
            self.workflow_state = "HOME"
        
        # 启动用户交互线程
        self._user_input_thread = threading.Thread(target=self._user_interaction_loop, daemon=True)
        self._user_input_thread.start()
        
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
        
        # 🔧 使用方案1（optical直接）- 用户确认此方案正确
        button_base = button_base_v1
        self.get_logger().info("✓ 使用方案1 (optical直接转换)")
        
        # 验证高度合理性（按钮应该在10cm~60cm之间）
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
    
    def _compute_approach_pose_base(self, button_position: np.ndarray, 
                                    normal_vector: np.ndarray, 
                                    approach_distance: float = 0.30) -> np.ndarray:
        """
        计算接近位姿（基座坐标系）- 正确版本
        
        参数:
            button_position: 按钮位置 [x, y, z] (基座系)
            normal_vector: 面板法向量 [nx, ny, nz] (基座系，垂直于面板指向外侧)
            approach_distance: 接近距离（米，默认30cm，**正值表示沿法向量反方向后退**）
        
        返回:
            4x4齐次变换矩阵（基座系下的接近位姿）
            
        关键逻辑：
            - 法向量指向面板外侧
            - 接近点 = 按钮位置 - 法向量 * distance（**沿法向量反方向后退**）
            - Gripper的+Z轴 = -法向量方向 → +Z 指向面板，用于按压/插入
            - 这样沿Gripper +Z前进 = 朝向面板，沿 -Z 后退 = 离开面板
        """
        # 归一化法向量
        normal = normal_vector / np.linalg.norm(normal_vector)
        
        # 🔧 关键修复：接近点应该沿法向量**反方向**后退
        # 接近点 = 按钮中心 - 法向量 * 距离（远离面板）
        approach_point = button_position - normal * approach_distance
        
        self.get_logger().info(f"  按钮位置: ({button_position[0]:.3f}, {button_position[1]:.3f}, {button_position[2]:.3f})")
        self.get_logger().info(f"  法向量: ({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})")
        self.get_logger().info(f"  后退距离: {approach_distance*100:.1f}cm")
        self.get_logger().info(f"  接近点: ({approach_point[0]:.3f}, {approach_point[1]:.3f}, {approach_point[2]:.3f})")

        rotation_matrix = self._build_orientation_from_normal(normal, gripper_z_toward_panel=True)
        T_approach = np.eye(4)
        T_approach[:3, :3] = rotation_matrix
        T_approach[:3, 3] = approach_point
        
        z_axis = rotation_matrix[:, 2]
        # 验证对齐性：Z轴与法向量的夹角应该≈0°（平行）
        dot_zn = np.dot(z_axis, -normal)
        angle_rad = np.arccos(np.clip(abs(dot_zn), 0, 1))
        angle_deg = np.degrees(angle_rad)
        
        # 判断标准：夹角 < 0.01弧度（约0.57°）认为已对齐
        is_aligned = angle_rad < 0.01
        
        self.get_logger().info(
            f"  ✓ 姿态验证: Gripper_Z ∥ (-Normal) | 夹角={angle_deg:.3f}° "
            f"({'✓已对齐' if is_aligned else '✗未对齐，需校正'})"
        )
        
        return T_approach

    def _build_orientation_from_normal(
        self,
        normal_vector: np.ndarray,
        gripper_z_toward_panel: bool = True,
    ) -> np.ndarray:
        """根据面板法向量构造右手坐标系。

        Args:
            normal_vector: 面板法向量 (基座系，指向面板外侧)
            gripper_z_toward_panel: True 时末端 +Z 指向面板（用于按压/插入）

        Returns:
            3x3 旋转矩阵
        """

        normal = normal_vector / np.linalg.norm(normal_vector)
        z_axis = -normal if gripper_z_toward_panel else normal

        # 选择一个与Z轴不平行的参考向量构造X轴
        reference = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(reference, z_axis)) > 0.95:
            reference = np.array([0.0, 1.0, 0.0])

        x_axis = reference - np.dot(reference, z_axis) * z_axis
        norm_x = np.linalg.norm(x_axis)
        if norm_x < 1e-6:
            reference = np.array([1.0, 0.0, 0.0])
            x_axis = reference - np.dot(reference, z_axis) * z_axis
            norm_x = np.linalg.norm(x_axis)
        x_axis /= norm_x

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rotation = np.eye(3)
        rotation[:, 0] = x_axis
        rotation[:, 1] = y_axis
        rotation[:, 2] = z_axis
        return rotation

    def _publish_target_marker(self, target_xyz) -> None:
        marker = Marker()
        marker.header.frame_id = "base_link"
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
