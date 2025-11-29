#!/usr/bin/env python3
"""
实时追踪面板法向量 (MoveIt2版本)
功能：让夹爪的Z轴始终垂直对齐于检测到的面板平面
"""
import sys
import time
import numpy as np
import pyrealsense2 as rs
import cv2
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
import time as time_module

from piper_sdk import C_PiperInterface_V2
from piper_arm import PiperArm
from utils.utils_plane import fit_plane_ransac
from utils.utils_math import quaternion_to_rotation_matrix

PI = np.pi

# ========================================
# 全局变量
# ========================================
piper = None
piper_arm = None
pipeline = None
ros_node = None
action_client = None
joint_state_publisher = None
ros2_executor = None
ros2_spin_thread = None


def initialize_hardware():
    """初始化机械臂和相机"""
    global piper, piper_arm, pipeline
    
    print("="*70)
    print("初始化硬件")
    print("="*70)
    
    # 1. 初始化机械臂
    print("\n[1/3] 初始化机械臂...")
    try:
        piper = C_PiperInterface_V2("can0")
        piper.ConnectPort()
        
        # 简单使能（不退出）
        print("  正在使能...")
        for _ in range(10):
            piper.EnableArm(7)
            time.sleep(0.5)
            status = piper.GetArmLowSpdInfoMsgs()
            enabled = (status.motor_1.foc_status.driver_enable_status and
                      status.motor_2.foc_status.driver_enable_status and
                      status.motor_3.foc_status.driver_enable_status and
                      status.motor_4.foc_status.driver_enable_status and
                      status.motor_5.foc_status.driver_enable_status and
                      status.motor_6.foc_status.driver_enable_status)
            if enabled:
                print("  ✓ 机械臂使能成功")
                break
        else:
            print("  ⚠️  部分关节可能未使能，但继续运行...")
        
        # 设置运动模式（高速追踪模式）
        piper.MotionCtrl_2(
            ctrl_mode=0x01,      # CAN控制
            move_mode=0x00,      # MOVEJ
            move_spd_rate_ctrl=50,  # 速度50%（从30%提高）
            is_mit_mode=0x00
        )
        
    except Exception as e:
        print(f"  ✗ 机械臂初始化失败: {e}")
        return False
    
    # 2. 初始化运动学
    print("\n[2/3] 初始化运动学...")
    try:
        piper_arm = PiperArm()
        print("  ✓ PiperArm 初始化成功")
    except Exception as e:
        print(f"  ✗ PiperArm 初始化失败: {e}")
        return False
    
    # 3. 初始化RealSense
    print("\n[3/3] 初始化RealSense相机...")
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        pipeline.start(config)
        
        # 预热相机
        for _ in range(10):
            pipeline.wait_for_frames()
        
        print("  ✓ RealSense 初始化成功")
    except Exception as e:
        print(f"  ✗ RealSense 初始化失败: {e}")
        return False
    
    print("\n✓✓✓ 所有硬件初始化完成 ✓✓✓\n")
    return True


def initialize_ros2() -> bool:
    """
    初始化ROS2节点和MoveIt2 action client
    
    返回:
        True: 成功, False: 失败
    """
    global ros_node, action_client, joint_state_publisher, ros2_executor, ros2_spin_thread
    
    try:
        import rclpy.executors
        import threading
        
        # 初始化ROS2
        if not rclpy.ok():
            rclpy.init()
        
        # 创建节点
        node_name = f'track_panel_normal_{int(time_module.time() * 1000)}'
        ros_node = Node(node_name)
        print(f"  ✓ 创建ROS2节点: {node_name}")
        
        # 创建MoveIt2 action client
        action_client = ActionClient(ros_node, MoveGroupAction, '/move_action')
        print("  ✓ Action Client已创建")
        
        # 创建joint_states发布器
        joint_state_publisher = ros_node.create_publisher(JointState, '/joint_states', 10)
        print("  ✓ joint_states发布器已启动")
        
        # 启动后台spin线程
        ros2_executor = rclpy.executors.SingleThreadedExecutor()
        ros2_executor.add_node(ros_node)
        ros2_spin_thread = threading.Thread(target=ros2_executor.spin, daemon=True)
        ros2_spin_thread.start()
        print("  ✓ ROS2 spin线程已启动")
        
        # 等待action server
        print("  ⏳ 等待MoveIt2 action server...")
        timeout = 10.0
        start_time = time_module.time()
        
        while not action_client.server_is_ready():
            time_module.sleep(0.2)
            elapsed = time_module.time() - start_time
            if elapsed > timeout:
                print("  ⚠️  MoveIt2 action server未启动，请先运行start_moveit2.sh")
                return False
        
        # ROS2 Foxy需要额外等待时间
        print("  ✓ Action server已就绪，等待服务完全启动...")
        time_module.sleep(2.0)
        
        print("✅ MoveIt2连接成功")
        return True
        
    except Exception as e:
        print(f"  ⚠️  ROS2初始化失败: {e}")
        return False


def get_current_joints() -> np.ndarray:
    """读取当前关节角度（弧度）"""
    msg = piper.GetArmJointMsgs()
    return np.array([
        msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_6 * 1e-3 * PI / 180.0
    ])


def detect_panel_normal(depth_frame, color_frame, 
                       center_x: int = 320, center_y: int = 240,
                       roi_size: int = 60) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    检测面板法向量和中心点位置（相机坐标系）
    
    参数:
        depth_frame: 深度帧
        color_frame: 彩色帧
        center_x, center_y: 检测中心点（屏幕像素坐标）
        roi_size: ROI半径（像素）
    
    返回:
        (法向量, 中心点3D位置) 或 None
    """
    # 获取深度内参
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    
    # 定义ROI
    x_min = max(0, center_x - roi_size)
    x_max = min(depth_intrin.width, center_x + roi_size)
    y_min = max(0, center_y - roi_size)
    y_max = min(depth_intrin.height, center_y + roi_size)
    
    # 提取ROI内的3D点云（稀疏采样加速）
    points_3d = []
    for y in range(y_min, y_max, 4):  # 每隔4像素采样，提高速度
        for x in range(x_min, x_max, 4):
            depth_value = depth_frame.get_distance(x, y)
            if 0.1 < depth_value < 2.0:  # 有效深度范围
                point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth_value)
                points_3d.append(point_3d)
    
    if len(points_3d) < 30:  # 降低最小点数要求
        return None
    
    points_3d = np.array(points_3d)
    
    # RANSAC拟合平面（减少迭代次数加速）
    normal_camera, d, inliers = fit_plane_ransac(
        points_3d,
        distance_threshold=0.010,  # 放宽到10mm容差
        max_iterations=200  # 从500降到200次迭代
    )
    
    if normal_camera is None:
        return None
    
    # 确保法向量指向相机（Z分量为负）
    if normal_camera[2] > 0:
        normal_camera = -normal_camera
    
    # 计算ROI中心的3D点位置
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    center_depth = depth_frame.get_distance(center_x, center_y)
    
    if center_depth < 0.1 or center_depth > 2.0:
        return None
    
    center_point_3d = np.array(rs.rs2_deproject_pixel_to_point(
        depth_intrin, [center_x, center_y], center_depth
    ))
    
    return normal_camera, center_point_3d


def transform_normal_to_base(normal_camera: np.ndarray, current_joints: np.ndarray) -> np.ndarray:
    """
    将法向量从相机坐标系转换到基座标系
    
    参数:
        normal_camera: 法向量（相机系）
        current_joints: 当前关节角度
    
    返回:
        法向量（基座系）
    """
    # 基座到link6的变换
    base_T_link6 = piper_arm.forward_kinematics(current_joints)
    
    # link6到相机的变换
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera
    
    # 组合变换
    base_T_cam = base_T_link6 @ link6_T_cam
    
    # 只转换旋转部分（法向量是方向）
    R_base_cam = base_T_cam[:3, :3]
    normal_base = R_base_cam @ normal_camera
    normal_base = normal_base / np.linalg.norm(normal_base)
    
    return normal_base


def compute_aligned_pose_with_distance(current_joints: np.ndarray, 
                                       normal_base: np.ndarray,
                                       panel_center_base: np.ndarray,
                                       distance: float = 0.30) -> Optional[np.ndarray]:
    """
    计算对齐法向量的末端位姿，保持指定距离
    
    策略：优先保持当前位置，只调整姿态对齐法向量
    
    参数:
        current_joints: 当前关节角度
        normal_base: 面板法向量（基座系，指向外侧）
        panel_center_base: 面板中心点（基座系）
        distance: 与面板的距离（米，默认30cm）
    
    返回:
        目标关节角度 或 None
    """
    # 获取当前末端位置
    current_T = piper_arm.forward_kinematics(current_joints)
    current_position = current_T[:3, 3]
    
    # 策略1：只调整姿态，保持当前位置（避免大幅移动导致超限）
    target_position = current_position
    
    # 构造目标旋转矩阵：Z轴对齐法向量（指向面板）
    z_axis = normal_base / np.linalg.norm(normal_base)
    
    # 选择一个"上"方向
    world_up = np.array([0, 0, 1])
    if abs(np.dot(z_axis, world_up)) > 0.95:
        world_up = np.array([1, 0, 0])
    
    x_axis = np.cross(world_up, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    
    y_axis = np.cross(z_axis, x_axis)
    
    # 目标变换矩阵
    target_T = np.eye(4)
    target_T[:3, 0] = x_axis
    target_T[:3, 1] = y_axis
    target_T[:3, 2] = z_axis
    target_T[:3, 3] = target_position
    
    # IK求解（使用refined版本，更稳定）
    try:
        target_joints = piper_arm.inverse_kinematics_refined(target_T, initial_guess=current_joints)
        if target_joints is None:
            return None
        return target_joints
    except Exception as e:
        # 如果refined失败，尝试基础版本
        try:
            target_joints = piper_arm.inverse_kinematics(target_T)
            if target_joints is None:
                return None
            return target_joints
        except:
            return None


def compute_aligned_pose(current_joints: np.ndarray, 
                        normal_base: np.ndarray,
                        offset_distance: float = 0.0) -> Optional[np.ndarray]:
    """
    计算对齐法向量的末端位姿（旧版本，保留兼容性）
    
    参数:
        current_joints: 当前关节角度
        normal_base: 面板法向量（基座系）
        offset_distance: 偏移距离（米，正值远离面板）
    
    返回:
        目标关节角度 或 None
    """
    # 获取当前末端位置
    current_T = piper_arm.forward_kinematics(current_joints)
    current_position = current_T[:3, 3]
    
    # 计算目标位置（加上偏移）
    target_position = current_position + normal_base * offset_distance
    
    # 构造目标旋转矩阵：Z轴对齐法向量
    z_axis = normal_base / np.linalg.norm(normal_base)
    
    # 选择一个"上"方向
    world_up = np.array([0, 0, 1])
    if abs(np.dot(z_axis, world_up)) > 0.95:
        world_up = np.array([1, 0, 0])
    
    x_axis = np.cross(world_up, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    
    y_axis = np.cross(z_axis, x_axis)
    
    # 目标变换矩阵
    target_T = np.eye(4)
    target_T[:3, 0] = x_axis
    target_T[:3, 1] = y_axis
    target_T[:3, 2] = z_axis
    target_T[:3, 3] = target_position
    
    # IK求解（使用refined版本，更稳定）
    try:
        target_joints = piper_arm.inverse_kinematics_refined(target_T, initial_guess=current_joints)
        if target_joints is None:
            return None
        return target_joints
    except Exception as e:
        # 如果refined失败，尝试基础版本
        try:
            target_joints = piper_arm.inverse_kinematics(target_T)
            if target_joints is None:
                return None
            return target_joints
        except:
            return None


def move_to_joints_moveit(target_joints: np.ndarray) -> bool:
    """
    使用MoveIt2移动到目标关节角度
    
    参数:
        target_joints: 目标关节角度（弧度）
    
    返回:
        True: 成功, False: 失败
    """
    global action_client, joint_state_publisher, ros_node
    
    if action_client is None:
        print("  ⚠️  MoveIt2 action client未初始化")
        return False
    
    # 检查关节限位
    joint_limits_rad = [
        (-PI, PI),          # J1: ±180°
        (0, PI),            # J2: 0~180°
        (-PI, 0),           # J3: -180~0°
        (-PI, PI),          # J4: ±180°
        (-70*PI/180, 70*PI/180),  # J5: ±70°
        (-PI, PI)           # J6: ±180°
    ]
    
    for i, (joint_val, (min_val, max_val)) in enumerate(zip(target_joints, joint_limits_rad)):
        if joint_val < min_val or joint_val > max_val:
            print(f"  ⚠️  关节{i+1}超限: {joint_val*180/PI:.1f}° (范围: {min_val*180/PI:.1f}~{max_val*180/PI:.1f}°)")
            return False
    
    # 创建MoveGroup.action goal消息
    goal_msg = MoveGroupAction.Goal()
    goal_msg.request = MotionPlanRequest()
    goal_msg.request.group_name = 'piper'
    goal_msg.request.num_planning_attempts = 5
    goal_msg.request.allowed_planning_time = 2.0
    goal_msg.request.max_velocity_scaling_factor = 0.5
    goal_msg.request.max_acceleration_scaling_factor = 0.5
    
    # 设置关节约束
    goal_msg.request.goal_constraints.append(Constraints())
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    
    for i, (name, position) in enumerate(zip(joint_names, target_joints)):
        constraint = JointConstraint()
        constraint.joint_name = name
        constraint.position = float(position)
        constraint.tolerance_above = 0.01
        constraint.tolerance_below = 0.01
        constraint.weight = 1.0
        goal_msg.request.goal_constraints[0].joint_constraints.append(constraint)
    
    # 发送目标（非阻塞）
    try:
        action_client.send_goal_async(goal_msg)
        
        # 同时发布joint_states供其他节点使用
        if joint_state_publisher:
            js_msg = JointState()
            js_msg.header.stamp = ros_node.get_clock().now().to_msg()
            js_msg.name = joint_names
            js_msg.position = target_joints.tolist()
            joint_state_publisher.publish(js_msg)
        
        return True
    except Exception as e:
        print(f"  ⚠️  MoveIt2发送目标失败: {e}")
        return False


def main():
    """主循环：实时追踪面板法向量"""
    print("\n" + "="*70)
    print("实时追踪面板法向量 (优化版)")
    print("="*70)
    print("功能：夹爪Z轴始终垂直对齐于面板")
    print("操作：")
    print("  - 移动相机对准面板")
    print("  - 按 'q' 退出")
    print("  - 按 's' 暂停/继续追踪")
    print("  - 按 'd' 切换显示模式（提速）")
    print("="*70 + "\n")
    
    # 初始化硬件
    if not initialize_hardware():
        print("硬件初始化失败，退出")
        return
    
    # 初始化ROS2和MoveIt2
    if not initialize_ros2():
        print("ROS2初始化失败，退出")
        return
    
    # 创建深度对齐对象
    align = rs.align(rs.stream.color)
    
    # 状态变量
    tracking_enabled = True
    display_enabled = True  # 显示开关
    frame_count = 0
    last_update_time = time.time()
    
    print("\n开始追踪...\n")
    print("💡 提示: 按 'd' 关闭显示可以提高追踪速度到 15Hz+\n")
    
    try:
        while True:
            frame_count += 1
            
            # 获取对齐的帧
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # 转换为numpy数组（仅在显示时）
            if display_enabled:
                color_image = np.asanyarray(color_frame.get_data())
                
                # 在图像中心画十字
                center_x, center_y = 320, 240
                cv2.line(color_image, (center_x - 30, center_y), (center_x + 30, center_y), (0, 255, 0), 2)
                cv2.line(color_image, (center_x, center_y - 30), (center_x, center_y + 30), (0, 255, 0), 2)
                cv2.circle(color_image, (center_x, center_y), 60, (255, 0, 0), 2)
            else:
                color_image = None
                center_x, center_y = 320, 240
            
            # 每2帧更新一次（提高响应速度）
            if tracking_enabled and frame_count % 2 == 0:
                # 检测法向量和面板中心点
                detection_result = detect_panel_normal(depth_frame, color_frame, center_x, center_y)
                
                if detection_result is not None:
                    normal_camera, panel_center_camera = detection_result
                    
                    # 读取当前关节角度
                    current_joints = get_current_joints()
                    
                    # 转换法向量到基座标系
                    normal_base = transform_normal_to_base(normal_camera, current_joints)
                    
                    # 转换面板中心点到基座标系
                    base_T_link6 = piper_arm.forward_kinematics(current_joints)
                    link6_T_cam = np.eye(4)
                    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
                    link6_T_cam[:3, 3] = piper_arm.link6_t_camera
                    base_T_cam = base_T_link6 @ link6_T_cam
                    
                    panel_center_camera_h = np.array([panel_center_camera[0], panel_center_camera[1], panel_center_camera[2], 1.0])
                    panel_center_base = base_T_cam @ panel_center_camera_h
                    panel_center_base = panel_center_base[:3]
                    
                    # 🔧 简化策略：只调整姿态对齐法向量，不改变位置
                    # 这样可以避免IK求解超限问题
                    target_joints = compute_aligned_pose(
                        current_joints, normal_base, offset_distance=0.0
                    )
                    
                    if target_joints is not None:
                        # 使用MoveIt2移动机械臂
                        success = move_to_joints_moveit(target_joints)
                        if not success:
                            # 如果超限，打印当前关节角度
                            print(f"  当前关节角(度): {np.array(current_joints)*180/PI}")
                            continue
                        
                        # 显示信息
                        now = time.time()
                        dt = now - last_update_time
                        last_update_time = now
                        
                        # 计算当前末端位置与面板中心的距离
                        current_T_check = piper_arm.forward_kinematics(get_current_joints())
                        current_pos = current_T_check[:3, 3]
                        actual_distance = np.linalg.norm(current_pos - panel_center_base)
                        
                        # 计算姿态对齐程度（Z轴与法向量的夹角）
                        current_z_axis = current_T_check[:3, 2]
                        alignment_angle = np.arccos(np.clip(np.dot(current_z_axis, normal_base), -1, 1)) * 180 / PI
                        
                        print(f"[追踪] 法向量: ({normal_base[0]:+.3f}, {normal_base[1]:+.3f}, {normal_base[2]:+.3f}) | "
                              f"距离: {actual_distance*100:.1f}cm | 对齐角度: {alignment_angle:.1f}° | 更新: {dt:.2f}s")
                        
                        # 在图像上显示状态
                        if display_enabled:
                            cv2.putText(color_image, "Tracking: ON", (10, 30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            cv2.putText(color_image, f"Normal: ({normal_base[0]:.2f}, {normal_base[1]:.2f}, {normal_base[2]:.2f})", 
                                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.putText(color_image, f"Distance: {actual_distance*100:.1f}cm (target: 30cm)", 
                                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    else:
                        if display_enabled:
                            cv2.putText(color_image, "IK Failed", (10, 30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    if display_enabled:
                        cv2.putText(color_image, "No Panel Detected", (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                if display_enabled:
                    cv2.putText(color_image, "Tracking: OFF (Press 's')", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 显示图像（仅在显示模式下）
            if display_enabled and color_image is not None:
                cv2.imshow('Panel Normal Tracking', color_image)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n退出追踪...")
                break
            elif key == ord('s'):
                tracking_enabled = not tracking_enabled
                status = "ON" if tracking_enabled else "OFF"
                print(f"\n追踪状态: {status}")
            elif key == ord('d'):
                display_enabled = not display_enabled
                status = "ON" if display_enabled else "OFF (提速模式)"
                print(f"\n显示状态: {status}")
                if not display_enabled:
                    cv2.destroyAllWindows()
    
    except KeyboardInterrupt:
        print("\n\n程序被中断")
    
    finally:
        # 清理资源
        if pipeline:
            pipeline.stop()
        cv2.destroyAllWindows()
        
        # 关闭ROS2
        if ros2_executor:
            ros2_executor.shutdown()
        if ros_node:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        print("\n程序结束")


if __name__ == "__main__":
    main()
