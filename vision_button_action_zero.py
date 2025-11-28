#!/usr/bin/env python3
"""
视觉按钮操作整合器
整合视觉检测 + button_actions.py 的动作执行
"""
import rospy
import sys
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker

# 导入必需的库
from piper_sdk import *
from piper_arm import PiperArm
import math

# 导入 button_actions.py 的功能（但不导入需要初始化的对象）
import button_actions
from button_actions import (
    # 动作函数
    action_toggle, action_plugin, action_push, action_knob,
    
    # 常量
    PI, factor
)

# 初始化自己的 piper 和 piper_arm 对象
piper = None
piper_arm = None
move_group = None

# ========================================
# 全局变量
# ========================================
receive_button_center = False
button_center = None  # 相机坐标系下的按钮位置 [x, y, z]
button_type = None    # 按钮类型字符串 'toggle', 'plugin', 'push', 'knob'


# ========================================
# 初始化函数
# ========================================
def initialize_hardware():
    """
    初始化机械臂硬件和 MoveIt（如果需要）
    """
    global piper, piper_arm, move_group
    
    print("\n" + "="*70)
    print("初始化硬件")
    print("="*70)
    
    # 1. 初始化机械臂SDK
    print("\n[1/3] 初始化 Piper SDK...")
    try:
        piper = C_PiperInterface_V2("can0")
        piper.ConnectPort()
        piper.EnableArm(7)  # 使能所有关节+夹爪
        
        # 使能所有关节
        from utils.utils_piper import enable_fun
        enable_fun(piper=piper)
        
        print("  ✓ Piper SDK 初始化成功")
    except Exception as e:
        print(f"  ✗ Piper SDK 初始化失败: {e}")
        return False
    
    # 2. 初始化运动学模块
    print("\n[2/3] 初始化 PiperArm...")
    try:
        piper_arm = PiperArm()
        print("  ✓ PiperArm 初始化成功")
    except Exception as e:
        print(f"  ✗ PiperArm 初始化失败: {e}")
        return False
    
    # 3. 初始化 MoveIt（如果启用）
    if button_actions.USE_MOVEIT and button_actions.MOVEIT_AVAILABLE:
        print("\n[3/3] 初始化 MoveIt...")
        try:
            import moveit_commander
            moveit_commander.roscpp_initialize(sys.argv)
            
            # 创建 MoveGroup
            group_name = "arm"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            
            # 配置规划参数
            move_group.set_planning_time(5.0)
            move_group.set_num_planning_attempts(10)
            move_group.set_max_velocity_scaling_factor(1.0)
            move_group.set_max_acceleration_scaling_factor(1.0)
            move_group.set_pose_reference_frame("arm_base")
            
            print(f"  ✓ MoveIt 初始化成功")
            print(f"    规划组: {group_name}")
            print(f"    末端: {move_group.get_end_effector_link()}")
            
            # 更新 button_actions 的 move_group
            button_actions.move_group = move_group
        except Exception as e:
            print(f"  ✗ MoveIt 初始化失败: {e}")
            print("  将使用 SDK 直接控制模式")
            move_group = None
            button_actions.move_group = None
    else:
        print("\n[3/3] 跳过 MoveIt (SDK 模式)")
    
    # 4. 更新 button_actions 的全局对象
    button_actions.piper = piper
    button_actions.piper_arm = piper_arm
    
    print("\n" + "="*70)
    print("✓ 硬件初始化完成")
    print("="*70 + "\n")
    
    return True

# ========================================
# 订阅回调函数
# ========================================
def object_point_callback(msg):
    """
    接收按钮的 3D 位置（相机坐标系）
    
    话题: /object_point
    类型: geometry_msgs/PointStamped
    """
    global receive_button_center, button_center
    
    if np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z):
        return
    
    button_center = [msg.point.x, msg.point.y, msg.point.z]
    receive_button_center = True
    
    print(f"\n{'='*70}")
    print(f"接收到按钮位置 (相机坐标系):")
    print(f"  X: {button_center[0]:.4f} m")
    print(f"  Y: {button_center[1]:.4f} m")
    print(f"  Z: {button_center[2]:.4f} m")
    print(f"{'='*70}\n")


def button_type_callback(msg):
    """
    接收按钮类型
    
    话题: /button_type
    类型: std_msgs/String
    """
    global button_type
    
    button_type = msg.data.lower()  # 转为小写: 'toggle', 'plugin', 'push', 'knob'
    
    print(f"接收到按钮类型: {button_type}")


# ========================================
# 坐标转换函数
# ========================================
def transform_camera_to_base(button_center_camera, current_joints):
    """
    将相机坐标系的按钮位置转换到基座坐标系
    
    Args:
        button_center_camera: [x, y, z] 相机坐标系
        current_joints: [j1, j2, j3, j4, j5, j6] 当前关节角度
    
    Returns:
        button_base: [x, y, z, 1] 基座坐标系（齐次坐标）
    """
    # 使用 piper_arm 的正运动学
    from utils.utils_math import quaternion_to_rotation_matrix
    
    # 末端到基座的变换
    base_T_link6 = piper_arm.forward_kinematics(current_joints)
    
    # 相机到末端的变换（从标定数据）
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera
    
    # 按钮在相机坐标系的齐次坐标
    button_cam_homogeneous = np.array([
        button_center_camera[0],
        button_center_camera[1],
        button_center_camera[2],
        1.0
    ])
    
    # 转换到基座坐标系
    button_base = base_T_link6 @ link6_T_cam @ button_cam_homogeneous
    
    return button_base


# ========================================
# 获取当前关节角度
# ========================================
def get_current_joints():
    """
    从机械臂获取当前关节角度
    
    Returns:
        joints: [j1, j2, j3, j4, j5, j6] 弧度
    """
    msg = piper.GetArmJointMsgs()
    
    theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
    theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
    theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
    theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
    theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
    theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0
    
    return [theta1, theta2, theta3, theta4, theta5, theta6]


# ========================================
# 应用夹爪TCP偏移（夹爪自身坐标系）
# ========================================
def apply_tcp_offset(button_base, current_joints, tcp_offset_local):
    """
    应用夹爪自身坐标系的TCP偏移
    
    Args:
        button_base: [x, y, z, 1] 基座坐标系的按钮位置（齐次坐标）
        current_joints: [j1, j2, j3, j4, j5, j6] 当前关节角度
        tcp_offset_local: [dx, dy, dz] 夹爪坐标系的偏移量
    
    Returns:
        target_base: [x, y, z, 1] 补偿后的目标位置（基座坐标系）
    
    说明:
        如果 tcp_offset_local = [0, 0, 0.07]，表示夹爪接触点在夹爪Z轴正方向7cm处
        函数会根据夹爪当前姿态，将这个偏移量转换到基座坐标系并应用补偿
    """
    # 获取末端姿态（link6在基座坐标系的变换）
    base_T_link6 = piper_arm.forward_kinematics(current_joints)
    
    # 提取旋转矩阵（描述夹爪的姿态）
    R_base_link6 = base_T_link6[:3, :3]
    
    # 将夹爪坐标系的偏移量转换到基座坐标系
    # offset_base = R * offset_local
    offset_base = R_base_link6 @ np.array(tcp_offset_local)
    
    # 应用偏移：目标位置 = 按钮位置 - 偏移量
    # （因为我们希望夹爪的TCP接触点到达按钮位置）
    target_base = button_base.copy()
    target_base[:3] = button_base[:3] - offset_base
    
    return target_base


# ========================================
# 动作执行函数
# ========================================
def execute_button_action(button_center_camera, button_type_str):
    """
    执行按钮操作
    
    Args:
        button_center_camera: [x, y, z] 相机坐标系
        button_type_str: 按钮类型 'toggle'/'plugin'/'push'/'knob'
    
    Returns:
        success: True/False
    """
    print("\n" + "="*70)
    print("开始执行按钮操作")
    print("="*70)
    
    # 1. 获取当前关节角度
    print("\n[步骤 1/4] 获取当前关节角度...")
    current_joints = get_current_joints()
    print(f"  当前关节角度 (度): {np.array(current_joints) * 180 / PI}")
    
    # 2. 坐标转换
    print("\n[步骤 2/4] 坐标转换: 相机坐标系 → 基座坐标系...")
    button_base = transform_camera_to_base(button_center_camera, current_joints)
    print(f"  相机坐标系: ({button_center_camera[0]:.4f}, {button_center_camera[1]:.4f}, {button_center_camera[2]:.4f})")
    print(f"  基座坐标系: ({button_base[0]:.4f}, {button_base[1]:.4f}, {button_base[2]:.4f})")
    
    # 发布可视化标记（基座坐标系）
    pub_target_marker = rospy.Publisher('/target_button_base', Marker, queue_size=10)
    from utils.utils_ros import publish_sphere_marker
    publish_sphere_marker(
        pub_target_marker, 
        button_base[:3], 
        frame_id="arm_base", 
        color=(0.0, 1.0, 0.0, 1.0),
        radius=0.02
    )
    
    # 3. 应用TCP偏移补偿 (夹爪自身坐标系)
    print("\n[步骤 3/4] 应用TCP偏移补偿...")
    
    # TCP偏移: 夹爪实际接触点相对于link6在夹爪自身坐标系的偏移
    # 如果夹爪接触点在夹爪Z轴正方向7cm处，设置为 [0, 0, 0.07]
    # 如果在X轴负方向6cm处，设置为 [-0.06, 0, 0]
    # TCP_OFFSET_LOCAL = np.array([-0.035, 0.01, 0.058])  # [dx, dy, dz] 单位: 米 (夹爪坐标系)
    # TCP_OFFSET_LOCAL = np.array([-0.0135, +0.0088, 0.058])  # [dx, dy, dz] 单位: 米 (夹爪坐标系)
    # TCP_OFFSET_LOCAL = np.array([-0.018, 0.007, 0.063]) 
    TCP_OFFSET_LOCAL = np.array([-0.05, 0.013, 0.03]) 

    # 根据当前末端姿态，将夹爪坐标系的偏移转换为基座坐标系的补偿
    target_base = apply_tcp_offset(button_base, current_joints, TCP_OFFSET_LOCAL)
    
    print(f"  按钮位置 (基座系): ({button_base[0]:.4f}, {button_base[1]:.4f}, {button_base[2]:.4f})")
    print(f"  TCP偏移 (夹爪系): ({TCP_OFFSET_LOCAL[0]:.4f}, {TCP_OFFSET_LOCAL[1]:.4f}, {TCP_OFFSET_LOCAL[2]:.4f})")
    print(f"  目标位置 (基座系): ({target_base[0]:.4f}, {target_base[1]:.4f}, {target_base[2]:.4f})")
    
    # 4. 更新 button_actions.py 的目标位置
    print("\n[步骤 4/5] 更新 button_actions 目标位置...")
    button_actions.TARGET_X = target_base[0]
    button_actions.TARGET_Y = target_base[1]
    button_actions.TARGET_Z = target_base[2]
    
    print(f"  ✓ 已更新 TARGET_X = {button_actions.TARGET_X:.4f}")
    print(f"  ✓ 已更新 TARGET_Y = {button_actions.TARGET_Y:.4f}")
    print(f"  ✓ 已更新 TARGET_Z = {button_actions.TARGET_Z:.4f}")
    
    # 5. 根据按钮类型调用对应的动作函数
    print(f"\n[步骤 5/5] 执行动作: {button_type_str.upper()}...")
    
    action_map = {
        'toggle': action_toggle,
        'plugin': action_plugin,
        'push': action_push,
        'knob': action_knob
    }
    
    if button_type_str not in action_map:
        print(f"\n✗ 错误: 未知的按钮类型 '{button_type_str}'")
        print(f"  支持的类型: {list(action_map.keys())}")
        return False
    
    # 调用动作函数
    try:
        success = action_map[button_type_str]()
        
        if success:
            print("\n" + "="*70)
            print("✓✓✓ 按钮操作执行成功！✓✓✓")
            print("="*70 + "\n")
        else:
            print("\n" + "="*70)
            print("✗✗✗ 按钮操作执行失败 ✗✗✗")
            print("="*70 + "\n")
        
        return success
    
    except Exception as e:
        print(f"\n✗ 动作执行过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False


# ========================================
# 主函数
# ========================================
def main():
    global receive_button_center, button_center, button_type
    
    print("\n" + "="*70)
    print("视觉按钮操作整合器")
    print("="*70)
    print("功能: 整合视觉检测 + button_actions.py 动作执行")
    print("订阅话题:")
    print("  - /object_point (按钮3D位置)")
    print("  - /button_type (按钮类型)")
    print("="*70 + "\n")
    
    # 初始化 ROS 节点
    rospy.init_node('vision_button_action_node', anonymous=True)
    
    # 初始化硬件
    if not initialize_hardware():
        rospy.logerr("硬件初始化失败，退出程序")
        return
    
    # 订阅话题
    rospy.Subscriber('/object_point', PointStamped, object_point_callback, queue_size=10)
    rospy.Subscriber('/button_type', String, button_type_callback, queue_size=10)
    
    print("✓ ROS 节点已初始化")
    print("✓ 等待接收按钮信息...\n")
    print("提示: 在 realsense_yolo_button_interactive 窗口中:")
    print("  1. 点击选择按钮")
    print("  2. 按 ENTER 确认\n")
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # 检查是否同时接收到位置和类型
        if receive_button_center and button_type is not None:
            print(f"\n{'='*70}")
            print(f"准备执行操作:")
            print(f"  按钮类型: {button_type}")
            print(f"  按钮位置: {button_center}")
            print(f"{'='*70}")
            
            # 执行动作
            success = execute_button_action(button_center, button_type)
            
            # 重置标志，等待下一次检测
            receive_button_center = False
            button_type = None
            button_center = None
            
            if success:
                print("\n等待下一个按钮...")
            else:
                print("\n操作失败，等待下一个按钮...")
        
        rate.sleep()
    
    print("\n程序退出")


if __name__ == "__main__":
    main()
