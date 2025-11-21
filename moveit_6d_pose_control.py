#!/usr/bin/env python3
"""
MoveIt 6D位姿控制脚本
功能：
1. 订阅6D目标位姿 (XYZ + Roll/Pitch/Yaw欧拉角)
   - 使用Float64MultiArray: [x, y, z, roll, pitch, yaw]
2. 使用MoveIt RRTconnect算法规划轨迹
3. 三个频率控制：
   - 轨迹发布到RViz的频率
   - SDK执行轨迹的频率
   - 执行命令发布的频率
4. RViz可视化末端执行器轨迹（带尾迹）
"""

import rospy
import sys
import time
import numpy as np
from math import pi

# Piper SDK
from piper_sdk import C_PiperInterface_V2
from piper_arm import PiperArm

# MoveIt
import moveit_commander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import DisplayTrajectory

# ROS消息类型
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler

# ========================================
# 用户配置参数
# ========================================

# === 频率控制 ===
RVIZ_PUBLISH_RATE = 10          # 轨迹发布到RViz的频率 (Hz)
SDK_EXECUTE_RATE = 50           # SDK执行轨迹的频率 (Hz) - 推荐50Hz
COMMAND_PUBLISH_RATE = 100      # 执行命令发布的频率 (Hz)
TRAJECTORY_SAMPLE_POINTS = 20   # 轨迹采样点数

# === MoveIt 规划器配置 ===
PLANNER_ID = "RRTconnect"       # 规划算法: RRTconnect
PLANNING_TIME = 10.0            # 规划时间限制 (秒) - 增加到10秒
PLANNING_ATTEMPTS = 20          # 规划尝试次数 - 增加到20次
VELOCITY_SCALING = 0.3          # 速度缩放 (0.0-1.0) - 降低速度增加成功率
ACCELERATION_SCALING = 0.3      # 加速度缩放 (0.0-1.0)
GOAL_POSITION_TOLERANCE = 0.01  # 位置容差 (m)
GOAL_ORIENTATION_TOLERANCE = 0.1  # 姿态容差 (rad)

# === 其他配置 ===
TOPIC_TARGET_POSE = "/target_6d_pose"     # 订阅的目标位姿话题 (Float64MultiArray: [x, y, z, roll, pitch, yaw])
TOPIC_TRAJECTORY = "/move_group/display_planned_path"  # MoveIt轨迹可视化
TOPIC_EE_PATH = "/end_effector_path"      # 末端执行器路径（带尾迹）
TOPIC_EE_TRAIL = "/end_effector_trail"    # 末端执行器轨迹标记

# ========================================
# 全局变量
# ========================================
PI = pi
factor = 1000 * 180 / PI

piper = None
piper_arm = None
move_group = None

# 发布器
display_trajectory_publisher = None
ee_path_publisher = None
ee_trail_publisher = None

# 轨迹记录（用于显示尾迹）
ee_trail_points = []
MAX_TRAIL_POINTS = 100  # 最大尾迹点数


# ========================================
# 辅助函数
# ========================================

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数 (ZYX顺序)
    
    Args:
        roll: 绕X轴旋转 (rad)
        pitch: 绕Y轴旋转 (rad)
        yaw: 绕Z轴旋转 (rad)
    
    Returns:
        (x, y, z, w): 四元数
    """
    return quaternion_from_euler(roll, pitch, yaw, 'sxyz')


def create_pose_from_xyzrpy(x, y, z, roll, pitch, yaw):
    """
    从XYZ位置和RPY欧拉角创建Pose消息
    
    Args:
        x, y, z: 位置 (m)
        roll, pitch, yaw: 欧拉角 (rad)
    
    Returns:
        geometry_msgs/Pose
    """
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    
    # 转换欧拉角为四元数
    quat = euler_to_quaternion(roll, pitch, yaw)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    
    return pose


# ========================================
# SDK控制函数
# ========================================

def control_arm_sdk(joints_list, speed=50):
    """
    使用SDK直接控制机械臂
    
    Args:
        joints_list: 关节角度列表 [rad]
        speed: 运动速度 (0-100)
    """
    global piper
    
    joints_int = [int(joints_list[i] * factor) for i in range(6)]
    joints_int[4] = max(-70000, joints_int[4])
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joints_int)


def execute_trajectory_with_sdk(trajectory):
    """
    使用SDK执行MoveIt规划的轨迹
    
    Args:
        trajectory: MoveIt规划的轨迹 (RobotTrajectory)
    
    Returns:
        bool: 执行是否成功
    """
    global piper
    
    try:
        traj_points = trajectory.joint_trajectory.points
        
        if len(traj_points) == 0:
            rospy.logerr("轨迹为空")
            return False
        
        rospy.loginfo(f"开始执行轨迹 (总点数: {len(traj_points)})")
        
        # 采样轨迹点
        sample_indices = np.linspace(0, len(traj_points)-1, 
                                    min(TRAJECTORY_SAMPLE_POINTS, len(traj_points)), 
                                    dtype=int)
        sample_points = [traj_points[i] for i in sample_indices]
        
        rospy.loginfo(f"采样点数: {len(sample_points)}, SDK执行频率: {SDK_EXECUTE_RATE}Hz")
        
        # 设置运动模式
        piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        
        # 使用ROS Rate控制执行频率
        rate = rospy.Rate(SDK_EXECUTE_RATE)
        command_rate = rospy.Rate(COMMAND_PUBLISH_RATE)
        
        for idx, point in enumerate(sample_points):
            # 转换为SDK格式
            joints_int = [int(point.positions[i] * factor) for i in range(6)]
            joints_int[4] = max(-70000, joints_int[4])
            
            # 发送关节命令（使用高频率发布）
            piper.JointCtrl(*joints_int)
            
            # 更新末端执行器轨迹
            current_joints = [point.positions[i] for i in range(6)]
            update_ee_trail(current_joints)
            
            # 进度显示
            if idx % 5 == 0:
                progress = (idx + 1) / len(sample_points) * 100
                rospy.loginfo(f"执行进度: {progress:.1f}%")
            
            # 频率控制
            if idx < len(sample_points) - 1:
                rate.sleep()
        
        # 到达最终位置
        final_joints_int = [int(traj_points[-1].positions[i] * factor) for i in range(6)]
        final_joints_int[4] = max(-70000, final_joints_int[4])
        piper.JointCtrl(*final_joints_int)
        
        rospy.sleep(0.5)
        rospy.loginfo("✓ 轨迹执行完成")
        return True
        
    except Exception as e:
        rospy.logerr(f"轨迹执行失败: {e}")
        import traceback
        traceback.print_exc()
        return False


# ========================================
# 末端执行器轨迹可视化
# ========================================

def update_ee_trail(joints):
    """
    更新末端执行器轨迹（尾迹可视化）
    
    Args:
        joints: 当前关节角度列表
    """
    global piper_arm, ee_trail_points, ee_path_publisher, ee_trail_publisher
    
    # 正向运动学计算末端位置
    T = piper_arm.forward_kinematics(joints)
    ee_position = T[:3, 3]
    
    # 添加到轨迹点列表
    ee_trail_points.append(ee_position.copy())
    
    # 限制轨迹点数量（保持最近的N个点）
    if len(ee_trail_points) > MAX_TRAIL_POINTS:
        ee_trail_points.pop(0)
    
    # 发布末端路径（Path消息）
    if ee_path_publisher is not None:
        publish_ee_path()
    
    # 发布末端轨迹标记（LineStrip可视化）
    if ee_trail_publisher is not None:
        publish_ee_trail_marker()


def publish_ee_path():
    """发布末端执行器路径（Path消息）"""
    global ee_path_publisher, ee_trail_points
    
    if len(ee_trail_points) < 2:
        return
    
    path_msg = Path()
    path_msg.header.frame_id = "arm_base"
    path_msg.header.stamp = rospy.Time.now()
    
    for point in ee_trail_points:
        pose_stamped = PoseStamped()
        pose_stamped.header = path_msg.header
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.position.z = point[2]
        pose_stamped.pose.orientation.w = 1.0
        path_msg.poses.append(pose_stamped)
    
    ee_path_publisher.publish(path_msg)


def publish_ee_trail_marker():
    """发布末端执行器轨迹标记（LineStrip可视化，带渐变颜色）"""
    global ee_trail_publisher, ee_trail_points
    
    if len(ee_trail_points) < 2:
        return
    
    marker = Marker()
    marker.header.frame_id = "arm_base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "ee_trail"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    
    # 线条样式
    marker.scale.x = 0.003  # 线宽 3mm
    marker.pose.orientation.w = 1.0
    
    # 添加所有轨迹点（带渐变颜色）
    num_points = len(ee_trail_points)
    for i, point in enumerate(ee_trail_points):
        # 点位置
        from geometry_msgs.msg import Point
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2]
        marker.points.append(p)
        
        # 渐变颜色（从蓝色渐变到红色）
        from std_msgs.msg import ColorRGBA
        color = ColorRGBA()
        ratio = i / max(1, num_points - 1)
        color.r = ratio          # 红色分量增加
        color.g = 0.0
        color.b = 1.0 - ratio    # 蓝色分量减少
        color.a = 0.8           # 透明度
        marker.colors.append(color)
    
    ee_trail_publisher.publish(marker)


def clear_ee_trail():
    """清空末端执行器轨迹"""
    global ee_trail_points
    ee_trail_points = []
    rospy.loginfo("末端执行器轨迹已清空")


# ========================================
# MoveIt规划与执行
# ========================================

def plan_and_execute_to_pose(target_pose):
    """
    使用MoveIt规划并执行到目标6D位姿
    
    Args:
        target_pose: geometry_msgs/Pose 目标位姿
    
    Returns:
        bool: 是否成功
    """
    global move_group, display_trajectory_publisher
    
    try:
        rospy.loginfo("="*70)
        rospy.loginfo("开始MoveIt规划")
        rospy.loginfo("="*70)
        rospy.loginfo(f"目标位置: ({target_pose.position.x:.3f}, "
                     f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        rospy.loginfo(f"目标姿态: (x={target_pose.orientation.x:.3f}, "
                     f"y={target_pose.orientation.y:.3f}, "
                     f"z={target_pose.orientation.z:.3f}, "
                     f"w={target_pose.orientation.w:.3f})")
        
        # 获取当前位姿
        current_pose = move_group.get_current_pose().pose
        rospy.loginfo(f"当前位置: ({current_pose.position.x:.3f}, "
                     f"{current_pose.position.y:.3f}, {current_pose.position.z:.3f})")
        
        # 计算距离
        distance = np.sqrt(
            (target_pose.position.x - current_pose.position.x)**2 +
            (target_pose.position.y - current_pose.position.y)**2 +
            (target_pose.position.z - current_pose.position.z)**2
        )
        rospy.loginfo(f"移动距离: {distance:.3f}m ({distance*100:.1f}cm)")
        
        # 清除旧目标
        move_group.clear_pose_targets()
        move_group.stop()
        
        # 设置目标位姿
        move_group.set_pose_target(target_pose)
        
        # 开始规划
        rospy.loginfo(f"使用 {PLANNER_ID} 算法规划中...")
        rospy.loginfo(f"规划参数: {PLANNING_ATTEMPTS}次尝试, {PLANNING_TIME}秒超时")
        plan = move_group.plan()
        
        # 检查规划结果（兼容不同MoveIt版本）
        if isinstance(plan, tuple):
            success, trajectory = plan[0], plan[1]
        else:
            success = plan.joint_trajectory.points != []
            trajectory = plan
        
        if not success:
            rospy.logerr("❌ 规划失败")
            rospy.logerr("可能原因:")
            rospy.logerr("  1. 目标位姿超出工作空间")
            rospy.logerr("  2. 目标姿态不可达")
            rospy.logerr("  3. 存在碰撞")
            rospy.logerr("  4. 规划时间不足")
            rospy.logerr("\n建议:")
            rospy.logerr("  - 尝试更近的目标位置")
            rospy.logerr("  - 调整姿态角度")
            rospy.logerr("  - 增加规划时间/尝试次数")
            return False
        
        # 提取轨迹点
        if isinstance(plan, tuple):
            traj_points = plan[1].joint_trajectory.points
        else:
            traj_points = plan.joint_trajectory.points
        
        rospy.loginfo(f"✓ 规划成功！轨迹点数: {len(traj_points)}")
        
        # 发布轨迹到RViz可视化
        if display_trajectory_publisher is not None:
            rospy.loginfo(f"发布轨迹到RViz (频率: {RVIZ_PUBLISH_RATE}Hz)...")
            display_msg = DisplayTrajectory()
            display_msg.trajectory_start = move_group.get_current_state()
            display_msg.trajectory.append(trajectory if isinstance(plan, tuple) else plan)
            
            # 使用指定频率发布
            rate = rospy.Rate(RVIZ_PUBLISH_RATE)
            for _ in range(3):  # 发布3次确保RViz接收
                display_trajectory_publisher.publish(display_msg)
                rate.sleep()
            
            rospy.loginfo("✓ 轨迹已发布到RViz")
        
        # 使用SDK执行轨迹
        rospy.loginfo("="*70)
        rospy.loginfo("使用SDK执行轨迹")
        rospy.loginfo("="*70)
        
        success = execute_trajectory_with_sdk(trajectory if isinstance(plan, tuple) else plan)
        
        if success:
            rospy.loginfo("="*70)
            rospy.loginfo("✓✓✓ 任务完成！✓✓✓")
            rospy.loginfo("="*70)
        
        return success
        
    except Exception as e:
        rospy.logerr(f"规划或执行失败: {e}")
        import traceback
        traceback.print_exc()
        return False


# ========================================
# ROS回调函数
# ========================================

def target_pose_callback(msg):
    """
    接收目标6D位姿 (XYZ + Roll/Pitch/Yaw) 并执行规划
    
    Args:
        msg: std_msgs/Float64MultiArray
             data: [x, y, z, roll, pitch, yaw]
             - x, y, z: 位置 (m)
             - roll, pitch, yaw: 欧拉角 (rad)
    """
    try:
        # 检查数据长度
        if len(msg.data) != 6:
            rospy.logerr(f"目标位姿数据长度错误: 期望6个值 [x,y,z,roll,pitch,yaw], 收到{len(msg.data)}个值")
            return
        
        # 解析数据
        x, y, z, roll, pitch, yaw = msg.data
        
        rospy.loginfo("="*70)
        rospy.loginfo("收到新的目标位姿")
        rospy.loginfo(f"  位置: ({x:.3f}, {y:.3f}, {z:.3f}) m")
        rospy.loginfo(f"  姿态: Roll={roll:.3f}, Pitch={pitch:.3f}, Yaw={yaw:.3f} rad")
        rospy.loginfo(f"        Roll={np.degrees(roll):.1f}°, Pitch={np.degrees(pitch):.1f}°, Yaw={np.degrees(yaw):.1f}°")
        rospy.loginfo("="*70)
        
        # 清空之前的轨迹
        clear_ee_trail()
        
        # 创建Pose消息
        target_pose = create_pose_from_xyzrpy(x, y, z, roll, pitch, yaw)
        
        # 执行规划
        plan_and_execute_to_pose(target_pose)
        
    except Exception as e:
        rospy.logerr(f"处理目标位姿失败: {e}")
        import traceback
        traceback.print_exc()


# ========================================
# 主程序
# ========================================

def main():
    global piper, piper_arm, move_group
    global display_trajectory_publisher, ee_path_publisher, ee_trail_publisher
    
    rospy.loginfo("="*70)
    rospy.loginfo("MoveIt 6D位姿控制脚本")
    rospy.loginfo("="*70)
    rospy.loginfo("\n配置参数:")
    rospy.loginfo(f"  规划算法: {PLANNER_ID}")
    rospy.loginfo(f"  规划时间: {PLANNING_TIME}秒")
    rospy.loginfo(f"  规划尝试: {PLANNING_ATTEMPTS}次")
    rospy.loginfo(f"  速度缩放: {VELOCITY_SCALING}")
    rospy.loginfo(f"  加速度缩放: {ACCELERATION_SCALING}")
    rospy.loginfo(f"\n频率控制:")
    rospy.loginfo(f"  RViz发布频率: {RVIZ_PUBLISH_RATE}Hz")
    rospy.loginfo(f"  SDK执行频率: {SDK_EXECUTE_RATE}Hz")
    rospy.loginfo(f"  命令发布频率: {COMMAND_PUBLISH_RATE}Hz")
    rospy.loginfo(f"\n订阅话题:")
    rospy.loginfo(f"  目标位姿: {TOPIC_TARGET_POSE}")
    rospy.loginfo(f"\n可视化话题:")
    rospy.loginfo(f"  MoveIt轨迹: {TOPIC_TRAJECTORY}")
    rospy.loginfo(f"  末端路径: {TOPIC_EE_PATH}")
    rospy.loginfo(f"  末端尾迹: {TOPIC_EE_TRAIL}")
    rospy.loginfo("="*70)
    
    # 初始化ROS节点
    rospy.init_node('moveit_6d_pose_control', anonymous=True)
    
    # 初始化硬件
    rospy.loginfo("\n初始化机械臂硬件...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    
    for i in range(7):
        piper.EnableArm(i + 1)
        rospy.sleep(0.1)
    
    piper.GripperCtrl(70000, 1000, 0x01, 0)
    rospy.loginfo("✓ 硬件初始化完成")
    
    # 初始化PiperArm（用于正向运动学）
    piper_arm = PiperArm()
    
    # 初始化MoveIt
    rospy.loginfo("\n初始化MoveIt...")
    try:
        import os
        # 获取项目根目录（moveit_6d_pose_control.py 所在目录）
        project_root = os.path.dirname(os.path.abspath(__file__))
        piper_ros_path = os.path.join(project_root, "piper_ros")
        src_path = os.path.join(piper_ros_path, 'src')
        current_path = os.environ.get('ROS_PACKAGE_PATH', '')
        if src_path not in current_path:
            os.environ['ROS_PACKAGE_PATH'] = f"{src_path}:{current_path}"
        
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        
        # 创建MoveGroup
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        # 配置MoveIt规划器
        move_group.set_planner_id(PLANNER_ID)
        move_group.set_planning_time(PLANNING_TIME)
        move_group.set_num_planning_attempts(PLANNING_ATTEMPTS)
        move_group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
        move_group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)
        
        # 设置目标容差
        move_group.set_goal_position_tolerance(GOAL_POSITION_TOLERANCE)
        move_group.set_goal_orientation_tolerance(GOAL_ORIENTATION_TOLERANCE)
        
        rospy.loginfo(f"✓ MoveIt初始化完成 (规划组: {group_name})")
        rospy.loginfo(f"  规划器: {PLANNER_ID}")
        rospy.loginfo(f"  规划时间: {PLANNING_TIME}秒")
        rospy.loginfo(f"  规划尝试: {PLANNING_ATTEMPTS}次")
        rospy.loginfo(f"  位置容差: {GOAL_POSITION_TOLERANCE}m")
        rospy.loginfo(f"  姿态容差: {GOAL_ORIENTATION_TOLERANCE}rad")
        
    except Exception as e:
        rospy.logerr(f"MoveIt初始化失败: {e}")
        return
    
    # 创建发布器
    rospy.loginfo("\n创建ROS发布器...")
    display_trajectory_publisher = rospy.Publisher(
        TOPIC_TRAJECTORY,
        DisplayTrajectory,
        queue_size=20
    )
    
    ee_path_publisher = rospy.Publisher(
        TOPIC_EE_PATH,
        Path,
        queue_size=10
    )
    
    ee_trail_publisher = rospy.Publisher(
        TOPIC_EE_TRAIL,
        Marker,
        queue_size=10
    )
    
    rospy.loginfo("✓ 发布器创建完成")
    
    # 回零位
    rospy.loginfo("\n回零位...")
    joints_zero = [0, 0, 0, 0, 0, 0]
    control_arm_sdk(joints_zero, 100)
    rospy.sleep(2)
    rospy.loginfo("✓ 已回零位")
    
    # 订阅目标位姿话题
    rospy.loginfo(f"\n订阅目标位姿话题: {TOPIC_TARGET_POSE}")
    rospy.loginfo("  消息格式: std_msgs/Float64MultiArray")
    rospy.loginfo("  数据格式: [x, y, z, roll, pitch, yaw]")
    rospy.Subscriber(TOPIC_TARGET_POSE, Float64MultiArray, target_pose_callback, queue_size=1)
    
    rospy.loginfo("\n" + "="*70)
    rospy.loginfo("✓ 系统就绪！等待目标位姿...")
    rospy.loginfo("="*70)
    rospy.loginfo(f"\n发布示例命令 (命令行):")
    rospy.loginfo(f"rostopic pub {TOPIC_TARGET_POSE} std_msgs/Float64MultiArray \"data: [0.2, 0.0, 0.25, 0.0, 0.0, 0.0]\"")
    rospy.loginfo(f"\n说明:")
    rospy.loginfo(f"  data[0-2]: x, y, z 位置 (米)")
    rospy.loginfo(f"  data[3-5]: roll, pitch, yaw 欧拉角 (弧度)")
    rospy.loginfo(f"\n示例Python代码:")
    rospy.loginfo(f"  from std_msgs.msg import Float64MultiArray")
    rospy.loginfo(f"  pub = rospy.Publisher('{TOPIC_TARGET_POSE}', Float64MultiArray, queue_size=1)")
    rospy.loginfo(f"  msg = Float64MultiArray()")
    rospy.loginfo(f"  msg.data = [0.2, 0.0, 0.25, 0.0, 0.0, 0.0]  # x, y, z, roll, pitch, yaw")
    rospy.loginfo(f"  pub.publish(msg)")
    rospy.loginfo("="*70 + "\n")
    
    # 保持节点运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("\n程序被用户中断")
    finally:
        # 清理资源
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("程序结束")


if __name__ == "__main__":
    main()
