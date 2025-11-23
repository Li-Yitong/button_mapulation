#!/usr/bin/env python3
"""
按钮操作执行器 - ROS2版本
支持四种按钮操作类型：Toggle, Plug-in, Push, Knob
集成ROS2 MoveIt2进行轨迹规划
兼容 ROS2 Foxy/Humble/Galactic
"""

# ========================================
# ROS2 条件导入机制（兼容多环境）
# ========================================
import sys
import os
import time
import math
import numpy as np
import traceback
import threading
import csv

# 检测ROS2环境
ROS_AVAILABLE = False
ros_distro = os.environ.get('ROS_DISTRO', 'unknown')

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    ROS_AVAILABLE = True
    print(f"✓ ROS2 ({ros_distro}) 环境检测成功")
except ImportError:
    ROS_AVAILABLE = False
    print("⚠️  未检测到ROS2环境，将使用纯SDK模式")
    
    # 提供Fake类以保证代码在无ROS环境下也能运行
    class FakeNode:
        class Clock:
            @staticmethod
            def now():
                class FakeTime:
                    def __init__(self):
                        self.nanoseconds = int(time.time() * 1e9)
                    def to_msg(self):
                        class FakeMsg:
                            def __init__(self, sec, nanosec):
                                self.sec = sec
                                self.nanosec = nanosec
                        return FakeMsg(int(time.time()), int((time.time() % 1) * 1e9))
                return FakeTime()
        
        def get_clock(self):
            return self.Clock()
        
        def create_publisher(self, *args, **kwargs):
            return FakePublisher()
        
        def create_timer(self, *args, **kwargs):
            return None
    
    class FakePublisher:
        def publish(self, msg):
            pass
        def get_num_connections(self):
            return 0
    
    class FakeRclpy:
        @staticmethod
        def init():
            pass
        @staticmethod
        def shutdown():
            pass
        @staticmethod
        def spin(node):
            pass
    
    rclpy = FakeRclpy()

# 导入Piper SDK
from piper_sdk import C_PiperInterface
from piper_arm import PiperArm

# ========================================
# 宏定义 - 用户配置区
# ========================================
PI = math.pi
factor = 1000 * 180 / PI

# === 目标位姿配置 (基座坐标系) ===
TARGET_X = 0.26      # X坐标 (单位：米)
TARGET_Y = 0.00      # Y坐标
TARGET_Z = 0.25      # Z坐标

# 姿态 (单位：弧度)
TARGET_ROLL = 0.0    # 绕末端X轴旋转
TARGET_PITCH = 0.0   # 绕末端Y轴旋转
TARGET_YAW = 0.0     # 绕末端Z轴旋转

USE_6D_POSE = True   # True=使用6D位姿, False=仅使用位置

# === 动作类型选择 ===
ACTION_TYPE = 'push'  # 'toggle'/'plugin'/'push'/'knob'

# === 控制模式 ===
USE_MOVEIT = True  # 启动脚本自动设置

# === Plugin (插拔连接器) 配置 ===
PLUGIN_GRIPPER_OPEN = 60000
PLUGIN_INSERT_DEPTH = 0.03
PLUGIN_GRIPPER_HOLD = 500
PLUGIN_INSERT_SPEED = 100
PLUGIN_EXTRACT_SPEED = 100

# === Toggle (拨动开关) 配置 ===
TOGGLE_GRIPPER_OPEN = 70000
TOGGLE_JOINT4_ROTATE = 90
TOGGLE_INSERT_DEPTH = 0.03
TOGGLE_GRIPPER_HOLD = 30000
TOGGLE_JOINT3_ANGLE = 30
TOGGLE_DIRECTION = 'left'
TOGGLE_INSERT_SPEED = 20
TOGGLE_TOGGLE_SPEED = 30

# === Push (按压按钮) 配置 ===
PUSH_GRIPPER_CLOSE = 0
PUSH_INSERT_DEPTH = 0.003
PUSH_HOLD_TIME = 0.01
PUSH_PRESS_SPEED = 30

# === Knob (旋转旋钮) 配置 ===
KNOB_GRIPPER_OPEN = 45000
KNOB_INSERT_DEPTH = 0.007
KNOB_GRIPPER_HOLD = 8000
KNOB_ROTATION_ANGLE = 45
KNOB_ROTATION_DIRECTION = 'ccw'
KNOB_INSERT_SPEED = 80
KNOB_ROTATION_SPEED = 60

# === 通用速度配置 ===
NORMAL_SPEED = 100
FAST_SPEED = 100

# ========================================
# MoveIt2 配置
# ========================================
RVIZ_PUBLISH_RATE = 10
COMMAND_SEND_RATE = 80
PLANNER_ID = "RRTConnect"  # ROS2推荐: RRTConnect, RRTstar, PRM, BKPIECE

DEBUG_TRAJECTORY = False
MAX_TRAIL_POINTS = 100

# ========================================
# MoveIt2 导入（ROS2 Action Client方式）
# ========================================
MOVEIT_AVAILABLE = False
move_group_action_client = None
moveit_node = None
joint_state_publisher = None
display_trajectory_publisher = None
ee_path_publisher = None
ee_trail_publisher = None

try:
    if USE_MOVEIT and ROS_AVAILABLE:
        # ROS2 MoveIt2 消息类型
        from moveit_msgs.action import MoveGroup as MoveGroupAction
        from moveit_msgs.msg import (
            DisplayTrajectory,
            RobotTrajectory,
            MotionPlanRequest,
            Constraints,
            JointConstraint,
            RobotState,
            WorkspaceParameters,
            PlanningOptions
        )
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        from geometry_msgs.msg import Point, PoseStamped
        from visualization_msgs.msg import Marker
        from std_msgs.msg import ColorRGBA
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        
        MOVEIT_AVAILABLE = True
        print(f"✓ MoveIt2 (ROS2 {ros_distro}) 已加载")
except ImportError as e:
    print(f"⚠️  MoveIt2未加载，将使用SDK模式: {e}")

# 全局变量
piper = None
piper_arm = None
ee_trail_points = []
planned_trajectory = []
executed_trajectory = []
trajectory_save_dir = "trajectory"

# ========================================
# ROS2 QoS配置
# ========================================
def get_default_qos():
    """获取默认QoS配置（兼容MoveIt2）"""
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )
    return qos

# ========================================
# 轨迹保存和可视化函数
# ========================================
def ensure_trajectory_dir():
    """确保轨迹保存目录存在"""
    if not os.path.exists(trajectory_save_dir):
        os.makedirs(trajectory_save_dir)
        print(f"  ✓ 创建轨迹保存目录: {trajectory_save_dir}/")

def save_trajectory_to_csv(traj_points, filename_prefix):
    """保存轨迹到CSV文件（ROS2版本）"""
    from datetime import datetime
    
    ensure_trajectory_dir()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{trajectory_save_dir}/{filename_prefix}_trajectory_{timestamp}.csv"
    
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            header = ['点号', '时间(s)', '时间间隔(ms)']
            header += [f'关节{i+1}(°)' for i in range(6)]
            header += [f'速度{i+1}(°/s)' for i in range(6)]
            header += [f'加速度{i+1}(°/s²)' for i in range(6)]
            header += ['末端X(m)', '末端Y(m)', '末端Z(m)']
            writer.writerow(header)
            
            prev_time = 0.0
            for idx, point in enumerate(traj_points):
                row = [idx]
                
                # 时间处理（ROS2使用Duration）
                time_sec = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                row.append(f"{time_sec:.4f}")
                
                dt = (time_sec - prev_time) * 1000
                row.append(f"{dt:.2f}")
                prev_time = time_sec
                
                # 关节角度
                joints = [point.positions[i] * 180 / PI for i in range(6)]
                row.extend([f"{j:.2f}" for j in joints])
                
                # 速度
                if len(point.velocities) >= 6:
                    velocities = [point.velocities[i] * 180 / PI for i in range(6)]
                    row.extend([f"{v:.2f}" for v in velocities])
                else:
                    row.extend(['0.00'] * 6)
                
                # 加速度
                if len(point.accelerations) >= 6:
                    accelerations = [point.accelerations[i] * 180 / PI for i in range(6)]
                    row.extend([f"{a:.2f}" for a in accelerations])
                else:
                    row.extend(['0.00'] * 6)
                
                # 末端位置
                joints_rad = [point.positions[i] for i in range(6)]
                T = piper_arm.forward_kinematics(joints_rad)
                xyz = T[:3, 3]
                row.extend([f"{xyz[0]:.6f}", f"{xyz[1]:.6f}", f"{xyz[2]:.6f}"])
                
                writer.writerow(row)
        
        print(f"  ✓ 轨迹已保存: {filename}")
        return filename
    except Exception as e:
        print(f"  ⚠️  保存CSV失败: {e}")
        return None

def publish_dual_trajectory_markers(planned_xyz, executed_xyz):
    """在RViz2中发布规划路径和执行路径的对比"""
    if not MOVEIT_AVAILABLE or len(planned_xyz) == 0 or moveit_node is None:
        return
    
    # 创建临时publisher
    marker_pub = moveit_node.create_publisher(Marker, '/trajectory_comparison', get_default_qos())
    time.sleep(0.1)
    
    # 规划路径（蓝色线）
    planned_marker = Marker()
    planned_marker.header.frame_id = "base_link"
    planned_marker.header.stamp = moveit_node.get_clock().now().to_msg()
    planned_marker.ns = "planned_trajectory"
    planned_marker.id = 0
    planned_marker.type = Marker.LINE_STRIP
    planned_marker.action = Marker.ADD
    planned_marker.scale.x = 0.005
    planned_marker.color.r = 0.0
    planned_marker.color.g = 0.5
    planned_marker.color.b = 1.0
    planned_marker.color.a = 0.8
    planned_marker.pose.orientation.w = 1.0
    
    for xyz in planned_xyz:
        p = Point()
        p.x = float(xyz[0])
        p.y = float(xyz[1])
        p.z = float(xyz[2])
        planned_marker.points.append(p)
    
    # 执行路径（红色线）
    executed_marker = Marker()
    executed_marker.header.frame_id = "base_link"
    executed_marker.header.stamp = moveit_node.get_clock().now().to_msg()
    executed_marker.ns = "executed_trajectory"
    executed_marker.id = 1
    executed_marker.type = Marker.LINE_STRIP
    executed_marker.action = Marker.ADD
    executed_marker.scale.x = 0.003
    executed_marker.color.r = 1.0
    executed_marker.color.g = 0.0
    executed_marker.color.b = 0.0
    executed_marker.color.a = 0.9
    executed_marker.pose.orientation.w = 1.0
    
    if len(executed_xyz) > 0:
        for xyz in executed_xyz:
            p = Point()
            p.x = float(xyz[0])
            p.y = float(xyz[1])
            p.z = float(xyz[2])
            executed_marker.points.append(p)
    
    # 发布标记（ROS2需要多次发布）
    for _ in range(3):
        marker_pub.publish(planned_marker)
        marker_pub.publish(executed_marker)
        time.sleep(0.1)
    
    print(f"  ✓ 轨迹对比已发布到 RViz2 (/trajectory_comparison)")
    print(f"    🔵 蓝色 = 规划路径 ({len(planned_xyz)}个点)")
    if len(executed_xyz) > 0:
        print(f"    🔴 红色 = 执行路径 ({len(executed_xyz)}个点)")

def plot_trajectory_comparison(planned_xyz, executed_xyz, planned_times, executed_times):
    """使用Matplotlib绘制轨迹对比图"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from datetime import datetime
        
        chinese_fonts = ['Noto Sans CJK JP', 'AR PL UMing CN', 'AR PL UKai CN', 'DejaVu Sans']
        plt.rcParams['font.sans-serif'] = chinese_fonts
        plt.rcParams['axes.unicode_minus'] = False
    except ImportError:
        print("  ⚠️  matplotlib 未安装，跳过绘图")
        return None
    
    ensure_trajectory_dir()
    
    planned_xyz = np.array(planned_xyz)
    executed_xyz = np.array(executed_xyz) if len(executed_xyz) > 0 else np.array([])
    
    fig = plt.figure(figsize=(16, 10))
    
    # 3D轨迹对比
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(planned_xyz[:, 0], planned_xyz[:, 1], planned_xyz[:, 2], 
             'b-', linewidth=2, label='Planned', alpha=0.7)
    if len(executed_xyz) > 0:
        ax1.plot(executed_xyz[:, 0], executed_xyz[:, 1], executed_xyz[:, 2], 
                 'r-', linewidth=2, label='Executed', alpha=0.7)
    ax1.scatter(planned_xyz[0, 0], planned_xyz[0, 1], planned_xyz[0, 2], 
                c='g', s=100, marker='o', label='Start')
    ax1.scatter(planned_xyz[-1, 0], planned_xyz[-1, 1], planned_xyz[-1, 2], 
                c='orange', s=100, marker='s', label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('End-Effector 3D Trajectory')
    ax1.legend()
    
    # X/Y/Z随时间变化
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(planned_times, planned_xyz[:, 0], 'b-', linewidth=2, label='Planned', alpha=0.7)
    if len(executed_xyz) > 0:
        ax2.plot(executed_times, executed_xyz[:, 0], 'r--', linewidth=2, label='Executed', alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('X (m)')
    ax2.set_title('X Coordinate vs Time')
    ax2.legend()
    ax2.grid(True)
    
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(planned_times, planned_xyz[:, 1], 'b-', linewidth=2, label='Planned', alpha=0.7)
    if len(executed_xyz) > 0:
        ax3.plot(executed_times, executed_xyz[:, 1], 'r--', linewidth=2, label='Executed', alpha=0.7)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Y (m)')
    ax3.set_title('Y Coordinate vs Time')
    ax3.legend()
    ax3.grid(True)
    
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(planned_times, planned_xyz[:, 2], 'b-', linewidth=2, label='Planned', alpha=0.7)
    if len(executed_xyz) > 0:
        ax4.plot(executed_times, executed_xyz[:, 2], 'r--', linewidth=2, label='Executed', alpha=0.7)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Z (m)')
    ax4.set_title('Z Coordinate vs Time')
    ax4.legend()
    ax4.grid(True)
    
    # XY平面投影
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(planned_xyz[:, 0], planned_xyz[:, 1], 'b-', linewidth=2, label='Planned', alpha=0.7)
    if len(executed_xyz) > 0:
        ax5.plot(executed_xyz[:, 0], executed_xyz[:, 1], 'r--', linewidth=2, label='Executed', alpha=0.7)
    ax5.scatter(planned_xyz[0, 0], planned_xyz[0, 1], c='g', s=100, marker='o', label='Start')
    ax5.scatter(planned_xyz[-1, 0], planned_xyz[-1, 1], c='orange', s=100, marker='s', label='End')
    ax5.set_xlabel('X (m)')
    ax5.set_ylabel('Y (m)')
    ax5.set_title('XY Plane Projection')
    ax5.legend()
    ax5.grid(True)
    ax5.axis('equal')
    
    # 轨迹误差
    ax6 = fig.add_subplot(2, 3, 6)
    if len(executed_xyz) > 0 and len(executed_xyz) == len(planned_xyz):
        errors = np.linalg.norm(executed_xyz - planned_xyz, axis=1) * 100
        ax6.plot(planned_times, errors, 'r-', linewidth=2)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Error (cm)')
        ax6.set_title(f'Tracking Error (Avg: {np.mean(errors):.2f}cm)')
        ax6.grid(True)
    else:
        ax6.text(0.5, 0.5, f'Planned: {len(planned_xyz)} pts\nExecuted: {len(executed_xyz)} pts', 
                 ha='center', va='center', fontsize=12)
        ax6.set_title('Trajectory Statistics')
    
    plt.tight_layout()
    
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{trajectory_save_dir}/trajectory_comparison_{timestamp}.png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"  ✓ 轨迹对比图已保存: {filename}")
    
    plt.close()
    return filename

# ========================================
# ROS2 Joint States 发布器
# ========================================
def publish_joint_states_callback():
    """ROS2定时器回调：发布当前关节状态"""
    global piper, joint_state_publisher, moveit_node
    
    if not ROS_AVAILABLE or piper is None or joint_state_publisher is None:
        return
    
    try:
        msg_data = piper.GetArmJointMsgs()
        
        msg = JointState()
        msg.header.stamp = moveit_node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        # 转换单位：SDK返回毫度(milli-degree) → 弧度
        msg.position = [
            msg_data.joint_state.joint_1 * 1e-3 * PI / 180.0,
            msg_data.joint_state.joint_2 * 1e-3 * PI / 180.0,
            msg_data.joint_state.joint_3 * 1e-3 * PI / 180.0,
            msg_data.joint_state.joint_4 * 1e-3 * PI / 180.0,
            msg_data.joint_state.joint_5 * 1e-3 * PI / 180.0,
            msg_data.joint_state.joint_6 * 1e-3 * PI / 180.0,
            0.0  # joint7 (gripper)
        ]
        
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        
        joint_state_publisher.publish(msg)
    except Exception as e:
        print(f"  ⚠️  发布joint_states失败: {e}")

def setup_joint_state_publisher():
    """设置joint_states发布器（10Hz）"""
    global joint_state_publisher, moveit_node
    
    if not ROS_AVAILABLE or moveit_node is None:
        return
    
    qos = get_default_qos()
    joint_state_publisher = moveit_node.create_publisher(JointState, '/joint_states', qos)
    
    # 创建10Hz定时器
    moveit_node.create_timer(0.1, publish_joint_states_callback)
    print("  ✓ Joint States 发布器已启动 (10Hz)")

# ========================================
# ROS2 Spin 后台线程
# ========================================
def setup_ros2_spin_thread():
    """启动ROS2 spin线程（后台处理消息）"""
    global moveit_node
    
    if not ROS_AVAILABLE or moveit_node is None:
        return
    
    def spin_thread():
        try:
            rclpy.spin(moveit_node)
        except Exception as e:
            print(f"  ⚠️  ROS2 spin线程异常: {e}")
    
    thread = threading.Thread(target=spin_thread, daemon=True)
    thread.start()
    print("  ✓ ROS2 spin线程已启动")

# ========================================
# MoveIt2 Action Client 初始化
# ========================================
def setup_moveit2():
    """设置MoveIt2 Action Client"""
    global move_group_action_client, moveit_node
    
    if not MOVEIT_AVAILABLE or moveit_node is None:
        return
    
    try:
        # 创建Action Client连接到 /move_action
        move_group_action_client = ActionClient(
            moveit_node,
            MoveGroupAction,
            '/move_action'
        )
        
        print("  ⏳ 等待MoveIt2服务...")
        if move_group_action_client.wait_for_server(timeout_sec=5.0):
            print("  ✓ MoveIt2 Action Client 已连接 (/move_action)")
        else:
            print("  ⚠️  MoveIt2服务未响应，将使用SDK模式")
            move_group_action_client = None
    except Exception as e:
        print(f"  ❌ 初始化MoveIt2失败: {e}")
        move_group_action_client = None

# ========================================
# 控制函数
# ========================================
def update_ee_trail(joints):
    """更新末端执行器轨迹"""
    global piper_arm, ee_trail_points
    
    if piper_arm is None:
        return
    
    T = piper_arm.forward_kinematics(joints)
    ee_position = T[:3, 3]
    ee_trail_points.append(ee_position.copy())
    
    if len(ee_trail_points) > MAX_TRAIL_POINTS:
        ee_trail_points.pop(0)

def clear_ee_trail():
    """清空末端轨迹"""
    global ee_trail_points
    ee_trail_points = []

def clear_trajectory_records():
    """清空轨迹记录"""
    global planned_trajectory, executed_trajectory
    planned_trajectory = []
    executed_trajectory = []
    print("  ✓ 已清空轨迹记录")

def save_and_visualize_trajectory():
    """保存并可视化轨迹"""
    global planned_trajectory, executed_trajectory
    
    if len(planned_trajectory) == 0:
        print("  ⚠️  没有规划轨迹记录")
        return
    
    print("\n" + "="*70)
    print("📊 保存和可视化轨迹...")
    print("="*70)
    print(f"  📍 规划轨迹点数: {len(planned_trajectory)}")
    print(f"  📍 执行轨迹点数: {len(executed_trajectory)}")
    
    # 发布到RViz2
    if len(executed_trajectory) > 0:
        publish_dual_trajectory_markers(planned_trajectory, executed_trajectory)
    
    # 绘制对比图
    if DEBUG_TRAJECTORY and len(executed_trajectory) > 0:
        planned_times = np.linspace(0, len(planned_trajectory)*0.1, len(planned_trajectory))
        executed_times = np.linspace(0, len(executed_trajectory)*0.0125, len(executed_trajectory))
        plot_trajectory_comparison(planned_trajectory, executed_trajectory, 
                                 planned_times, executed_times)
    
    print("="*70)

def control_arm_sdk(joints, speed=50, gripper_value=None):
    """SDK直接控制模式"""
    global piper
    
    # 关键修复：确保机械臂使能（防止规划失败后失能导致摔落）
    piper.EnableArm(7)  # 使能所有关节 + 夹爪
    time.sleep(0.05)  # 等待使能生效
    
    joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
    joints_int[4] = max(-70000, joints_int[4])
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joints_int)
    
    # 🔧 关键修复：等待机械臂到达目标位置
    # 估算运动时间（基于速度和关节角度差异）
    current = get_current_joints()
    max_joint_diff = max([abs(joints[i] - current[i]) for i in range(6)])
    estimated_time = max_joint_diff / (speed / 100.0 * 2.0) + 0.5  # 保守估计
    estimated_time = min(estimated_time, 10.0)  # 最长等待10秒
    print(f"  [SDK] 移动中... (预计{estimated_time:.1f}秒)")
    time.sleep(estimated_time)
    
    if gripper_value is not None:
        gripper_int = int(gripper_value)
        piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    elif len(joints) > 6:
        gripper_int = int(joints[6] * 1000000)
        piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    
    return True

def control_arm_moveit(joints, speed=50, gripper_value=None):
    """MoveIt2规划控制模式（ROS2 Action Client）"""
    global piper, move_group_action_client, moveit_node
    global planned_trajectory, executed_trajectory
    
    if move_group_action_client is None:
        return control_arm_sdk(joints, speed, gripper_value)
    
    try:
        # 获取当前关节角度
        current_joints = get_current_joints()
        target_joints = joints[:6] if len(joints) > 6 else joints
        
        # 构建MoveGroup.action Goal
        goal_msg = MoveGroupAction.Goal()
        
        # 1. 设置规划组名称（必须与SRDF中的group名称一致）
        goal_msg.request.group_name = "arm"  # 修复：从 "piper_arm" 改为 "arm"
        
        # 2. 设置工作空间参数
        goal_msg.request.workspace_parameters = WorkspaceParameters()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.header.stamp = moveit_node.get_clock().now().to_msg()
        goal_msg.request.workspace_parameters.min_corner.x = -0.5
        goal_msg.request.workspace_parameters.min_corner.y = -0.5
        goal_msg.request.workspace_parameters.min_corner.z = -0.5
        goal_msg.request.workspace_parameters.max_corner.x = 0.5
        goal_msg.request.workspace_parameters.max_corner.y = 0.5
        goal_msg.request.workspace_parameters.max_corner.z = 0.5
        
        # 3. 设置起始状态（当前关节角度）
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = JointState()
        goal_msg.request.start_state.joint_state.header.stamp = moveit_node.get_clock().now().to_msg()
        goal_msg.request.start_state.joint_state.name = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        goal_msg.request.start_state.joint_state.position = list(current_joints)
        goal_msg.request.start_state.is_diff = False
        
        # 4. 设置目标约束（目标关节角度）
        constraints = Constraints()
        for i, angle in enumerate(target_joints):
            jc = JointConstraint()
            jc.joint_name = f'joint{i+1}'
            jc.position = float(angle)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints = [constraints]
        
        # 5. 设置规划器参数
        goal_msg.request.planner_id = PLANNER_ID
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = speed / 100.0
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # 6. 设置规划选项
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        print("  [MoveIt2] 发送规划请求...")
        print(f"  📍 起始点: [{', '.join([f'{j:.4f}' for j in current_joints])}]")
        print(f"  📍 目标点: [{', '.join([f'{j:.4f}' for j in target_joints])}]")
        
        # 7. 发送goal并等待结果（同步方式）
        future = move_group_action_client.send_goal_async(goal_msg)
        
        # 等待goal被接受
        rclpy.spin_until_future_complete(moveit_node, future, timeout_sec=10.0)
        
        if not future.done():
            print("  ❌ 规划请求超时，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            print("  ❌ 规划请求被拒绝，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        # 等待规划结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(moveit_node, result_future, timeout_sec=10.0)
        
        if not result_future.done():
            print("  ❌ 规划超时，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        result = result_future.result()
        
        if result.result.error_code.val != 1:  # 1 = SUCCESS
            print(f"  ❌ 规划失败 (错误码: {result.result.error_code.val})，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        # 提取轨迹
        trajectory = result.result.planned_trajectory.joint_trajectory
        traj_points = trajectory.points
        
        if len(traj_points) == 0:
            print("  ❌ 规划的轨迹为空，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        print(f"  ✓ 规划成功 (轨迹点: {len(traj_points)})")
        
        # 提取规划的末端轨迹
        global planned_trajectory
        for point in traj_points:
            joints_rad = [point.positions[i] for i in range(6)]
            T = piper_arm.forward_kinematics(joints_rad)
            xyz = T[:3, 3]
            planned_trajectory.append(xyz.copy())
        
        print(f"  ✓ 已提取规划轨迹 (累计: {len(planned_trajectory)}个点)")
        
        # 保存轨迹
        if DEBUG_TRAJECTORY:
            save_trajectory_to_csv(traj_points, "planned")
        
        # SDK执行轨迹
        print(f"  [SDK] 执行完整轨迹 (点数: {len(traj_points)}, 速度: {speed})")
        
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        
        start_time = time.time()
        command_rate_delay = 1.0 / COMMAND_SEND_RATE
        
        executed_trajectory.clear()
        
        for idx, point in enumerate(traj_points):
            # 发送关节命令
            joints_int = [int(point.positions[i] * factor) for i in range(6)]
            joints_int[4] = max(-70000, joints_int[4])
            piper.JointCtrl(*joints_int)
            
            # 记录执行轨迹
            if idx % 5 == 0:
                joints_rad = [point.positions[i] for i in range(6)]
                T = piper_arm.forward_kinematics(joints_rad)
                xyz = T[:3, 3]
                executed_trajectory.append(xyz.copy())
                update_ee_trail(joints_rad)
            
            # 按固定频率发送
            time.sleep(command_rate_delay)
        
        total_time = time.time() - start_time
        print(f"  ✓ 执行完成 (耗时: {total_time:.2f}s)")
        
        # 等待到达目标
        time.sleep(0.1)
        
        # 控制夹爪
        if gripper_value is not None:
            piper.GripperCtrl(abs(int(gripper_value)), 1000, 0x01, 0)
        
        return True
        
    except Exception as e:
        print(f"  ❌ MoveIt2执行失败: {e}")
        traceback.print_exc()
        return control_arm_sdk(joints, speed, gripper_value)

def control_arm(joints, speed=50, use_moveit=False, gripper_value=None):
    """统一控制接口"""
    if gripper_value is None:
        gripper_value = joints[6] * 1000000 if len(joints) > 6 else None
    
    if use_moveit and MOVEIT_AVAILABLE and move_group_action_client is not None:
        return control_arm_moveit(joints[:6], speed, gripper_value)
    else:
        return control_arm_sdk(joints, speed, gripper_value)

def get_current_joints():
    """获取当前关节角度"""
    global piper
    msg = piper.GetArmJointMsgs()
    return [
        msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_6 * 1e-3 * PI / 180.0,
    ]

def euler_to_rotation_matrix(roll, pitch, yaw):
    """欧拉角转旋转矩阵（ZYX顺序）"""
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    return Rz @ Ry @ Rx

def create_target_transform(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, use_6d=False):
    """创建目标位姿变换矩阵"""
    T = np.eye(4)
    
    if use_6d:
        # 默认姿态（末端朝前）
        R_default = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
        R_delta = euler_to_rotation_matrix(roll, pitch, yaw)
        T[:3, :3] = R_default @ R_delta
    else:
        # 默认姿态
        T[:3, :3] = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
    
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    
    return T

def move_along_end_effector_z(current_joints, distance, speed=20):
    """沿末端执行器Z轴方向移动（保持姿态）"""
    global piper_arm
    
    # 当前末端位姿
    T_current = piper_arm.forward_kinematics(current_joints)
    
    # 末端Z轴方向
    z_axis = T_current[:3, 2]
    
    # 计算新位置
    new_position = T_current[:3, 3] + z_axis * distance
    
    # 创建新目标位姿
    T_target = T_current.copy()
    T_target[:3, 3] = new_position
    
    # 逆运动学求解
    new_joints = piper_arm.inverse_kinematics(T_target)
    
    if new_joints is None:
        print(f"  ⚠️  IK求解失败（距离={distance:.4f}m），保持当前位置")
        return current_joints
    
    # 执行移动
    control_arm(new_joints, speed=speed, use_moveit=USE_MOVEIT)
    time.sleep(abs(distance) / 0.05)
    
    return new_joints

# ========================================
# 按钮操作函数
# ========================================
def action_toggle():
    """拨动开关操作"""
    print("\n" + "="*70)
    print("🎯 开始执行 Toggle (拨动开关) 操作")
    print("="*70)
    
    clear_trajectory_records()
    clear_ee_trail()
    
    # 1. 移动到目标位置上方
    print("\n📍 步骤1: 移动到目标上方")
    T_target = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z + 0.05,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
    joints = piper_arm.inverse_kinematics(T_target)
    if joints is None:
        print("  ❌ IK求解失败")
        return
    control_arm(joints, speed=FAST_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(1.0)
    
    # 2. 张开夹爪
    print("\n📍 步骤2: 张开夹爪")
    piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 3. 旋转joint4到指定角度
    print(f"\n📍 步骤3: 旋转joint4到{TOGGLE_JOINT4_ROTATE}°")
    current = get_current_joints()
    current[3] += TOGGLE_JOINT4_ROTATE * PI / 180.0
    control_arm(current, speed=NORMAL_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(0.5)
    
    # 4. 插入
    print(f"\n📍 步骤4: 插入深度={TOGGLE_INSERT_DEPTH}m")
    current = get_current_joints()
    current = move_along_end_effector_z(current, TOGGLE_INSERT_DEPTH, TOGGLE_INSERT_SPEED)
    time.sleep(0.5)
    
    # 5. 夹持
    print(f"\n📍 步骤5: 夹持（宽度={TOGGLE_GRIPPER_HOLD}）")
    piper.GripperCtrl(TOGGLE_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 6. 拨动
    direction_sign = -1 if TOGGLE_DIRECTION == 'left' else 1
    print(f"\n📍 步骤6: 拨动（方向={TOGGLE_DIRECTION}, 角度={TOGGLE_JOINT3_ANGLE}°）")
    current = get_current_joints()
    current[2] += direction_sign * TOGGLE_JOINT3_ANGLE * PI / 180.0
    control_arm(current, speed=TOGGLE_TOGGLE_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(0.5)
    
    # 7. 复位joint3
    print("\n📍 步骤7: 复位joint3")
    current[2] -= direction_sign * TOGGLE_JOINT3_ANGLE * PI / 180.0
    control_arm(current, speed=TOGGLE_TOGGLE_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(0.5)
    
    # 8. 松开夹爪
    print("\n📍 步骤8: 松开夹爪")
    piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 9. 退出
    print("\n📍 步骤9: 退出")
    current = get_current_joints()
    new_joints = move_along_end_effector_z(current, -TOGGLE_INSERT_DEPTH, TOGGLE_INSERT_SPEED)
    if new_joints is not None:
        current = new_joints
    time.sleep(0.5)
    
    # 10. 复位joint4
    print("\n📍 步骤10: 复位joint4")
    current = get_current_joints()
    current[3] -= TOGGLE_JOINT4_ROTATE * PI / 180.0
    control_arm(current, speed=NORMAL_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(0.5)
    
    print("\n✅ Toggle操作完成")
    save_and_visualize_trajectory()

def action_plugin():
    """插拔连接器操作"""
    print("\n" + "="*70)
    print("🎯 开始执行 Plug-in (插拔连接器) 操作")
    print("="*70)
    
    clear_trajectory_records()
    clear_ee_trail()
    
    # 1. 移动到目标上方
    print("\n📍 步骤1: 移动到目标上方")
    T_target = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z + 0.05,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
    joints = piper_arm.inverse_kinematics(T_target)
    if joints is None:
        print("  ❌ IK求解失败")
        return
    control_arm(joints, speed=FAST_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(1.0)
    
    # 2. 张开夹爪
    print("\n📍 步骤2: 张开夹爪")
    piper.GripperCtrl(PLUGIN_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 3. 插入
    print(f"\n📍 步骤3: 插入深度={PLUGIN_INSERT_DEPTH}m")
    current = get_current_joints()
    current = move_along_end_effector_z(current, PLUGIN_INSERT_DEPTH, PLUGIN_INSERT_SPEED)
    time.sleep(0.5)
    
    # 4. 夹持
    print(f"\n📍 步骤4: 夹持（宽度={PLUGIN_GRIPPER_HOLD}）")
    piper.GripperCtrl(PLUGIN_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 5. 拔出
    print("\n📍 步骤5: 拔出")
    current = move_along_end_effector_z(current, -PLUGIN_INSERT_DEPTH, PLUGIN_EXTRACT_SPEED)
    time.sleep(0.5)
    
    # 6. 松开夹爪
    print("\n📍 步骤6: 松开夹爪")
    piper.GripperCtrl(PLUGIN_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.5)
    
    print("\n✅ Plug-in操作完成")
    save_and_visualize_trajectory()

def action_push():
    """按压按钮操作"""
    print("\n" + "="*70)
    print("🎯 开始执行 Push (按压按钮) 操作")
    print("="*70)
    
    clear_trajectory_records()
    clear_ee_trail()
    
    # 1. 闭合夹爪
    print("\n📍 步骤1: 闭合夹爪")
    piper.GripperCtrl(PUSH_GRIPPER_CLOSE, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 2. 移动到目标上方
    print("\n📍 步骤2: 移动到目标上方")
    T_target = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z + 0.05,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
    joints = piper_arm.inverse_kinematics(T_target)
    if joints is None:
        print("  ❌ IK求解失败")
        return
    control_arm(joints, speed=FAST_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(1.0)
    
    # 3. 按压
    print(f"\n📍 步骤3: 按压深度={PUSH_INSERT_DEPTH}m")
    current = get_current_joints()
    current = move_along_end_effector_z(current, PUSH_INSERT_DEPTH, PUSH_PRESS_SPEED)
    time.sleep(PUSH_HOLD_TIME)
    
    # 4. 抬起
    print("\n📍 步骤4: 抬起")
    current = move_along_end_effector_z(current, -PUSH_INSERT_DEPTH, PUSH_PRESS_SPEED)
    time.sleep(0.5)
    
    print("\n✅ Push操作完成")
    save_and_visualize_trajectory()

def action_knob():
    """旋转旋钮操作"""
    print("\n" + "="*70)
    print("🎯 开始执行 Knob (旋转旋钮) 操作")
    print("="*70)
    
    clear_trajectory_records()
    clear_ee_trail()
    
    # 1. 移动到目标上方
    print("\n📍 步骤1: 移动到目标上方")
    T_target = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z + 0.05,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
    joints = piper_arm.inverse_kinematics(T_target)
    if joints is None:
        print("  ❌ IK求解失败")
        return
    control_arm(joints, speed=FAST_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(1.0)
    
    # 2. 张开夹爪
    print("\n📍 步骤2: 张开夹爪")
    piper.GripperCtrl(KNOB_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 3. 插入
    print(f"\n📍 步骤3: 插入深度={KNOB_INSERT_DEPTH}m")
    current = get_current_joints()
    current = move_along_end_effector_z(current, KNOB_INSERT_DEPTH, KNOB_INSERT_SPEED)
    time.sleep(0.5)
    
    # 4. 夹持
    print(f"\n📍 步骤4: 夹持（宽度={KNOB_GRIPPER_HOLD}）")
    piper.GripperCtrl(KNOB_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 5. 旋转
    direction_sign = -1 if KNOB_ROTATION_DIRECTION == 'ccw' else 1
    print(f"\n📍 步骤5: 旋转（方向={KNOB_ROTATION_DIRECTION}, 角度={KNOB_ROTATION_ANGLE}°）")
    current = get_current_joints()
    current[5] += direction_sign * KNOB_ROTATION_ANGLE * PI / 180.0
    control_arm(current, speed=KNOB_ROTATION_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(0.5)
    
    # 6. 松开夹爪
    print("\n📍 步骤6: 松开夹爪")
    piper.GripperCtrl(KNOB_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 7. 退出
    print("\n📍 步骤7: 退出")
    current = get_current_joints()
    new_joints = move_along_end_effector_z(current, -KNOB_INSERT_DEPTH, KNOB_INSERT_SPEED)
    if new_joints is not None:
        current = new_joints
    time.sleep(0.5)
    
    # 8. 复位joint6
    print("\n📍 步骤8: 复位joint6")
    current = get_current_joints()
    current[5] -= direction_sign * KNOB_ROTATION_ANGLE * PI / 180.0
    control_arm(current, speed=KNOB_ROTATION_SPEED, use_moveit=USE_MOVEIT)
    time.sleep(0.5)
    
    print("\n✅ Knob操作完成")
    save_and_visualize_trajectory()

# ========================================
# 主程序
# ========================================
def main():
    global piper, piper_arm, moveit_node
    
    print("\n" + "="*70)
    print("🤖 Piper按钮操作控制器 - ROS2版本")
    print("="*70)
    
    # 1. 初始化硬件
    print("\n初始化硬件...")
    piper = C_PiperInterface()
    piper_arm = PiperArm()
    piper.ConnectPort()
    time.sleep(0.1)
    
    piper.EnableArm(7, 0x01)  # 0x01=启用, 0x02=禁用
    time.sleep(0.1)
    print("  ✓ 机械臂已使能")
    
    # 2. 初始化ROS2
    if MOVEIT_AVAILABLE:
        print("\n初始化ROS2...")
        rclpy.init()
        
        moveit_node = Node('button_action_controller')
        print("  ✓ ROS2节点已创建 (button_action_controller)")
        
        # 启动joint_states发布器
        setup_joint_state_publisher()
        
        # 启动后台spin线程
        setup_ros2_spin_thread()
        
        # 初始化MoveIt2
        setup_moveit2()
        
        time.sleep(1.0)
    
    # 3. 显示配置
    print("\n当前配置:")
    print(f"  目标位置: X={TARGET_X:.3f}, Y={TARGET_Y:.3f}, Z={TARGET_Z:.3f}")
    print(f"  目标姿态: Roll={TARGET_ROLL:.3f}, Pitch={TARGET_PITCH:.3f}, Yaw={TARGET_YAW:.3f}")
    print(f"  动作类型: {ACTION_TYPE}")
    print(f"  控制模式: {'MoveIt2' if USE_MOVEIT and MOVEIT_AVAILABLE else 'SDK'}")
    
    # 4. 执行动作
    try:
        if ACTION_TYPE == 'toggle':
            action_toggle()
        elif ACTION_TYPE == 'plugin':
            action_plugin()
        elif ACTION_TYPE == 'push':
            action_push()
        elif ACTION_TYPE == 'knob':
            action_knob()
        else:
            print(f"  ⚠️  未知的动作类型: {ACTION_TYPE}")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断")
    except Exception as e:
        print(f"\n\n❌ 执行异常: {e}")
        traceback.print_exc()
    finally:
        # 5. 清理
        print("\n清理资源...")
        if MOVEIT_AVAILABLE and ROS_AVAILABLE:
            rclpy.shutdown()
            print("  ✓ ROS2已关闭")
        
        print("\n" + "="*70)
        print("程序结束")
        print("="*70)

if __name__ == "__main__":
    main()
