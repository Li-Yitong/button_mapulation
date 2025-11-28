#!/usr/bin/env python3
"""
按钮操作执行器 - 独立版本
支持四种按钮操作类型：Toggle, Plug-in, Push, Knob
所有参数通过宏定义配置，无需视觉检测
"""
from piper_sdk import *
import rospy
import time
import numpy as np
import math
from piper_arm import PiperArm

# ========================================
# 宏定义 - 用户配置区
# ========================================
# ========================================
# 全局常量
# ========================================
PI = math.pi
factor = 1000 * 180 / PI

# === 预设零位配置 (基座坐标系) ===
# 这是机械臂的"家位置"（home position），所有动作开始和结束都会回到这个位置
# 单位：度（°）
# PRESET_ZERO_J1 = 1.411      # Joint 1
# PRESET_ZERO_J2 = 8.211      # Joint 2
# PRESET_ZERO_J3 = -39.652    # Joint 3
# PRESET_ZERO_J4 = -8.139     # Joint 4
# PRESET_ZERO_J5 = 30.782     # Joint 5
# PRESET_ZERO_J6 = 6.237      # Joint 6

PRESET_ZERO_J1 = 0     # Joint 1
PRESET_ZERO_J2 = 0    # Joint 2
PRESET_ZERO_J3 = 0   # Joint 3
PRESET_ZERO_J4 = 0    # Joint 4
PRESET_ZERO_J5 = 0    # Joint 5
PRESET_ZERO_J6 = 0    # Joint 6

# 转换为弧度（内部使用）
PRESET_ZERO_JOINTS = [
    PRESET_ZERO_J1 * PI / 180,
    PRESET_ZERO_J2 * PI / 180,
    PRESET_ZERO_J3 * PI / 180,
    PRESET_ZERO_J4 * PI / 180,
    PRESET_ZERO_J5 * PI / 180,
    PRESET_ZERO_J6 * PI / 180,
]

# === 目标位姿配置 (基座坐标系) ===
# 位置 (单位：米)
TARGET_X = 0.26  # X坐标 (降低以保证可达性)
TARGET_Y = 0.00  # Y坐标
TARGET_Z = 0.25  # Z坐标 (提高以保证可达性)

# 姿态 (单位：弧度) - 相对于默认姿态（末端朝前）的旋转
# 注意：Roll=Pitch=Yaw=0 表示默认姿态（末端朝前），这是一个可达的姿态
TARGET_ROLL = 0.0    # 绕末端X轴旋转 (翻滚) - 正值：向右倾斜 [建议范围: -0.5~0.5 rad]
TARGET_PITCH = 0.0   # 绕末端Y轴旋转 (俯仰) - 正值：向上抬起 [建议范围: -0.3~0.3 rad]
TARGET_YAW = 0.0     # 绕末端Z轴旋转 (偏航) - 正值：逆时针旋转 [建议范围: -1.0~1.0 rad]

# 姿态模式选择
USE_6D_POSE = True   # True=使用6D位姿(含姿态), False=仅使用位置(末端朝前)

# 注意：IK精度说明
# 由于piper_arm.py的IK算法存在精度限制，实际到达的位置可能与目标位置有几厘米的偏差。
# 这是正常现象，不影响按钮操作的执行。如果需要更高精度，请考虑使用MoveIt的笛卡尔路径规划。

# === 动作类型选择 ===
ACTION_TYPE = 'push'  # 'toggle'/'plugin'/'push'/'knob'

# === 控制模式 ===
USE_MOVEIT = True  # 启动脚本自动设置

# === Plugin (插拔连接器) 配置 ===
PLUGIN_GRIPPER_OPEN = 60000     # 张开宽度 (单位: 0.001mm, 范围: 0~70000, 即0~70mm)
PLUGIN_INSERT_DEPTH = 0.03      # 插入深度 (单位: 米, 范围: -0.1~0.1, 建议: 0.01~0.05)
PLUGIN_GRIPPER_HOLD = 500     # 闭合夹持宽度 (单位: 0.001mm, 范围: 0~70000, 建议: 20000~40000)
PLUGIN_INSERT_SPEED = 100       # 插入速度 (单位: 无量纲, 范围: 0~100)
PLUGIN_EXTRACT_SPEED = 100      # 拔出速度 (单位: 无量纲, 范围: 0~100)

# === Toggle (拨动开关) 配置 ===
TOGGLE_GRIPPER_OPEN = 70000     # 张开宽度 (单位: 0.001mm, 范围: 0~70000, 即0~70mm)
TOGGLE_JOINT4_ROTATE = 90       # joint4旋转角度 (单位: 度, 范围: -180~180, 建议: -90~90)
TOGGLE_INSERT_DEPTH = 0.03      # 插入深度 (单位: 米, 范围: -0.1~0.1, 建议: 0.01~0.05)
TOGGLE_GRIPPER_HOLD = 30000     # 闭合夹持宽度 (单位: 0.001mm, 范围: 0~70000, 建议: 20000~40000)
TOGGLE_JOINT3_ANGLE = 30        # joint3拨动角度 (单位: 度, 范围: -180~180, 建议: 10~45)
TOGGLE_DIRECTION = 'left'       # 拨动方向: 'left'(左拨) / 'right'(右拨)
TOGGLE_INSERT_SPEED = 20        # 插入速度 (单位: 无量纲, 范围: 0~100)
TOGGLE_TOGGLE_SPEED = 30        # 拨动速度 (单位: 无量纲, 范围: 0~100)

# === Push (按压按钮) 配置 ===
PUSH_GRIPPER_CLOSE = 0          # 夹爪闭合值 (单位: 0.001mm, 范围: 0~70000, 0=完全闭合)
PUSH_INSERT_DEPTH = 0.003        # 按压深度 (单位: 米, 范围: -0.1~0.1, 建议: 0.01~0.05)
PUSH_HOLD_TIME = 0.01              # 保持时间 (单位: 秒, 范围: 0~无限, 建议: 1~5)
PUSH_PRESS_SPEED = 30           # 按压速度 (单位: 无量纲, 范围: 0~100, 建议: 20~50慢速按压)

# === Knob (旋转旋钮) 配置 ===
KNOB_GRIPPER_OPEN = 35000       # 张开宽度 (单位: 0.001mm, 范围: 0~70000, 即0~70mm)
KNOB_INSERT_DEPTH = 0.0015        # 插入深度 (单位: 米, 范围: -0.1~0.1, 建议: 0.005~0.02)
KNOB_GRIPPER_HOLD = 8000       # 闭合夹持宽度 (单位: 0.001mm, 范围: 0~70000, 建议: 15000~35000)
KNOB_ROTATION_ANGLE = 45        # 旋转角度 (单位: 度, 范围: -360~360, 建议: 30~180)
KNOB_ROTATION_DIRECTION = 'ccw'  # 旋转方向: 'cw'=顺时针(右旋), 'ccw'=逆时针(左旋)
KNOB_INSERT_SPEED = 80          # 插入速度 (单位: 无量纲, 范围: 0~100)
KNOB_ROTATION_SPEED = 60        # 旋转速度 (单位: 无量纲, 范围: 0~100)

# === 通用速度配置 ===
NORMAL_SPEED = 100              # 正常移动速度 (单位: 无量纲, 范围: 0~100, SDK硬限制)
FAST_SPEED = 100                # 快速移动速度 (单位: 无量纲, 范围: 0~100, SDK硬限制)



# ========================================
# MoveIt 配置 (可选)
# ========================================
# 轨迹执行频率控制
RVIZ_PUBLISH_RATE = 10          # 轨迹发布到RViz的频率 (Hz)
COMMAND_SEND_RATE = 80          # 命令发送频率 (Hz) - 在轨迹点之间持续发送命令
PLANNER_ID = "BKPIECE"       # 可选: "RRTstar", "PRM", "BKPIECE", "EST"

# 调试配置
DEBUG_TRAJECTORY = False        # 是否显示详细的轨迹调试信息（关闭以提高速度）

# 尾迹可视化配置
MAX_TRAIL_POINTS = 100          # 最大尾迹点数

MOVEIT_AVAILABLE = False
move_group = None
try:
    if USE_MOVEIT:
        import moveit_commander
        from moveit_msgs.msg import DisplayTrajectory
        from nav_msgs.msg import Path
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point, PoseStamped
        from std_msgs.msg import ColorRGBA
        MOVEIT_AVAILABLE = True
        print("✓ MoveIt已加载")
except ImportError:
    print("⚠️  MoveIt未加载，将使用SDK模式")

# 全局变量
piper = None
piper_arm = None
display_trajectory_publisher = None
ee_path_publisher = None
ee_trail_publisher = None

# 轨迹记录（用于显示尾迹）
ee_trail_points = []

# 轨迹记录（规划 vs 执行）
planned_trajectory = []      # MoveIt规划的轨迹（末端XYZ）
executed_trajectory = []     # 实际执行的轨迹（末端XYZ）
trajectory_save_dir = "trajectory"  # 轨迹保存目录


# ========================================
# 轨迹可视化和保存函数
# ========================================

def ensure_trajectory_dir():
    """确保轨迹保存目录存在"""
    import os
    if not os.path.exists(trajectory_save_dir):
        os.makedirs(trajectory_save_dir)
        print(f"  ✓ 创建轨迹保存目录: {trajectory_save_dir}/")


def save_trajectory_to_csv(traj_points, filename_prefix):
    """
    保存轨迹详细信息到CSV文件
    
    Args:
        traj_points: MoveIt生成的轨迹点列表
        filename_prefix: 文件名前缀 (如 "planned" 或 "executed")
    """
    import csv
    from datetime import datetime
    
    ensure_trajectory_dir()
    
    # 生成带时间戳的文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{trajectory_save_dir}/{filename_prefix}_trajectory_{timestamp}.csv"
    
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # 写入表头
            header = ['点号', '时间(s)', '时间间隔(ms)']
            header += [f'关节{i+1}(°)' for i in range(6)]
            header += [f'速度{i+1}(°/s)' for i in range(6)]
            header += [f'加速度{i+1}(°/s²)' for i in range(6)]
            header += ['末端X(m)', '末端Y(m)', '末端Z(m)']
            writer.writerow(header)
            
            # 写入数据
            prev_time = 0.0
            for idx, point in enumerate(traj_points):
                row = []
                
                # 点号
                row.append(idx)
                
                # 时间
                time = point.time_from_start.to_sec()
                row.append(f"{time:.4f}")
                
                # 时间间隔
                dt = (time - prev_time) * 1000  # 转换为毫秒
                row.append(f"{dt:.2f}")
                prev_time = time
                
                # 关节角度（转换为度）
                joints = [point.positions[i] * 180 / PI for i in range(6)]
                row.extend([f"{j:.2f}" for j in joints])
                
                # 关节速度（转换为度/秒）
                if len(point.velocities) >= 6:
                    velocities = [point.velocities[i] * 180 / PI for i in range(6)]
                    row.extend([f"{v:.2f}" for v in velocities])
                else:
                    row.extend(['0.00'] * 6)
                
                # 关节加速度（转换为度/秒²）
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


def save_execution_trajectory_to_csv(executed_points, filename_prefix="executed"):
    """
    保存实际执行轨迹到CSV文件
    
    Args:
        executed_points: 实际执行的轨迹点列表 [(time, joints, xyz), ...]
        filename_prefix: 文件名前缀
    """
    import csv
    from datetime import datetime
    
    ensure_trajectory_dir()
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{trajectory_save_dir}/{filename_prefix}_trajectory_{timestamp}.csv"
    
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # 写入表头
            header = ['点号', '时间(s)', '时间间隔(ms)']
            header += [f'关节{i+1}(°)' for i in range(6)]
            header += ['末端X(m)', '末端Y(m)', '末端Z(m)']
            writer.writerow(header)
            
            # 写入数据
            prev_time = 0.0
            for idx, (time, joints, xyz) in enumerate(executed_points):
                row = []
                
                row.append(idx)
                row.append(f"{time:.4f}")
                
                dt = (time - prev_time) * 1000
                row.append(f"{dt:.2f}")
                prev_time = time
                
                # 关节角度（转换为度）
                joints_deg = [joints[i] * 180 / PI for i in range(6)]
                row.extend([f"{j:.2f}" for j in joints_deg])
                
                # 末端位置
                row.extend([f"{xyz[0]:.6f}", f"{xyz[1]:.6f}", f"{xyz[2]:.6f}"])
                
                writer.writerow(row)
        
        print(f"  ✓ 执行轨迹已保存: {filename}")
        return filename
    except Exception as e:
        print(f"  ⚠️  保存执行轨迹失败: {e}")
        return None


def publish_dual_trajectory_markers(planned_xyz, executed_xyz):
    """
    在RViz中发布规划路径和执行路径的对比可视化
    
    Args:
        planned_xyz: 规划的末端XYZ轨迹 (N×3 array)
        executed_xyz: 执行的末端XYZ轨迹 (M×3 array)
    """
    if not MOVEIT_AVAILABLE or len(planned_xyz) == 0:
        return
    
    marker_pub = rospy.Publisher('/trajectory_comparison', Marker, queue_size=10)
    rospy.sleep(0.1)
    
    # 发布规划路径（蓝色线）
    planned_marker = Marker()
    planned_marker.header.frame_id = "arm_base"
    planned_marker.header.stamp = rospy.Time.now()
    planned_marker.ns = "planned_trajectory"
    planned_marker.id = 0
    planned_marker.type = Marker.LINE_STRIP
    planned_marker.action = Marker.ADD
    planned_marker.scale.x = 0.005  # 线宽 5mm
    planned_marker.color.r = 0.0
    planned_marker.color.g = 0.5
    planned_marker.color.b = 1.0
    planned_marker.color.a = 0.8
    planned_marker.pose.orientation.w = 1.0
    
    for xyz in planned_xyz:
        p = Point()
        p.x = xyz[0]
        p.y = xyz[1]
        p.z = xyz[2]
        planned_marker.points.append(p)
    
    # 发布执行路径（红色线）
    executed_marker = Marker()
    executed_marker.header.frame_id = "arm_base"
    executed_marker.header.stamp = rospy.Time.now()
    executed_marker.ns = "executed_trajectory"
    executed_marker.id = 1
    executed_marker.type = Marker.LINE_STRIP
    executed_marker.action = Marker.ADD
    executed_marker.scale.x = 0.003  # 线宽 3mm
    executed_marker.color.r = 1.0
    executed_marker.color.g = 0.0
    executed_marker.color.b = 0.0
    executed_marker.color.a = 0.9
    executed_marker.pose.orientation.w = 1.0
    
    if len(executed_xyz) > 0:
        for xyz in executed_xyz:
            p = Point()
            p.x = xyz[0]
            p.y = xyz[1]
            p.z = xyz[2]
            executed_marker.points.append(p)
    
    # 发布标记
    for _ in range(3):
        marker_pub.publish(planned_marker)
        marker_pub.publish(executed_marker)
        rospy.sleep(0.1)
    
    print(f"  ✓ 轨迹对比已发布到 RViz (/trajectory_comparison)")
    print(f"    🔵 蓝色 = 规划路径 ({len(planned_xyz)}个点)")
    if len(executed_xyz) > 0:
        print(f"    🔴 红色 = 执行路径 ({len(executed_xyz)}个点)")


def plot_trajectory_comparison(planned_xyz, executed_xyz, planned_times, executed_times):
    """
    使用Matplotlib绘制规划路径vs执行路径对比图
    
    Args:
        planned_xyz: 规划的末端XYZ轨迹 (N×3 array)
        executed_xyz: 执行的末端XYZ轨迹 (M×3 array)
        planned_times: 规划的时间序列 (N array)
        executed_times: 执行的时间序列 (M array)
    """
    try:
        import matplotlib
        matplotlib.use('Agg')  # 非交互式后端
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from datetime import datetime
        
        # 设置中文字体支持 - 使用系统已安装的字体
        # 优先级：Noto Sans CJK > AR PL UMing > AR PL UKai
        chinese_fonts = ['Noto Sans CJK JP', 'AR PL UMing CN', 'AR PL UKai CN', 'DejaVu Sans']
        plt.rcParams['font.sans-serif'] = chinese_fonts
        plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    except ImportError:
        print("  ⚠️  matplotlib 未安装，跳过绘图")
        return None
    
    ensure_trajectory_dir()
    
    planned_xyz = np.array(planned_xyz)
    executed_xyz = np.array(executed_xyz) if len(executed_xyz) > 0 else np.array([])
    
    fig = plt.figure(figsize=(16, 10))
    
    # 子图1: 3D轨迹对比
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(planned_xyz[:, 0], planned_xyz[:, 1], planned_xyz[:, 2], 
             'b-', linewidth=2, label='Planned Path', alpha=0.7)
    if len(executed_xyz) > 0:
        ax1.plot(executed_xyz[:, 0], executed_xyz[:, 1], executed_xyz[:, 2], 
                 'r-', linewidth=2, label='Executed Path', alpha=0.7)
    ax1.scatter(planned_xyz[0, 0], planned_xyz[0, 1], planned_xyz[0, 2], 
                c='g', s=100, marker='o', label='Start')
    ax1.scatter(planned_xyz[-1, 0], planned_xyz[-1, 1], planned_xyz[-1, 2], 
                c='orange', s=100, marker='s', label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('End-Effector 3D Trajectory')
    ax1.legend()
    
    # 子图2: X坐标随时间变化
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(planned_times, planned_xyz[:, 0], 'b-', linewidth=2, label='Planned', alpha=0.7)
    if len(executed_xyz) > 0:
        ax2.plot(executed_times, executed_xyz[:, 0], 'r--', linewidth=2, label='Executed', alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('X (m)')
    ax2.set_title('X Coordinate vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # 子图3: Y坐标随时间变化
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(planned_times, planned_xyz[:, 1], 'b-', linewidth=2, label='Planned', alpha=0.7)
    if len(executed_xyz) > 0:
        ax3.plot(executed_times, executed_xyz[:, 1], 'r--', linewidth=2, label='Executed', alpha=0.7)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Y (m)')
    ax3.set_title('Y Coordinate vs Time')
    ax3.legend()
    ax3.grid(True)
    
    # 子图4: Z坐标随时间变化
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(planned_times, planned_xyz[:, 2], 'b-', linewidth=2, label='Planned', alpha=0.7)
    if len(executed_xyz) > 0:
        ax4.plot(executed_times, executed_xyz[:, 2], 'r--', linewidth=2, label='Executed', alpha=0.7)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Z (m)')
    ax4.set_title('Z Coordinate vs Time')
    ax4.legend()
    ax4.grid(True)
    
    # 子图5: XY平面投影
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
    
    # 子图6: 轨迹误差（如果有执行数据）
    ax6 = fig.add_subplot(2, 3, 6)
    if len(executed_xyz) > 0 and len(executed_xyz) == len(planned_xyz):
        errors = np.linalg.norm(executed_xyz - planned_xyz, axis=1) * 100  # 转换为cm
        ax6.plot(planned_times, errors, 'r-', linewidth=2)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Error (cm)')
        ax6.set_title(f'Tracking Error (Avg: {np.mean(errors):.2f}cm)')
        ax6.grid(True)
    else:
        ax6.text(0.5, 0.5, f'Planned: {len(planned_xyz)} pts\nExecuted: {len(executed_xyz)} pts', 
                 ha='center', va='center', fontsize=12)
        ax6.set_title('Trajectory Point Statistics')
    
    plt.tight_layout()
    
    # 保存图表
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{trajectory_save_dir}/trajectory_comparison_{timestamp}.png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"  ✓ 轨迹对比图已保存: {filename}")
    
    plt.close()
    return filename


# ========================================
# 控制函数
# ========================================

def update_ee_trail(joints):
    """
    更新末端执行器轨迹（尾迹可视化）
    
    Args:
        joints: 当前关节角度列表
    """
    global piper_arm, ee_trail_points, ee_path_publisher, ee_trail_publisher
    
    if not MOVEIT_AVAILABLE or piper_arm is None:
        return
    
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
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2]
        marker.points.append(p)
        
        # 渐变颜色（从蓝色渐变到红色）
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


def clear_trajectory_records():
    """清空轨迹记录（在新的动作序列开始前调用）"""
    global planned_trajectory, executed_trajectory
    planned_trajectory = []
    executed_trajectory = []
    print("  ✓ 已清空轨迹记录")


def save_and_visualize_trajectory():
    """
    保存并可视化完整的轨迹记录（在动作序列结束后调用）
    """
    global planned_trajectory, executed_trajectory
    
    if len(planned_trajectory) == 0:
        print("  ⚠️  没有规划轨迹记录")
        return
    
    print("\n" + "="*70)
    print("📊 保存和可视化轨迹...")
    print("="*70)
    print(f"  📍 规划轨迹点数: {len(planned_trajectory)}")
    print(f"  📍 执行轨迹点数: {len(executed_trajectory)}")
    
    # 1. 保存规划轨迹到CSV（简化版：只有XYZ）
    if DEBUG_TRAJECTORY:
        from datetime import datetime
        import csv
        
        ensure_trajectory_dir()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 保存规划轨迹
        planned_file = f"{trajectory_save_dir}/planned_trajectory_{timestamp}.csv"
        with open(planned_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['点号', 'X (m)', 'Y (m)', 'Z (m)'])
            for i, xyz in enumerate(planned_trajectory):
                writer.writerow([i+1, f"{xyz[0]:.6f}", f"{xyz[1]:.6f}", f"{xyz[2]:.6f}"])
        print(f"  ✓ 规划轨迹已保存: {planned_file}")
        
        # 保存执行轨迹
        if len(executed_trajectory) > 0:
            executed_file = f"{trajectory_save_dir}/executed_trajectory_{timestamp}.csv"
            with open(executed_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['点号', 'X (m)', 'Y (m)', 'Z (m)'])
                for i, xyz in enumerate(executed_trajectory):
                    writer.writerow([i+1, f"{xyz[0]:.6f}", f"{xyz[1]:.6f}", f"{xyz[2]:.6f}"])
            print(f"  ✓ 执行轨迹已保存: {executed_file}")
    
    # 2. 发布轨迹对比到RViz
    if len(executed_trajectory) > 0:
        publish_dual_trajectory_markers(planned_trajectory, executed_trajectory)
    
    # 3. 绘制Matplotlib对比图
    if DEBUG_TRAJECTORY and len(executed_trajectory) > 0:
        # 生成时间序列（简化：假设均匀采样）
        planned_times = np.linspace(0, len(planned_trajectory)*0.1, len(planned_trajectory))
        executed_times = np.linspace(0, len(executed_trajectory)*0.0625, len(executed_trajectory))
        plot_trajectory_comparison(planned_trajectory, executed_trajectory, 
                                 planned_times, executed_times)
    
    print("="*70)


def control_arm_sdk(joints, speed=50, gripper_value=None):
    """SDK 直接控制模式"""
    global piper
    
    joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
    joints_int[4] = max(-70000, joints_int[4])
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joints_int)
    
    if gripper_value is not None:
        gripper_int = int(gripper_value)
        piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    elif len(joints) > 6:
        gripper_int = int(joints[6] * 1000000)
        piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    
    return True


def control_arm_moveit(joints, speed=50, gripper_value=None):
    """MoveIt 规划控制模式"""
    global piper, move_group, display_trajectory_publisher
    global planned_trajectory, executed_trajectory
    
    if move_group is None:
        return control_arm_sdk(joints, speed, gripper_value)
    
    try:
        # 获取当前关节角度（起始点）
        current_joints = get_current_joints()
        
        move_group.clear_pose_targets()
        move_group.stop()
        
        # 【关键】设置起始状态为当前实际位置（在clear之后）
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        robot_state = RobotState()
        robot_state.joint_state = JointState()
        robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        robot_state.joint_state.position = list(current_joints)
        move_group.set_start_state(robot_state)
        
        target_joints = joints[:6] if len(joints) > 6 else joints
        move_group.set_joint_value_target(target_joints)
        
        # MoveIt 规划
        print("  [MoveIt] 规划轨迹...")
        print(f"  📍 起始点 (弧度): [{', '.join([f'{j:.4f}' for j in current_joints])}]")
        print(f"  📍 目标点 (弧度): [{', '.join([f'{j:.4f}' for j in target_joints])}]")
        
        plan = move_group.plan()
        if isinstance(plan, tuple):
            success, trajectory = plan[0], plan[1]
        else:
            success, trajectory = True, plan
        
        if not success or not trajectory.joint_trajectory.points:
            print("  ❌ 规划失败，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        traj_points = trajectory.joint_trajectory.points
        print(f"  ✓ 规划成功 (轨迹点: {len(traj_points)})")
        print(f"  📊 轨迹点数由MoveIt根据路径长短、速度、加速度自动计算")
        
        # 提取规划的末端轨迹（XYZ）- 累积到全局变量
        global planned_trajectory
        step_planned = []
        for point in traj_points:
            joints_rad = [point.positions[i] for i in range(6)]
            T = piper_arm.forward_kinematics(joints_rad)
            xyz = T[:3, 3]
            step_planned.append(xyz.copy())
            planned_trajectory.append(xyz.copy())  # 累积到全局
        
        print(f"  ✓ 已提取规划轨迹的末端XYZ (本步骤: {len(step_planned)}个点, 累计: {len(planned_trajectory)}个点)")
        
        # 保存规划轨迹到CSV
        if DEBUG_TRAJECTORY:
            save_trajectory_to_csv(traj_points, "planned")
        
        # 打印轨迹详细信息（简化版：只显示XYZ）
        if DEBUG_TRAJECTORY:
            import tf.transformations as tft
            
            # 计算起点和终点的末端XYZ
            start_joints = current_joints
            start_T = piper_arm.forward_kinematics(start_joints)
            start_xyz = start_T[:3, 3]
            
            end_joints = [traj_points[-1].positions[i] for i in range(6)]
            end_T = piper_arm.forward_kinematics(end_joints)
            end_xyz = end_T[:3, 3]
            
            print(f"\n  起点 XYZ: [{start_xyz[0]:.4f}, {start_xyz[1]:.4f}, {start_xyz[2]:.4f}]")
            print(f"  终点 XYZ: [{end_xyz[0]:.4f}, {end_xyz[1]:.4f}, {end_xyz[2]:.4f}]")
            print(f"  轨迹点数: {len(traj_points)}, 总时长: {traj_points[-1].time_from_start.to_sec():.2f}s")
            
            # 只在点数较少时显示详细轨迹
            if len(traj_points) <= 20:
                print(f"  轨迹点详情 (XYZ):")
                for idx in [0, len(traj_points)//4, len(traj_points)//2, 3*len(traj_points)//4, len(traj_points)-1]:
                    if idx < len(traj_points):
                        point = traj_points[idx]
                        point_joints = [point.positions[i] for i in range(6)]
                        point_T = piper_arm.forward_kinematics(point_joints)
                        point_xyz = point_T[:3, 3]
                        print(f"    点#{idx}: [{point_xyz[0]:.4f}, {point_xyz[1]:.4f}, {point_xyz[2]:.4f}] @ {point.time_from_start.to_sec():.2f}s")
            else:
                print(f"  (轨迹点较多，仅显示起止点)")
            print()
        
        # 发布轨迹到RViz可视化（使用RVIZ_PUBLISH_RATE频率）
        if display_trajectory_publisher is not None and display_trajectory_publisher.get_num_connections() > 0:
            display_msg = DisplayTrajectory()
            display_msg.trajectory_start = move_group.get_current_state()
            display_msg.trajectory.append(trajectory)
            
            rviz_rate = rospy.Rate(RVIZ_PUBLISH_RATE)
            for _ in range(3):  # 发布3次确保RViz接收
                display_trajectory_publisher.publish(display_msg)
                rviz_rate.sleep()
            print(f"  ✓ 轨迹已发布到RViz (频率: {RVIZ_PUBLISH_RATE}Hz)")
        
        # SDK 执行完整轨迹（使用插值平滑执行）+ 记录实际轨迹
        print(f"  [SDK] 执行完整轨迹 (点数: {len(traj_points)}, 速度: {speed}, 发送频率: {COMMAND_SEND_RATE}Hz)")
        
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        
        if DEBUG_TRAJECTORY:
            print("\n  " + "="*70)
            print("  🚀 开始执行完整轨迹 (高频插值模式 + 记录实际轨迹):")
            print("  " + "="*70)
        
        start_time = rospy.Time.now()
        command_rate = rospy.Rate(COMMAND_SEND_RATE)
        
        # 清空执行轨迹记录
        executed_trajectory = []
        execution_records = []  # [(time, joints, xyz), ...]
        
        current_point_idx = 0
        next_point_idx = 1
        
        while next_point_idx < len(traj_points):
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            # 找到当前时间对应的轨迹段
            while next_point_idx < len(traj_points) and elapsed >= traj_points[next_point_idx].time_from_start.to_sec():
                current_point_idx = next_point_idx
                next_point_idx += 1
            
            if next_point_idx >= len(traj_points):
                break
            
            # 获取当前段的两个端点
            point_current = traj_points[current_point_idx]
            point_next = traj_points[next_point_idx]
            
            # 计算插值比例
            t_current = point_current.time_from_start.to_sec()
            t_next = point_next.time_from_start.to_sec()
            
            if t_next > t_current:
                ratio = (elapsed - t_current) / (t_next - t_current)
                ratio = max(0.0, min(1.0, ratio))  # 限制在[0,1]
            else:
                ratio = 1.0
            
            # 线性插值计算当前应该发送的关节角度
            joints_interpolated = []
            for i in range(6):
                pos_current = point_current.positions[i]
                pos_next = point_next.positions[i]
                pos_interp = pos_current + ratio * (pos_next - pos_current)
                joints_interpolated.append(pos_interp)
            
            # 发送插值后的关节命令
            joints_int = [int(joints_interpolated[i] * factor) for i in range(6)]
            joints_int[4] = max(-70000, joints_int[4])
            piper.JointCtrl(*joints_int)
            
            # 记录实际执行的轨迹（每N个周期记录一次）
            if len(execution_records) == 0 or int(elapsed * COMMAND_SEND_RATE) % 5 == 0:
                T = piper_arm.forward_kinematics(joints_interpolated)
                xyz = T[:3, 3]
                execution_records.append((elapsed, joints_interpolated.copy(), xyz.copy()))
                executed_trajectory.append(xyz.copy())
            
            # 更新末端执行器轨迹（降低更新频率以减少计算）
            if int(elapsed * COMMAND_SEND_RATE) % 5 == 0:  # 每5个周期更新一次
                update_ee_trail(joints_interpolated)
            
            # 打印执行信息（每10个点打印一次）
            if DEBUG_TRAJECTORY and current_point_idx % 10 == 0 and int(elapsed * 100) % 50 == 0:
                print(f"  执行点 #{current_point_idx}/{len(traj_points)-1} | 已用时: {elapsed:.3f}s | 插值比例: {ratio:.2f}")
            
            # 按照固定频率发送命令
            command_rate.sleep()
        
        # 发送最终位置
        final_point = traj_points[-1]
        joints_int = [int(final_point.positions[i] * factor) for i in range(6)]
        joints_int[4] = max(-70000, joints_int[4])
        piper.JointCtrl(*joints_int)
        
        final_joints_rad = [final_point.positions[i] for i in range(6)]
        T_final = piper_arm.forward_kinematics(final_joints_rad)
        xyz_final = T_final[:3, 3]
        
        elapsed_final = (rospy.Time.now() - start_time).to_sec()
        execution_records.append((elapsed_final, final_joints_rad, xyz_final.copy()))
        executed_trajectory.append(xyz_final.copy())
        
        update_ee_trail(final_joints_rad)
        
        total_exec_time = (rospy.Time.now() - start_time).to_sec()
        if DEBUG_TRAJECTORY:
            print(f"\n  ✓ 轨迹命令发送完成，实际用时: {total_exec_time:.3f}s")
            print(f"  ✓ 记录了 {len(execution_records)} 个实际执行点")
            print("  " + "="*70 + "\n")
        else:
            print(f"  ✓ 轨迹命令发送完成 (用时: {total_exec_time:.3f}s)")
        
        # 等待机械臂真正到达目标位置
        print("  ⏳ 等待机械臂到达目标位置...")
        target_reached = False
        wait_start = rospy.Time.now()
        max_wait_time = 3.0  # 最多等待3秒
        position_threshold = 0.01  # 位置误差阈值 (弧度，约0.57度)
        
        while not target_reached and (rospy.Time.now() - wait_start).to_sec() < max_wait_time:
            current_joints_actual = get_current_joints()
            
            # 计算与目标位置的误差
            max_error = max([abs(current_joints_actual[i] - final_joints_rad[i]) for i in range(6)])
            
            if max_error < position_threshold:
                target_reached = True
                print(f"  ✓ 机械臂已到达目标位置 (最大误差: {max_error:.5f} rad)")
            else:
                rospy.sleep(0.05)  # 等待50ms后再检查
        
        if not target_reached:
            print(f"  ⚠️  等待超时，当前最大误差: {max_error:.5f} rad")
        
        # 额外等待一小段时间确保稳定
        rospy.sleep(0.01)
        
        # 控制夹爪
        if gripper_value is not None:
            gripper_int = int(gripper_value)
            piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
        
        print(f"  ✓ 执行完成")
        return True
    except Exception as e:
        print(f"  ❌ MoveIt执行失败: {e}，切换到SDK模式")
        import traceback
        traceback.print_exc()
        return control_arm_sdk(joints, speed, gripper_value)


def control_arm(joints, speed=50, use_moveit=False, gripper_value=None):
    """统一控制接口"""
    if gripper_value is None:
        gripper_value = joints[6] * 1000000 if len(joints) > 6 else None
    
    if use_moveit and MOVEIT_AVAILABLE and move_group is not None:
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
    """
    欧拉角转旋转矩阵 (ZYX顺序)
    
    参数:
        roll: 绕X轴旋转 (弧度)
        pitch: 绕Y轴旋转 (弧度)
        yaw: 绕Z轴旋转 (弧度)
    
    返回:
        3x3 旋转矩阵
    """
    # 绕X轴旋转
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # 绕Y轴旋转
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # 绕Z轴旋转
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # ZYX顺序: R = Rz * Ry * Rx
    return Rz @ Ry @ Rx


def create_target_transform(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, use_6d=False):
    """
    创建目标位姿变换矩阵
    
    参数:
        x, y, z: 位置 (米)
        roll, pitch, yaw: 姿态 (弧度) - 相对于默认姿态的旋转
        use_6d: 是否使用6D位姿
    
    返回:
        4x4 齐次变换矩阵
    
    说明:
        - 当 use_6d=False 时: 使用默认姿态（末端朝前）
        - 当 use_6d=True 时:
          - Roll=Pitch=Yaw=0 表示默认姿态（末端朝前）
          - Roll/Pitch/Yaw 是在默认姿态基础上的相对旋转（末端坐标系）
    """
    T = np.eye(4)
    
    if use_6d:
        # 默认姿态（末端朝前）
        R_default = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        
        # 相对旋转矩阵（末端坐标系下）
        R_relative = euler_to_rotation_matrix(roll, pitch, yaw)
        
        # 最终姿态 = 默认姿态 × 相对旋转
        T[:3, :3] = R_default @ R_relative
    else:
        # 默认姿态：末端朝前
        T[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    
    return T


def move_along_end_effector_z(current_joints, distance, speed=20):
    """
    沿末端执行器z轴方向移动（保持当前姿态）
    使用MoveIt笛卡尔路径规划以提高可靠性
    
    参数:
        current_joints: 当前关节角度 (弧度)
        distance: 移动距离 (米)，正值=沿末端+Z轴方向，负值=沿末端-Z轴方向
        speed: 移动速度
    
    返回:
        新的关节角度
    
    说明:
        末端坐标系Z轴 = 旋转矩阵第3列
        直接沿末端Z轴方向移动，正值=+Z方向，负值=-Z方向
    """
    global piper_arm, move_group, piper
    
    # 获取当前末端位姿
    current_T = piper_arm.forward_kinematics(current_joints)
    print(f"  当前位置: ({current_T[0,3]:.3f}, {current_T[1,3]:.3f}, {current_T[2,3]:.3f})")
    
    # 沿末端z轴移动 - 末端坐标系的Z轴是旋转矩阵的第3列
    # 正值distance = 沿末端+Z轴方向移动（插入）
    # 注意：根据实际测试，不需要取反
    z_axis = current_T[:3, 2]
    print(f"  移动距离: {distance*100:.1f}cm，末端Z轴方向: ({z_axis[0]:.3f}, {z_axis[1]:.3f}, {z_axis[2]:.3f})")
    
    # 计算新的目标位置
    target_T = current_T.copy()
    target_T[:3, 3] += z_axis * distance
    print(f"  目标位置: ({target_T[0,3]:.3f}, {target_T[1,3]:.3f}, {target_T[2,3]:.3f})")
    
    # 尝试使用MoveIt笛卡尔路径规划
    if USE_MOVEIT and MOVEIT_AVAILABLE and move_group is not None:
        try:
            from geometry_msgs.msg import Pose
            from moveit_msgs.msg import RobotState
            from sensor_msgs.msg import JointState
            import tf.transformations as tft
            
            # 【关键】强制设置MoveIt的起始状态为当前实际关节角度
            robot_state = RobotState()
            robot_state.joint_state = JointState()
            robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            robot_state.joint_state.position = list(current_joints)
            move_group.set_start_state(robot_state)
            print(f"  ✓ 已设置MoveIt起始状态为实际关节角度: [{', '.join([f'{j:.4f}' for j in current_joints])}]")
            
            # 创建目标位姿
            target_pose = Pose()
            target_pose.position.x = target_T[0, 3]
            target_pose.position.y = target_T[1, 3]
            target_pose.position.z = target_T[2, 3]
            
            # 从旋转矩阵转换为四元数
            quat = tft.quaternion_from_matrix(target_T)
            target_pose.orientation.x = quat[0]
            target_pose.orientation.y = quat[1]
            target_pose.orientation.z = quat[2]
            target_pose.orientation.w = quat[3]
            
            # 生成笛卡尔路径（多个中间点）
            waypoints = []
            num_steps = max(5, int(abs(distance) * 1))  # 每厘米至少5个点
            for i in range(num_steps + 1):
                fraction = i / num_steps
                intermediate_T = current_T.copy()
                # 沿末端Z轴方向移动
                intermediate_T[:3, 3] += z_axis * distance * fraction
                
                intermediate_pose = Pose()
                intermediate_pose.position.x = intermediate_T[0, 3]
                intermediate_pose.position.y = intermediate_T[1, 3]
                intermediate_pose.position.z = intermediate_T[2, 3]
                
                quat = tft.quaternion_from_matrix(intermediate_T)
                intermediate_pose.orientation.x = quat[0]
                intermediate_pose.orientation.y = quat[1]
                intermediate_pose.orientation.z = quat[2]
                intermediate_pose.orientation.w = quat[3]
                
                waypoints.append(intermediate_pose)
            
            print(f"  [MoveIt笛卡尔] 规划路径（{len(waypoints)}个路径点）...")
            
            # 计算笛卡尔路径
            # Python API: compute_cartesian_path(waypoints, eef_step, avoid_collisions)
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints,     # waypoints to follow (list of Pose objects)
                0.01,          # eef_step (1cm)
                True           # avoid_collisions
            )
            
            if fraction < 0.95:
                print(f"  ⚠️  笛卡尔路径规划覆盖率较低: {fraction*100:.1f}%，尝试简单IK...")
                raise Exception("低覆盖率")
            
            print(f"  ✓ 笛卡尔路径规划成功 (覆盖率: {fraction*100:.1f}%)")
            
            # 执行笛卡尔路径
            traj_points = plan.joint_trajectory.points
            if len(traj_points) == 0:
                raise Exception("轨迹为空")
            
            # 打印笛卡尔轨迹详细信息（简化版：只显示XYZ）
            if DEBUG_TRAJECTORY:
                import tf.transformations as tft
                
                # 计算当前末端位姿（起始点）
                current_T = piper_arm.forward_kinematics(current_joints)
                current_xyz = current_T[:3, 3]
                
                # 计算终点末端位姿
                end_joints = [traj_points[-1].positions[i] for i in range(6)]
                end_T = piper_arm.forward_kinematics(end_joints)
                end_xyz = end_T[:3, 3]
                
                # 计算位移
                delta_xyz = end_xyz - current_xyz
                
                print(f"\n  起点 XYZ: [{current_xyz[0]:.4f}, {current_xyz[1]:.4f}, {current_xyz[2]:.4f}]")
                print(f"  终点 XYZ: [{end_xyz[0]:.4f}, {end_xyz[1]:.4f}, {end_xyz[2]:.4f}]")
                print(f"  位移: ΔX={delta_xyz[0]*100:.2f}cm, ΔY={delta_xyz[1]*100:.2f}cm, ΔZ={delta_xyz[2]*100:.2f}cm")
                print(f"  轨迹点数: {len(traj_points)}, 总时长: {traj_points[-1].time_from_start.to_sec():.2f}s")
                
                # 显示关键点
                if len(traj_points) <= 20:
                    print(f"  轨迹点详情 (XYZ):")
                    for idx in [0, len(traj_points)//4, len(traj_points)//2, 3*len(traj_points)//4, len(traj_points)-1]:
                        if idx < len(traj_points):
                            point = traj_points[idx]
                            point_joints = [point.positions[i] for i in range(6)]
                            point_T = piper_arm.forward_kinematics(point_joints)
                            point_xyz = point_T[:3, 3]
                            print(f"    点#{idx}: [{point_xyz[0]:.4f}, {point_xyz[1]:.4f}, {point_xyz[2]:.4f}] @ {point.time_from_start.to_sec():.2f}s")
                else:
                    print(f"  (轨迹点较多，仅显示起止点)")
                print()
            
            # 发布轨迹到RViz（使用RVIZ_PUBLISH_RATE频率）
            if display_trajectory_publisher is not None and display_trajectory_publisher.get_num_connections() > 0:
                display_msg = DisplayTrajectory()
                display_msg.trajectory_start = move_group.get_current_state()
                display_msg.trajectory.append(plan)
                
                rviz_rate = rospy.Rate(RVIZ_PUBLISH_RATE)
                for _ in range(3):
                    display_trajectory_publisher.publish(display_msg)
                    rviz_rate.sleep()
                print(f"  ✓ 笛卡尔轨迹已发布到RViz (频率: {RVIZ_PUBLISH_RATE}Hz)")
            
            # 执行完整笛卡尔轨迹（使用插值平滑执行）
            print(f"  [SDK] 执行完整笛卡尔轨迹 (点数: {len(traj_points)}, 速度: {speed}, 发送频率: {COMMAND_SEND_RATE}Hz)")
            
            piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
            
            if DEBUG_TRAJECTORY:
                print("\n  " + "="*70)
                print("  🚀 开始执行笛卡尔轨迹 (高频插值模式):")
                print("  " + "="*70)
            
            start_time = rospy.Time.now()
            command_rate = rospy.Rate(COMMAND_SEND_RATE)
            
            current_point_idx = 0
            next_point_idx = 1
            
            while next_point_idx < len(traj_points):
                elapsed = (rospy.Time.now() - start_time).to_sec()
                
                # 找到当前时间对应的轨迹段
                while next_point_idx < len(traj_points) and elapsed >= traj_points[next_point_idx].time_from_start.to_sec():
                    current_point_idx = next_point_idx
                    next_point_idx += 1
                
                if next_point_idx >= len(traj_points):
                    break
                
                # 获取当前段的两个端点
                point_current = traj_points[current_point_idx]
                point_next = traj_points[next_point_idx]
                
                # 计算插值比例
                t_current = point_current.time_from_start.to_sec()
                t_next = point_next.time_from_start.to_sec()
                
                if t_next > t_current:
                    ratio = (elapsed - t_current) / (t_next - t_current)
                    ratio = max(0.0, min(1.0, ratio))
                else:
                    ratio = 1.0
                
                # 线性插值计算当前应该发送的关节角度
                joints_interpolated = []
                for i in range(6):
                    pos_current = point_current.positions[i]
                    pos_next = point_next.positions[i]
                    pos_interp = pos_current + ratio * (pos_next - pos_current)
                    joints_interpolated.append(pos_interp)
                
                # 发送插值后的关节命令
                joints_int = [int(joints_interpolated[i] * factor) for i in range(6)]
                joints_int[4] = max(-70000, joints_int[4])
                piper.JointCtrl(*joints_int)
                
                # 更新末端执行器轨迹
                if int(elapsed * COMMAND_SEND_RATE) % 5 == 0:
                    update_ee_trail(joints_interpolated)
                
                # 打印执行信息
                if DEBUG_TRAJECTORY and current_point_idx % 10 == 0 and int(elapsed * 100) % 50 == 0:
                    print(f"  执行点 #{current_point_idx}/{len(traj_points)-1} | 已用时: {elapsed:.3f}s | 插值比例: {ratio:.2f}")
                
                command_rate.sleep()
            
            # 发送最终位置
            final_point = traj_points[-1]
            joints_int = [int(final_point.positions[i] * factor) for i in range(6)]
            joints_int[4] = max(-70000, joints_int[4])
            piper.JointCtrl(*joints_int)
            
            final_joints = [final_point.positions[i] for i in range(6)]
            update_ee_trail(final_joints)
            
            total_exec_time = (rospy.Time.now() - start_time).to_sec()
            if DEBUG_TRAJECTORY:
                print(f"\n  ✓ 笛卡尔轨迹命令发送完成，实际用时: {total_exec_time:.3f}s")
                print("  " + "="*70 + "\n")
            else:
                print(f"  ✓ 笛卡尔轨迹命令发送完成 (用时: {total_exec_time:.3f}s)")
            
            # 等待机械臂真正到达目标位置
            print("  ⏳ 等待机械臂到达目标位置...")
            target_reached = False
            wait_start = rospy.Time.now()
            max_wait_time = 3.0
            position_threshold = 0.01
            
            final_joints_target = [traj_points[-1].positions[i] for i in range(6)]
            
            while not target_reached and (rospy.Time.now() - wait_start).to_sec() < max_wait_time:
                current_joints_actual = get_current_joints()
                max_error = max([abs(current_joints_actual[i] - final_joints_target[i]) for i in range(6)])
                
                if max_error < position_threshold:
                    target_reached = True
                    print(f"  ✓ 机械臂已到达目标位置 (最大误差: {max_error:.5f} rad)")
                else:
                    rospy.sleep(0.05)
            
            if not target_reached:
                print(f"  ⚠️  等待超时，当前最大误差: {max_error:.5f} rad")
            
            # 额外等待确保稳定
            rospy.sleep(0.2)
            
            return final_joints_target
            
        except Exception as e:
            print(f"  ⚠️  MoveIt笛卡尔规划失败: {e}，回退到简单IK...")
    
    # 回退方案：使用简单IK
    target_joints = piper_arm.inverse_kinematics(target_T)
    if not target_joints:
        print(f"  ❌ IK求解失败，目标位置可能不可达")
        return None
    
    print(f"  [简单IK] 执行运动...")
    if not control_arm(target_joints, speed, USE_MOVEIT):
        return None
    
    return target_joints


# ========================================
# 四种按钮操作函数（重写版）
# ========================================

def action_plugin():
    """
    插拔连接器操作
    流程: 张开 → 到达 → 插入(z轴前进) → 闭合 → 拔出(z轴后退) → 张开 → 回零闭合
    """
    global piper_arm
    
    # 清空轨迹记录
    clear_trajectory_records()
    clear_ee_trail()
    
    print("="*70)
    print("动作类型: Plugin (插拔连接器)")
    print("="*70)
    print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"插入深度: {PLUGIN_INSERT_DEPTH*100:.1f}cm")
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(PLUGIN_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    targetT = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
    
    joints_target = piper_arm.inverse_kinematics(targetT)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    if not control_arm(joints_target, NORMAL_SPEED, USE_MOVEIT, PLUGIN_GRIPPER_OPEN):
        return False
    time.sleep(0.1)
    
    # 步骤3: 沿末端z轴插入
    # 使用实际到达的关节角度，而不是IK计算的理论值
    print(f"\n步骤3: 沿末端z轴插入 {PLUGIN_INSERT_DEPTH*100:.1f}cm...")
    actual_joints = get_current_joints()  # 获取实际当前位置
    print(f"  使用实际关节角度作为起点")
    joints_insert = move_along_end_effector_z(actual_joints, PLUGIN_INSERT_DEPTH, PLUGIN_INSERT_SPEED)
    if not joints_insert:
        return False
    time.sleep(0.1)
    
    # 步骤4: 夹爪闭合
    print(f"\n步骤4: 夹爪闭合到 {PLUGIN_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(PLUGIN_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤5: 沿末端z轴拔出
    print(f"\n步骤5: 沿末端z轴拔出 {PLUGIN_INSERT_DEPTH*100:.1f}cm...")
    joints_extract = move_along_end_effector_z(joints_insert, -PLUGIN_INSERT_DEPTH, PLUGIN_EXTRACT_SPEED)
    if not joints_extract:
        return False
    time.sleep(0.1)
    
    # 步骤6: 夹爪张开
    print("\n步骤6: 夹爪张开...")
    piper.GripperCtrl(PLUGIN_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤7: 回预设零位
    print("\n步骤7: 回预设零位...")
    joints_zero = PRESET_ZERO_JOINTS.copy()
    if not control_arm(joints_zero, FAST_SPEED, USE_MOVEIT):
        return False
    time.sleep(0.1)
    
    # 步骤8: 夹爪闭合
    print("\n步骤8: 夹爪闭合...")
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 保存和可视化完整轨迹
    save_and_visualize_trajectory()
    
    print("="*70)
    print("✓✓✓ Plugin 操作完成！✓✓✓")
    print("="*70)
    return True


def action_toggle():
    """
    拨动开关操作
    流程: 张开 → 到达 → joint4旋转90° → 插入(z轴前进) → 闭合 → joint3拨动 → 张开 → 回零闭合
    """
    global piper_arm
    
    # 清空轨迹记录
    clear_trajectory_records()
    clear_ee_trail()
    
    print("="*70)
    print("动作类型: Toggle (拨动开关)")
    print("="*70)
    print(f"目标位姿: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"joint4旋转: {TOGGLE_JOINT4_ROTATE}°, 插入: {TOGGLE_INSERT_DEPTH*100:.1f}cm")
    print(f"joint3拨动: {TOGGLE_JOINT3_ANGLE}° ({TOGGLE_DIRECTION})")
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    targetT = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
    
    joints_target = piper_arm.inverse_kinematics(targetT)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    if not control_arm(joints_target, NORMAL_SPEED, USE_MOVEIT, TOGGLE_GRIPPER_OPEN):
        return False
    time.sleep(1.0)
    
    # 步骤3: joint4旋转90度
    print(f"\n步骤3: joint4旋转 {TOGGLE_JOINT4_ROTATE}°...")
    joints_rotate = joints_target.copy()
    joints_rotate[3] += TOGGLE_JOINT4_ROTATE * PI / 180
    if not control_arm(joints_rotate, NORMAL_SPEED, USE_MOVEIT, TOGGLE_GRIPPER_OPEN):
        return False
    time.sleep(1.0)
    
    # 步骤4: 沿末端z轴插入
    # 使用实际到达的关节角度，而不是上一步计算的理论值
    print(f"\n步骤4: 沿末端z轴插入 {TOGGLE_INSERT_DEPTH*100:.1f}cm...")
    actual_joints = get_current_joints()  # 获取实际当前位置
    print(f"  使用实际关节角度作为起点")
    joints_insert = move_along_end_effector_z(actual_joints, TOGGLE_INSERT_DEPTH, TOGGLE_INSERT_SPEED)
    if not joints_insert:
        return False
    time.sleep(0.5)
    
    # 步骤5: 夹爪闭合
    print(f"\n步骤5: 夹爪闭合到 {TOGGLE_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(TOGGLE_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(1.0)
    
    # 步骤6: joint3拨动
    direction_sign = -1 if TOGGLE_DIRECTION == 'left' else 1
    print(f"\n步骤6: joint3 {TOGGLE_DIRECTION}拨 {TOGGLE_JOINT3_ANGLE}°...")
    joints_toggle = joints_insert.copy()
    joints_toggle[2] += direction_sign * TOGGLE_JOINT3_ANGLE * PI / 180
    if not control_arm(joints_toggle, TOGGLE_TOGGLE_SPEED, USE_MOVEIT, TOGGLE_GRIPPER_HOLD):
        return False
    time.sleep(1.0)
    
    # 步骤7: 夹爪张开
    print("\n步骤7: 夹爪张开...")
    piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤8: 回预设零位
    print("\n步骤8: 回预设零位...")
    joints_zero = PRESET_ZERO_JOINTS.copy()
    if not control_arm(joints_zero, FAST_SPEED, USE_MOVEIT):
        return False
    time.sleep(1.0)
    
    # 步骤9: 夹爪闭合
    print("\n步骤9: 夹爪闭合...")
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 保存和可视化完整轨迹
    save_and_visualize_trajectory()
    
    print("="*70)
    print("✓✓✓ Toggle 操作完成！✓✓✓")
    print("="*70)
    return True


def action_push():
    """
    按压按钮操作
    流程: 闭合 → 到达 → 插入(z轴前进) → 保持0.5s → 返回 → 回零
    """
    global piper_arm
    
    # 清空轨迹记录
    clear_trajectory_records()
    clear_ee_trail()
    
    print("="*70)
    print("动作类型: Push (按压按钮)")
    print("="*70)
    print(f"目标位姿: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"按压深度: {PUSH_INSERT_DEPTH*100:.1f}cm, 保持: {PUSH_HOLD_TIME}秒")
    
    # 步骤1: 夹爪闭合
    print("\n步骤1: 夹爪闭合...")
    piper.GripperCtrl(PUSH_GRIPPER_CLOSE, 1000, 0x01, 0)
    time.sleep(0.1)  # 减少等待时间：夹爪动作很快
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    print(f"  目标: XYZ=({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    
    # 获取当前位置
    current_joints = get_current_joints()
    current_T = piper_arm.forward_kinematics(current_joints)
    current_xyz = current_T[:3, 3]
    print(f"  当前: XYZ=({current_xyz[0]:.3f}, {current_xyz[1]:.3f}, {current_xyz[2]:.3f})")
    
    # 创建目标位姿
    targetT = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
    
    # 先用IK计算目标关节角度
    joints_target = piper_arm.inverse_kinematics(targetT)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    # 使用MoveIt规划到达（关节空间规划，比笛卡尔规划更可靠）
    if not control_arm(joints_target, NORMAL_SPEED, USE_MOVEIT, PUSH_GRIPPER_CLOSE):
        return False
    # time.sleep(1.0)  # 不需要：control_arm_moveit内部已等待到达
    
    # 验证到达位置
    final_joints = get_current_joints()
    final_T = piper_arm.forward_kinematics(final_joints)
    final_xyz = final_T[:3, 3]
    print(f"  实际到达: XYZ=({final_xyz[0]:.3f}, {final_xyz[1]:.3f}, {final_xyz[2]:.3f})")
    error = np.linalg.norm(final_xyz - np.array([TARGET_X, TARGET_Y, TARGET_Z]))
    print(f"  位置误差: {error*100:.2f}cm")
    
    # 如果误差较大且启用了MoveIt，尝试笛卡尔路径微调
    if error > 0.01 and USE_MOVEIT and MOVEIT_AVAILABLE and move_group is not None:
        print(f"  位置误差较大，尝试笛卡尔路径微调...")
        try:
            from geometry_msgs.msg import Pose
            import tf.transformations as tft
            
            target_pose = Pose()
            target_pose.position.x = TARGET_X
            target_pose.position.y = TARGET_Y
            target_pose.position.z = TARGET_Z
            
            quat = tft.quaternion_from_matrix(targetT)
            target_pose.orientation.x = quat[0]
            target_pose.orientation.y = quat[1]
            target_pose.orientation.z = quat[2]
            target_pose.orientation.w = quat[3]
            
            waypoints = [target_pose]
            (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, True)
            
            if fraction > 0.9 and len(plan.joint_trajectory.points) > 0:
                print(f"  ✓ 笛卡尔微调成功 (覆盖率: {fraction*100:.1f}%)")
                
                traj_points = plan.joint_trajectory.points
                piper.MotionCtrl_2(0x01, 0x01, NORMAL_SPEED, 0x00)
                
                start_time = rospy.Time.now()
                command_rate = rospy.Rate(COMMAND_SEND_RATE)
                
                current_point_idx = 0
                next_point_idx = 1
                
                while next_point_idx < len(traj_points):
                    elapsed = (rospy.Time.now() - start_time).to_sec()
                    
                    while next_point_idx < len(traj_points) and elapsed >= traj_points[next_point_idx].time_from_start.to_sec():
                        current_point_idx = next_point_idx
                        next_point_idx += 1
                    
                    if next_point_idx >= len(traj_points):
                        break
                    
                    point_current = traj_points[current_point_idx]
                    point_next = traj_points[next_point_idx]
                    
                    t_current = point_current.time_from_start.to_sec()
                    t_next = point_next.time_from_start.to_sec()
                    
                    if t_next > t_current:
                        ratio = (elapsed - t_current) / (t_next - t_current)
                        ratio = max(0.0, min(1.0, ratio))
                    else:
                        ratio = 1.0
                    
                    joints_interpolated = []
                    for i in range(6):
                        pos_current = point_current.positions[i]
                        pos_next = point_next.positions[i]
                        pos_interp = pos_current + ratio * (pos_next - pos_current)
                        joints_interpolated.append(pos_interp)
                    
                    joints_int = [int(joints_interpolated[i] * factor) for i in range(6)]
                    joints_int[4] = max(-70000, joints_int[4])
                    piper.JointCtrl(*joints_int)
                    
                    command_rate.sleep()
                
                joints_target = [traj_points[-1].positions[i] for i in range(6)]
                rospy.sleep(0.5)
                
                # 再次验证
                final_T = piper_arm.forward_kinematics(joints_target)
                final_xyz = final_T[:3, 3]
                print(f"  微调后位置: XYZ=({final_xyz[0]:.3f}, {final_xyz[1]:.3f}, {final_xyz[2]:.3f})")
                error = np.linalg.norm(final_xyz - np.array([TARGET_X, TARGET_Y, TARGET_Z]))
                print(f"  最终误差: {error*100:.2f}cm")
            else:
                print(f"  笛卡尔微调失败 (覆盖率: {fraction*100:.1f}%)，使用当前位置")
        except Exception as e:
            print(f"  笛卡尔微调异常: {e}，使用当前位置")
    
    # 步骤3: 沿末端z轴插入（按压）
    # 使用实际到达的关节角度，而不是IK计算的理论值
    print(f"\n步骤3: 沿末端z轴按压 {PUSH_INSERT_DEPTH*100:.1f}cm...")
    actual_joints = get_current_joints()  # 获取实际当前位置
    print(f"  使用实际关节角度作为起点")
    joints_press = move_along_end_effector_z(actual_joints, PUSH_INSERT_DEPTH, PUSH_PRESS_SPEED)
    if not joints_press:
        return False
    
    步骤4: 保持按压
    print(f"\n步骤4: 保持按压 {PUSH_HOLD_TIME}秒...")
    time.sleep(PUSH_HOLD_TIME)
    
    # 步骤5: 返回到目标位置
    print("\n步骤5: 返回目标位置...")
    if not control_arm(joints_target, PUSH_PRESS_SPEED, USE_MOVEIT, PUSH_GRIPPER_CLOSE):
        return False
    time.sleep(0.1)
    
    # 步骤6: 回预设零位
    print("\n步骤6: 回预设零位...")
    joints_zero = PRESET_ZERO_JOINTS.copy()
    if not control_arm(joints_zero, FAST_SPEED, USE_MOVEIT, PUSH_GRIPPER_CLOSE):
        return False
    time.sleep(0.1)

    # 保存和可视化完整轨迹
    save_and_visualize_trajectory()
    
    print("="*70)
    print("✓✓✓ Push 操作完成！✓✓✓")
    print("="*70)
    return True


def action_knob():
    """
    旋转旋钮操作
    流程: 张开 → 到达 → 插入(z轴前进) → 闭合 → 旋转 → 回零闭合
    """
    global piper_arm
    
    # 清空轨迹记录
    clear_trajectory_records()
    clear_ee_trail()
    
    print("="*70)
    print("动作类型: Knob (旋转旋钮)")
    print("="*70)
    print(f"目标位姿: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"插入: {KNOB_INSERT_DEPTH*100:.1f}cm, 旋转: {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})")
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(KNOB_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    targetT = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
    
    joints_target = piper_arm.inverse_kinematics(targetT)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    if not control_arm(joints_target, NORMAL_SPEED, USE_MOVEIT, KNOB_GRIPPER_OPEN):
        return False
    time.sleep(0.1)
    
    # 步骤3: 沿末端z轴插入
    # 使用实际到达的关节角度，而不是IK计算的理论值
    print(f"\n步骤3: 沿末端z轴插入 {KNOB_INSERT_DEPTH*100:.1f}cm...")
    actual_joints = get_current_joints()  # 获取实际当前位置
    print(f"  使用实际关节角度作为起点")
    joints_insert = move_along_end_effector_z(actual_joints, KNOB_INSERT_DEPTH, KNOB_INSERT_SPEED)
    if not joints_insert:
        return False
    time.sleep(0.1)
    
    # 步骤4: 夹爪闭合
    print(f"\n步骤4: 夹爪闭合到 {KNOB_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(KNOB_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤5: 旋转joint6
    direction_sign = 1 if KNOB_ROTATION_DIRECTION == 'cw' else -1
    print(f"\n步骤5: 旋转 {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})...")
    # 【关键】使用实际当前位置，而不是之前记录的joints_insert
    current_joints_before_rotate = get_current_joints()
    print(f"  当前实际关节角度: [{', '.join([f'{j:.4f}' for j in current_joints_before_rotate])}]")
    
    joints_rotate = current_joints_before_rotate.copy()
    joints_rotate[5] += direction_sign * KNOB_ROTATION_ANGLE * PI / 180
    print(f"  目标关节角度: [{', '.join([f'{j:.4f}' for j in joints_rotate])}]")
    if not control_arm(joints_rotate, KNOB_ROTATION_SPEED, USE_MOVEIT, KNOB_GRIPPER_HOLD):
        return False
    time.sleep(0.1)
    
    # 步骤6: 夹爪张开（松开旋钮）
    print("\n步骤6: 夹爪张开...")
    piper.GripperCtrl(KNOB_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤7: 回预设零位
    print("\n步骤7: 回预设零位...")
    joints_zero = PRESET_ZERO_JOINTS.copy()
    if not control_arm(joints_zero, FAST_SPEED, USE_MOVEIT, KNOB_GRIPPER_OPEN):
        return False
    time.sleep(0.1)
    
    # 步骤8: 夹爪闭合
    print("\n步骤8: 夹爪闭合...")
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 保存和可视化完整轨迹
    save_and_visualize_trajectory()
    
    print("="*70)
    print("✓✓✓ Knob 操作完成！✓✓✓")
    print("="*70)
    return True


# ========================================
# 主程序
# ========================================

def main():
    global piper, piper_arm, move_group, display_trajectory_publisher, ee_path_publisher, ee_trail_publisher
    
    print("="*70)
    print("按钮操作执行器 - 独立版本")
    print("="*70)
    print(f"\n📍 目标位姿: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    if USE_6D_POSE:
        print(f"   姿态: Roll={TARGET_ROLL:.3f}, Pitch={TARGET_PITCH:.3f}, Yaw={TARGET_YAW:.3f} (弧度)")
    print(f"🎯 动作类型: {ACTION_TYPE.upper()}")
    print(f"🔧 控制模式: {'MoveIt' if USE_MOVEIT and MOVEIT_AVAILABLE else 'SDK'}")
    
    # 显示动作特定参数
    if ACTION_TYPE == 'plugin':
        print(f"\n插拔连接器配置:")
        print(f"  插入深度: {PLUGIN_INSERT_DEPTH*100:.1f}cm")
        print(f"  夹持宽度: {PLUGIN_GRIPPER_HOLD/1000:.1f}mm")
    elif ACTION_TYPE == 'toggle':
        print(f"\n拨动开关配置:")
        print(f"  joint4旋转: {TOGGLE_JOINT4_ROTATE}°")
        print(f"  插入深度: {TOGGLE_INSERT_DEPTH*100:.1f}cm")
        print(f"  joint3拨动: {TOGGLE_JOINT3_ANGLE}° ({TOGGLE_DIRECTION})")
    elif ACTION_TYPE == 'push':
        print(f"\n按压按钮配置:")
        print(f"  按压深度: {PUSH_INSERT_DEPTH*100:.1f}cm")
        print(f"  保持时间: {PUSH_HOLD_TIME}秒")
    elif ACTION_TYPE == 'knob':
        print(f"\n旋转旋钮配置:")
        print(f"  插入深度: {KNOB_INSERT_DEPTH*100:.1f}cm")
        print(f"  旋转角度: {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})")
    
    print("="*70)
    
    # 初始化硬件
    print("\n初始化机械臂...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    
    for i in range(7):
        piper.EnableArm(i + 1)
        time.sleep(0.1)
    
    piper.GripperCtrl(70000, 1000, 0x01, 0)
    print("  ✓ 硬件初始化完成")
    
    # 初始化 ROS
    print("\n初始化ROS...")
    rospy.init_node('button_action_node', anonymous=True)
    
    # 初始化 MoveIt (如果需要)
    if USE_MOVEIT and MOVEIT_AVAILABLE:
        try:
            import os
            piper_ros_path = "/home/robot/button/V4.0/project2/piper_ros"
            src_path = os.path.join(piper_ros_path, 'src')
            current_path = os.environ.get('ROS_PACKAGE_PATH', '')
            if src_path not in current_path:
                os.environ['ROS_PACKAGE_PATH'] = f"{src_path}:{current_path}"
            
            moveit_commander.roscpp_initialize([])
            robot = moveit_commander.RobotCommander()
            move_group = moveit_commander.MoveGroupCommander("arm")
            move_group.set_planning_time(2.0)  # 减少规划时间：从5秒降到2秒
            move_group.set_max_velocity_scaling_factor(1.0)  # 最大速度
            move_group.set_max_acceleration_scaling_factor(1.0)  # 最大加速度
            move_group.set_planner_id(PLANNER_ID)  # 使用快速规划器
            
            # 创建轨迹可视化发布器
            display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                DisplayTrajectory,
                queue_size=20
            )
            
            # 创建末端执行器路径发布器
            ee_path_publisher = rospy.Publisher(
                '/end_effector_path',
                Path,
                queue_size=10
            )
            
            # 创建末端执行器轨迹标记发布器
            ee_trail_publisher = rospy.Publisher(
                '/end_effector_trail',
                Marker,
                queue_size=10
            )
            
            print("  ✓ MoveIt初始化完成")
            print(f"  ✓ 规划器: {PLANNER_ID}")
            print("  ✓ 轨迹可视化发布器已创建")
            print(f"     - /move_group/display_planned_path (DisplayTrajectory)")
            print(f"     - /end_effector_path (Path)")
            print(f"     - /end_effector_trail (Marker with gradient)")
            print(f"  ✓ 频率配置:")
            print(f"     - RViz发布: {RVIZ_PUBLISH_RATE}Hz")
            print(f"     - 命令发送: {COMMAND_SEND_RATE}Hz (高频插值)")
        except Exception as e:
            print(f"  ⚠️  MoveIt初始化失败: {e}")
            print("  将使用SDK模式")
    
    # 初始化 Piper Arm
    piper_arm = PiperArm()
    
    # 回预设零位
    print("\n回预设零位...")
    joints_zero = PRESET_ZERO_JOINTS.copy()
    control_arm_sdk(joints_zero, 100)
    time.sleep(2)
    print("  ✓ 已回预设零位")
    
    print("\n="*70)
    print("开始执行动作...")
    print("="*70)
    
    # 执行对应动作
    action_functions = {
        'plugin': action_plugin,
        'toggle': action_toggle,
        'push': action_push,
        'knob': action_knob
    }
    
    if ACTION_TYPE not in action_functions:
        print(f"❌ 未知动作类型: {ACTION_TYPE}")
        print(f"   支持的类型: {list(action_functions.keys())}")
        return
    
    try:
        success = action_functions[ACTION_TYPE]()
        if success:
            print("\n✓ 动作执行成功！")
        else:
            print("\n❌ 动作执行失败")
    except Exception as e:
        print(f"\n❌ 动作执行异常: {e}")
        import traceback
        traceback.print_exc()
    
    # 清理资源
    if MOVEIT_AVAILABLE:
        moveit_commander.roscpp_shutdown()
    
    print("\n程序结束")


if __name__ == "__main__":
    main()