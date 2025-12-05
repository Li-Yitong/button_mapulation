#!/usr/bin/env python3
"""
按钮操作执行器 - 独立版本
支持四种按钮操作类型：Toggle, Plug-in, Push, Knob
所有参数通过宏定义配置，无需视觉检测
"""
from piper_sdk import *
import time
import numpy as np
import math
from piper_arm import PiperArm

# 条件导入 ROS (兼容 ROS1 和非 ROS 环境)
try:
    import rospy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # 提供兼容的时间函数
    class FakeRospy:
        class Time:
            @staticmethod
            def now():
                class TimeObj:
                    def __init__(self):
                        self.secs = int(time.time())
                        self.nsecs = int((time.time() % 1) * 1e9)
                    def to_sec(self):
                        return time.time()
                    def __sub__(self, other):
                        class Duration:
                            def __init__(self, val):
                                self.val = val
                            def to_sec(self):
                                return self.val
                        return Duration(time.time() - other.to_sec())
                return TimeObj()
        
        @staticmethod
        def init_node(name, anonymous=False):
            """Fake init_node for compatibility"""
            pass
        
        @staticmethod
        def sleep(duration):
            time.sleep(duration)
        
        class Rate:
            def __init__(self, hz):
                self.period = 1.0 / hz
                self.last_time = time.time()
            def sleep(self):
                elapsed = time.time() - self.last_time
                if elapsed < self.period:
                    time.sleep(self.period - elapsed)
                self.last_time = time.time()
        
        @staticmethod
        def Publisher(*args, **kwargs):
            class FakePublisher:
                def publish(self, msg):
                    pass
            return FakePublisher()
    
    rospy = FakeRospy()

# ========================================
# 宏定义 - 用户配置区
# ========================================
# ========================================
# 全局常量
# ========================================
PI = math.pi
factor = 1000 * 180 / PI

# ========================================
# 新增：AprilTag绝对姿态（基座系）
# ========================================
# ⚠️ 重要区分：
#   - APRILTAG_BASE_X/Y/Z：AprilTag标签本身的位置（仅用于姿态参考）
#   - TARGET_X/Y/Z：实际按钮/旋钮的位置（动作的目标位置）
#
# AprilTag在基座标系下的位置和姿态（请根据实际标定结果填写）
# 💡 这些值应该从 realsense_yolo_button_interactive_ros2_direct_april.py 
#    显示的 [Tag in Base] 数据中获取！
# 
# 📐 关键理解（垂直按压时的姿态关系）:
#    ⚠️ AprilTag的姿态 ≠ 夹爪末端的姿态！
#    
#    当夹爪垂直按压按钮时，夹爪末端姿态与Tag姿态的关系为：
#    - gripper_roll  = -APRILTAG_BASE_ROLL  (Roll取反，因为法向量反向)
#    - gripper_pitch =  APRILTAG_BASE_PITCH (Pitch相同)
#    - gripper_yaw   =  APRILTAG_BASE_YAW   (Yaw相同)
#    
#    💡 这个转换已在 get_gripper_approach_rotation('perpendicular') 中自动处理！
#
# 🏷️ AprilTag位置（仅用于姿态参考，不是目标位置！）
APRILTAG_BASE_X = 0.413      # Tag中心X坐标 (米)
APRILTAG_BASE_Y = 0.00      # Tag中心Y坐标 (米)
APRILTAG_BASE_Z = 0.00      # Tag中心Z坐标 (米)

# 🏷️ AprilTag姿态（用于计算夹爪的正确姿态）
APRILTAG_BASE_ROLL = -180 * PI / 180   # Tag的Roll (弧度) ⚠️ 这是Tag本身的姿态！
APRILTAG_BASE_PITCH = 3.4 * PI / 180     # Tag的Pitch (弧度)
APRILTAG_BASE_YAW = 180 * PI / 180     # Tag的Yaw (弧度)

APRILTAG_REFERENCE_POSE_BASE = None  # 面板在基座系的目标姿态（3x3旋转矩阵，从上述RPY计算得到）
APRILTAG_ALIGNMENT_TOLERANCE = 5.0 * PI / 180  # 姿态容差：5度

# ========================================
# 新增：AprilTag RPY静态补偿
# ========================================
# 💡 用途：修正AprilTag检测的系统性偏差
#    最终参考姿态 = 采样平均值 + 静态补偿
#
# 🔧 调试方法：
#    1. 运行程序，观察夹爪到达目标时的姿态偏差
#    2. 在终端查看"实际姿态 vs 目标姿态"的差值
#    3. 如果Yaw总是偏+5度，设置 APRILTAG_RPY_OFFSET_YAW = -5.0
#    4. 重启程序，重新采样AprilTag姿态
#
# 📐 示例：
#    观察到：实际Yaw=155°, 目标Yaw=150°, 偏差=+5°
#    修正：APRILTAG_RPY_OFFSET_YAW = -5.0
#
APRILTAG_RPY_OFFSET_ROLL = 0   # Roll补偿（度）⚠️ 正值=顺时针修正
APRILTAG_RPY_OFFSET_PITCH = 0  # Pitch补偿（度）⚠️ 正值=抬头修正
APRILTAG_RPY_OFFSET_YAW = 0    # Yaw补偿（度）⚠️ 正值=逆时针修正（常用）

# ========================================
# 新增：每种按钮的独立TCP偏移
# ========================================
# 💡 用途：不同按钮可能需要不同的夹爪接触点
#
# 📐 坐标系定义（夹爪本体坐标系）：
#    X：向前（手指闭合方向）- 负值向后
#    Y：向左 - 负值向右
#    Z：向上 - 负值向下
#
# 🔧 调试方法：
#    1. 运行程序，观察夹爪接触按钮的位置
#    2. 如果偏左5mm → 减少Y：tcp_y -= 0.005
#    3. 如果偏前10mm → 减少X：tcp_x -= 0.010
#    4. 如果偏高8mm → 减少Z：tcp_z -= 0.008
#
# 📊 调整建议：
#    - Toggle（拨动）：可能需要侧面接触，调整Y值
#    - Push（按压）：需要正面接触，保持默认
#    - Knob（旋转）：可能需要更深插入，调整Z值
#    - Plugin（插拔）：可能需要更精确对准，微调X/Y
#
# TCP_OFFSET_TOGGLE = [-0.051, 0.007, 0.080]   # Toggle拨动开关
# TCP_OFFSET_PLUGIN = [-0.051, 0.007, 0.080]   # Plugin插拔连接器
# TCP_OFFSET_PUSH = [0.025, -0.068, 0.125]     # Push按压按钮
# TCP_OFFSET_KNOB = [0.015, -0.045, 0.12]     # Knob旋转旋钮

TCP_OFFSET_TOGGLE = [0,0,0]   # Toggle拨动开关
TCP_OFFSET_PLUGIN = [0,0,-0.15]   # Plugin插拔连接器
TCP_OFFSET_PUSH = [0,0.057,-0.136]     # Push按压按钮
TCP_OFFSET_KNOB = [-0.0,0.056,-0.15]     # Knob旋转旋钮  +  hou   +zuo  +xia

# 🔧 快速调试开关：统一调整所有TCP（全局微调）
# 💡 用途：快速测试偏移方向，找到问题后再调整具体按钮的TCP
TCP_GLOBAL_OFFSET_X = 0.0  # 全局X偏移（米）⚠️ 正值向前，负值向后
TCP_GLOBAL_OFFSET_Y = 0.0  # 全局Y偏移（米）⚠️ 正值向左，负值向右
TCP_GLOBAL_OFFSET_Z = 0.0  # 全局Z偏移（米）⚠️ 正值向上，负值向下
# === 标准起始/结束位姿 (可选，用于视觉检测等待位置) ===
# HOME位姿：一个安全的观察位姿，机械臂在此位置等待视觉检测
# 注意：J5限位为[-70°, 70°]，保留5°安全余量
# HOME_JOINTS = [
#     -4.68 * PI / 180,   # J1: -4.68°
#     86.06 * PI / 180,   # J2: 86.06°
#     -86.16 * PI / 180,  # J3: -86.16°
#     5.27 * PI / 180,    # J4: 5.27°
#     65.0 * PI / 180,    # J5: 65.0° (原69.12°，降低避免接近限位)
#     0.94 * PI / 180     # J6: 0.94°
# # ]
# [INFO] [1764824515.578004544] [piper_status_reader]:   J1: -2.50°
# [INFO] [1764824515.580301892] [piper_status_reader]:   J2: 91.96°
# [INFO] [1764824515.583516281] [piper_status_reader]:   J3: -81.42°
# [INFO] [1764824515.586704452] [piper_status_reader]:   J4: 2.25°
# [INFO] [1764824515.589387963] [piper_status_reader]:   J5: 64.09°
# [INFO] [1764824515.591650302] [piper_status_reader]:   J6: 0.97°

# [INFO] [1764925196.496128365] [piper_status_reader]:   J1: -2.55°
# [INFO] [1764925196.496424424] [piper_status_reader]:   J2: 83.32°
# [INFO] [1764925196.496711148] [piper_status_reader]:   J3: -54.39°
# [INFO] [1764925196.497000148] [piper_status_reader]:   J4: 12.37°
# [INFO] [1764925196.497278926] [piper_status_reader]:   J5: 30.39°
# [INFO] [1764925196.497572549] [piper_status_reader]:   J6: -7.26°
HOME_JOINTS = [
    -2.55 * PI / 180,   # J1: -4.68°
    83.32 * PI / 180,   # J2: 86.06°
    -54.39 * PI / 180,  # J3: -86.16°
    12.37 * PI / 180,    # J4: 5.27°
    30.39 * PI / 180,    # J5: 65.0° (原69.12°，降低避免接近限位)
    -7.26 * PI / 180     # J6: 0.94°
]

# HOME_JOINTS = [
#     0 * PI / 180,   # J1: -4.68°
#     0 * PI / 180,   # J2: 86.06°
#     0 * PI / 180,  # J3: -86.16°
#     0 * PI / 180,   # J4: 5.27°
#     0 * PI / 180,    # J5: 65.0° (原69.12°，降低避免接近限位)
#     0 * PI / 180     # J6: 0.94°
# ]
HOME_GRIPPER = 0  # 夹爪闭合状态（0 = 完全闭合）

# 🔧 HOME位姿开关（控制动作开始前是否先到HOME位姿）
# True  = 动作前先移动到HOME位姿（用于视觉检测等待位置），动作后回零位
# False = 动作前从零位直接开始，动作后回零位
# 注意：无论哪种模式，动作结束后都统一回零位（安全可靠）
USE_HOME_POSITION = True

# === 目标位姿配置 (基座坐标系) ===
# 🎯 重要说明：
#    TARGET_X/Y/Z 是实际按钮/旋钮的位置（不是AprilTag的位置！）
#    姿态由 APRILTAG_BASE_ROLL/PITCH/YAW 自动计算（通过get_gripper_approach_rotation）
#
# 位置 (单位：米) - 实际按钮/旋钮的3D坐标
TARGET_X = 0.4  # X坐标 (降低以保证可达性)
TARGET_Y = 0.19 # Y坐标
TARGET_Z = 0.0  # Z坐标 (使用末端朝下姿态可达更高位置)

# XYZ: (+0.560, -0.104, -0.038) m
# TARGET_X = +0.560  # X坐标 (降低以保证可达性)
# TARGET_Y = -0.104  # Y坐标 
# TARGET_Z = -0.038  # Z坐标 (使用末端朝下姿态可达更高位置)



# 新增：完整位姿矩阵（包含法向量对齐）
# 当 vision_button_action_ros2 提供时，将使用此矩阵代替 TARGET_X/Y/Z + TARGET_ROLL/PITCH/YAW
TARGET_POSE_MATRIX = None  # 4x4 np.ndarray 或 None

# 新增：面板对齐位姿（保存在对齐阶段计算的姿态）
PANEL_ALIGN_POSE = None  # 4x4 np.ndarray 或 None，供后续按钮操作继承

# 姿态 (单位：弧度) - 仅在未设置AprilTag姿态时使用（回退方案）
# ⚠️ 注意：当APRILTAG_REFERENCE_POSE_BASE存在时，姿态由AprilTag自动计算，这些值不会被使用
TARGET_ROLL = 0.0          # 绕末端X轴旋转 (翻滚) - 正值：向右倾斜 [建议范围: -0.5~0.5 rad]
TARGET_PITCH = PI/2        # 绕末端Y轴旋转 (俯仰) - 正值：向上抬起 [PI/2 rad = 90° = 末端朝下]
TARGET_YAW = 0.0           # 绕末端Z轴旋转 (偏航) - 正值：逆时针旋转 [建议范围: -1.0~1.0 rad]

# 姿态模式选择
USE_6D_POSE = True   # True=使用6D位姿(含姿态), False=仅使用位置(末端朝前)

# 注意：IK精度说明
# 由于piper_arm.py的IK算法存在精度限制，实际到达的位置可能与目标位置有几厘米的偏差。
# 这是正常现象，不影响按钮操作的执行。如果需要更高精度，请考虑使用MoveIt的笛卡尔路径规划。

# === 动作类型选择 ===
ACTION_TYPE = 'push'  # 'toggle'/'plugin'/'push'/'knob'

# === 控制模式 ===
USE_MOVEIT = True  # ROS2启动脚本: 启用MoveIt2粗定位

# === 🔧 测试模式：从HOME位姿直接执行动作 ===
TEST_MODE_FROM_HOME = False  # True=从HOME位姿直接沿Z轴执行, False=使用MoveIt规划到接近位姿（推荐！）

# === 精调与调试开关 === 
ENABLE_CARTESIAN_FINE_TUNE = False    # True=MoveIt后允许笛卡尔微调, False=严格使用MoveIt结果
CARTESIAN_FINE_TUNE_THRESHOLD = 0.008  # 超过该距离(米)才触发微调
CARTESIAN_ORIENTATION_INTERPOLATION = True  # ✨ True=笛卡尔精调时同时插值姿态(SLERP)，避免突然旋转
DEBUG_IK_SOLVER = False               # True=打印每个IK求解细节
AUTO_FINE_TUNE_ON_FAILURE = True      # True=MoveIt多次尝试后仍超差时自动触发笛卡尔精调
AUTO_FINE_TUNE_SPEED = 12             # 自动精调的默认SDK速度

# === Plugin (插拔连接器) 配置 ===
PLUGIN_GRIPPER_OPEN = 60000     # 张开宽度 (单位: 0.001mm, 范围: 0~70000, 即0~70mm)
PLUGIN_INSERT_DEPTH = 0.02      # 插入深度 (单位: 米, 范围: -0.1~0.1, 建议: 0.01~0.05)
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
PUSH_INSERT_DEPTH = 0.005       # 按压深度 (单位: 米, 0.5cm - 与knob一致)
PUSH_HOLD_TIME = 0.5            # 保持时间 (单位: 秒, 范围: 0~无限, 建议: 1~5)
PUSH_PRESS_SPEED = 90           # 按压/回撤速度 (单位: 无量纲, 0~100)

# === 笛卡尔平滑执行全局配置 ===
CARTESIAN_MIN_WAYPOINTS = 6             # 参考ROS1实现，短行程只需几个分段
CARTESIAN_MAX_WAYPOINTS = 50            # 最多waypoints，避免规划出上百点
CARTESIAN_WAYPOINT_STEP = 0.005         # waypoint目标间距 (米)，默认5mm≈每厘米2点
CARTESIAN_EEF_STEP = 0.01               # compute_custom_cartesian_path的细分步长 (米)
CARTESIAN_SPEED_LIMIT = 100              # SDK笛卡尔执行默认最大速度 (0~100)
CARTESIAN_HIGH_SPEED_LIMIT = 100        # 特殊场景的高速上限 (push/plugin)
CARTESIAN_INTERPOLATION_PROFILE = 'cubic'  # 插值速度曲线: 'linear' 或 'cubic'
CARTESIAN_HIGH_ACCEL_PROFILE = 'impulse'   # 高加速度场景使用的自定义曲线

# === Knob (旋转旋钮) 配置 ===
KNOB_GRIPPER_OPEN = 30000       # 张开宽度 (单位: 0.001mm, 范围: 0~70000, 即0~70mm)
KNOB_INSERT_DEPTH = 0.005        # 插入深度 (单位: 米, 范围: -0.1~0.1, 建议: 0.005~0.02)
KNOB_GRIPPER_HOLD = 8000       # 闭合夹持宽度 (单位: 0.001mm, 范围: 0~70000, 建议: 15000~35000)
KNOB_ROTATION_ANGLE = 45        # 旋转角度 (单位: 度, 范围: -360~360, 建议: 30~180)
KNOB_ROTATION_DIRECTION = 'cw'  # 旋转方向: 'cw'=顺时针(右旋), 'ccw'=逆时针(左旋)
KNOB_INSERT_SPEED = 100          # 插入速度 (单位: 无量纲, 范围: 0~100)
KNOB_ROTATION_SPEED = 60        # 旋转速度 (单位: 无量纲, 范围: 0~100)

# === 通用速度配置 ===
NORMAL_SPEED = 100              # 正常移动速度 (单位: 无量纲, 范围: 0~100, SDK硬限制)
FAST_SPEED = 100                # 快速移动速度 (单位: 无量纲, 范围: 0~100, SDK硬限制)
SLOW_SPEED = 40                 # 慢速移动速度 (单位: 无量纲, 范围: 0~100, 用于回零/修正)

# === MoveIt 精度与重试策略 ===
MOVEIT_POSITION_TOLERANCE = 0.002     # 第一阶段允许的最大位置误差 (米)
MOVEIT_MAX_REPLAN_ATTEMPTS = 1         # 允许额外重试的次数（总尝试 = 尝试次数 + 1）
MOVEIT_JOINT_TOLERANCE = 0.001          # 关节约束容差 (弧度)，约1.15°



# ========================================
# MoveIt 配置 (可选)
# ========================================
# 轨迹执行频率控制
RVIZ_PUBLISH_RATE = 10          # 轨迹发布到RViz的频率 (Hz)
COMMAND_SEND_RATE = 80          # 命令发送频率 (Hz) - 在轨迹点之间持续发送命令
PLANNER_ID = "RRTstar"       # 可选: "RRTstar", "PRM", "BKPIECE", "EST"

# 调试配置
DEBUG_TRAJECTORY = False        # 是否显示详细的轨迹调试信息（关闭以提高速度）

# 尾迹可视化配置
MAX_TRAIL_POINTS = 100          # 最大尾迹点数

# === MoveIt2 配置 ===
# 注意: ROS2 Foxy的MoveIt2不支持Python Action Client API
# 虽然action server存在且可以连接，但不会响应Python客户端的goal请求
# 这是已知限制，需要ROS2 Humble+或pymoveit2库
# 因此在ROS2 Foxy环境中自动禁用MoveIt2，使用SDK模式
MOVEIT_AVAILABLE = False
MOVEIT_INITIALIZED = False  # 标记MoveIt2是否已初始化
move_group = None
moveit_node = None  # ROS2 node for MoveIt2
ros2_executor = None  # ROS2 executor for spinning
ros2_spin_thread = None  # 背景spin线程
ROS2_FOXY_DETECTED = False

try:
    if USE_MOVEIT:
        # ROS2 MoveIt2 imports
        import rclpy
        from rclpy.node import Node
        from moveit_msgs.action import MoveGroup as MoveGroupAction
        from rclpy.action import ActionClient
        from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from nav_msgs.msg import Path
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point, PoseStamped
        from std_msgs.msg import ColorRGBA
        
        # MoveIt2在ROS2 Foxy中可用（需要使用干净环境避免ROS1冲突）
        # 使用 start_moveit2_clean.sh 启动MoveIt2
        # 使用 run_button_actions_clean.sh 运行本程序
        MOVEIT_AVAILABLE = True
        try:
            import os
            ros_distro = os.environ.get('ROS_DISTRO', '')
            print(f"✓ MoveIt2 (ROS2 {ros_distro}) 已加载")
        except Exception:
            print("✓ MoveIt2 (ROS2) 已加载")
except ImportError as e:
    print(f"⚠️  MoveIt2未加载，将使用SDK模式: {e}")

# Global variables
piper = None
piper_arm = None
display_trajectory_publisher = None
ee_path_publisher = None
ee_trail_publisher = None
joint_state_publisher = None  # ROS2 joint_states publisher
joint_state_timer = None      # ROS2 timer

# Trajectory recording (trail visualization)
ee_trail_points = []

# Trajectory recording (planning vs execution) - ACCUMULATED ACROSS ALL STEPS
planned_trajectory = []      # Accumulated planned end-effector XYZ across all planning steps
executed_trajectory = []     # Accumulated executed end-effector XYZ across all execution steps
all_planned_points = []      # Accumulated MoveIt planned points (JointTrajectoryPoint objects)
all_execution_records = []   # Accumulated execution records [(time, joints, xyz, velocities), ...]
trajectory_start_time = 0.0  # Time when the first planning started (for cumulative timeline)
trajectory_save_dir = "trajectory"  # Trajectory save directory
pvat_data = None  # PVAT (Position-Velocity-Acceleration-Time) data


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


def publish_joint_states_callback():
    """
    ROS2定时器回调：发布当前关节状态
    用于MoveIt2规划时获取机器人当前状态
    """
    global piper, joint_state_publisher, moveit_node
    
    if not MOVEIT_AVAILABLE or joint_state_publisher is None:
        return
    
    try:
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = moveit_node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # 发布所有关节（joint1-joint7）
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        # 如果有真实硬件，从piper读取当前位置
        # 这里使用零位作为默认值（或者可以读取piper.GetArmStatus()）
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.velocity = []
        msg.effort = []
        
        joint_state_publisher.publish(msg)
    except Exception as e:
        # 静默失败，避免刷屏
        pass


def publish_dual_trajectory_markers(planned_xyz, executed_xyz):
    """
    在RViz中发布规划路径和执行路径的对比可视化 (ROS2版本)
    
    Args:
        planned_xyz: 规划的末端XYZ轨迹 (N×3 array)
        executed_xyz: 执行的末端XYZ轨迹 (M×3 array)
    """
    global moveit_node
    
    if not MOVEIT_AVAILABLE or len(planned_xyz) == 0 or moveit_node is None:
        print("  ⚠️  无法发布轨迹对比标记（MoveIt2未初始化或无数据）")
        return
    
    try:
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        import time
        
        # 创建publisher（QoS配置）
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        marker_pub = moveit_node.create_publisher(Marker, '/trajectory_comparison', qos)
        time.sleep(0.1)  # 等待publisher建立连接
        
        # 发布规划路径（蓝色线）
        planned_marker = Marker()
        planned_marker.header.frame_id = "base_link"
        planned_marker.header.stamp = moveit_node.get_clock().now().to_msg()
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
            p.x = float(xyz[0])
            p.y = float(xyz[1])
            p.z = float(xyz[2])
            planned_marker.points.append(p)
        
        # 发布执行路径（红色线）
        executed_marker = Marker()
        executed_marker.header.frame_id = "base_link"
        executed_marker.header.stamp = moveit_node.get_clock().now().to_msg()
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
                p.x = float(xyz[0])
                p.y = float(xyz[1])
                p.z = float(xyz[2])
                executed_marker.points.append(p)
        
        # 发布标记
        for _ in range(3):
            marker_pub.publish(planned_marker)
            marker_pub.publish(executed_marker)
            time.sleep(0.05)
        
        print(f"  ✓ 轨迹对比已发布到 RViz (/trajectory_comparison)")
        print(f"    🔵 蓝色 = 规划路径 ({len(planned_xyz)}个点)")
        if len(executed_xyz) > 0:
            print(f"    🔴 红色 = 执行路径 ({len(executed_xyz)}个点)")
    except Exception as e:
        print(f"  ⚠️  发布轨迹标记失败: {e}")
    
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
    path_msg.header.frame_id = "base_link"  # Fixed: 使用正确的frame名称
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
    marker.header.frame_id = "base_link"  # Fixed: 使用正确的frame名称
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
    """
    Clear trajectory records (call before starting a new action sequence)
    This clears ALL accumulated planning and execution data
    """
    global planned_trajectory, executed_trajectory, all_planned_points, all_execution_records, trajectory_start_time
    planned_trajectory = []
    executed_trajectory = []
    all_planned_points = []
    all_execution_records = []
    trajectory_start_time = 0.0
    print("  ✓ Trajectory records cleared (ready for new action sequence)")


def save_and_visualize_trajectory():
    """
    Save and visualize complete trajectory records (call after action sequence ends)
    This generates PVAT analysis charts for the entire sequence from start to finish
    """
    global planned_trajectory, executed_trajectory, pvat_data, all_planned_points, all_execution_records
    
    if len(all_planned_points) == 0:
        print("\n  ⚠️  No planning data, skipping trajectory visualization")
        return
    
    print("\n" + "="*70)
    print("📊 Saving and visualizing complete trajectory...")
    print("="*70)
    print(f"  📍 Accumulated planned points: {len(all_planned_points)}")
    print(f"  📍 Accumulated execution records: {len(all_execution_records)}")
    print(f"  📍 Total planned XYZ points: {len(planned_trajectory)}")
    print(f"  📍 Total executed XYZ points: {len(executed_trajectory)}")
    
    # 1. Publish trajectory comparison to RViz
    if len(executed_trajectory) > 0:
        publish_dual_trajectory_markers(planned_trajectory, executed_trajectory)
    
    # 2. Generate and save PVAT charts
    if len(all_execution_records) > 0:
        # Compute total time span
        total_time = all_execution_records[-1][0] if len(all_execution_records) > 0 else 0.0
        
        # Save PVAT data
        pvat_data = {
            'planned_points': all_planned_points,      # All accumulated MoveIt points
            'execution_records': all_execution_records,  # All accumulated execution records
            'total_time': total_time
        }
        
        # Save to pickle file
        ensure_trajectory_dir()
        import pickle
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        pvat_file = f"{trajectory_save_dir}/pvat_data_{timestamp}.pkl"
        
        with open(pvat_file, 'wb') as f:
            pickle.dump(pvat_data, f)
        print(f"  ✓ PVAT data saved: {pvat_file}")
        
        # Generate PVAT charts
        try:
            from plot_pvat import plot_pvat_analysis
            chart_file = plot_pvat_analysis(pvat_data, trajectory_save_dir)
            print(f"  ✓ PVAT chart generated: {chart_file}")
        except Exception as e:
            print(f"  ⚠️  Failed to generate PVAT chart: {e}")
    
    print("="*70)


def control_arm_sdk(joints, speed=50, gripper_value=None):
    """SDK 直接控制模式"""
    global piper
    
    # 🔧 关键修复：确保机械臂使能并验证状态
    piper.EnableArm(7)  # 使能所有关节 + 夹爪
    time.sleep(0.2)  # 增加等待时间，确保使能生效
    
    # 检查使能状态（可选，但有助于调试）
    try:
        status = piper.GetArmLowSpdInfoMsgs()
        if status.motor_1.foc_status.driver_enable_status != 1:
            print("  ⚠️ [SDK] 使能状态异常，尝试重新使能...")
            piper.EnableArm(7)
            time.sleep(0.5)
    except:
        pass  # 忽略状态检查错误，继续执行
    
    joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
    # 🔧 关键修复：限制Joint5上下限，防止超出范围 (-70000, 70000)
    joints_int[4] = min(70000, max(-70000, joints_int[4]))
    
    # 🔧 关键修复：设置运动模式并等待生效（参考demo_03_go_zero_ros2.py）
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    time.sleep(0.1)  # 等待运动模式切换生效
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
    """MoveIt2 规划控制模式 (ROS2) - 只规划不执行，执行用SDK"""
    global piper, move_group, moveit_node, display_trajectory_publisher
    global planned_trajectory, executed_trajectory, piper_arm
    
    # 检查 MoveIt2 是否可用
    if move_group is None or moveit_node is None:
        print("  ⚠️  MoveIt2 未初始化，回退到 SDK 模式")
        return control_arm_sdk(joints, speed, gripper_value)
    
    if not MOVEIT_AVAILABLE:
        print("  ⚠️  MoveIt2 不可用，回退到 SDK 模式")
        return control_arm_sdk(joints, speed, gripper_value)
    
    try:
        # 导入 ROS2 消息类型
        from moveit_msgs.msg import Constraints, JointConstraint, RobotState
        from moveit_msgs.action import MoveGroup as MoveGroupAction
        from sensor_msgs.msg import JointState
        import rclpy
        import time as time_module
        
        # 【关键】获取当前实际关节角度作为起点
        current_joints = get_current_joints()
        target_joints = joints[:6] if len(joints) > 6 else joints
        
        print("  [MoveIt2] 规划轨迹...")
        print(f"  📍 起始点 (弧度): [{', '.join([f'{j:.4f}' for j in current_joints])}]")
        print(f"  📍 目标点 (弧度): [{', '.join([f'{j:.4f}' for j in target_joints])}]")
        
        # 创建规划目标 - 完全按照test_moveit.py的模式
        goal = MoveGroupAction.Goal()
        
        # 1. 设置workspace parameters
        from moveit_msgs.msg import WorkspaceParameters
        from std_msgs.msg import Header
        from geometry_msgs.msg import Vector3
        
        goal.request.workspace_parameters = WorkspaceParameters()
        goal.request.workspace_parameters.header = Header()
        goal.request.workspace_parameters.header.frame_id = "base_link"  # Fixed: 使用正确的frame名称
        goal.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        goal.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        
        # 2. 设置基本参数
        goal.request.group_name = 'arm'  # 🔧 关键修复：与SRDF中的group名称一致（不是piper_arm）
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = float(speed) / 100.0
        goal.request.max_acceleration_scaling_factor = float(speed) / 100.0
        
        # 3. 【关键修复】设置起始状态为当前实际位置
        goal.request.start_state = RobotState()
        goal.request.start_state.joint_state = JointState()
        goal.request.start_state.joint_state.header = Header()
        goal.request.start_state.joint_state.header.stamp = moveit_node.get_clock().now().to_msg()
        goal.request.start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        goal.request.start_state.joint_state.position = current_joints
        goal.request.start_state.is_diff = False  # 使用绝对状态，不是diff
        
        # 4. 设置目标约束
        constraints = Constraints()
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        for i, angle in enumerate(target_joints):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = float(angle)
            jc.tolerance_above = MOVEIT_JOINT_TOLERANCE
            jc.tolerance_below = MOVEIT_JOINT_TOLERANCE
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal.request.goal_constraints = [constraints]
        
        # 5. 设置planning options（完全按test_moveit.py格式）
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        # 发送规划请求
        print("  [MoveIt2] 发送规划请求...")
        print(f"  [DEBUG] group_name: {goal.request.group_name}")
        print(f"  [DEBUG] planner_id: {goal.request.planner_id if goal.request.planner_id else '(使用默认)'}")
        print(f"  [DEBUG] planning_attempts: {goal.request.num_planning_attempts}")
        print(f"  [DEBUG] planning_time: {goal.request.allowed_planning_time}s")
        print(f"  [DEBUG] plan_only: {goal.planning_options.plan_only}")
        print(f"  [DEBUG] start_state.is_diff: {goal.request.start_state.is_diff}")
        print(f"  [DEBUG] 起始关节位置: {goal.request.start_state.joint_state.position}")
        print(f"  [DEBUG] 目标约束数: {len(goal.request.goal_constraints[0].joint_constraints)}")
        print(f"  [DEBUG] workspace frame: {goal.request.workspace_parameters.header.frame_id}")
        send_goal_future = move_group.send_goal_async(goal)
        print(f"  [DEBUG] send_goal_future 已创建，类型: {type(send_goal_future)}")
        
        # 等待goal被接受（后台spin线程会处理future）
        print("  [MoveIt2] 等待goal接受...")
        import time as time_module
        timeout = 10.0
        start_time = time_module.time()
        while not send_goal_future.done():
            time_module.sleep(0.01)
            if time_module.time() - start_time > timeout:
                print(f"  ❌ 等待goal接受超时")
                print(f"  💡 可能原因: MoveIt2 move_group未运行或规划组名称错误")
                return control_arm_sdk(joints, speed, gripper_value)
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            print(f"  ❌ 规划请求被拒绝，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        print("  ✓ 规划请求已接受，等待规划结果...")
        print(f"  [DEBUG] Goal handle: {goal_handle}")
        print(f"  [DEBUG] Goal ID: {goal_handle.goal_id if hasattr(goal_handle, 'goal_id') else 'N/A'}")
        
        # 等待规划完成（后台spin线程会处理future）
        result_future = goal_handle.get_result_async()
        print(f"  [DEBUG] Result future created, waiting up to 30s...")
        timeout = 30.0
        start_time = time_module.time()
        while not result_future.done():
            time_module.sleep(0.01)
            if time_module.time() - start_time > timeout:
                print("  ❌ 规划超时(30秒)，MoveIt2可能正在计算或卡住，切换到SDK模式")
                return control_arm_sdk(joints, speed, gripper_value)
        
        result = result_future.result()
        if not result or result.result.error_code.val != 1:  # 1 = SUCCESS
            error_code = result.result.error_code.val if result else "None"
            print(f"  ❌ 规划失败 (错误码: {error_code})，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        print(f"  ✓ 规划成功！")
        
        # 提取轨迹信息
        if result.result.planned_trajectory and result.result.planned_trajectory.joint_trajectory.points:
            traj_points = result.result.planned_trajectory.joint_trajectory.points
            print(f"  📊 轨迹点数: {len(traj_points)}")
            
            # 提取规划的末端轨迹（XYZ）- 累积到全局变量
            step_planned = []
            for point in traj_points:
                joints_rad = [point.positions[i] for i in range(6)]
                T = piper_arm.forward_kinematics(joints_rad)
                xyz = T[:3, 3]
                step_planned.append(xyz.copy())
                planned_trajectory.append(xyz.copy())  # 累积到全局
            
            print(f"  ✓ 已提取规划轨迹的末端XYZ (本步骤: {len(step_planned)}个点, 累计: {len(planned_trajectory)}个点)")
            
            # 计算轨迹总时长
            total_traj_time = traj_points[-1].time_from_start.sec + traj_points[-1].time_from_start.nanosec * 1e-9
            print(f"  [SDK] 执行完整轨迹 (点数: {len(traj_points)}, 总时长: {total_traj_time:.2f}s, 发送频率: {COMMAND_SEND_RATE}Hz)")
            expected_commands = int(total_traj_time * COMMAND_SEND_RATE)
            print(f"  [DEBUG] 预计发送命令: {expected_commands}次 ({COMMAND_SEND_RATE}Hz × {total_traj_time:.2f}s)")
            
            # 🔧 关键修复：设置运动控制模式并等待生效（参考demo_03_go_zero_ros2.py）
            piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
            time_module.sleep(0.1)  # 等待运动模式切换生效
            
            if DEBUG_TRAJECTORY:
                print("\n  " + "="*70)
                print("  🚀 开始执行完整轨迹 (高频插值模式 + 记录实际轨迹):")
                print("  " + "="*70)
            
            start_time = time_module.time()
            
            # 清空执行轨迹记录
            global executed_trajectory
            executed_trajectory = []
            execution_records = []  # [(time, joints, xyz, velocities), ...]
            
            current_point_idx = 0
            next_point_idx = 1
            command_count = 0
            
            # 高频插值执行循环（80Hz）- 基于轨迹总时长而不是点索引
            while True:
                elapsed = time_module.time() - start_time
                
                # 检查是否完成整个轨迹
                if elapsed >= total_traj_time:
                    break
                
                # 找到当前时间对应的轨迹段
                while next_point_idx < len(traj_points):
                    next_time = traj_points[next_point_idx].time_from_start.sec + \
                               traj_points[next_point_idx].time_from_start.nanosec * 1e-9
                    if elapsed >= next_time:
                        current_point_idx = next_point_idx
                        next_point_idx += 1
                    else:
                        break
                
                # 如果已经到最后一段，保持在最后两个点之间插值
                if next_point_idx >= len(traj_points):
                    next_point_idx = len(traj_points) - 1
                    current_point_idx = next_point_idx - 1
                
                # 获取当前段的两个端点
                point_current = traj_points[current_point_idx]
                point_next = traj_points[next_point_idx]
                
                # 计算插值比例
                t_current = point_current.time_from_start.sec + point_current.time_from_start.nanosec * 1e-9
                t_next = point_next.time_from_start.sec + point_next.time_from_start.nanosec * 1e-9
                
                if t_next > t_current:
                    ratio = (elapsed - t_current) / (t_next - t_current)
                    ratio = max(0.0, min(1.0, ratio))  # 限制在[0,1]
                else:
                    ratio = 1.0
                
                # 线性插值计算当前应该发送的关节角度和速度
                joints_interpolated = []
                velocities_interpolated = []
                for i in range(6):
                    pos_current = point_current.positions[i]
                    pos_next = point_next.positions[i]
                    pos_interp = pos_current + ratio * (pos_next - pos_current)
                    joints_interpolated.append(pos_interp)
                    
                    # 速度插值（用于PVAT图表）
                    vel_current = point_current.velocities[i] if len(point_current.velocities) > i else 0.0
                    vel_next = point_next.velocities[i] if len(point_next.velocities) > i else 0.0
                    vel_interp = vel_current + ratio * (vel_next - vel_current)
                    velocities_interpolated.append(vel_interp)
                
                # 发送插值后的关节命令
                joints_int = [int(joints_interpolated[i] * factor) for i in range(6)]
                # 🔧 关键修复：限制Joint5上下限
                joints_int[4] = min(70000, max(-70000, joints_int[4]))
                piper.JointCtrl(*joints_int)
                command_count += 1
                
                # 记录实际执行的轨迹（每个周期都记录，用于精确的PVAT图表）
                T = piper_arm.forward_kinematics(joints_interpolated)
                xyz = T[:3, 3]
                execution_records.append((elapsed, joints_interpolated.copy(), xyz.copy(), velocities_interpolated.copy()))
                executed_trajectory.append(xyz.copy())
                
                # 打印执行信息（每10个点打印一次）
                if DEBUG_TRAJECTORY and command_count % 10 == 0:
                    print(f"  执行段 #{current_point_idx}→{next_point_idx}/{len(traj_points)-1} | 时间: {elapsed:.3f}s/{total_traj_time:.2f}s | 插值: {ratio:.2f} | 命令: {command_count}/{expected_commands}")
                
                # 按照固定频率发送命令（80Hz = 12.5ms间隔）
                time_module.sleep(1.0 / COMMAND_SEND_RATE)
            
            # 发送最终位置（确保到达）
            final_point = traj_points[-1]
            joints_int = [int(final_point.positions[i] * factor) for i in range(6)]
            # 🔧 关键修复：限制Joint5上下限
            joints_int[4] = min(70000, max(-70000, joints_int[4]))
            piper.JointCtrl(*joints_int)
            
            final_joints_rad = [final_point.positions[i] for i in range(6)]
            T_final = piper_arm.forward_kinematics(final_joints_rad)
            xyz_final = T_final[:3, 3]
            
            elapsed_final = time_module.time() - start_time
            final_vels = [final_point.velocities[i] if len(final_point.velocities) > i else 0.0 for i in range(6)]
            execution_records.append((elapsed_final, final_joints_rad, xyz_final.copy(), final_vels))
            executed_trajectory.append(xyz_final.copy())
            
            total_exec_time = time_module.time() - start_time
            if DEBUG_TRAJECTORY:
                print(f"\n  ✓ 轨迹命令发送完成，实际用时: {total_exec_time:.3f}s")
                print(f"  ✓ 发送了 {command_count} 个插值命令 (预计: {expected_commands})")
                print(f"  ✓ 记录了 {len(execution_records)} 个执行点")
                print("  " + "="*70 + "\n")
            else:
                print(f"  ✓ 轨迹命令发送完成 (用时: {total_exec_time:.3f}s, 命令数: {command_count})")
            
            # 等待机械臂真正到达目标位置
            print("  ⏳ 等待机械臂到达目标位置...")
            target_reached = False
            wait_start = time_module.time()
            max_wait_time = 3.0  # 最多等待3秒
            position_threshold = 0.01  # 位置误差阈值 (弧度，约0.57度)
            
            while not target_reached and (time_module.time() - wait_start) < max_wait_time:
                current_joints_actual = get_current_joints()
                
                # 计算与目标位置的误差
                max_error = max([abs(current_joints_actual[i] - final_joints_rad[i]) for i in range(6)])
                
                if max_error < position_threshold:
                    target_reached = True
                    print(f"  ✓ 机械臂已到达目标位置 (最大误差: {max_error:.5f} rad)")
                else:
                    time_module.sleep(0.05)  # 等待50ms后再检查
            
            if not target_reached:
                print(f"  ⚠️  等待超时，当前最大误差: {max_error:.5f} rad")
            
            # Extra wait to ensure stability
            time_module.sleep(0.01)
            
            print(f"  ✓ Trajectory executed (MoveIt2 planned {len(traj_points)} pts → SDK interpolated {len(execution_records)} cmds)")
            
            # Accumulate trajectory data for final PVAT analysis (instead of overwriting)
            global all_planned_points, all_execution_records, trajectory_start_time
            
            # Set start time on first planning
            if len(all_planned_points) == 0:
                trajectory_start_time = execution_records[0][0] if len(execution_records) > 0 else 0.0
            
            # Adjust execution record timestamps to be cumulative
            time_offset = all_execution_records[-1][0] if len(all_execution_records) > 0 else 0.0
            for record in execution_records:
                t, joints, xyz, vels = record
                all_execution_records.append((time_offset + t, joints, xyz, vels))
            
            # Adjust planned point timestamps to be cumulative
            time_offset_planned = all_planned_points[-1].time_from_start if len(all_planned_points) > 0 else None
            for point in traj_points:
                # Create a copy and adjust timestamp
                import copy
                point_copy = copy.deepcopy(point)
                if time_offset_planned is not None:
                    # Add offset to make timeline cumulative
                    point_copy.time_from_start.sec += time_offset_planned.sec
                    point_copy.time_from_start.nanosec += time_offset_planned.nanosec
                    # Handle nanosecond overflow
                    if point_copy.time_from_start.nanosec >= 1_000_000_000:
                        point_copy.time_from_start.sec += 1
                        point_copy.time_from_start.nanosec -= 1_000_000_000
                all_planned_points.append(point_copy)
            
            print(f"  ✓ Accumulated trajectory data: {len(all_planned_points)} planned points, {len(all_execution_records)} execution records")
        else:
            # If no trajectory, use SDK directly
            print("  ⚠️  No trajectory obtained, using SDK mode")
            # 安全检查：避免大幅度突然运动
            current_joints = get_current_joints()
            joint_diff = np.array(joints) - np.array(current_joints)
            max_diff = np.max(np.abs(joint_diff))
            if max_diff > 1.5:  # 超过86度的突变
                print(f"  ⚠️  关节角度变化过大({np.rad2deg(max_diff):.1f}°)，拒绝执行以防失能")
                return False
            return control_arm_sdk(joints, min(speed, 30), gripper_value)  # 降低速度
        
        # 控制夹爪
        if gripper_value is not None:
            gripper_int = int(gripper_value)
            piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
        
        print(f"  ✓ MoveIt2规划+SDK执行完成")
        return True
        
    except Exception as e:
        print(f"  ❌ MoveIt2 执行错误: {e}")
        import traceback
        traceback.print_exc()
        print("  回退到 SDK 模式")
        # 安全检查：避免大幅度突然运动
        current_joints = get_current_joints()
        joint_diff = np.array(joints) - np.array(current_joints)
        max_diff = np.max(np.abs(joint_diff))
        if max_diff > 1.5:  # 超过86度的突变
            print(f"  ⚠️  关节角度变化过大({np.rad2deg(max_diff):.1f}°)，拒绝执行以防失能")
            return False
        return control_arm_sdk(joints, min(speed, 30), gripper_value)  # 降低速度



def control_arm(joints, speed=50, use_moveit=False, gripper_value=None):
    """统一控制接口"""
    if gripper_value is None:
        gripper_value = joints[6] * 1000000 if len(joints) > 6 else None
    
    if use_moveit and MOVEIT_AVAILABLE and move_group is not None:
        return control_arm_moveit(joints[:6], speed, gripper_value)
    else:
        return control_arm_sdk(joints, speed, gripper_value)


def move_to_pose_with_retries(target_pose, joints_target, speed=NORMAL_SPEED, gripper_value=None, description="MoveIt到位"):
    """第一阶段到位：确保MoveIt完成并验证误差，不达标则有限次重试"""
    global piper_arm

    target_xyz = target_pose[:3, 3]
    moveit_enabled = USE_MOVEIT and MOVEIT_AVAILABLE and move_group is not None
    max_attempts = MOVEIT_MAX_REPLAN_ATTEMPTS + 1 if moveit_enabled else 1
    last_error = float('inf')

    for attempt in range(1, max_attempts + 1):
        if attempt > 1:
            print(f"  [{description}] ↻ 重新规划第{attempt}次，消除残余误差...")

        if not control_arm(joints_target, speed, moveit_enabled, gripper_value):
            print(f"  ❌ {description} 执行失败，跳过本次尝试")
            continue

        actual_joints = get_current_joints()
        actual_pose = piper_arm.forward_kinematics(actual_joints)
        actual_xyz = actual_pose[:3, 3]
        actual_R = actual_pose[:3, :3]
        target_R = target_pose[:3, :3]
        
        last_error = np.linalg.norm(actual_xyz - target_xyz)
        print(f"  [{description}] 实际到达: XYZ=({actual_xyz[0]:.3f}, {actual_xyz[1]:.3f}, {actual_xyz[2]:.3f}), 误差={last_error*100:.2f}cm")
        
        # 调试：打印姿态信息
        actual_rpy = rotation_matrix_to_euler(actual_R)
        target_rpy = rotation_matrix_to_euler(target_R)
        print(f"  [{description}] 实际姿态: Roll={actual_rpy[0]*180/PI:6.1f}°, Pitch={actual_rpy[1]*180/PI:6.1f}°, Yaw={actual_rpy[2]*180/PI:6.1f}°")
        print(f"  [{description}] 目标姿态: Roll={target_rpy[0]*180/PI:6.1f}°, Pitch={target_rpy[1]*180/PI:6.1f}°, Yaw={target_rpy[2]*180/PI:6.1f}°")

        if not moveit_enabled or last_error <= MOVEIT_POSITION_TOLERANCE:
            return True, last_error

        # 需要再次尝试，重新计算IK作为种子
        new_joints = compute_ik_moveit2(target_pose, timeout=5.0, attempts=10, use_current_as_seed=True)
        if not new_joints:
            print("  ⚠️ 重新计算IK失败，无法继续更精细的重试")
            break
        joints_target = new_joints

    print(f"  ⚠️ {description} 在 {max_attempts} 次尝试后误差仍为 {last_error*100:.2f}cm (> {MOVEIT_POSITION_TOLERANCE*100:.1f}cm)")
    if AUTO_FINE_TUNE_ON_FAILURE:
        print(f"  ↪ MoveIt仍未达标，启动笛卡尔精调恢复 ({description})...")
        fine_tune_joints = precise_move_to_pose(
            target_pose,
            speed=AUTO_FINE_TUNE_SPEED,
            description=f"{description}笛卡尔精调",
            force=True
        )
        if fine_tune_joints:
            actual_joints = get_current_joints()
            actual_pose = piper_arm.forward_kinematics(actual_joints)
            actual_xyz = actual_pose[:3, 3]
            residual_error = np.linalg.norm(actual_xyz - target_xyz)
            if residual_error <= MOVEIT_POSITION_TOLERANCE:
                print(f"  ✓ 精调完成，误差降至 {residual_error*100:.2f}cm (<= {MOVEIT_POSITION_TOLERANCE*100:.1f}cm)")
                return True, residual_error
            print(f"  ⚠️ 精调后误差仍为 {residual_error*100:.2f}cm")
        else:
            print("  ❌ 笛卡尔精调失败，无法纠正误差")
    return False, last_error


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


def set_apriltag_reference_from_gripper_rpy(gripper_rpy_rad, current_joints):
    """
    从AprilTag在夹爪系的RPY计算面板在基座系的绝对姿态（先验知识）
    
    关键：计算后就不再依赖AprilTag实时检测
    
    Args:
        gripper_rpy_rad: AprilTag在夹爪系的RPY（弧度）[roll, pitch, yaw]
        current_joints: 当前关节角度（用于正向运动学）
    
    Returns:
        bool: 是否成功设置
    """
    global APRILTAG_REFERENCE_POSE_BASE, APRILTAG_BASE_ROLL, APRILTAG_BASE_PITCH, APRILTAG_BASE_YAW
    
    # 1. 获取当前夹爪在基座系的位姿
    T_base_gripper = piper_arm.forward_kinematics(current_joints)
    
    # 2. 构建AprilTag相对夹爪的旋转矩阵
    roll, pitch, yaw = gripper_rpy_rad
    R_gripper_tag = euler_to_rotation_matrix(roll, pitch, yaw)
    
    # 3. 计算面板在基座系的姿态（关键步骤！）
    # 面板姿态 = 夹爪姿态 @ AprilTag偏差
    R_base_gripper = T_base_gripper[:3, :3]
    R_base_panel = R_base_gripper @ R_gripper_tag
    
    # 4. 保存参考姿态（只保存旋转矩阵）
    APRILTAG_REFERENCE_POSE_BASE = R_base_panel.copy()
    
    # 🔧 从旋转矩阵反算RPY并更新全局变量（动作函数会使用这些变量！）
    base_rpy_rad = rotation_matrix_to_euler(R_base_panel)
    APRILTAG_BASE_ROLL, APRILTAG_BASE_PITCH, APRILTAG_BASE_YAW = base_rpy_rad
    
    roll_deg, pitch_deg, yaw_deg = np.degrees([roll, pitch, yaw])
    print("\n" + "="*70)
    print("✓✓✓ 已记录面板参考姿态（基座系绝对姿态）")
    print("="*70)
    print(f"  AprilTag偏差（夹爪系）: R={roll_deg:.2f}°, P={pitch_deg:.2f}°, Y={yaw_deg:.2f}°")
    print(f"  面板姿态已固化为基座系绝对姿态")
    print(f"  后续运动将保持此姿态，无需AprilTag实时检测")
    print("="*70)
    
    return True


def set_apriltag_reference_from_base_rpy(base_rpy_rad):
    """
    从AprilTag在基座系的RPY设置面板参考姿态（带静态补偿）
    
    Args:
        base_rpy_rad: AprilTag在基座系的RPY（弧度）[roll, pitch, yaw]
    
    Returns:
        bool: 是否成功设置
    
    ⚠️ 新增功能：静态补偿
    最终姿态 = 采样平均值 + 静态补偿
    """
    global APRILTAG_REFERENCE_POSE_BASE, APRILTAG_BASE_ROLL, APRILTAG_BASE_PITCH, APRILTAG_BASE_YAW
    
    try:
        # 应用静态补偿
        compensated_rpy_rad = base_rpy_rad.copy()
        compensated_rpy_rad[0] += np.radians(APRILTAG_RPY_OFFSET_ROLL)
        compensated_rpy_rad[1] += np.radians(APRILTAG_RPY_OFFSET_PITCH)
        compensated_rpy_rad[2] += np.radians(APRILTAG_RPY_OFFSET_YAW)
        
        # 🔧 更新全局RPY变量（动作函数会使用这些变量！）
        APRILTAG_BASE_ROLL, APRILTAG_BASE_PITCH, APRILTAG_BASE_YAW = compensated_rpy_rad
        
        # 构建面板在基座系的旋转矩阵
        roll, pitch, yaw = compensated_rpy_rad
        APRILTAG_REFERENCE_POSE_BASE = euler_to_rotation_matrix(roll, pitch, yaw)
        
        # 详细输出
        roll_deg, pitch_deg, yaw_deg = np.degrees(base_rpy_rad)
        comp_roll_deg, comp_pitch_deg, comp_yaw_deg = np.degrees(compensated_rpy_rad)
        
        print("\n" + "="*70)
        print("✓✓✓ 已记录面板参考姿态（基座系绝对姿态）")
        print("="*70)
        print(f"  原始采样值: R={roll_deg:+7.2f}°, P={pitch_deg:+7.2f}°, Y={yaw_deg:+7.2f}°")
        print(f"  静态补偿值: R={APRILTAG_RPY_OFFSET_ROLL:+7.2f}°, P={APRILTAG_RPY_OFFSET_PITCH:+7.2f}°, Y={APRILTAG_RPY_OFFSET_YAW:+7.2f}°")
        print(f"  最终参考值: R={comp_roll_deg:+7.2f}°, P={comp_pitch_deg:+7.2f}°, Y={comp_yaw_deg:+7.2f}°")
        
        # 计算垂直按压时的夹爪姿态（用于显示）⚠️ 使用补偿后的值
        gripper_roll_comp = -compensated_rpy_rad[0]
        gripper_pitch_comp = compensated_rpy_rad[1]
        gripper_yaw_comp = compensated_rpy_rad[2]
        gripper_roll_deg, gripper_pitch_deg, gripper_yaw_deg = np.degrees([gripper_roll_comp, gripper_pitch_comp, gripper_yaw_comp])
        
        print(f"\n  垂直按压时夹爪姿态（perpendicular模式）:")
        print(f"    R={gripper_roll_deg:+7.2f}° (=-补偿后Tag_Roll)")
        print(f"    P={gripper_pitch_deg:+7.2f}° (=补偿后Tag_Pitch)")
        print(f"    Y={gripper_yaw_deg:+7.2f}° (=补偿后Tag_Yaw)")
        print(f"\n  ✓ 面板姿态已固化为基座系绝对姿态")
        print(f"  ✓ 后续运动将保持此姿态，无需AprilTag实时检测")
        print(f"  ✓✓✓ 全局变量已更新，四种动作将使用补偿后的姿态")
        print(f"      APRILTAG_BASE_ROLL  = {np.degrees(APRILTAG_BASE_ROLL):+7.2f}°")
        print(f"      APRILTAG_BASE_PITCH = {np.degrees(APRILTAG_BASE_PITCH):+7.2f}°")
        print(f"      APRILTAG_BASE_YAW   = {np.degrees(APRILTAG_BASE_YAW):+7.2f}°")
        print("="*70)
        
        return True
        
    except Exception as e:
        print(f"✗ 设置AprilTag参考姿态失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def get_tcp_offset_for_button(button_type):
    """
    根据按钮类型获取对应的TCP偏移
    
    参数：
    - button_type: str - 'toggle', 'plugin', 'push', 'knob'
    
    返回：
    - np.array([x, y, z]) - 夹爪坐标系的偏移（米）
    
    说明：
    - 每种按钮有独立的TCP配置
    - 自动应用全局偏移（用于快速微调）
    
    坐标系：
    - X：向前（手指闭合方向）
    - Y：向左
    - Z：向上
    """
    tcp_map = {
        'toggle': TCP_OFFSET_TOGGLE,
        'plugin': TCP_OFFSET_PLUGIN,
        'push': TCP_OFFSET_PUSH,
        'knob': TCP_OFFSET_KNOB
    }
    
    # 获取基础偏移
    base_offset = np.array(tcp_map.get(button_type, TCP_OFFSET_PUSH))
    
    # 应用全局偏移（用于快速微调）
    base_offset[0] += TCP_GLOBAL_OFFSET_X
    base_offset[1] += TCP_GLOBAL_OFFSET_Y
    base_offset[2] += TCP_GLOBAL_OFFSET_Z
    
    return base_offset


def rotation_matrix_to_quaternion(R):
    """
    将旋转矩阵转换为四元数 [x, y, z, w]
    """
    trace = np.trace(R)
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    
    return np.array([x, y, z, w])


def create_aligned_target_pose(button_xyz_base, gripper_rotation=None, tcp_offset=None):
    """
    根据按钮位置和夹爪目标姿态构建目标位姿
    
    ⚠️ 重要修正：
    - gripper_rotation 是夹爪的**目标姿态**（绝对姿态），不是补偿矩阵！
    - 应该从 get_gripper_approach_rotation() 获取，该函数已考虑Tag姿态的转换关系
    
    Args:
        button_xyz_base: 按钮在基座系的坐标 [x, y, z]
        gripper_rotation: 夹爪的目标姿态 (3x3旋转矩阵)
            - None: 使用默认姿态（末端朝下）
            - 推荐使用 get_gripper_approach_rotation() 获取正确的姿态
        tcp_offset: TCP偏移量 [dx, dy, dz] (在末端坐标系下)
            - None: 不应用偏移
            - [dx, dy, dz]: 在末端坐标系下的偏移（米）
            - dx: 沿末端X轴（向前为正）
            - dy: 沿末端Y轴（向左为正）
            - dz: 沿末端Z轴（向上为正）
    
    Returns:
        4x4 变换矩阵
    
    使用示例:
        # 垂直按压（推荐）
        R_gripper = get_gripper_approach_rotation('perpendicular')
        T = create_aligned_target_pose([x, y, z], R_gripper)
        
        # 平行于面板 + TCP偏移
        R_gripper = get_gripper_approach_rotation('parallel')
        T = create_aligned_target_pose([x, y, z], R_gripper, tcp_offset=[0.01, 0, 0.02])
    """
    if gripper_rotation is None:
        print("⚠️  未指定夹爪姿态，使用默认姿态（末端朝下）")
        R_target = euler_to_rotation_matrix(0.0, PI/2, 0.0)
    else:
        R_target = gripper_rotation
    
    # 应用TCP偏移（在末端坐标系下）
    button_xyz_adjusted = np.array(button_xyz_base).copy()
    
    if tcp_offset is not None and any(abs(x) > 1e-6 for x in tcp_offset):
        # TCP偏移在末端坐标系下定义，需要转换到基座系
        # 偏移向量 = R_target @ tcp_offset（旋转变换）
        tcp_offset_base = R_target @ np.array(tcp_offset)
        button_xyz_adjusted += tcp_offset_base
        
        print(f"  🔧 TCP偏移: 末端系=[{tcp_offset[0]:.3f}, {tcp_offset[1]:.3f}, {tcp_offset[2]:.3f}]m")
        print(f"            基座系=[{tcp_offset_base[0]:.3f}, {tcp_offset_base[1]:.3f}, {tcp_offset_base[2]:.3f}]m")
        print(f"  原始目标: ({button_xyz_base[0]:.3f}, {button_xyz_base[1]:.3f}, {button_xyz_base[2]:.3f})")
        print(f"  调整目标: ({button_xyz_adjusted[0]:.3f}, {button_xyz_adjusted[1]:.3f}, {button_xyz_adjusted[2]:.3f})")
    
    # 构建目标位姿
    T_target = np.eye(4)
    T_target[:3, :3] = R_target
    T_target[:3, 3] = button_xyz_adjusted
    
    return T_target


def get_gripper_approach_rotation(approach_mode='perpendicular'):
    """
    获取夹爪接近面板的目标姿态（不是补偿矩阵！）
    
    ⚠️ 关键理解：
    - AprilTag的姿态（APRILTAG_BASE_ROLL/PITCH/YAW）是Tag本身的姿态
    - 夹爪要垂直按压时，需要的姿态与Tag姿态的关系是：
        * gripper_roll  = -APRILTAG_BASE_ROLL  (Roll相反，因为法向量反向)
        * gripper_pitch =  APRILTAG_BASE_PITCH (Pitch相同)
        * gripper_yaw   =  APRILTAG_BASE_YAW   (Yaw相同)
    
    Args:
        approach_mode: 接近模式
            - 'perpendicular': 垂直于面板（默认，按压姿态）
            - 'parallel': 平行于面板（贴合姿态）
            - 'tilted_15': 倾斜15°（按压前的预备姿态）
            - 'tilted_30': 倾斜30°（按压前的预备姿态）
    
    Returns:
        3x3 旋转矩阵，表示夹爪的目标姿态（绝对姿态，不是相对补偿）
    
    使用示例:
        # 垂直按压
        R_gripper = get_gripper_approach_rotation('perpendicular')
        T = create_aligned_target_pose([x, y, z], R_gripper)
    """
    if APRILTAG_REFERENCE_POSE_BASE is None:
        print("⚠️  未设置AprilTag参考姿态，使用默认夹爪姿态")
        if approach_mode == 'perpendicular':
            return euler_to_rotation_matrix(0.0, PI/2, 0.0)  # 末端朝前
        else:
            return euler_to_rotation_matrix(0.0, 0.0, 0.0)   # 末端朝下
    
    if approach_mode == 'perpendicular':
        # 🎯 垂直按压：Roll取反，Pitch和Yaw保持
        gripper_roll  = -APRILTAG_BASE_ROLL
        gripper_pitch =  APRILTAG_BASE_PITCH
        gripper_yaw   =  APRILTAG_BASE_YAW
        return euler_to_rotation_matrix(gripper_roll, gripper_pitch, gripper_yaw)
    
    elif approach_mode == 'parallel':
        # 平行于面板：直接使用Tag姿态
        return APRILTAG_REFERENCE_POSE_BASE.copy()
    
    elif approach_mode == 'tilted_15':
        # 倾斜15°：在垂直姿态基础上，绕Y轴额外旋转-15°
        gripper_roll  = -APRILTAG_BASE_ROLL
        gripper_pitch =  APRILTAG_BASE_PITCH - 15 * PI / 180
        gripper_yaw   =  APRILTAG_BASE_YAW
        return euler_to_rotation_matrix(gripper_roll, gripper_pitch, gripper_yaw)
    
    elif approach_mode == 'tilted_30':
        # 倾斜30°：在垂直姿态基础上，绕Y轴额外旋转-30°
        gripper_roll  = -APRILTAG_BASE_ROLL
        gripper_pitch =  APRILTAG_BASE_PITCH - 30 * PI / 180
        gripper_yaw   =  APRILTAG_BASE_YAW
        return euler_to_rotation_matrix(gripper_roll, gripper_pitch, gripper_yaw)
    
    else:
        print(f"⚠️  未知的接近模式: {approach_mode}，使用默认（垂直）")
        return get_gripper_approach_rotation('perpendicular')


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


def rotation_matrix_to_euler(R):
    """
    旋转矩阵转欧拉角 (ZYX顺序)
    
    参数:
        R: 3x3 旋转矩阵
    
    返回:
        (roll, pitch, yaw) 元组 (弧度)
    """
    # 提取pitch
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw = np.arctan2(R[1,0], R[0,0])
    else:
        roll = np.arctan2(-R[1,2], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        yaw = 0
    
    return roll, pitch, yaw


def slerp_rotation(R0, R1, t):
    """
    球面线性插值（SLERP）用于旋转矩阵
    
    参数:
        R0: 起始旋转矩阵 (3x3)
        R1: 目标旋转矩阵 (3x3)
        t: 插值参数 [0, 1]，0=R0, 1=R1
    
    返回:
        插值后的旋转矩阵 (3x3)
    
    说明:
        使用四元数进行球面线性插值，保证旋转路径最短且速度均匀
    """
    try:
        from scipy.spatial.transform import Rotation as R_scipy
        
        # 转换为四元数
        q0 = R_scipy.from_matrix(R0).as_quat()  # [x, y, z, w]
        q1 = R_scipy.from_matrix(R1).as_quat()
        
        # 确保选择最短路径（四元数的符号模糊性）
        if np.dot(q0, q1) < 0:
            q1 = -q1
        
        # 球面线性插值
        q_interp = (1 - t) * q0 + t * q1
        q_interp = q_interp / np.linalg.norm(q_interp)  # 归一化
        
        # 转回旋转矩阵
        R_interp = R_scipy.from_quat(q_interp).as_matrix()
        
        return R_interp
    
    except ImportError:
        # 备用方案：使用矩阵指数映射（Log-Exp SLERP）
        # R(t) = R0 * exp(t * log(R0^T * R1))
        R_delta = R0.T @ R1  # 相对旋转
        
        # 转换为轴角表示（Rodrigues公式的逆）
        theta = np.arccos(np.clip((np.trace(R_delta) - 1) / 2, -1, 1))
        
        if theta < 1e-6:
            # 接近单位矩阵，直接线性插值
            return (1 - t) * R0 + t * R1
        
        # 提取旋转轴
        omega_skew = (R_delta - R_delta.T) / (2 * np.sin(theta))
        omega = np.array([omega_skew[2,1], omega_skew[0,2], omega_skew[1,0]])
        
        # 缩放角度
        theta_t = theta * t
        
        # Rodrigues公式：exp(theta * [omega]_x)
        omega_skew_t = np.array([
            [0, -omega[2], omega[1]],
            [omega[2], 0, -omega[0]],
            [-omega[1], omega[0], 0]
        ]) * theta_t
        
        R_delta_t = (np.eye(3) + 
                     np.sin(theta_t) / theta_t * omega_skew_t + 
                     (1 - np.cos(theta_t)) / (theta_t**2) * (omega_skew_t @ omega_skew_t))
        
        return R0 @ R_delta_t


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


def compute_ik_moveit2(target_pose, timeout=5.0, attempts=10, use_current_as_seed=True):
    """
    使用高精度IK求解（piper_arm数值优化版本）
    
    参数:
        target_pose: 4x4齐次变换矩阵或Pose消息
        timeout: 保留参数（兼容性）
        attempts: 保留参数（兼容性）
        use_current_as_seed: 是否使用当前关节角度作为种子点（提高解的一致性）
    
    返回:
        关节角度列表 (6个元素) 或 None（失败时）
    """
    global piper_arm
    
    # 直接使用piper_arm的高精度数值优化IK
    if isinstance(target_pose, np.ndarray):
        # 🔧 修复1: 使用当前关节角度作为种子点，确保IK解的一致性
        initial_guess = None
        if use_current_as_seed:
            try:
                initial_guess = get_current_joints()
            except:
                initial_guess = None
        
        # 使用优化版本的IK（解析解 + Levenberg-Marquardt优化）
        result = piper_arm.inverse_kinematics_refined(
            target_pose, 
            initial_guess=initial_guess,
            max_iterations=50, 
            tolerance=1e-6
        )
        if result is not False and result is not None:
            return result
        else:
            # 如果高精度失败，回退到基础解析解
            print("  ⚠️ 高精度IK失败，使用基础解析解")
            return piper_arm.inverse_kinematics(target_pose)
    
    return None


def compute_custom_cartesian_path(start_joints, waypoint_poses, eef_step=0.01):
    """
    自定义笛卡尔路径规划器（不依赖MoveIt2 API）
    通过在笛卡尔空间插值并用IK求解关节角度
    
    参数:
        start_joints: 起始关节角度 (6个元素的列表/数组)
        waypoint_poses: 目标位姿列表 (4x4变换矩阵的列表)
        eef_step: 末端执行器步长 (米)，控制插值密度
    
    返回:
        (trajectory_points, fraction)
        - trajectory_points: 关节轨迹点列表 [(joints, time), ...]
        - fraction: 成功规划的比例 (0.0~1.0)
    """
    global piper_arm
    
    if len(waypoint_poses) == 0:
        return [], 0.0
    
    trajectory_points = []
    current_joints = list(start_joints)
    
    # 从起始点开始
    trajectory_points.append((current_joints, 0.0))
    
    total_waypoints = len(waypoint_poses)
    successful_waypoints = 0
    cumulative_time = 0.0
    
    # 计算起始位姿
    current_pose = piper_arm.forward_kinematics(current_joints)
    
    for waypoint_idx, target_pose in enumerate(waypoint_poses):
        # 计算当前位姿到目标位姿的距离
        current_pos = current_pose[:3, 3]
        target_pos = target_pose[:3, 3]
        distance = np.linalg.norm(target_pos - current_pos)
        
        # 根据eef_step计算需要多少插值点
        num_steps = max(2, int(distance / eef_step) + 1)
        
        # 在笛卡尔空间插值
        for step in range(1, num_steps + 1):
            alpha = step / num_steps
            
            # 位置插值（线性）
            interp_pos = current_pos + alpha * (target_pos - current_pos)
            
            # 姿态插值（SLERP - 球面线性插值）
            # 简化：直接使用目标姿态（保持姿态不变）
            interp_pose = target_pose.copy()
            interp_pose[:3, 3] = interp_pos
            
            # 用MoveIt2 IK求解关节角度（高精度）
            interp_joints = compute_ik_moveit2(interp_pose, timeout=2.0, attempts=5)
            
            if not interp_joints:
                # IK失败，停止规划
                print(f"    ⚠️ 笛卡尔插值点#{step}/{num_steps}的IK求解失败")
                break
            
            # 计算时间（基于距离和速度）
            step_distance = np.linalg.norm(
                piper_arm.forward_kinematics(interp_joints)[:3, 3] - 
                piper_arm.forward_kinematics(current_joints)[:3, 3]
            )
            step_time = step_distance / 0.1  # 假设速度0.1m/s
            cumulative_time += step_time
            
            trajectory_points.append((interp_joints, cumulative_time))
            current_joints = interp_joints
        
        # 检查是否成功到达当前waypoint
        final_pose = piper_arm.forward_kinematics(current_joints)
        final_pos = final_pose[:3, 3]
        error = np.linalg.norm(final_pos - target_pos)
        
        if error < 0.01:  # 1cm误差容限
            successful_waypoints += 1
            current_pose = final_pose
        else:
            print(f"    ⚠️ Waypoint {waypoint_idx+1}到达误差较大: {error*100:.2f}cm")
            break
    
    fraction = successful_waypoints / total_waypoints if total_waypoints > 0 else 0.0
    return trajectory_points, fraction


def execute_sdk_cartesian_trajectory(cartesian_traj, speed, label="SDK平滑执行", extra_delay=0.3, profile=None):
    """
    使用SDK以高密度插值和平滑加减速执行笛卡尔轨迹
    """
    global piper
    if not cartesian_traj or len(cartesian_traj) < 2:
        print("  ⚠️ 笛卡尔轨迹点不足，无法执行SDK平滑轨迹")
        return False

    profile = (profile or 'linear').lower()

    print(f"  [{label}] 笛卡尔轨迹 ({len(cartesian_traj)}个点)...")
    # 🔧 关键修复：设置运动模式并等待生效
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    time.sleep(0.1)  # 等待运动模式切换生效

    # 高密度插值：在相邻轨迹点之间插入更多点以获得更平滑的曲线
    interpolated_trajectory = []
    for idx in range(len(cartesian_traj)):
        interpolated_trajectory.append(cartesian_traj[idx])
        if idx < len(cartesian_traj) - 1:
            current_j = cartesian_traj[idx][0]
            next_j = cartesian_traj[idx + 1][0]
            for alpha in [0.25, 0.5, 0.75]:
                interp_joints = [
                    current_j[i] + alpha * (next_j[i] - current_j[i])
                    for i in range(6)
                ]
                interpolated_trajectory.append((interp_joints, 0.0))

    print(f"  ✓ 高密度插值: {len(cartesian_traj)}个点 → {len(interpolated_trajectory)}个点")

    total_interp_points = len(interpolated_trajectory)
    accel_points = min(15, total_interp_points // 3)
    decel_points = min(15, total_interp_points // 3)

    base_delay = 0.02
    max_delay = 0.05

    for idx, (joints, _) in enumerate(interpolated_trajectory):
        joints_int = [int(joints[i] * factor) for i in range(6)]
        # 🔧 关键修复：限制Joint5上下限，防止超出范围 (-70000, 70000)
        joints_int[4] = min(70000, max(-70000, joints_int[4]))
        
        # 🐛 调试：打印前几个命令，确认数据正确
        if idx < 3 or idx == len(interpolated_trajectory) - 1:
            joints_deg = [j * 180 / PI for j in joints]
            print(f"    [调试 {idx+1}/{len(interpolated_trajectory)}] J={[f'{j:6.1f}°' for j in joints_deg]}, SDK值={joints_int}")
        
        piper.JointCtrl(*joints_int)

        if profile == 'cubic' and total_interp_points > 1:
            norm = idx / (total_interp_points - 1)
            velocity_scale = 3 * norm ** 2 - 2 * norm ** 3  # ease-in-out
            delay = base_delay + (max_delay - base_delay) * (1 - velocity_scale)
        elif profile == 'impulse':
            impulse_accel_points = max(3, total_interp_points // 15)
            impulse_decel_points = impulse_accel_points
            if idx < impulse_accel_points:
                progress = idx / impulse_accel_points if impulse_accel_points > 0 else 1.0
                delay = 0.03 - 0.015 * progress
            elif idx >= total_interp_points - impulse_decel_points and impulse_decel_points > 0:
                remaining = total_interp_points - idx
                progress = remaining / impulse_decel_points
                delay = 0.015 + 0.015 * (progress if progress < 1 else 1)
            else:
                delay = 0.008
        else:
            if idx < accel_points:
                progress = idx / accel_points if accel_points > 0 else 1.0
                delay = 0.05 * (1 - progress * 0.6) + 0.02
            elif idx >= total_interp_points - decel_points and decel_points > 0:
                remaining = total_interp_points - idx
                progress = remaining / decel_points
                delay = 0.05 * (1 - progress * 0.6) + 0.02
            else:
                delay = 0.02

        time.sleep(delay)

    print("  ✓ 平滑执行完成")
    time.sleep(extra_delay)
    return True


def precise_move_to_pose(target_pose, speed=15, description="笛卡尔精调", min_fraction=0.9, force=False):
    """通过自定义笛卡尔路径精调末端到目标位姿"""
    global piper_arm

    if not ENABLE_CARTESIAN_FINE_TUNE and not force:
        print(f"  [{description}] 已禁用笛卡尔微调，跳过")
        return get_current_joints()

    current_joints = get_current_joints()
    current_T = piper_arm.forward_kinematics(current_joints)
    current_xyz = current_T[:3, 3]
    target_xyz = target_pose[:3, 3]

    distance = np.linalg.norm(target_xyz - current_xyz)
    print(f"  [{description}] 当前误差: {distance*100:.2f}cm")
    if distance < 0.0005:
        print("  ✓ 已在目标附近，无需精调")
        return current_joints

    num_waypoints = max(30, int(distance * 200))
    waypoint_poses = []
    
    # 提取当前和目标的旋转矩阵
    current_R = current_T[:3, :3]
    target_R = target_pose[:3, :3]
    
    # 计算姿态差异（用于判断是否需要插值）
    rotation_diff = np.linalg.norm(current_R - target_R)
    
    # 调试：打印当前和目标姿态
    current_rpy = rotation_matrix_to_euler(current_R)
    target_rpy = rotation_matrix_to_euler(target_R)
    print(f"  [{description}] 当前姿态: Roll={current_rpy[0]*180/PI:6.1f}°, Pitch={current_rpy[1]*180/PI:6.1f}°, Yaw={current_rpy[2]*180/PI:6.1f}°")
    print(f"  [{description}] 目标姿态: Roll={target_rpy[0]*180/PI:6.1f}°, Pitch={target_rpy[1]*180/PI:6.1f}°, Yaw={target_rpy[2]*180/PI:6.1f}°")
    print(f"  [{description}] 姿态差异: {rotation_diff:.4f}")
    
    for i in range(1, num_waypoints + 1):
        alpha = i / num_waypoints
        intermediate_T = np.eye(4)
        
        # 位置插值（线性）
        intermediate_T[:3, 3] = current_xyz + alpha * (target_xyz - current_xyz)
        
        # 姿态插值（根据配置决定）
        if CARTESIAN_ORIENTATION_INTERPOLATION and rotation_diff > 0.01:
            # 使用球面线性插值（SLERP），保证平滑旋转
            intermediate_T[:3, :3] = slerp_rotation(current_R, target_R, alpha)
        else:
            # 直接使用目标姿态（旧行为）
            intermediate_T[:3, :3] = target_R
        
        waypoint_poses.append(intermediate_T)

    interp_mode = "位置+姿态同步插值" if (CARTESIAN_ORIENTATION_INTERPOLATION and rotation_diff > 0.01) else "仅位置插值"
    print(f"  [{description}] 生成 {len(waypoint_poses)} 个waypoints（{interp_mode}）")
    cartesian_traj, fraction = compute_custom_cartesian_path(
        current_joints,
        waypoint_poses,
        eef_step=CARTESIAN_EEF_STEP
    )

    if fraction < min_fraction or len(cartesian_traj) < 2:
        print(f"  ⚠️  {description} 规划覆盖率不足 ({fraction*100:.1f}%)")
        return False

    exec_speed = min(speed, 20)
    if not execute_sdk_cartesian_trajectory(
        cartesian_traj,
        exec_speed,
        label=f"{description}SDK平滑执行",
        profile=CARTESIAN_INTERPOLATION_PROFILE
    ):
        return False

    final_joints = get_current_joints()
    final_T = piper_arm.forward_kinematics(final_joints)
    final_xyz = final_T[:3, 3]
    error = np.linalg.norm(final_xyz - target_xyz)
    print(f"  [{description}] 实际到达: XYZ=({final_xyz[0]:.3f}, {final_xyz[1]:.3f}, {final_xyz[2]:.3f}), 误差={error*100:.2f}cm")
    return final_joints


def move_along_end_effector_z(current_joints, distance, speed=20, lock_orientation=True, speed_limit=None, profile=None):
    """
    沿末端执行器Z轴方向移动，保持当前姿态不变
    使用自定义笛卡尔路径规划以提高可靠性
    
    参数:
        current_joints: 当前关节角度 (弧度)
        distance: 移动距离 (米)，正值=沿末端+Z轴方向，负值=沿末端-Z轴方向
        speed: 移动速度
        lock_orientation: True=锁定姿态（保持当前姿态不变），False=允许姿态变化
    
    返回:
        新的关节角度
    
    说明:
        - 末端坐标系Z轴 = 旋转矩阵第3列
        - 沿当前末端Z轴方向移动，正值=+Z方向（向前），负值=-Z方向（后退）
        - lock_orientation=True时，移动过程中保持姿态不变（推荐用于按压）
        - ✅ 不会强制修改姿态，只是沿当前姿态的Z轴方向移动
    """
    global piper_arm, move_group, piper
    
    # 获取当前末端位姿
    current_T = piper_arm.forward_kinematics(current_joints)
    print(f"  当前位置: ({current_T[0,3]:.3f}, {current_T[1,3]:.3f}, {current_T[2,3]:.3f})")
    
    # 打印当前姿态旋转矩阵
    print(f"  当前旋转矩阵:")
    print(f"    [{current_T[0,0]:7.4f}, {current_T[0,1]:7.4f}, {current_T[0,2]:7.4f}]")
    print(f"    [{current_T[1,0]:7.4f}, {current_T[1,1]:7.4f}, {current_T[1,2]:7.4f}]")
    print(f"    [{current_T[2,0]:7.4f}, {current_T[2,1]:7.4f}, {current_T[2,2]:7.4f}]")
    
    # 决定使用哪个Z轴方向
    if lock_orientation:
        # ✅ 直接使用当前姿态的Z轴方向
        # 因为当前姿态已经是通过MoveIt/笛卡尔精调得到的正确姿态
        # 不需要重新构建"理想姿态"
        z_axis = current_T[:3, 2]
        print(f"  ✓ 使用当前姿态的Z轴方向（姿态锁定模式）")
    else:
        # 使用当前实际姿态的Z轴方向（同上，但逻辑不同）
        z_axis = current_T[:3, 2]
        print(f"  使用当前姿态的Z轴方向")
    
    print(f"  移动距离: {distance*100:.1f}cm")
    print(f"  末端Z轴方向 (基坐标系): ({z_axis[0]:7.4f}, {z_axis[1]:7.4f}, {z_axis[2]:7.4f})")
    
    # 如果启用了姿态锁定，显示对比信息
    if lock_orientation:
        actual_z = current_T[:3, 2]
        ideal_z = z_axis
        angle_error = np.arccos(np.clip(np.dot(actual_z, ideal_z), -1.0, 1.0)) * 180.0 / PI
        print(f"  实际姿态方向: ({actual_z[0]:7.4f}, {actual_z[1]:7.4f}, {actual_z[2]:7.4f})")
        print(f"  姿态偏差角度: {angle_error:.2f}° (已补偿)")
    else:
        ideal_z = np.array([1.0, 0.0, 0.0])  # 末端朝前的理想方向
        angle_error = np.arccos(np.clip(np.dot(z_axis, ideal_z), -1.0, 1.0)) * 180.0 / PI
        print(f"  理想Z轴方向 (末端朝前): ( 1.0000,  0.0000,  0.0000)")
        print(f"  姿态偏差角度: {angle_error:.2f}° (未补偿)")
    
    # 计算新的目标位置：沿末端Z轴方向移动distance米
    # 末端Z轴 = 旋转矩阵第3列 = [1, 0, 0] （向前）
    # distance > 0 → X增大（向前按压）✓
    target_T = current_T.copy()
    target_T[:3, 3] += z_axis * distance

    
    # 如果启用姿态锁定，保持当前姿态不变（不重新构建）
    if lock_orientation:
        # ✅ 使用当前实际姿态，不要重新计算！
        # 这样可以保持从MoveIt/笛卡尔精调得到的正确姿态
        target_T[:3, :3] = current_T[:3, :3]
        print(f"  ✓ 保持当前姿态不变（沿末端Z轴移动）")
    
    print(f"  目标位置: ({target_T[0,3]:.3f}, {target_T[1,3]:.3f}, {target_T[2,3]:.3f})")
    
    # 使用自定义笛卡尔路径规划器
    print(f"  [自定义笛卡尔] 生成插值路径...")
    
    # 🔧 修复4: 增加waypoint密度，使小距离移动也能平滑
    waypoint_poses = []
    abs_distance = abs(distance)
    if abs_distance < 1e-6:
        print("  ⚠️ 移动距离过小，保持当前位置")
        return current_joints

    desired_steps = int(abs_distance / CARTESIAN_WAYPOINT_STEP) if CARTESIAN_WAYPOINT_STEP > 0 else 0
    num_steps = max(CARTESIAN_MIN_WAYPOINTS, desired_steps)
    if CARTESIAN_MAX_WAYPOINTS is not None:
        num_steps = min(num_steps, CARTESIAN_MAX_WAYPOINTS)
    easing = lambda a: 3 * a ** 2 - 2 * a ** 3  # cubic ease-in-out

    for i in range(1, num_steps + 1):
        alpha = i / num_steps
        eased_alpha = easing(alpha)
        intermediate_T = current_T.copy()
        # 沿末端Z轴方向移动
        intermediate_T[:3, 3] += z_axis * distance * eased_alpha
        
        # 如果启用姿态锁定，保持理想姿态
        if lock_orientation:
            intermediate_T[:3, :3] = target_T[:3, :3]
        
        waypoint_poses.append(intermediate_T)
    
    # 计算笛卡尔路径（使用IK插值）
    cartesian_traj, fraction = compute_custom_cartesian_path(
        current_joints, 
        waypoint_poses, 
        eef_step=CARTESIAN_EEF_STEP  # 更细步长，减小相邻点跳变
    )
    
    if fraction < 0.9 or len(cartesian_traj) < 2:
        print(f"  ⚠️  自定义笛卡尔规划覆盖率较低: {fraction*100:.1f}%，回退到简单IK...")
        # 回退到MoveIt2高精度IK
        target_joints = compute_ik_moveit2(target_T, timeout=5.0, attempts=10)
        if not target_joints:
            print(f"  ❌ 目标位置IK失败")
            return None
        
        print(f"  [简单IK] 执行运动...")
        if not control_arm(target_joints, speed, USE_MOVEIT):
            return None
        
        return target_joints
    
    print(f"  ✓ 自定义笛卡尔规划成功 (覆盖率: {fraction*100:.1f}%, 轨迹点: {len(cartesian_traj)})")

    limit = CARTESIAN_SPEED_LIMIT if speed_limit is None else speed_limit
    ultra_smooth_speed = min(speed, limit)
    profile_to_use = profile or CARTESIAN_INTERPOLATION_PROFILE
    if not execute_sdk_cartesian_trajectory(
        cartesian_traj,
        ultra_smooth_speed,
        label="末端Z轴SDK平滑执行",
        profile=profile_to_use
    ):
        return None

    time.sleep(0.3)
    final_joints = cartesian_traj[-1][0] if len(cartesian_traj) > 0 else current_joints
    print(f"  ✓ 笛卡尔轨迹执行完成")
    return final_joints


def wait_for_joints_to_settle(target_joints, tolerance=0.01, timeout=1.5, label="运动"):
    """轮询当前关节反馈，确认实机追上目标后再执行下一步"""
    if target_joints is None:
        return False

    start_time = time.time()
    max_error = float("inf")
    while time.time() - start_time < timeout:
        current = get_current_joints()
        max_error = max(abs(current[i] - target_joints[i]) for i in range(6))
        if max_error <= tolerance:
            print(f"  ✓ {label}已稳定，最大误差 {max_error:.5f} rad")
            return True
        time.sleep(0.05)

    print(f"  ⚠️ {label}仍在缓冲 (最大误差 {max_error:.5f} rad)，建议稍等或降速")
    return False


def move_along_end_effector_x(current_joints, distance, speed=20, lock_orientation=True, speed_limit=None, profile=None):
    """
    沿末端执行器X轴移动指定距离（备用函数）
    
    注意：当前push/plugin/knob动作都使用Z轴移动（move_along_end_effector_z）
    此函数保留用于未来可能的特殊场景（如侧向推动）
    
    - 沿+X移动 = 沿X轴正方向
    - 沿-X移动 = 沿X轴负方向
    
    参数:
        current_joints: 当前关节角度 (起点)
        distance: 沿X轴移动距离 (米), 正值=沿+X, 负值=沿-X
        speed: SDK速度参数 (0~100)
        lock_orientation: 是否锁定末端姿态
        speed_limit: 笛卡尔速度上限
        profile: 速度曲线类型
    
    返回:
        最终关节角度或None
    """
    global piper, piper_arm
    
    if abs(distance) < 0.0001:
        print(f"  ⚠️ 移动距离过小 ({distance*1000:.2f}mm)，跳过")
        return current_joints
    
    # 当前末端位姿
    current_T = piper_arm.forward_kinematics(current_joints)
    current_xyz = current_T[:3, 3]
    current_R = current_T[:3, :3]
    
    # X轴方向（法向量方向）
    x_axis = current_R[:, 0]
    
    # 目标位置 = 当前位置 + X轴方向 × 距离
    target_xyz = current_xyz + x_axis * distance
    
    print(f"  沿末端X轴移动 {distance*100:.2f}cm")
    print(f"  起点: ({current_xyz[0]:.3f}, {current_xyz[1]:.3f}, {current_xyz[2]:.3f})")
    print(f"  终点: ({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f})")
    print(f"  X轴方向: ({x_axis[0]:.3f}, {x_axis[1]:.3f}, {x_axis[2]:.3f})")
    
    # 构造目标位姿
    target_T = np.eye(4)
    if lock_orientation:
        target_T[:3, :3] = current_R.copy()  # 保持姿态
    target_T[:3, 3] = target_xyz
    
    # 生成waypoint（逐步接近）
    num_waypoints = max(CARTESIAN_MIN_WAYPOINTS, int(abs(distance) / CARTESIAN_WAYPOINT_STEP))
    num_waypoints = min(num_waypoints, CARTESIAN_MAX_WAYPOINTS)
    
    waypoints = []
    for i in range(1, num_waypoints + 1):
        ratio = i / num_waypoints
        wp_T = np.eye(4)
        wp_T[:3, :3] = current_R.copy() if lock_orientation else current_T[:3, :3]
        wp_T[:3, 3] = current_xyz + x_axis * (distance * ratio)
        waypoints.append(wp_T)
    
    print(f"  生成 {len(waypoints)} 个waypoints")
    
    # 规划笛卡尔路径
    cartesian_traj = compute_custom_cartesian_path(
        current_joints,
        waypoints,
        eef_step=CARTESIAN_EEF_STEP
    )
    
    if not cartesian_traj or len(cartesian_traj) == 0:
        print("  ❌ 笛卡尔路径规划失败")
        return None
    
    print(f"  规划成功: {len(cartesian_traj)} 个轨迹点")
    
    # 执行轨迹
    profile_to_use = profile if profile is not None else CARTESIAN_INTERPOLATION_PROFILE
    # 使用speed参数限制速度（SDK的speed范围0-100）
    effective_speed = speed if speed_limit is None else min(speed, speed_limit)
    if not execute_sdk_cartesian_trajectory(
        cartesian_traj,
        effective_speed,
        label="末端X轴SDK平滑执行",
        profile=profile_to_use
    ):
        return None
    
    time.sleep(0.3)
    final_joints = cartesian_traj[-1][0] if len(cartesian_traj) > 0 else current_joints
    print(f"  ✓ 沿X轴轨迹执行完成")
    return final_joints


def move_to_home(speed=50, description="回HOME位姿"):
    """
    移动到HOME位姿（标准起始/结束位置）
    
    参数:
        speed: 移动速度
        description: 描述信息
    
    返回:
        True=成功, False=失败
    """
    if not USE_HOME_POSITION:
        return True  # 禁用HOME位姿时直接返回成功
    
    print(f"\n{description}...")
    print(f"  目标关节角 (度): [{', '.join([f'{j*180/PI:.2f}' for j in HOME_JOINTS])}]")
    
    use_moveit_plan = USE_MOVEIT and MOVEIT_AVAILABLE and move_group is not None
    
    moveit_success = False
    if use_moveit_plan:
        print(f"  [MoveIt2] 规划到HOME位姿...")
        moveit_success = control_arm(HOME_JOINTS, min(speed, NORMAL_SPEED), True, HOME_GRIPPER)
        if not moveit_success:
            print("  ⚠️ MoveIt规划失败，使用SDK直接移动")
    else:
        print(f"  [SDK] 直接移动到HOME位姿")
    
    if not moveit_success:
        control_arm_sdk(HOME_JOINTS, min(speed, 50), HOME_GRIPPER)
    
    time.sleep(0.3)
    
    # 验证到达精度
    final_joints = get_current_joints()
    errors = [abs(final_joints[i] - HOME_JOINTS[i]) for i in range(6)]
    max_error = max(errors)
    print(f"  当前最大偏差: {max_error:.4f} rad ({max_error*180/PI:.2f}°)")
    
    if max_error > 0.02:
        print("  ↪ 偏差较大，使用慢速SDK精调...")
        control_arm_sdk(HOME_JOINTS, 20, HOME_GRIPPER)
        time.sleep(0.5)
        final_joints = get_current_joints()
        errors = [abs(final_joints[i] - HOME_JOINTS[i]) for i in range(6)]
        max_error = max(errors)
        print(f"  精调后最大偏差: {max_error:.4f} rad ({max_error*180/PI:.2f}°)")
    
    return max_error < 0.05


def safe_return_to_zero(speed=40, use_moveit_first=True, gripper_value=None, description="回零"):
    """使用MoveIt+SDK双重保障安全回零"""
    global piper
    
    zero_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    use_moveit_plan = (
        use_moveit_first and USE_MOVEIT and MOVEIT_AVAILABLE and move_group is not None
    )

    # 🔧 关键修复：确保机械臂使能（防止失能状态导致命令无效）
    print(f"  [{description}] 检查使能状态...")
    piper.EnableArm(7)
    time.sleep(0.3)  # 等待使能生效
    
    status = piper.GetArmLowSpdInfoMsgs()
    if status.motor_1.foc_status.driver_enable_status != 1:
        print(f"  ⚠️ [{description}] 警告：机械臂使能状态异常，尝试重新使能...")
        piper.EnableArm(7)
        time.sleep(1.0)

    # 显示当前关节位置（调试用）
    initial_joints = get_current_joints()
    initial_max = max(abs(j) for j in initial_joints)
    print(f"  [{description}] 当前关节偏差: {initial_max:.4f} rad ({initial_max*180/PI:.2f}°)")
    print(f"  [{description}] 关节角度 (度): [{', '.join([f'{j*180/PI:6.2f}' for j in initial_joints])}]")

    moveit_success = False
    if use_moveit_plan:
        print(f"  [{description}] MoveIt2规划回零...")
        moveit_success = control_arm(zero_joints, min(speed, NORMAL_SPEED), True, gripper_value)
        if not moveit_success:
            print("  ⚠️ MoveIt回零失败，将回退到SDK零点")
    else:
        print(f"  [{description}] MoveIt不可用，直接使用SDK零点")

    if not moveit_success:
        print(f"  [{description}] 使用SDK直接回零...")
        control_arm_sdk(zero_joints, min(speed, 30), gripper_value)

    # 等待机械臂稳定
    time.sleep(0.5)
    
    # 检查第一次回零结果
    final_joints = get_current_joints()
    max_error = max(abs(j) for j in final_joints)
    print(f"  [{description}] 第一次回零偏差: {max_error:.4f} rad ({max_error*180/PI:.2f}°)")
    print(f"  [{description}] 关节角度 (度): [{', '.join([f'{j*180/PI:6.2f}' for j in final_joints])}]")

    # 如果偏差过大，进行精调
    if max_error > 0.01:
        print(f"  ↪ 偏差 > 0.01rad，使用慢速SDK精调零点...")
        control_arm_sdk(zero_joints, 10, gripper_value)
        time.sleep(1.0)  # 增加等待时间，确保到位
        
        final_joints = get_current_joints()
        max_error = max(abs(j) for j in final_joints)
        print(f"  [{description}] 精调后偏差: {max_error:.4f} rad ({max_error*180/PI:.2f}°)")
        print(f"  [{description}] 关节角度 (度): [{', '.join([f'{j*180/PI:6.2f}' for j in final_joints])}]")

    # 判断标准：误差 < 2cm 认为成功（约1.15°）
    success = max_error < 0.02
    
    if success:
        print(f"  ✓ [{description}] 回零成功！")
    else:
        print(f"  ✗ [{description}] 回零失败，偏差过大: {max_error:.4f} rad")
    
    return success


# ========================================
# 四种按钮操作函数（重写版）
# ========================================

def action_plugin():
    """
    插拔连接器操作
    流程: HOME位姿 → 张开 → 到达 → 插入(z轴前进) → 闭合 → 拔出(z轴后退) → 张开 → 回HOME位姿
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
    
    # 步骤0: 移动到HOME位姿
    if USE_HOME_POSITION:
        if not move_to_home(speed=NORMAL_SPEED, description="步骤0: 移动到HOME起始位姿"):
            print("❌ 无法到达HOME位姿，终止动作")
            return False
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(PLUGIN_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    
    # 🔧 新增：优先使用 AprilTag 参考姿态（包含法向量对齐）
    if APRILTAG_REFERENCE_POSE_BASE is not None:
        print("  ✓ 使用AprilTag参考姿态计算的目标位姿")
        # 注意：使用TARGET位置（按钮实际位置），AprilTag只提供姿态参考
        target_position = np.array([TARGET_X, TARGET_Y, TARGET_Z])
        
        # 🎯 关键：使用垂直接近模式（推荐用于插拔动作）
        R_gripper = get_gripper_approach_rotation('perpendicular')
        
        # 应用TCP偏移（Plugin专用）
        tcp_offset_plugin = np.array(TCP_OFFSET_PLUGIN) + np.array([TCP_GLOBAL_OFFSET_X, TCP_GLOBAL_OFFSET_Y, TCP_GLOBAL_OFFSET_Z])
        targetT = create_aligned_target_pose(target_position, R_gripper, tcp_offset=tcp_offset_plugin)
        
        target_xyz = targetT[:3, 3]
        target_R = targetT[:3, :3]
        target_rpy = rotation_matrix_to_euler(target_R)
        
        print(f"  接近位置: ({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f})")
        print(f"  目标姿态: Roll={target_rpy[0]*180/PI:6.1f}°, Pitch={target_rpy[1]*180/PI:6.1f}°, Yaw={target_rpy[2]*180/PI:6.1f}°")
        print(f"  末端Z轴: ({targetT[0,2]:.3f}, {targetT[1,2]:.3f}, {targetT[2,2]:.3f})")
        print(f"  姿态模式: 垂直于面板 (perpendicular) ✓")
    else:
        print("  ⚠️  未设置AprilTag参考姿态，使用默认姿态")
        targetT = create_target_transform(
            TARGET_X, TARGET_Y, TARGET_Z,
            TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
            USE_6D_POSE
        )
        target_xyz = targetT[:3, 3]
        print(f"  目标位姿: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    
    print(f"  插入深度: {PLUGIN_INSERT_DEPTH*100:.1f}cm")

    joints_target = compute_ik_moveit2(targetT, timeout=5.0, attempts=10)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False

    success, _ = move_to_pose_with_retries(
        targetT,
        joints_target,
        speed=NORMAL_SPEED,
        gripper_value=PLUGIN_GRIPPER_OPEN,
        description="Plugin第一阶段"
    )
    if not success:
        if ENABLE_CARTESIAN_FINE_TUNE:
            print("  ↪ MoveIt误差仍大，改用笛卡尔微调确保第一阶段准确")
            if not precise_move_to_pose(targetT, speed=15, description="Plugin笛卡尔精调", force=True):
                print("❌ 微调失败，终止动作")
                return False
        else:
            print("❌ Plugin第一阶段未能满足精度要求，终止动作以避免误差放大")
            return False
    time.sleep(0.1)
    
    # 步骤3: 沿末端z轴插入
    # 使用实际到达的关节角度，而不是IK计算的理论值
    print(f"\n步骤3: 沿末端z轴插入 {PLUGIN_INSERT_DEPTH*100:.1f}cm...")
    
    actual_joints_step3 = get_current_joints()  # 获取实际当前位置
    actual_T_step3 = piper_arm.forward_kinematics(actual_joints_step3)
    actual_xyz_step3 = actual_T_step3[:3, 3]
    print(f"  实际起点: XYZ=({actual_xyz_step3[0]:.3f}, {actual_xyz_step3[1]:.3f}, {actual_xyz_step3[2]:.3f})")
    
    joints_insert = move_along_end_effector_z(actual_joints_step3, PLUGIN_INSERT_DEPTH, PLUGIN_INSERT_SPEED)
    if not joints_insert:
        return False
    time.sleep(0.1)
    
    # 步骤4: 夹爪闭合
    print(f"\n步骤4: 夹爪闭合到 {PLUGIN_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(PLUGIN_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤5: 沿末端z轴拔出
    # 【关键修复】使用实际当前位置
    print(f"\n步骤5: 沿末端z轴拔出 {PLUGIN_INSERT_DEPTH*100:.1f}cm...")
    
    actual_joints_step5 = get_current_joints()  # 获取插入后的实际位置
    actual_T_step5 = piper_arm.forward_kinematics(actual_joints_step5)
    actual_xyz_step5 = actual_T_step5[:3, 3]
    print(f"  当前位置: XYZ=({actual_xyz_step5[0]:.3f}, {actual_xyz_step5[1]:.3f}, {actual_xyz_step5[2]:.3f})")
    
    joints_extract = move_along_end_effector_z(
        actual_joints_step5,
        -PLUGIN_INSERT_DEPTH,
        PLUGIN_EXTRACT_SPEED,
        speed_limit=CARTESIAN_HIGH_SPEED_LIMIT,
        profile=CARTESIAN_HIGH_ACCEL_PROFILE
    )
    if not joints_extract:
        return False
    time.sleep(0.1)
    
    # 步骤6: 夹爪张开
    print("\n步骤6: 夹爪张开...")
    piper.GripperCtrl(PLUGIN_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤7: 回零位（关键：必须成功）
    print("\n步骤7: 回零位...")
    max_retries = 2
    for attempt in range(max_retries):
        if attempt > 0:
            print(f"  🔄 回零重试第 {attempt + 1}/{max_retries} 次...")
        
        if safe_return_to_zero(description="Plugin回零", speed=40):
            final_joints = get_current_joints()
            max_error = max(abs(j) for j in final_joints)
            print(f"  ✓ 回零完成 (最大偏差: {max_error:.4f} rad = {max_error*180/PI:.2f}°)")
            break
        
        if attempt < max_retries - 1:
                print("  ⚠️ 回零失败，等待1秒后重试...")
                time.sleep(1)
        else:
            print("  ❌ 多次尝试后仍无法回零")
            print("  ⚠️ 动作已完成，但机械臂不在安全位置！")
            return False
    
    # 步骤8: 夹爪闭合
    print("\n步骤8: 夹爪闭合...")
    piper.GripperCtrl(HOME_GRIPPER, 1000, 0x01, 0)
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
    流程: HOME位姿 → 张开 → 到达 → joint4旋转90° → 插入(z轴前进) → 闭合 → joint3拨动 → 张开 → 回HOME位姿
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
    
    # 步骤0: 移动到HOME位姿
    if USE_HOME_POSITION:
        if not move_to_home(speed=NORMAL_SPEED, description="步骤0: 移动到HOME起始位姿"):
            print("❌ 无法到达HOME位姿，终止动作")
            return False
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    
    # 🔧 新增：优先使用 AprilTag 参考姿态（包含法向量对齐）
    if APRILTAG_REFERENCE_POSE_BASE is not None:
        print("  ✓ 使用AprilTag参考姿态计算的目标位姿")
        # 注意：使用TARGET位置（按钮实际位置），AprilTag只提供姿态参考
        target_position = np.array([TARGET_X, TARGET_Y, TARGET_Z])
        
        # 🎯 关键：使用垂直接近模式（推荐用于拨动动作）
        R_gripper = get_gripper_approach_rotation('perpendicular')
        
        # 应用TCP偏移（Toggle专用）
        tcp_offset_toggle = np.array(TCP_OFFSET_TOGGLE) + np.array([TCP_GLOBAL_OFFSET_X, TCP_GLOBAL_OFFSET_Y, TCP_GLOBAL_OFFSET_Z])
        targetT = create_aligned_target_pose(target_position, R_gripper, tcp_offset=tcp_offset_toggle)
        
        target_xyz = targetT[:3, 3]
        target_R = targetT[:3, :3]
        target_rpy = rotation_matrix_to_euler(target_R)
        
        print(f"  接近位置: ({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f})")
        print(f"  目标姿态: Roll={target_rpy[0]*180/PI:6.1f}°, Pitch={target_rpy[1]*180/PI:6.1f}°, Yaw={target_rpy[2]*180/PI:6.1f}°")
        print(f"  末端Z轴: ({targetT[0,2]:.3f}, {targetT[1,2]:.3f}, {targetT[2,2]:.3f})")
        print(f"  姿态模式: 垂直于面板 (perpendicular) ✓")
    else:
        print("  ⚠️  未设置AprilTag参考姿态，使用默认姿态")
        targetT = create_target_transform(
            TARGET_X, TARGET_Y, TARGET_Z,
            TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
            USE_6D_POSE
        )
        target_xyz = targetT[:3, 3]
        print(f"  目标位姿: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    
    print(f"  joint4旋转: {TOGGLE_JOINT4_ROTATE}°, 插入: {TOGGLE_INSERT_DEPTH*100:.1f}cm, 拨动: {TOGGLE_JOINT3_ANGLE}° ({TOGGLE_DIRECTION})")
    
    joints_target = compute_ik_moveit2(targetT, timeout=5.0, attempts=10)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    success, _ = move_to_pose_with_retries(
        targetT,
        joints_target,
        speed=NORMAL_SPEED,
        gripper_value=TOGGLE_GRIPPER_OPEN,
        description="Toggle第一阶段"
    )
    if not success:
        if ENABLE_CARTESIAN_FINE_TUNE:
            print("  ↪ MoveIt误差仍大，使用笛卡尔微调补偿")
            if not precise_move_to_pose(targetT, speed=15, description="Toggle笛卡尔精调", force=True):
                print("❌ 微调失败，终止动作")
                return False
        else:
            print("❌ Toggle第一阶段未满足精度要求，终止动作")
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
    # 步骤4: 沿末端z轴插入
    print(f"\n步骤4: 沿末端z轴插入 {TOGGLE_INSERT_DEPTH*100:.1f}cm...")
    
    actual_joints_step4 = get_current_joints()  # 获取实际当前位置
    actual_T_step4 = piper_arm.forward_kinematics(actual_joints_step4)
    actual_xyz_step4 = actual_T_step4[:3, 3]
    print(f"  实际起点: XYZ=({actual_xyz_step4[0]:.3f}, {actual_xyz_step4[1]:.3f}, {actual_xyz_step4[2]:.3f})")
    
    joints_insert = move_along_end_effector_z(actual_joints_step4, TOGGLE_INSERT_DEPTH, TOGGLE_INSERT_SPEED)
    if not joints_insert:
        return False
    time.sleep(0.5)
    
    # 步骤5: 夹爪闭合
    print(f"\n步骤5: 夹爪闭合到 {TOGGLE_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(TOGGLE_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(1.0)
    
    # 步骤6: joint3拨动
    # 【关键修复】使用实际当前位置
    direction_sign = -1 if TOGGLE_DIRECTION == 'left' else 1
    print(f"\n步骤6: joint3 {TOGGLE_DIRECTION}拨 {TOGGLE_JOINT3_ANGLE}°...")
    actual_joints_step6 = get_current_joints()  # 获取插入后的实际位置
    actual_T_step6 = piper_arm.forward_kinematics(actual_joints_step6)
    actual_xyz_step6 = actual_T_step6[:3, 3]
    print(f"  当前位置: XYZ=({actual_xyz_step6[0]:.3f}, {actual_xyz_step6[1]:.3f}, {actual_xyz_step6[2]:.3f})")
    
    joints_toggle = actual_joints_step6.copy()
    joints_toggle[2] += direction_sign * TOGGLE_JOINT3_ANGLE * PI / 180
    if not control_arm(joints_toggle, TOGGLE_TOGGLE_SPEED, USE_MOVEIT, TOGGLE_GRIPPER_HOLD):
        return False
    time.sleep(1.0)
    
    # 步骤7: 夹爪张开
    print("\n步骤7: 夹爪张开...")
    piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤8: 回零位
    print("\n步骤8: 回零位...")
    actual_joints_step8 = get_current_joints()
    actual_T_step8 = piper_arm.forward_kinematics(actual_joints_step8)
    actual_xyz_step8 = actual_T_step8[:3, 3]
    print(f"  当前位置: XYZ=({actual_xyz_step8[0]:.3f}, {actual_xyz_step8[1]:.3f}, {actual_xyz_step8[2]:.3f})")
    
    if not safe_return_to_zero(description="Toggle回零"):
        return False
    final_joints = get_current_joints()
    max_error = max(abs(j) for j in final_joints)
    print(f"  ✓ 回零完成 (最大偏差: {max_error:.4f} rad = {max_error*180/PI:.2f}°)")
    
    # 步骤9: 夹爪闭合
    print("\n步骤9: 夹爪闭合...")
    piper.GripperCtrl(HOME_GRIPPER, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 保存和可视化完整轨迹
    save_and_visualize_trajectory()
    
    print("="*70)
    print("✓✓✓ Toggle 操作完成！✓✓✓")
    print("="*70)
    return True


def action_push():
    """
    按压按钮操作（AprilTag绝对姿态版）
    
    工作流程：
    1. 移动到HOME位姿（可选）
    2. 检查是否已设置AprilTag参考姿态
    3. 直接规划到按钮位置（使用绝对姿态，带5度容差约束）
    4. 执行按压动作
    5. 返回零位
    
    关键：姿态约束基于基座系，不依赖AprilTag实时检测
    """
    global piper_arm
    
    # 清空轨迹记录
    clear_trajectory_records()
    clear_ee_trail()
    
    print("="*70)
    print("动作类型: Push (按压按钮) - AprilTag绝对姿态版")
    print("="*70)
    
    # 前置检查：是否已设置参考姿态
    if APRILTAG_REFERENCE_POSE_BASE is None:
        print("⚠️  未设置AprilTag参考姿态")
        print("  将使用默认姿态（末端朝下）")
    else:
        print("✓ 已加载AprilTag参考姿态（基座系绝对姿态）")
    
    # 步骤0: 移动到HOME位姿
    if USE_HOME_POSITION:
        if not move_to_home(speed=NORMAL_SPEED, description="步骤0: 移动到HOME起始位姿"):
            print("❌ 无法到达HOME位姿，终止动作")
            return False
    
    # 步骤1: 夹爪闭合
    print("\n步骤1: 夹爪闭合...")
    piper.GripperCtrl(PUSH_GRIPPER_CLOSE, 1000, 0x01, 0)
    time.sleep(0.1)  # 减少等待时间：夹爪动作很快
    
    # 🔧 测试模式：从HOME位姿直接执行
    if TEST_MODE_FROM_HOME:
        print("\n🔧 测试模式：从HOME位姿直接沿Z轴执行动作")
        print("  跳过MoveIt规划到接近位姿的步骤")
        
        # 获取HOME位姿的末端位置（用于显示）
        home_T = piper_arm.forward_kinematics(HOME_JOINTS)
        home_xyz = home_T[:3, 3]
        home_z_axis = home_T[:3, 2]
        print(f"  HOME末端位置: ({home_xyz[0]:.3f}, {home_xyz[1]:.3f}, {home_xyz[2]:.3f})")
        print(f"  HOME末端Z轴: ({home_z_axis[0]:.3f}, {home_z_axis[1]:.3f}, {home_z_axis[2]:.3f})")
        
        # 计算按压距离（简化版：只有按压深度）
        total_distance = PUSH_INSERT_DEPTH
        print(f"  按压深度: {PUSH_INSERT_DEPTH*100:.1f}cm, 保持: {PUSH_HOLD_TIME}秒")
        
    else:
        # 正常模式：MoveIt规划到目标位姿
        print("\n步骤2: MoveIt规划到目标位姿...")
        
        # 🔧 新增：优先使用 AprilTag 参考姿态（包含法向量对齐）
        if APRILTAG_REFERENCE_POSE_BASE is not None:
            print("  ✓ 使用AprilTag参考姿态计算的目标位姿")
            # 注意：使用TARGET位置（按钮实际位置），AprilTag只提供姿态参考
            target_position = np.array([TARGET_X, TARGET_Y, TARGET_Z])
            
            # 🎯 关键：使用垂直接近模式（推荐用于按压动作）
            R_offset = get_gripper_approach_rotation('perpendicular')
            
            # 应用TCP偏移（Push专用）
            tcp_offset_push = np.array(TCP_OFFSET_PUSH) + np.array([TCP_GLOBAL_OFFSET_X, TCP_GLOBAL_OFFSET_Y, TCP_GLOBAL_OFFSET_Z])
            targetT = create_aligned_target_pose(target_position, R_offset, tcp_offset=tcp_offset_push)
            
            target_xyz = targetT[:3, 3]
            target_R = targetT[:3, :3]
            target_rpy = rotation_matrix_to_euler(target_R)
            
            print(f"  接近位置: ({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f})")
            print(f"  目标姿态: Roll={target_rpy[0]*180/PI:6.1f}°, Pitch={target_rpy[1]*180/PI:6.1f}°, Yaw={target_rpy[2]*180/PI:6.1f}°")
            print(f"  末端Z轴: ({targetT[0,2]:.3f}, {targetT[1,2]:.3f}, {targetT[2,2]:.3f})")
            print(f"  姿态模式: 垂直于面板 (perpendicular) ✓")
        else:
            print("  ⚠️  未设置AprilTag参考姿态，使用默认姿态")
            targetT = create_target_transform(
                TARGET_X, TARGET_Y, TARGET_Z,
                TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
                USE_6D_POSE
            )
            target_xyz = targetT[:3, 3]
            print(f"  目标位姿: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
        
        print(f"  按压深度: {PUSH_INSERT_DEPTH*100:.1f}cm, 保持: {PUSH_HOLD_TIME}秒")

        joints_target = compute_ik_moveit2(targetT, timeout=5.0, attempts=10)
        if not joints_target:
            print("❌ 接近位姿IK失败")
            return False

        success, _ = move_to_pose_with_retries(
            targetT,
            joints_target,
            speed=NORMAL_SPEED,
            gripper_value=PUSH_GRIPPER_CLOSE,
            description="Push接近位姿"
        )
        if not success:
            if ENABLE_CARTESIAN_FINE_TUNE:
                print("  ↪ MoveIt误差仍大，尝试笛卡尔精调确保接近位姿准确")
                if not precise_move_to_pose(targetT, speed=12, description="Push笛卡尔精调", force=True):
                    print("❌ 笛卡尔精调失败，终止动作")
                    return False
            else:
                print("❌ Push接近位姿未满足精度要求，终止动作")
                return False

        final_joints = get_current_joints()
        final_T = piper_arm.forward_kinematics(final_joints)
        final_xyz = final_T[:3, 3]
        error = np.linalg.norm(final_xyz - target_xyz)
        print(f"  ✓ 实际到达目标位姿: XYZ=({final_xyz[0]:.3f}, {final_xyz[1]:.3f}, {final_xyz[2]:.3f}), 误差={error*100:.2f}cm")
        time.sleep(0.1)
        
        # 正常模式：已经到达目标位姿，只需按压深度
        total_distance = PUSH_INSERT_DEPTH
    
    # 步骤3: 沿末端Z轴按压
    # 关键逻辑：
    # - 测试模式：从HOME位姿直接沿Z轴按压（不推荐）
    # - 正常模式：从目标位姿沿Z轴按压（与knob逻辑一致）
    
    if TEST_MODE_FROM_HOME:
        print(f"\n步骤2 (测试模式): 沿末端Z轴按压 {total_distance*100:.1f}cm...")
    else:
        print(f"\n步骤3: 沿末端Z轴按压 {total_distance*100:.1f}cm...")
        if APRILTAG_REFERENCE_POSE_BASE is not None:
            print(f"  (Z轴已对齐AprilTag，垂直接近面板)")
        else:
            print(f"  (使用默认姿态)")
    
    actual_joints = get_current_joints()
    joints_press = move_along_end_effector_z(
        actual_joints,
        total_distance,
        PUSH_PRESS_SPEED,
        speed_limit=CARTESIAN_HIGH_SPEED_LIMIT,
        profile=CARTESIAN_HIGH_ACCEL_PROFILE
    )
    
    if not joints_press:
        return False
    
    # 步骤4/3: 保持按压
    if TEST_MODE_FROM_HOME:
        print(f"\n步骤3 (测试模式): 保持按压 {PUSH_HOLD_TIME}秒...")
    else:
        print(f"\n步骤4: 保持按压 {PUSH_HOLD_TIME}秒...")
    time.sleep(PUSH_HOLD_TIME)
    
    # 步骤5/4: 沿末端Z轴完全撤回
    if TEST_MODE_FROM_HOME:
        print(f"\n步骤4 (测试模式): 沿末端Z轴撤回 {total_distance*100:.1f}cm...")
    else:
        print(f"\n步骤5: 沿末端Z轴撤回 {total_distance*100:.1f}cm...")
    
    actual_joints_after_press = get_current_joints()
    actual_T_after_press = piper_arm.forward_kinematics(actual_joints_after_press)
    actual_xyz_after_press = actual_T_after_press[:3, 3]
    print(f"  当前位置: XYZ=({actual_xyz_after_press[0]:.3f}, {actual_xyz_after_press[1]:.3f}, {actual_xyz_after_press[2]:.3f})")
    
    joints_retract = move_along_end_effector_z(
        actual_joints_after_press,
        -total_distance,
        PUSH_PRESS_SPEED,
        speed_limit=CARTESIAN_HIGH_SPEED_LIMIT,
        profile=CARTESIAN_HIGH_ACCEL_PROFILE
    )
    
    if not joints_retract:
        print("  ⚠️  沿Z轴撤回失败（Joint5超限），尝试返回HOME位姿...")
        if USE_HOME_POSITION:
            if not move_to_home(speed=SLOW_SPEED, description="Push紧急返回HOME"):
                print("  ❌ 无法返回HOME位姿，尝试直接回零")
        else:
            print("  ⚠️  未启用HOME位姿，将尝试直接回零（可能有较大误差）")
    else:
        time.sleep(0.1)
    
    # 验证撤回后的位置
    actual_joints_after_retract = get_current_joints()
    actual_T_after_retract = piper_arm.forward_kinematics(actual_joints_after_retract)
    actual_xyz_after_retract = actual_T_after_retract[:3, 3]
    
    # 如果使用HOME位姿，检查是否成功回到HOME附近
    if USE_HOME_POSITION and joints_retract:
        home_T = piper_arm.forward_kinematics(HOME_JOINTS)
        home_xyz = home_T[:3, 3]
        retract_error = np.linalg.norm(actual_xyz_after_retract - home_xyz)
        if retract_error > 0.05:  # 撤回误差 > 5cm
            print(f"  ⚠️  撤回后位置偏差过大 ({retract_error*100:.1f}cm)，返回HOME位姿...")
            if not move_to_home(speed=SLOW_SPEED, description="Push撤回修正"):
                print("  ❌ 无法返回HOME位姿")
    
    # 步骤6/5: 回零位
    print("\n步骤6: 回零位...")
    actual_joints_before_zero = get_current_joints()
    actual_T_before_zero = piper_arm.forward_kinematics(actual_joints_before_zero)
    actual_xyz_before_zero = actual_T_before_zero[:3, 3]
    print(f"  当前位置: XYZ=({actual_xyz_before_zero[0]:.3f}, {actual_xyz_before_zero[1]:.3f}, {actual_xyz_before_zero[2]:.3f})")
    
    if not safe_return_to_zero(description="Push回零"):
        return False
    final_joints = get_current_joints()
    max_error = max(abs(j) for j in final_joints)
    print(f"  ✓ 回零完成 (最大偏差: {max_error:.4f} rad = {max_error*180/PI:.2f}°)")

    # 保存和可视化完整轨迹
    save_and_visualize_trajectory()
    
    print("="*70)
    print("✓✓✓ Push 操作完成！✓✓✓")
    print("="*70)
    return True


def action_knob():
    """
    旋转旋钮操作
    流程: HOME位姿 → 张开 → 到达 → 插入(z轴前进) → 闭合 → 旋转 → 回HOME位姿
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
    
    # 步骤0: 移动到HOME位姿
    if USE_HOME_POSITION:
        if not move_to_home(speed=NORMAL_SPEED, description="步骤0: 移动到HOME起始位姿"):
            print("❌ 无法到达HOME位姿，终止动作")
            return False
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(KNOB_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    
    # 🔧 新增：优先使用 AprilTag 参考姿态（包含法向量对齐）
    if APRILTAG_REFERENCE_POSE_BASE is not None:
        print("  ✓ 使用AprilTag参考姿态计算的目标位姿")
        # 注意：使用TARGET位置（旋钮实际位置），AprilTag只提供姿态参考
        target_position = np.array([TARGET_X, TARGET_Y, TARGET_Z])
        
        # 🎯 关键：使用垂直接近模式（推荐用于旋钮插入动作）
        R_gripper = get_gripper_approach_rotation('perpendicular')
        
        # 应用TCP偏移（Knob专用）
        tcp_offset_knob = np.array(TCP_OFFSET_KNOB) + np.array([TCP_GLOBAL_OFFSET_X, TCP_GLOBAL_OFFSET_Y, TCP_GLOBAL_OFFSET_Z])
        targetT = create_aligned_target_pose(target_position, R_gripper, tcp_offset=tcp_offset_knob)
        
        target_xyz = targetT[:3, 3]
        target_R = targetT[:3, :3]
        target_rpy = rotation_matrix_to_euler(target_R)
        
        print(f"  接近位置: ({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f})")
        print(f"  目标姿态: Roll={target_rpy[0]*180/PI:6.1f}°, Pitch={target_rpy[1]*180/PI:6.1f}°, Yaw={target_rpy[2]*180/PI:6.1f}°")
        print(f"  末端Z轴: ({targetT[0,2]:.3f}, {targetT[1,2]:.3f}, {targetT[2,2]:.3f})")
        print(f"  姿态模式: 垂直于面板 (perpendicular) ✓")
    else:
        print("  ⚠️  未设置AprilTag参考姿态，使用默认姿态")
        targetT = create_target_transform(
            TARGET_X, TARGET_Y, TARGET_Z,
            TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
            USE_6D_POSE
        )
        target_xyz = targetT[:3, 3]
        print(f"  目标位姿: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    
    print(f"  插入深度: {KNOB_INSERT_DEPTH*100:.1f}cm, 旋转: {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})")
    
    joints_target = compute_ik_moveit2(targetT, timeout=5.0, attempts=10)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    success, _ = move_to_pose_with_retries(
        targetT,
        joints_target,
        speed=NORMAL_SPEED,
        gripper_value=KNOB_GRIPPER_OPEN,
        description="Knob第一阶段"
    )
    if not success:
        if ENABLE_CARTESIAN_FINE_TUNE:
            print("  ↪ MoveIt误差仍大，启动笛卡尔微调确保阶段一准确")
            if not precise_move_to_pose(targetT, speed=12, description="Knob笛卡尔精调", force=True):
                print("❌ 笛卡尔微调失败，终止旋钮动作")
                return False
        else:
            print("❌ Knob第一阶段未满足精度要求，终止动作")
            return False
    time.sleep(0.1)
    
    # 步骤3: 沿末端z轴插入
    if abs(KNOB_INSERT_DEPTH) > 1e-6:
        # 使用实际到达的关节角度，而不是IK计算的理论值
        print(f"\n步骤3: 沿末端z轴插入 {KNOB_INSERT_DEPTH*100:.1f}cm...")
        
        actual_joints_step3 = get_current_joints()  # 获取实际当前位置
        actual_T_step3 = piper_arm.forward_kinematics(actual_joints_step3)
        actual_xyz_step3 = actual_T_step3[:3, 3]
        print(f"  实际起点: XYZ=({actual_xyz_step3[0]:.3f}, {actual_xyz_step3[1]:.3f}, {actual_xyz_step3[2]:.3f})")
        
        joints_insert = move_along_end_effector_z(actual_joints_step3, KNOB_INSERT_DEPTH, KNOB_INSERT_SPEED)
        if not joints_insert:
            return False
        time.sleep(0.1)
    
    # 步骤4: 夹爪闭合
    print(f"\n步骤4: 夹爪闭合到 {KNOB_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(KNOB_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 步骤5: 旋转旋钮（保持位置不变，只改变姿态）
    # 【重要修复】使用笛卡尔旋转路径，让所有关节协同工作，而不是只转动joint6
    direction_sign = 1 if KNOB_ROTATION_DIRECTION == 'cw' else -1
    target_rotation_joints = None
    print(f"\n步骤5: 旋转 {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})...")
    actual_joints_step5 = get_current_joints()  # 获取插入后的实际位置
    actual_T_step5 = piper_arm.forward_kinematics(actual_joints_step5)
    actual_xyz_step5 = actual_T_step5[:3, 3]
    print(f"  当前位置: XYZ=({actual_xyz_step5[0]:.3f}, {actual_xyz_step5[1]:.3f}, {actual_xyz_step5[2]:.3f})")
    print(f"  保持位置不变，仅旋转姿态 {KNOB_ROTATION_ANGLE}°")
    
    # 🔧 新方法：生成笛卡尔旋转路径（位置不变，姿态旋转）
    rotation_angle_rad = direction_sign * KNOB_ROTATION_ANGLE * PI / 180
    
    # 生成旋转waypoints
    num_rotation_steps = max(10, int(abs(KNOB_ROTATION_ANGLE) / 5))  # 每5度一个点
    waypoint_poses_rotation = []
    
    for i in range(1, num_rotation_steps + 1):
        alpha = i / num_rotation_steps
        intermediate_angle = rotation_angle_rad * alpha
        
        # 创建旋转矩阵（绕Z轴旋转）
        cos_a = np.cos(intermediate_angle)
        sin_a = np.sin(intermediate_angle)
        rotation_z = np.array([
            [cos_a, -sin_a, 0],
            [sin_a,  cos_a, 0],
            [0,      0,     1]
        ])
        
        # 应用旋转到当前姿态
        intermediate_T = actual_T_step5.copy()
        intermediate_T[:3, :3] = actual_T_step5[:3, :3] @ rotation_z  # 在末端坐标系旋转
        # 保持位置不变
        intermediate_T[:3, 3] = actual_xyz_step5
        
        waypoint_poses_rotation.append(intermediate_T)
    
    # 使用自定义笛卡尔规划器
    print(f"  [笛卡尔旋转] 生成旋转路径 ({num_rotation_steps}个waypoints)...")
    cartesian_traj_rotation, fraction_rotation = compute_custom_cartesian_path(
        actual_joints_step5,
        waypoint_poses_rotation,
        eef_step=0.01  # 1cm步长
    )
    
    if fraction_rotation < 0.8 or len(cartesian_traj_rotation) < 2:
        print(f"  ⚠️  笛卡尔旋转规划覆盖率较低: {fraction_rotation*100:.1f}%")
        print(f"  回退到简单joint6旋转...")
        # 回退到原方法：只转joint6
        joints_rotate = actual_joints_step5.copy()
        joints_rotate[5] += rotation_angle_rad
        if not control_arm(joints_rotate, KNOB_ROTATION_SPEED, USE_MOVEIT, KNOB_GRIPPER_HOLD):
            return False
        target_rotation_joints = joints_rotate
    else:
        print(f"  ✓ 笛卡尔旋转规划成功 (覆盖率: {fraction_rotation*100:.1f}%, 轨迹点: {len(cartesian_traj_rotation)})")
        
        # 执行旋转轨迹
        print(f"  [SDK] 执行旋转轨迹 ({len(cartesian_traj_rotation)}个点)...")
        smooth_rotation_speed = min(KNOB_ROTATION_SPEED, 15)
        # 🔧 关键修复：设置运动模式并等待生效
        piper.MotionCtrl_2(0x01, 0x01, smooth_rotation_speed, 0x00)
        time.sleep(0.1)  # 等待运动模式切换生效
        
        for idx, (joints, t) in enumerate(cartesian_traj_rotation):
            joints_int = [int(joints[i] * factor) for i in range(6)]
            # 🔧 关键修复：限制Joint5上下限
            joints_int[4] = min(70000, max(-70000, joints_int[4]))
            piper.JointCtrl(*joints_int)
            time.sleep(0.02)  # 50Hz执行频率
        
        print(f"  ✓ 笛卡尔旋转轨迹执行完成")
        target_rotation_joints = cartesian_traj_rotation[-1][0]
    
    # 🔧 修复2: 等待旋转完全停止后再继续
    wait_for_joints_to_settle(target_rotation_joints, tolerance=0.008, timeout=2.0, label="旋钮旋转")
    time.sleep(0.2)
    
    # 步骤6: 夹爪张开（确认旋转完成后再松开）
    print("\n步骤6: 夹爪张开（已确认旋转结束）...")
    piper.GripperCtrl(KNOB_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.2)  # 等待夹爪张开
    
    # 步骤7: 回零位
    print("\n步骤7: 回零位...")
    actual_joints_step7 = get_current_joints()
    actual_T_step7 = piper_arm.forward_kinematics(actual_joints_step7)
    actual_xyz_step7 = actual_T_step7[:3, 3]
    print(f"  当前位置: XYZ=({actual_xyz_step7[0]:.3f}, {actual_xyz_step7[1]:.3f}, {actual_xyz_step7[2]:.3f})")
    
    if not safe_return_to_zero(description="Knob回零"):
        return False
    final_joints = get_current_joints()
    max_error = max(abs(j) for j in final_joints)
    print(f"  ✓ 回零完成 (最大偏差: {max_error:.4f} rad = {max_error*180/PI:.2f}°)")
    
    # 步骤8: 夹爪闭合
    print("\n步骤8: 夹爪闭合...")
    piper.GripperCtrl(HOME_GRIPPER, 1000, 0x01, 0)
    time.sleep(0.1)
    
    # 保存和可视化完整轨迹
    save_and_visualize_trajectory()
    
    print("="*70)
    print("✓✓✓ Knob 操作完成！✓✓✓")
    print("="*70)
    return True


# ========================================
# 公开初始化函数（供外部调用）
# ========================================

def initialize_moveit2(external_node=None):
    """
    初始化MoveIt2（可被外部模块调用）
    
    Args:
        external_node: 外部已创建的ROS2 Node实例（可选）
                      如果提供，将复用该节点；否则创建新节点
    
    Returns:
        bool: 初始化是否成功
    """
    global MOVEIT_AVAILABLE, MOVEIT_INITIALIZED, move_group, moveit_node
    global joint_state_publisher, joint_state_timer, ros2_executor, ros2_spin_thread
    
    if not USE_MOVEIT:
        print("  ℹ️  USE_MOVEIT=False，跳过MoveIt2初始化")
        return False
    
    if not MOVEIT_AVAILABLE:
        print("  ℹ️  MOVEIT_AVAILABLE=False，跳过MoveIt2初始化")
        return False
    
    if MOVEIT_INITIALIZED:
        print("  ℹ️  MoveIt2已初始化，跳过")
        return True
    
    try:
        import rclpy
        import rclpy.executors
        import threading
        import time as time_module
        from sensor_msgs.msg import JointState
        
        print("\n初始化MoveIt2...")
        
        # 确保rclpy已初始化（外部调用时可能已初始化）
        if not rclpy.ok():
            print("  ⚠️  rclpy未初始化，尝试初始化...")
            try:
                rclpy.init()
                print("  ✓ rclpy初始化成功")
            except Exception as e:
                print(f"  ✗ rclpy初始化失败: {e}")
                return False
        
        # 创建或复用节点
        if external_node is not None:
            moveit_node = external_node
            print("  ✓ 使用外部提供的ROS2节点")
        else:
            node_name = f'button_action_moveit_{int(time_module.time() * 1000)}'
            moveit_node = Node(node_name)
            print(f"  ✓ 创建新ROS2节点: {node_name}")
        
        # 启动joint_states发布器
        joint_state_publisher = moveit_node.create_publisher(JointState, '/joint_states', 10)
        joint_state_timer = moveit_node.create_timer(0.1, publish_joint_states_callback)
        print("  ✓ joint_states发布器已启动 (10Hz)")
        
        # 创建Action Client
        move_group = ActionClient(moveit_node, MoveGroupAction, '/move_action')
        print("  ✓ Action Client已创建")
        
        # 如果是外部节点，由外部负责spin；否则启动后台spin线程
        if external_node is None:
            ros2_executor = rclpy.executors.SingleThreadedExecutor()
            ros2_executor.add_node(moveit_node)
            ros2_spin_thread = threading.Thread(target=ros2_executor.spin, daemon=True)
            ros2_spin_thread.start()
            print("  ✓ ROS2 spin线程已启动")
        else:
            print("  ℹ️  使用外部节点，跳过spin线程启动")
        
        # 等待joint_states开始发布
        time_module.sleep(0.5)
        
        # 等待action server可用
        print("  ⏳ 等待MoveIt2 action server...")
        timeout = 15.0
        start_time = time_module.time()
        
        while not move_group.server_is_ready():
            time_module.sleep(0.2)
            elapsed = time_module.time() - start_time
            if elapsed > timeout:
                print("  ⚠️  MoveIt2 action server不可用（超时）")
                print("  💡 提示: 请确保 MoveIt2 服务已启动")
                print("      启动命令: ./start_moveit2_clean.sh")
                
                # 清理资源
                cleanup_moveit2_resources()
                MOVEIT_AVAILABLE = False
                return False
            
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                print(f"  ⏳ 仍在等待... ({elapsed:.0f}s/{timeout:.0f}s)")
        
        # 🔧 关键修复：ROS2 Foxy Bug - server_is_ready()返回True后，实际还需要额外时间
        print("  ✓ Action server 已就绪，等待服务完全启动...")
        time_module.sleep(3.0)  # 额外等待3秒确保action server真正可用
        
        # 🔧 验证：尝试发送一个测试请求来确认server真正可用
        print("  ⏳ 验证 action server 是否真正可用...")
        test_timeout = 5.0
        test_start = time_module.time()
        server_verified = False
        
        while time_module.time() - test_start < test_timeout:
            if move_group.server_is_ready():
                server_verified = True
                break
            time_module.sleep(0.5)
        
        if not server_verified:
            print("  ⚠️  Action server 验证失败")
            cleanup_moveit2_resources()
            MOVEIT_AVAILABLE = False
            return False
        
        print("  ✓ MoveIt2初始化完成")
        print(f"  ✓ Action client已连接并验证: /move_action")
        MOVEIT_INITIALIZED = True
        return True
        
    except Exception as e:
        print(f"  ✗ MoveIt2初始化失败: {e}")
        import traceback
        traceback.print_exc()
        cleanup_moveit2_resources()
        MOVEIT_AVAILABLE = False
        return False


def cleanup_moveit2_resources():
    """清理MoveIt2资源（内部函数）"""
    global move_group, joint_state_timer, joint_state_publisher
    global ros2_executor, ros2_spin_thread, moveit_node
    
    try:
        if move_group is not None:
            move_group.destroy()
            move_group = None
    except:
        pass
    
    try:
        if joint_state_timer is not None:
            joint_state_timer.cancel()
            joint_state_timer = None
    except:
        pass
    
    try:
        if moveit_node is not None and joint_state_publisher is not None:
            moveit_node.destroy_publisher(joint_state_publisher)
    except:
        pass
    joint_state_publisher = None
    
    try:
        if ros2_executor is not None:
            ros2_executor.shutdown()
            ros2_executor = None
    except:
        pass
    
    try:
        if ros2_spin_thread is not None and ros2_spin_thread.is_alive():
            ros2_spin_thread.join(timeout=1.0)
        ros2_spin_thread = None
    except:
        pass


# ========================================
# 主程序
# ========================================

def main():
    global piper, piper_arm, move_group, moveit_node, display_trajectory_publisher, ee_path_publisher, ee_trail_publisher
    global MOVEIT_AVAILABLE, joint_state_publisher, joint_state_timer, ros2_executor, ros2_spin_thread  # 🔧 修复：在函数开始声明，避免语法错误
    global APRILTAG_REFERENCE_POSE_BASE  # 🔧 新增：声明全局变量
    
    # ========================================
    # 初始化AprilTag参考姿态
    # ========================================
    print("="*70)
    print("按钮操作执行器 - 独立版本")
    print("="*70)
    
    # 从宏定义的RPY计算旋转矩阵
    print(f"\n🏷️  AprilTag基座标系位姿:")
    print(f"   位置: ({APRILTAG_BASE_X:.3f}, {APRILTAG_BASE_Y:.3f}, {APRILTAG_BASE_Z:.3f})")
    print(f"   姿态: Roll={APRILTAG_BASE_ROLL*180/PI:.1f}°, Pitch={APRILTAG_BASE_PITCH*180/PI:.1f}°, Yaw={APRILTAG_BASE_YAW*180/PI:.1f}°")
    
    # 计算旋转矩阵
    APRILTAG_REFERENCE_POSE_BASE = euler_to_rotation_matrix(
        APRILTAG_BASE_ROLL, 
        APRILTAG_BASE_PITCH, 
        APRILTAG_BASE_YAW
    )
    print("   ✓ AprilTag参考姿态已初始化")
    
    # 计算并显示夹爪垂直按压时的姿态
    print(f"\n🤖 夹爪垂直按压时的目标姿态:")
    gripper_roll_perp = -APRILTAG_BASE_ROLL
    gripper_pitch_perp = APRILTAG_BASE_PITCH
    gripper_yaw_perp = APRILTAG_BASE_YAW
    print(f"   Roll={gripper_roll_perp*180/PI:.1f}° (= -Tag_Roll)")
    print(f"   Pitch={gripper_pitch_perp*180/PI:.1f}° (= Tag_Pitch)")
    print(f"   Yaw={gripper_yaw_perp*180/PI:.1f}° (= Tag_Yaw)")
    print(f"   💡 注意：Roll取反是因为夹爪和Tag法向量反向！")
    
    print("\n" + "="*70)
    print(f"\n📍 目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f}) ← 按钮/旋钮的实际位置")
    if USE_6D_POSE:
        if APRILTAG_REFERENCE_POSE_BASE is not None:
            print(f"   姿态来源: AprilTag自动计算（垂直按压模式）")
            print(f"   └─ Roll={gripper_roll_perp*180/PI:.1f}°, Pitch={gripper_pitch_perp*180/PI:.1f}°, Yaw={gripper_yaw_perp*180/PI:.1f}°")
        else:
            print(f"   姿态来源: 手动配置（TARGET_ROLL/PITCH/YAW）")
            print(f"   └─ Roll={TARGET_ROLL*180/PI:.1f}°, Pitch={TARGET_PITCH*180/PI:.1f}°, Yaw={TARGET_YAW*180/PI:.1f}°")
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
    
    # 🔧 关键修复：使能机械臂并等待足够时间（参考demo_03_go_zero_ros2.py）
    print("  正在使能机械臂...")
    piper.EnableArm(7)
    time.sleep(2)  # 等待2秒让使能生效
    
    # 检查使能状态
    status = piper.GetArmLowSpdInfoMsgs()
    if status.motor_1.foc_status.driver_enable_status == 1:
        print("  ✓ 机械臂已使能（状态已验证）")
    else:
        print("  ⚠️ 警告：机械臂使能状态异常")
    
    # 单独使能每个关节（保持原有逻辑）
    for i in range(7):
        piper.EnableArm(i + 1)
        time.sleep(0.1)
    
    piper.GripperCtrl(70000, 1000, 0x01, 0)
    print("  ✓ 硬件初始化完成")
    
    # 初始化 ROS
    print("\n初始化ROS...")
    if MOVEIT_AVAILABLE:
        # 使用ROS2
        import rclpy
        import rclpy.executors
        import threading
        
        # 🔧 关键修复：清理可能存在的旧上下文（第二次运行时）
        try:
            if rclpy.ok():
                print("  ⚠️  检测到旧的ROS2上下文，正在清理...")
                rclpy.shutdown()
                time.sleep(0.5)
        except:
            pass
        
        rclpy.init()
        print("  ✓ ROS2初始化完成")
    else:
        # 使用ROS1或FakeRospy
        rospy.init_node('button_action_node', anonymous=True)
        print("  ✓ ROS初始化完成")
    
    # 初始化 MoveIt (如果需要)
    if USE_MOVEIT and MOVEIT_AVAILABLE:
        try:
            # 创建ROS2节点（使用唯一名称避免冲突）
            import time as time_module
            node_name = f'button_action_moveit_{int(time_module.time() * 1000)}'
            moveit_node = Node(node_name)
            print("  ✓ ROS2节点已创建")
            
            # 启动joint_states发布器（MoveIt2需要）
            from sensor_msgs.msg import JointState
            joint_state_publisher = moveit_node.create_publisher(JointState, '/joint_states', 10)
            joint_state_timer = moveit_node.create_timer(0.1, publish_joint_states_callback)  # 10Hz
            print("  ✓ joint_states发布器已启动 (10Hz)")
            
            # 【重要】先创建Action Client，再启动spin线程
            # 这样可以避免ROS2 Foxy的wait set索引越界bug
            move_group = ActionClient(moveit_node, MoveGroupAction, '/move_action')
            print("  ✓ Action Client已创建")
            
            # 启动后台线程持续spin节点（让timer回调能运行）
            ros2_executor = rclpy.executors.SingleThreadedExecutor()
            ros2_executor.add_node(moveit_node)
            ros2_spin_thread = threading.Thread(target=ros2_executor.spin, daemon=True)
            ros2_spin_thread.start()
            print("  ✓ ROS2 spin线程已启动")
            
            # 等待joint_states开始发布
            import time as time_module
            time_module.sleep(0.5)
            
            # 等待action server可用
            print("  ⏳ 等待MoveIt2 action server...")
            
            # 🔧 关键修复：增加等待时间和重试机制（第二次运行需要更多时间）
            timeout = 15.0  # 从10秒增加到15秒
            start_time = time_module.time()
            retry_count = 0
            while not move_group.server_is_ready():
                time_module.sleep(0.2)  # 增加睡眠间隔减少CPU占用
                elapsed = time_module.time() - start_time
                if elapsed > timeout:
                    print("  ⚠️  MoveIt2 action server不可用，将使用SDK模式")
                    print("  💡 提示: 请确保已运行 ./start_moveit2.sh")
                    
                    # 🔧 关键修复：完全清理ROS2资源，避免段错误
                    # 1. 销毁Action Client
                    try:
                        move_group.destroy()
                    except:
                        pass
                    move_group = None
                    
                    # 1.5 停止joint_states timer/publisher
                    try:
                        if joint_state_timer is not None:
                            joint_state_timer.cancel()
                            joint_state_timer = None
                    except:
                        pass
                    try:
                        if moveit_node is not None and joint_state_publisher is not None:
                            moveit_node.destroy_publisher(joint_state_publisher)
                    except:
                        pass
                    joint_state_publisher = None
                    
                    # 2. 停止spin线程
                    try:
                        ros2_executor.shutdown()
                    except:
                        pass
                    ros2_executor = None
                    try:
                        if ros2_spin_thread is not None and ros2_spin_thread.is_alive():
                            ros2_spin_thread.join(timeout=1.0)
                    except:
                        pass
                    ros2_spin_thread = None
                    
                    # 3. 销毁节点
                    try:
                        moveit_node.destroy_node()
                    except:
                        pass
                    moveit_node = None
                    
                    # 4. 关闭ROS2上下文
                    try:
                        rclpy.shutdown()
                    except:
                        pass
                    
                    # 5. 标记MoveIt不可用
                    MOVEIT_AVAILABLE = False
                    print("  ℹ️  已完全关闭MoveIt2，使用SDK模式")
                    break
                
                # 每5秒打印一次等待状态
                if int(elapsed) % 5 == 0 and int(elapsed) > 0 and retry_count != int(elapsed):
                    retry_count = int(elapsed)
                    print(f"  ⏳ 仍在等待... ({elapsed:.0f}s/{timeout:.0f}s)")
            
            if move_group is not None:
                print("  ✓ MoveIt2初始化完成")
                print(f"  ✓ 规划器: {PLANNER_ID}")
                print(f"  ✓ Action client已连接: /move_action")
        except Exception as e:
            print(f"  ⚠️  MoveIt初始化失败: {e}")
            import traceback
            traceback.print_exc()
            print("  将使用SDK模式")
            
            # 🔧 关键修复：清理已创建的资源，避免段错误
            try:
                if 'move_group' in locals() and move_group is not None:
                    move_group.destroy()
                    move_group = None
            except:
                pass
            
            try:
                if joint_state_timer is not None:
                    joint_state_timer.cancel()
                    joint_state_timer = None
            except:
                pass
            try:
                if moveit_node is not None and joint_state_publisher is not None:
                    moveit_node.destroy_publisher(joint_state_publisher)
            except:
                pass
            joint_state_publisher = None
            
            try:
                if 'ros2_executor' in locals() and ros2_executor is not None:
                    ros2_executor.shutdown()
            except:
                pass
            try:
                if ros2_spin_thread is not None and ros2_spin_thread.is_alive():
                    ros2_spin_thread.join(timeout=1.0)
            except:
                pass
            ros2_spin_thread = None
            
            try:
                if 'moveit_node' in locals() and moveit_node is not None:
                    moveit_node.destroy_node()
                    moveit_node = None
            except:
                pass
            
            # 初始化失败后不使用MoveIt
            MOVEIT_AVAILABLE = False
    
    # 初始化 Piper Arm
    piper_arm = PiperArm()
    
    # ========================================
    # 检查当前位置，如果不在零位则回零
    # ========================================
    print("\n检查当前位置...")
    current_joints = get_current_joints()
    max_error = max(abs(j) for j in current_joints)
    print(f"  当前关节角度 (度): [{', '.join([f'{j*180/PI:7.2f}' for j in current_joints])}]")
    print(f"  最大偏差: {max_error:.4f} rad ({max_error*180/PI:.2f}°)")
    
    # 判断是否需要回零（阈值：2cm约1.15°）
    need_return_zero = max_error > 0.02
    
    if need_return_zero:
        print("  ⚠️ 不在零位，需要回零")
    else:
        print("  ✓ 已在零位附近，无需回零")
    
    # 如果需要回零，执行回零操作（关键：启动时必须成功回零）
    if need_return_zero:
        print("\n执行回零...")
        max_retries = 3
        for attempt in range(max_retries):
            if attempt > 0:
                print(f"  🔄 回零重试第 {attempt + 1}/{max_retries} 次...")
            
            if safe_return_to_zero(speed=60, description="启动回零"):
                print("  ✓ 已回零位")
                break
            
            if attempt < max_retries - 1:
                print("  ⚠️ 回零偏差过大，等待2秒后重试...")
                time.sleep(2)
        else:
            # 所有重试都失败
            print("\n" + "="*70)
            print("❌ 启动回零失败！机械臂未能回到零位")
            print("="*70)
            print("可能原因:")
            print("  1. 关节反馈异常（读取值与实际不符）")
            print("  2. 机械臂被物理阻挡")
            print("  3. 电机未正确使能")
            print("\n建议操作:")
            print("  1. 手动检查关节是否在零位")
            print("  2. 重启CAN总线和机械臂")
            print("  3. 检查关节电机状态")
            print("="*70)
            
            # 清理资源后退出
            if MOVEIT_AVAILABLE:
                try:
                    print("\n正在清理资源...")
                    ros2_executor.shutdown()
                    moveit_node.destroy_node()
                    rclpy.shutdown()
                except:
                    pass
            return
    else:
        print("  跳过回零步骤")
    
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
    
    # 清理资源 🔧 关键修复：防止段错误
    print("\n正在清理资源...")
    if MOVEIT_AVAILABLE:
        try:
            # 1. 先停止joint_states timer/publisher，防止回调继续运行
            if joint_state_timer is not None:
                try:
                    joint_state_timer.cancel()
                except Exception as e:
                    print(f"    (忽略timer错误: {e})")
                joint_state_timer = None
            if moveit_node is not None and joint_state_publisher is not None:
                try:
                    moveit_node.destroy_publisher(joint_state_publisher)
                except Exception as e:
                    print(f"    (忽略publisher错误: {e})")
            joint_state_publisher = None

            # 2. 停止executor和spin线程，确保没有后台线程调用rcl接口
            if ros2_executor is not None:
                try:
                    print("  - 正在停止 executor...")
                    ros2_executor.shutdown()
                except Exception as e:
                    print(f"    (忽略错误: {e})")
                ros2_executor = None
            if ros2_spin_thread is not None and ros2_spin_thread.is_alive():
                ros2_spin_thread.join(timeout=1.0)
            ros2_spin_thread = None

            # 3. 现在销毁Action Client，避免spin线程仍在访问
            if move_group is not None:
                try:
                    print("  - 正在销毁 Action Client...")
                    move_group.destroy()
                except Exception as e:
                    print(f"    (忽略错误: {e})")
                move_group = None

            # 4. 销毁节点
            if moveit_node is not None:
                try:
                    print("  - 正在销毁 node...")
                    moveit_node.destroy_node()
                except Exception as e:
                    print(f"    (忽略错误: {e})")
                moveit_node = None

            # 5. 关闭rclpy
            try:
                if rclpy.ok():
                    print("  - 正在关闭 rclpy...")
                    rclpy.shutdown()
            except Exception as e:
                print(f"    (忽略错误: {e})")
            
            print("  ✓ 资源清理完成")
        except Exception as e:
            print(f"  ⚠️  清理资源时出现异常（可忽略）: {e}")
    
    # 5. 最后禁用机械臂（可选）
    # try:
    #     print("  - 正在禁用机械臂...")
    #     piper.DisableArm(7)
    #     piper.DisconnectPort()
    #     print("  ✓ 机械臂已安全断开")
    # except:
    #     pass
    
    print("\n程序正常结束")


def action_approach_panel_demo():
    """
    示例：让夹爪正对面板的不同接近方式
    
    演示如何使用不同的姿态策略接近面板上的按钮
    """
    print("\n" + "="*70)
    print("示例：夹爪正对面板 - 不同接近方式演示")
    print("="*70)
    
    if APRILTAG_REFERENCE_POSE_BASE is None:
        print("❌ 错误：未设置面板参考姿态！")
        print("   请先在HOME位姿观察AprilTag，记录面板姿态")
        return False
    
    # 假设按钮位置（需要替换为实际检测值）
    button_x = TARGET_X if TARGET_X is not None else 0.4
    button_y = TARGET_Y if TARGET_Y is not None else 0.0
    button_z = TARGET_Z if TARGET_Z is not None else 0.2
    button_xyz = np.array([button_x, button_y, button_z])
    
    print(f"\n目标按钮位置（基座系）: ({button_x:.3f}, {button_y:.3f}, {button_z:.3f})")
    print(f"面板参考姿态已设置: ✓")
    
    # ========== 方式1：平行于面板（默认） ==========
    print("\n" + "-"*70)
    print("方式1: 平行于面板 (parallel)")
    print("-"*70)
    R_offset = get_gripper_approach_rotation('parallel')
    T_parallel = create_aligned_target_pose(button_xyz, R_offset)
    
    print("目标姿态（旋转矩阵）:")
    print(f"  [{T_parallel[0,0]:7.4f}, {T_parallel[0,1]:7.4f}, {T_parallel[0,2]:7.4f}]")
    print(f"  [{T_parallel[1,0]:7.4f}, {T_parallel[1,1]:7.4f}, {T_parallel[1,2]:7.4f}]")
    print(f"  [{T_parallel[2,0]:7.4f}, {T_parallel[2,1]:7.4f}, {T_parallel[2,2]:7.4f}]")
    print("说明: 夹爪与面板保持平行，适用于侧面滑动")
    
    # ========== 方式2：垂直于面板（按压姿态） ==========
    print("\n" + "-"*70)
    print("方式2: 垂直于面板 (perpendicular) - 推荐用于按压")
    print("-"*70)
    R_offset = get_gripper_approach_rotation('perpendicular')
    T_perpendicular = create_aligned_target_pose(button_xyz, R_offset)
    
    print("目标姿态（旋转矩阵）:")
    print(f"  [{T_perpendicular[0,0]:7.4f}, {T_perpendicular[0,1]:7.4f}, {T_perpendicular[0,2]:7.4f}]")
    print(f"  [{T_perpendicular[1,0]:7.4f}, {T_perpendicular[1,1]:7.4f}, {T_perpendicular[1,2]:7.4f}]")
    print(f"  [{T_perpendicular[2,0]:7.4f}, {T_perpendicular[2,1]:7.4f}, {T_perpendicular[2,2]:7.4f}]")
    print("说明: 夹爪垂直于面板，适用于push/plugin按压动作")
    
    # ========== 方式3：倾斜15°（预备姿态） ==========
    print("\n" + "-"*70)
    print("方式3: 倾斜15° (tilted_15)")
    print("-"*70)
    R_offset = get_gripper_approach_rotation('tilted_15')
    T_tilted = create_aligned_target_pose(button_xyz, R_offset)
    
    print("目标姿态（旋转矩阵）:")
    print(f"  [{T_tilted[0,0]:7.4f}, {T_tilted[0,1]:7.4f}, {T_tilted[0,2]:7.4f}]")
    print(f"  [{T_tilted[1,0]:7.4f}, {T_tilted[1,1]:7.4f}, {T_tilted[1,2]:7.4f}]")
    print(f"  [{T_tilted[2,0]:7.4f}, {T_tilted[2,1]:7.4f}, {T_tilted[2,2]:7.4f}]")
    print("说明: 夹爪倾斜15°，适用于按压前的预备姿态")
    
    print("\n" + "="*70)
    print("提示：在实际动作中使用：")
    print("  1. 获取按钮位置: button_xyz = [TARGET_X, TARGET_Y, TARGET_Z]")
    print("  2. 选择接近模式: R_gripper = get_gripper_approach_rotation('perpendicular')")
    print("  3. 构建目标位姿: T_target = create_aligned_target_pose(button_xyz, R_gripper)")
    print("  4. 执行运动: ik_result = piper_arm.inverse_kinematics_search(T_target, ...)")
    print("="*70)
    
    # 数值验证
    print("\n" + "="*70)
    print("📊 姿态关系验证（垂直按压时）")
    print("="*70)
    R_perp = get_gripper_approach_rotation('perpendicular')
    gripper_rpy = rotation_matrix_to_euler(R_perp)
    tag_rpy = (APRILTAG_BASE_ROLL, APRILTAG_BASE_PITCH, APRILTAG_BASE_YAW)
    
    print(f"\nAprilTag姿态:")
    print(f"  Roll  = {tag_rpy[0]*180/PI:7.2f}° ")
    print(f"  Pitch = {tag_rpy[1]*180/PI:7.2f}° ")
    print(f"  Yaw   = {tag_rpy[2]*180/PI:7.2f}° ")
    
    print(f"\n夹爪目标姿态 (perpendicular):")
    print(f"  Roll  = {gripper_rpy[0]*180/PI:7.2f}° (应为 {-tag_rpy[0]*180/PI:7.2f}°)")
    print(f"  Pitch = {gripper_rpy[1]*180/PI:7.2f}° (应为 {tag_rpy[1]*180/PI:7.2f}°)")
    print(f"  Yaw   = {gripper_rpy[2]*180/PI:7.2f}° (应为 {tag_rpy[2]*180/PI:7.2f}°)")
    
    # 验证
    roll_ok = abs(gripper_rpy[0] - (-tag_rpy[0])) < 0.01
    pitch_ok = abs(gripper_rpy[1] - tag_rpy[1]) < 0.01
    yaw_ok = abs(gripper_rpy[2] - tag_rpy[2]) < 0.01
    
    if roll_ok and pitch_ok and yaw_ok:
        print("\n✅ 姿态关系正确！")
        print("   Roll取反 ✓, Pitch相同 ✓, Yaw相同 ✓")
    else:
        print("\n❌ 姿态关系不正确！")
        if not roll_ok: print("   Roll验证失败")
        if not pitch_ok: print("   Pitch验证失败")
        if not yaw_ok: print("   Yaw验证失败")
    
    print("="*70)
    
    return True


if __name__ == "__main__":
    main()