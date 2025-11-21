# #!/usr/bin/env python3
# from piper_sdk import *
# import rospy
# import time
# import sys
# import numpy as np
# import math
# from piper_arm import PiperArm
# from utils.utils_piper import read_joints
# from utils.utils_piper import enable_fun
# from utils.utils_ros import publish_tf, publish_sphere_marker, publish_trajectory
# from utils.utils_math import quaternion_to_rotation_matrix
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PointStamped
# from nav_msgs.msg import Path

# PI = math.pi
# factor = 1000 * 180 / PI
# receive_object_center = False
# object_center = []
# simulation = True

# # 用户可自定义参数
# GRIPPER_CLOSE_VALUE = 40000  # 夹爪闭合值(单位:0.001mm) 默认40mm
# ROTATION_ANGLE = 90  # 旋转角度(度) 默认90度
# ROTATION_DIRECTION = 1  # 旋转方向: 1=右旋(顺时针), -1=左旋(逆时针)


# def control_arm(joints, speed=2):

#     # joints [rad]

#     position = joints

#     joint_0 = int(position[0] * factor)
#     joint_1 = int(position[1] * factor)
#     joint_2 = int(position[2] * factor)
#     joint_3 = int(position[3] * factor)
#     joint_4 = int(position[4] * factor)
#     joint_5 = int(position[5] * factor)

#     if (joint_4 < -70000) :
#         joint_4 = -70000

#     # piper.MotionCtrl_1()
#     piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
#     piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

#     if len(joints) > 6:
#         joint_6 = round(position[6] * 1000 * 1000)
#         piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

#     print(piper.GetArmStatus())
#     print(position)

# def object_point_callback(msg):
#     # print("Receive visual detection result", msg.point.x, msg.point.y, msg.point.z)
#     if(np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z)):
#         return
#     global receive_object_center, object_center
#     receive_object_center = True
#     object_center = [msg.point.x, msg.point.y, msg.point.z]


# def move_and_grasp(object_center, joints, piper_arm, gripper_close_value=None, rotation_angle=None, rotation_direction=None):
#     """移动并抓取物体
#     Args:
#         object_center: 目标物体中心坐标
#         joints: 当前关节角度
#         piper_arm: 机械臂对象
#         gripper_close_value: 夹爪闭合值(0.001mm单位), None=使用全局默认值
#         rotation_angle: 旋转角度(度), None=使用全局默认值
#         rotation_direction: 旋转方向(1=右旋,-1=左旋), None=使用全局默认值
#     """
#     # 使用传入参数或全局默认值
#     if gripper_close_value is None:
#         gripper_close_value = GRIPPER_CLOSE_VALUE
#     if rotation_angle is None:
#         rotation_angle = ROTATION_ANGLE
#     if rotation_direction is None:
#         rotation_direction = ROTATION_DIRECTION
    
#     print("prepare to grasp point under camera frame", object_center[0], object_center[1], object_center[2])
#     print(f"抓取前: 夹爪完全打开 (70mm)")
#     print(f"抓取后: 夹爪闭合到 {gripper_close_value/1000:.1f}mm")
#     print(f"旋转设置: {'右旋' if rotation_direction == 1 else '左旋'} {rotation_angle}度")
    
#     # 步骤1: 先确保夹爪完全打开 (准备抓取)
#     print("\n步骤1: 夹爪完全打开...")
#     piper.GripperCtrl(70000, 1000, 0x01, 0)  # 70000 = 70mm 完全打开
#     time.sleep(1)

#     # transfer point from camera frame to base_link frame
#     base_T_link6 = piper_arm.forward_kinematics(joints)
#     link6_T_cam = np.eye(4)
#     link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
#     link6_T_cam[:3, 3] = piper_arm.link6_t_camera

#     base_ob_center = base_T_link6 @ link6_T_cam @ np.array([object_center[0], object_center[1], object_center[2], 1])

#     # publish target object center
#     print("point under base frame", base_ob_center)
#     pub = rospy.Publisher('/target_point_under_based', Marker, queue_size=10)
#     publish_sphere_marker(pub, base_ob_center, frame_id="arm_base", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)

#     targetT = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
#     targetT[0, 3] = base_ob_center[0]
#     targetT[1, 3] = base_ob_center[1]
#     targetT[2, 3] = base_ob_center[2]


#     # inverse kinematics
#     joints = piper_arm.inverse_kinematics(targetT)
#     joints_array = np.array(joints)
#     print("base ob center", base_ob_center)
#     if not joints :
#         print("ik fail")
#         return False
#     print("Planed ik[degree]:", joints_array / PI * 180)

#     # time_now = rospy.Time.now()
#     # publish_tf(piper_arm, joints, time_now)

#     # 步骤2: 移动到目标位置，夹爪保持完全打开状态 (70mm)
#     joints.append(0.07)  # 70mm = 0.07m 完全打开
#     print("\n步骤2: 移动到目标位置 (夹爪保持打开 70mm)...")
#     control_arm(joints, 20)
#     time.sleep(10)
    
#     # 步骤3: 闭合夹爪抓取物体 (闭合到用户自定义值)
#     print(f"\n步骤3: 闭合夹爪抓取 (从70mm闭合到{gripper_close_value/1000:.1f}mm)...")
#     joints[6] = gripper_close_value / 1000000  # 转换为米
#     control_arm(joints, 20)
#     time.sleep(2)
#     print(f"✓ 夹爪已闭合到 {gripper_close_value/1000:.1f}mm (物体已抓取)")
    
#     # 步骤4: 旋转夹爪 (带物体旋转)
#     actual_rotation = rotation_angle * rotation_direction
#     print(f"\n步骤4: 旋转夹爪 {'右旋' if rotation_direction == 1 else '左旋'} {rotation_angle}度 (保持夹爪闭合)...")
#     joints[5] += actual_rotation * PI / 180  # 在当前角度基础上旋转
#     control_arm(joints, 20)
#     time.sleep(2)
#     print(f"✓ 旋转完成")
    
#     # 步骤5: 返回安全位置 (保持夹爪闭合和旋转状态)
#     print(f"\n步骤5: 返回安全位置 (保持夹爪闭合{gripper_close_value/1000:.1f}mm和旋转状态)...")
#     joints_safe = [0, 0, -0.4, 0, 0, joints[5], joints[6]]  # 保持旋转角度和夹爪状态
#     control_arm(joints_safe, 20)
#     time.sleep(2)
#     print("✓ 抓取任务完成！")

#     return True



# if __name__ == "__main__":
#     # 用户可在此处自定义参数
#     print("="*60)
#     print("Piper 视觉抓取程序")
#     print("="*60)
#     print("\n⚠️  用户自定义参数:")
#     print(f"  夹爪闭合值: {GRIPPER_CLOSE_VALUE} (0.001mm) = {GRIPPER_CLOSE_VALUE/1000:.1f}mm")
#     print(f"  旋转角度: {ROTATION_ANGLE}度")
#     print(f"  旋转方向: {'右旋(顺时针)' if ROTATION_DIRECTION == 1 else '左旋(逆时针)'}")
#     print("\n💡 修改方法: 编辑文件顶部的全局变量")
#     print("  GRIPPER_CLOSE_VALUE = 40000  # 40mm")
#     print("  ROTATION_ANGLE = 90  # 90度")
#     print("  ROTATION_DIRECTION = 1  # 1=右旋, -1=左旋")
#     print("="*60)
    
#     # 允许用户临时修改参数
#     use_custom = input("\n是否使用自定义参数? (y/n, 默认n): ").strip().lower()
    
#     custom_gripper = GRIPPER_CLOSE_VALUE
#     custom_angle = ROTATION_ANGLE
#     custom_direction = ROTATION_DIRECTION
    
#     if use_custom == 'y':
#         try:
#             val = input(f"输入夹爪闭合值(0.001mm单位, 0-70000, 默认{GRIPPER_CLOSE_VALUE}): ").strip()
#             if val:
#                 custom_gripper = int(val)
#                 if not (0 <= custom_gripper <= 70000):
#                     print(f"⚠️  值超出范围，使用默认值 {GRIPPER_CLOSE_VALUE}")
#                     custom_gripper = GRIPPER_CLOSE_VALUE
            
#             val = input(f"输入旋转角度(度, 默认{ROTATION_ANGLE}): ").strip()
#             if val:
#                 custom_angle = float(val)
            
#             dir_input = input(f"输入旋转方向(1=右旋, -1=左旋, 默认{ROTATION_DIRECTION}): ").strip()
#             if dir_input:
#                 custom_direction = int(dir_input)
#                 if custom_direction not in [1, -1]:
#                     print(f"⚠️  方向无效，使用默认值 {ROTATION_DIRECTION}")
#                     custom_direction = ROTATION_DIRECTION
#         except:
#             print("⚠️  输入无效，使用默认值")
    
#     print("\n最终使用参数:")
#     print(f"  夹爪闭合值: {custom_gripper} = {custom_gripper/1000:.1f}mm")
#     print(f"  旋转: {'右旋' if custom_direction == 1 else '左旋'} {custom_angle}度")
#     print("="*60)
    
#     piper = C_PiperInterface_V2("can0")
#     piper.ConnectPort()
#     piper.EnableArm(7)
#     enable_fun(piper=piper)
#     piper.GripperCtrl(70000, 1000, 0x01, 0)  # 初始化: 完全打开

#     # 设置初始位置
#     joints = [0, 0, 0, 0, 0, 0, 0]
#     control_arm(joints, 100)
#     time.sleep(2)


#     # 初始化节点
#     rospy.init_node('vison_grasp_node', anonymous=True)

#     piper_arm = PiperArm()
#     sub = rospy.Subscriber('/object_point',
#                            PointStamped,
#                            object_point_callback,
#                            queue_size=10,
#                            tcp_nodelay=True)

#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         # time_now = rospy.Time.now()
#         # publish_tf(piper_arm, joints, time_now)
#         if (receive_object_center):
#             msg = piper.GetArmJointMsgs()

#             theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
#             theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
#             theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
#             theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
#             theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
#             theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

#             joints = [theta1, theta2, theta3, theta4, theta5, theta6]

#             if move_and_grasp(object_center, joints, piper_arm, custom_gripper, custom_angle, custom_direction):
#                 break
#             receive_object_center = False

#         rate.sleep()


#!/usr/bin/env python3
"""
Piper 视觉抓取程序 - MoveIt 增强版
支持 MoveIt 轨迹规划 + SDK 执行的混合控制架构
"""
from piper_sdk import *
import rospy
import time
import sys
import numpy as np
import math
from piper_arm import PiperArm
from utils.utils_piper import read_joints, enable_fun
from utils.utils_ros import publish_tf, publish_sphere_marker, publish_trajectory
from utils.utils_math import quaternion_to_rotation_matrix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path

# ===== MoveIt 导入 (支持降级) =====
MOVEIT_AVAILABLE = False
display_trajectory_publisher = None
end_effector_path_publisher = None
try:
    import moveit_commander
    from moveit_msgs.msg import DisplayTrajectory
    from geometry_msgs.msg import PoseStamped
    MOVEIT_AVAILABLE = True
    print("✓ MoveIt已加载，将使用MoveIt轨迹规划")
except ImportError as e:
    print(f"⚠️  MoveIt导入失败: {e}")
    print("   将使用SDK模式（直接控制）")

# ===== 全局常量 =====
PI = math.pi
factor = 1000 * 180 / PI  # 弧度 → SDK 整数转换因子

# ===== MoveIt 配置参数 =====
TRAJECTORY_PUBLISH_RATE = 10   # 轨迹发布到RViz的频率 (Hz)
TRAJECTORY_EXECUTE_RATE = 50   # SDK执行轨迹的频率 (Hz)
TRAJECTORY_SAMPLE_POINTS = 20  # 轨迹采样点数

# ===== 用户可自定义参数 =====
GRIPPER_CLOSE_VALUE = 40000    # 夹爪闭合值(单位:0.001mm) 默认40mm
ROTATION_ANGLE = 90            # 旋转角度(度) 默认90度
ROTATION_DIRECTION = 1         # 旋转方向: 1=右旋(顺时针), -1=左旋(逆时针)

# 抓取流程配置
PRE_GRASP_OFFSET = 0.10        # 预抓取偏移 (10cm，物体上方)
LIFT_HEIGHT = 0.05             # 抬起高度 (5cm)
APPROACH_SPEED = 30            # 接近速度（慢速）
NORMAL_SPEED = 60              # 正常速度
FAST_SPEED = 80                # 快速（返回安全位置）

# ===== 四种按钮操作类型配置 =====
ACTION_CONFIGS = {
    'toggle': {
        'gripper_open': 70000,         # 夹爪完全打开 (70mm)
        'approach_offset': 0.05,       # 接近偏移 5cm
        'push_distance': 0.03,         # 推动距离 3cm
        'approach_speed': 20,          # 接近速度（极慢）
        'push_speed': 40,              # 推动速度
        'hold_time': 0.5,              # 保持时间
    },
    'plugin': {
        'gripper_hold': 30000,         # 夹持连接器 30mm
        'gripper_release': 50000,      # 松开 50mm
        'lift_height': 0.05,           # 抬起高度 5cm
        'insert_depth': 0.04,          # 插入深度 4cm
        'insert_speed': 15,            # 插入速度（极慢）
        'extract_speed': 20,           # 拔出速度
        'approach_offset': 0.10,       # 接近偏移 10cm
    },
    'push': {
        'gripper_close': 0,            # 夹爪闭合（形成按压面）
        'approach_offset': 0.08,       # 接近偏移 8cm
        'press_depth': 0.01,           # 按压深度 1cm
        'press_speed': 30,             # 按压速度
        'hold_time': 2.0,              # 保持按压 2秒
    },
    'knob': {
        'gripper_offset': 5000,        # 夹爪比旋钮大5mm (例如20mm旋钮用25mm夹持)
        'approach_offset': 0.05,       # 接近偏移 5cm
        'rotation_speed': 40,          # 旋转速度
        'hold_time': 0.5,              # 保持时间
        'max_single_rotation': 180,    # 单次最大旋转角度
    }
}

# ===== 全局变量 =====
receive_object_center = False
object_center = []
piper = None  # SDK 接口


# ========================================
# MoveIt 控制函数
# ========================================

def initialize_moveit():
    """初始化 MoveIt 规划器"""
    if not MOVEIT_AVAILABLE:
        return None
    
    try:
        # 1. 设置 ROS 包路径
        import os
        # 获取项目根目录（grasp_action_copy.py 所在目录）
        project_root = os.path.dirname(os.path.abspath(__file__))
        piper_ros_path = os.path.join(project_root, "piper_ros")
        src_path = os.path.join(piper_ros_path, 'src')
        current_path = os.environ.get('ROS_PACKAGE_PATH', '')
        if src_path not in current_path:
            os.environ['ROS_PACKAGE_PATH'] = f"{src_path}:{current_path}"
        
        # 2. 初始化 MoveIt Commander
        moveit_commander.roscpp_initialize([])
        robot = moveit_commander.RobotCommander()
        
        # 3. 创建规划组
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        # 4. 配置规划参数
        move_group.set_planning_time(5.0)
        move_group.set_max_velocity_scaling_factor(1.0)
        move_group.set_max_acceleration_scaling_factor(1.0)
        
        # 5. 初始化可视化发布器
        global display_trajectory_publisher, end_effector_path_publisher
        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            DisplayTrajectory,
            queue_size=20
        )
        end_effector_path_publisher = rospy.Publisher(
            '/end_effector_path',
            Path,
            queue_size=10
        )
        
        print(f"  ✓ MoveIt就绪 (组: {group_name})")
        print(f"  ✓ 轨迹可视化: /move_group/display_planned_path")
        print(f"  ✓ 末端路径: /end_effector_path")
        print(f"  ✓ 执行频率: {TRAJECTORY_EXECUTE_RATE}Hz")
        
        return move_group
        
    except Exception as e:
        print(f"  ❌ MoveIt初始化失败: {e}")
        return None


def control_arm_moveit(joints, speed=50, gripper_value=None):
    """
    使用 MoveIt 规划轨迹，然后用 SDK 执行
    
    参数:
        joints: 目标关节角度 [j0,j1,j2,j3,j4,j5]，单位：弧度
        speed: SDK 执行速度 (1-100)
        gripper_value: 夹爪位置（米），None=不控制夹爪
    
    返回:
        True: 成功, False: 失败
    """
    global piper, move_group
    
    try:
        # 清理旧状态
        move_group.clear_pose_targets()
        move_group.stop()
        
        # 设置目标（只使用前6个关节）
        target_joints = joints[:6] if len(joints) > 6 else joints
        move_group.set_joint_value_target(target_joints)
        
        # MoveIt 规划
        print("  [MoveIt] 规划轨迹...")
        plan = move_group.plan()
        
        # 解析规划结果
        if isinstance(plan, tuple):
            success, trajectory = plan[0], plan[1]
        else:
            success, trajectory = True, plan
        
        if not success or not trajectory.joint_trajectory.points:
            print("  ❌ 规划失败")
            return False
        
        traj_points = trajectory.joint_trajectory.points
        print(f"  ✓ 规划成功 (轨迹点: {len(traj_points)})")
        
        # 发布关节轨迹到 RViz
        if display_trajectory_publisher is not None:
            display_msg = DisplayTrajectory()
            display_msg.trajectory_start = move_group.get_current_state()
            display_msg.trajectory.append(trajectory)
            display_trajectory_publisher.publish(display_msg)
            print(f"  ✓ 关节轨迹已发布 (话题: /move_group/display_planned_path)")
        
        # 发布末端路径到 RViz
        if end_effector_path_publisher is not None:
            path_msg = Path()
            path_msg.header.frame_id = "dummy_link"
            path_msg.header.stamp = rospy.Time.now()
            
            # 简化：直接发布轨迹点数量（实际应用中可计算末端位姿）
            for point in traj_points:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                path_msg.poses.append(pose_stamped)
            
            end_effector_path_publisher.publish(path_msg)
            print(f"  ✓ 末端轨迹已发布 (点数: {len(path_msg.poses)}, 话题: /end_effector_path)")
        
        # 轨迹采样
        sample_indices = np.linspace(
            0, 
            len(traj_points) - 1, 
            min(TRAJECTORY_SAMPLE_POINTS, len(traj_points)), 
            dtype=int
        )
        sample_points = [traj_points[i] for i in sample_indices]
        
        # SDK 执行
        print(f"  [SDK] 执行轨迹 (采样点: {len(sample_points)}, 频率: {TRAJECTORY_EXECUTE_RATE}Hz)")
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        
        rate = rospy.Rate(TRAJECTORY_EXECUTE_RATE)
        for idx, point in enumerate(sample_points):
            joints_int = [int(point.positions[i] * factor) for i in range(6)]
            joints_int[4] = max(-70000, joints_int[4])
            piper.JointCtrl(*joints_int)
            
            if idx < len(sample_points) - 1:
                rate.sleep()
        
        # 最终位置确认
        print("  到达最终目标位置...")
        final_joints = [int(traj_points[-1].positions[i] * factor) for i in range(6)]
        final_joints[4] = max(-70000, final_joints[4])
        piper.JointCtrl(*final_joints)
        rospy.sleep(0.5)
        
        # 控制夹爪
        if gripper_value is not None:
            gripper_int = int(gripper_value * 1000000)  # 米 → 0.001mm
            piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
        
        print(f"  ✓ SDK执行完成")
        return True
        
    except Exception as e:
        print(f"  ❌ MoveIt控制失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def control_arm_sdk(joints, speed=50, gripper_value=None):
    """
    SDK 直接控制模式（不使用 MoveIt）
    
    参数:
        joints: 关节角度 [j0,j1,j2,j3,j4,j5] 或 [j0,...,j5,gripper]，单位：弧度
        speed: 速度 (1-100)
        gripper_value: 夹爪位置（米），None=不控制夹爪
    """
    global piper
    
    # 转换关节角度
    joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
    joints_int[4] = max(-70000, joints_int[4])
    
    # 发送关节命令
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joints_int)
    
    # 控制夹爪
    if gripper_value is not None:
        gripper_int = int(gripper_value * 1000000)
        piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    elif len(joints) > 6:
        gripper_int = int(joints[6] * 1000000)
        piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    
    print(f"  [SDK] 直接控制完成")
    return True


def control_arm(joints, speed=50, use_moveit=True):
    """
    统一的机械臂控制接口（智能选择模式）
    
    参数:
        joints: 关节角度（弧度）
        speed: 速度 (1-100)
        use_moveit: True=使用MoveIt, False=使用SDK
    """
    # 提取夹爪值
    gripper_value = joints[6] if len(joints) > 6 else None
    
    # 选择控制模式
    if use_moveit and MOVEIT_AVAILABLE and move_group is not None:
        return control_arm_moveit(joints[:6], speed, gripper_value)
    else:
        return control_arm_sdk(joints, speed, gripper_value)


# ========================================
# 视觉检测回调
# ========================================

def object_point_callback(msg):
    """接收视觉检测到的物体中心点"""
    if np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z):
        return
    
    global receive_object_center, object_center
    receive_object_center = True
    object_center = [msg.point.x, msg.point.y, msg.point.z]


# ========================================
# 四种按钮操作函数
# ========================================

def action_toggle(target_pos, piper_arm, toggle_direction='up', push_distance=None, use_moveit=True):
    """
    拨动开关操作
    
    参数:
        target_pos: 拨片中心位置 [x, y, z] (基座坐标系)
        piper_arm: 机械臂对象
        toggle_direction: 拨动方向 'up'/'down'/'left'/'right'
        push_distance: 拨动行程 (米)，None=使用默认值
        use_moveit: 是否使用 MoveIt
    
    返回:
        True: 成功, False: 失败
    """
    config = ACTION_CONFIGS['toggle']
    if push_distance is None:
        push_distance = config['push_distance']
    
    print("="*70)
    print("动作类型: Toggle (拨动开关)")
    print("="*70)
    print(f"目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
    print(f"拨动方向: {toggle_direction}, 行程: {push_distance*100:.1f}cm")
    
    # 方向向量映射（基座坐标系）
    direction_vectors = {
        'up':    [0, 0, push_distance],
        'down':  [0, 0, -push_distance],
        'left':  [0, push_distance, 0],
        'right': [0, -push_distance, 0]
    }
    
    if toggle_direction not in direction_vectors:
        print(f"❌ 无效的拨动方向: {toggle_direction}")
        return False
    
    # 步骤1: 夹爪完全打开
    print("\n步骤1: 夹爪完全打开...")
    piper.GripperCtrl(config['gripper_open'], 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤2: 移动到拨片前方（预接近位置）
    print(f"\n步骤2: 移动到拨片前方 {config['approach_offset']*100:.0f}cm...")
    targetT_pre = np.eye(4)
    targetT_pre[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])  # 夹爪朝向
    targetT_pre[0, 3] = target_pos[0] - config['approach_offset']
    targetT_pre[1, 3] = target_pos[1]
    targetT_pre[2, 3] = target_pos[2]
    
    joints_pre = piper_arm.inverse_kinematics(targetT_pre)
    if not joints_pre:
        print("❌ 预接近位置IK失败")
        return False
    
    joints_pre.append(config['gripper_open'] / 1000000)
    if not control_arm(joints_pre, NORMAL_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    
    # 步骤3: 接触拨片
    print("\n步骤3: 接触拨片...")
    targetT_contact = targetT_pre.copy()
    targetT_contact[0, 3] = target_pos[0]
    
    joints_contact = piper_arm.inverse_kinematics(targetT_contact)
    if not joints_contact:
        print("❌ 接触位置IK失败")
        return False
    
    joints_contact.append(config['gripper_open'] / 1000000)
    if not control_arm(joints_contact, config['approach_speed'], use_moveit):
        return False
    time.sleep(0.5)
    
    # 步骤4: 推动拨片
    print(f"\n步骤4: 推动拨片 ({toggle_direction})...")
    direction_offset = direction_vectors[toggle_direction]
    targetT_push = targetT_contact.copy()
    targetT_push[0, 3] += direction_offset[0]
    targetT_push[1, 3] += direction_offset[1]
    targetT_push[2, 3] += direction_offset[2]
    
    joints_push = piper_arm.inverse_kinematics(targetT_push)
    if not joints_push:
        print("❌ 推动位置IK失败")
        return False
    
    joints_push.append(config['gripper_open'] / 1000000)
    if not control_arm(joints_push, config['push_speed'], use_moveit):
        return False
    
    # 步骤5: 保持
    print(f"\n步骤5: 保持 {config['hold_time']}秒...")
    time.sleep(config['hold_time'])
    
    # 步骤6: 回退
    print("\n步骤6: 回退到预接近位置...")
    if not control_arm(joints_pre, config['push_speed'], use_moveit):
        return False
    time.sleep(0.5)
    
    # 步骤7: 返回安全位置
    print("\n步骤7: 返回安全位置...")
    joints_safe = [0, 0, -0.4, 0, 0, 0, config['gripper_open'] / 1000000]
    if not control_arm(joints_safe, FAST_SPEED, use_moveit):
        return False
    
    print("="*70)
    print("✓✓✓ Toggle 操作完成！✓✓✓")
    print("="*70)
    return True


def action_plugin(target_pos, piper_arm, action_type='plug', insert_depth=None, knob_diameter=0.02, use_moveit=True):
    """
    插拔连接器操作
    
    参数:
        target_pos: 连接器中心位置 [x, y, z] (基座坐标系)
        piper_arm: 机械臂对象
        action_type: 'plug'(插入) / 'unplug'(拔出)
        insert_depth: 插入深度 (米)，None=使用默认值
        knob_diameter: 连接器直径 (米)，用于确定夹持宽度
        use_moveit: 是否使用 MoveIt
    
    返回:
        True: 成功, False: 失败
    """
    config = ACTION_CONFIGS['plugin']
    if insert_depth is None:
        insert_depth = config['insert_depth']
    
    print("="*70)
    print(f"动作类型: Plug-in ({'插入' if action_type == 'plug' else '拔出'}连接器)")
    print("="*70)
    print(f"目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
    print(f"插入深度: {insert_depth*100:.1f}cm, 夹持宽度: {config['gripper_hold']/1000:.1f}mm")
    
    if action_type == 'plug':
        # === 插入流程 ===
        # 步骤1: 夹爪打开
        print("\n步骤1: 夹爪打开...")
        piper.GripperCtrl(config['gripper_release'], 1000, 0x01, 0)
        time.sleep(0.8)
        
        # 步骤2: 移动到连接器上方
        print(f"\n步骤2: 移动到连接器上方 {config['approach_offset']*100:.0f}cm...")
        targetT_above = np.eye(4)
        targetT_above[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        targetT_above[0, 3] = target_pos[0]
        targetT_above[1, 3] = target_pos[1]
        targetT_above[2, 3] = target_pos[2] + config['approach_offset']
        
        joints_above = piper_arm.inverse_kinematics(targetT_above)
        if not joints_above:
            print("❌ 上方位置IK失败")
            return False
        
        joints_above.append(config['gripper_release'] / 1000000)
        if not control_arm(joints_above, NORMAL_SPEED, use_moveit):
            return False
        time.sleep(1.0)
        
        # 步骤3: 下降并夹持连接器
        print("\n步骤3: 下降并夹持连接器...")
        targetT_grasp = targetT_above.copy()
        targetT_grasp[2, 3] = target_pos[2]
        
        joints_grasp = piper_arm.inverse_kinematics(targetT_grasp)
        if not joints_grasp:
            print("❌ 夹持位置IK失败")
            return False
        
        joints_grasp.append(config['gripper_hold'] / 1000000)
        if not control_arm(joints_grasp, config['insert_speed'], use_moveit):
            return False
        time.sleep(1.0)
        
        # 步骤4: 抬起连接器
        print(f"\n步骤4: 抬起连接器 {config['lift_height']*100:.0f}cm...")
        targetT_lift = targetT_grasp.copy()
        targetT_lift[2, 3] += config['lift_height']
        
        joints_lift = piper_arm.inverse_kinematics(targetT_lift)
        if not joints_lift:
            print("❌ 抬起位置IK失败")
            return False
        
        joints_lift.append(config['gripper_hold'] / 1000000)
        if not control_arm(joints_lift, APPROACH_SPEED, use_moveit):
            return False
        time.sleep(0.8)
        
        # 步骤5: 移动到插座位置（假设插座在连接器旁边）
        print("\n步骤5: 移动到插座上方...")
        # 这里需要根据实际情况调整插座位置
        # 简化：假设插座在连接器旁边 10cm
        targetT_socket = targetT_lift.copy()
        targetT_socket[0, 3] += 0.10
        
        joints_socket = piper_arm.inverse_kinematics(targetT_socket)
        if not joints_socket:
            print("❌ 插座位置IK失败")
            return False
        
        joints_socket.append(config['gripper_hold'] / 1000000)
        if not control_arm(joints_socket, NORMAL_SPEED, use_moveit):
            return False
        time.sleep(1.0)
        
        # 步骤6: 垂直插入
        print(f"\n步骤6: 垂直插入 (深度{insert_depth*100:.1f}cm)...")
        targetT_insert = targetT_socket.copy()
        targetT_insert[2, 3] -= insert_depth
        
        joints_insert = piper_arm.inverse_kinematics(targetT_insert)
        if not joints_insert:
            print("❌ 插入位置IK失败")
            return False
        
        joints_insert.append(config['gripper_hold'] / 1000000)
        if not control_arm(joints_insert, config['insert_speed'], use_moveit):
            return False
        time.sleep(1.0)
        
        # 步骤7: 松开夹爪
        print("\n步骤7: 松开夹爪...")
        joints_insert[6] = config['gripper_release'] / 1000000
        control_arm_sdk(joints_insert, 10)
        time.sleep(0.5)
        
        # 步骤8: 垂直上升
        print("\n步骤8: 垂直上升...")
        if not control_arm(joints_socket, APPROACH_SPEED, use_moveit):
            return False
        
    else:  # unplug
        # === 拔出流程 ===
        # 步骤1: 夹爪打开
        print("\n步骤1: 夹爪打开...")
        piper.GripperCtrl(config['gripper_release'], 1000, 0x01, 0)
        time.sleep(0.8)
        
        # 步骤2: 移动到连接器位置
        print("\n步骤2: 移动到连接器位置...")
        targetT_connector = np.eye(4)
        targetT_connector[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        targetT_connector[0, 3] = target_pos[0]
        targetT_connector[1, 3] = target_pos[1]
        targetT_connector[2, 3] = target_pos[2]
        
        joints_connector = piper_arm.inverse_kinematics(targetT_connector)
        if not joints_connector:
            print("❌ 连接器位置IK失败")
            return False
        
        joints_connector.append(config['gripper_hold'] / 1000000)
        if not control_arm(joints_connector, config['insert_speed'], use_moveit):
            return False
        time.sleep(1.0)
        
        # 步骤3: 垂直拔出
        print(f"\n步骤3: 垂直拔出 (行程{insert_depth*100:.1f}cm)...")
        targetT_extract = targetT_connector.copy()
        targetT_extract[2, 3] += insert_depth
        
        joints_extract = piper_arm.inverse_kinematics(targetT_extract)
        if not joints_extract:
            print("❌ 拔出位置IK失败")
            return False
        
        joints_extract.append(config['gripper_hold'] / 1000000)
        if not control_arm(joints_extract, config['extract_speed'], use_moveit):
            return False
        time.sleep(1.0)
        
        # 步骤4: 松开连接器
        print("\n步骤4: 松开连接器...")
        joints_extract[6] = config['gripper_release'] / 1000000
        control_arm_sdk(joints_extract, 10)
        time.sleep(0.5)
    
    # 返回安全位置
    print("\n返回安全位置...")
    joints_safe = [0, 0, -0.4, 0, 0, 0, config['gripper_release'] / 1000000]
    if not control_arm(joints_safe, FAST_SPEED, use_moveit):
        return False
    
    print("="*70)
    print(f"✓✓✓ Plug-in ({'插入' if action_type == 'plug' else '拔出'}) 操作完成！✓✓✓")
    print("="*70)
    return True


def action_push(target_pos, piper_arm, press_depth=None, hold_time=None, use_moveit=True):
    """
    按压按钮操作
    
    参数:
        target_pos: 按钮中心位置 [x, y, z] (基座坐标系)
        piper_arm: 机械臂对象
        press_depth: 按压深度 (米)，None=使用默认值
        hold_time: 保持按压时间 (秒)，None=使用默认值
        use_moveit: 是否使用 MoveIt
    
    返回:
        True: 成功, False: 失败
    """
    config = ACTION_CONFIGS['push']
    if press_depth is None:
        press_depth = config['press_depth']
    if hold_time is None:
        hold_time = config['hold_time']
    
    print("="*70)
    print("动作类型: Push (按压按钮)")
    print("="*70)
    print(f"目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
    print(f"按压深度: {press_depth*100:.1f}cm, 保持时间: {hold_time}秒")
    
    # 步骤1: 夹爪闭合（形成按压面）
    print("\n步骤1: 夹爪闭合（形成按压面）...")
    piper.GripperCtrl(config['gripper_close'], 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤2: 移动到按钮上方
    print(f"\n步骤2: 移动到按钮上方 {config['approach_offset']*100:.0f}cm...")
    targetT_above = np.eye(4)
    targetT_above[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    targetT_above[0, 3] = target_pos[0] - config['approach_offset']
    targetT_above[1, 3] = target_pos[1]
    targetT_above[2, 3] = target_pos[2]
    
    joints_above = piper_arm.inverse_kinematics(targetT_above)
    if not joints_above:
        print("❌ 上方位置IK失败")
        return False
    
    joints_above.append(config['gripper_close'] / 1000000)
    if not control_arm(joints_above, NORMAL_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    
    # 步骤3: 缓慢接近按钮表面
    print("\n步骤3: 缓慢接近按钮表面...")
    targetT_surface = targetT_above.copy()
    targetT_surface[0, 3] = target_pos[0]
    
    joints_surface = piper_arm.inverse_kinematics(targetT_surface)
    if not joints_surface:
        print("❌ 表面位置IK失败")
        return False
    
    joints_surface.append(config['gripper_close'] / 1000000)
    if not control_arm(joints_surface, config['press_speed'], use_moveit):
        return False
    time.sleep(0.5)
    
    # 步骤4: 按压到指定深度
    print(f"\n步骤4: 按压到指定深度 {press_depth*100:.1f}cm...")
    targetT_press = targetT_surface.copy()
    targetT_press[0, 3] += press_depth
    
    joints_press = piper_arm.inverse_kinematics(targetT_press)
    if not joints_press:
        print("❌ 按压位置IK失败")
        return False
    
    joints_press.append(config['gripper_close'] / 1000000)
    if not control_arm(joints_press, config['press_speed'], use_moveit):
        return False
    
    # 步骤5: 保持按压
    print(f"\n步骤5: 保持按压 {hold_time}秒...")
    time.sleep(hold_time)
    
    # 步骤6: 释放（回到表面）
    print("\n步骤6: 释放按压...")
    if not control_arm(joints_surface, config['press_speed'], use_moveit):
        return False
    time.sleep(0.5)
    
    # 步骤7: 返回上方位置
    print("\n步骤7: 返回上方位置...")
    if not control_arm(joints_above, NORMAL_SPEED, use_moveit):
        return False
    time.sleep(0.5)
    
    # 步骤8: 返回安全位置
    print("\n步骤8: 返回安全位置...")
    joints_safe = [0, 0, -0.4, 0, 0, 0, config['gripper_close'] / 1000000]
    if not control_arm(joints_safe, FAST_SPEED, use_moveit):
        return False
    
    print("="*70)
    print("✓✓✓ Push 操作完成！✓✓✓")
    print("="*70)
    return True


def action_knob(target_pos, piper_arm, rotation_angle=90, rotation_direction='cw', knob_diameter=0.02, use_moveit=True):
    """
    旋转旋钮操作
    
    参数:
        target_pos: 旋钮中心位置 [x, y, z] (基座坐标系)
        piper_arm: 机械臂对象
        rotation_angle: 旋转角度 (度)
        rotation_direction: 'cw'(顺时针) / 'ccw'(逆时针)
        knob_diameter: 旋钮直径 (米)
        use_moveit: 是否使用 MoveIt
    
    返回:
        True: 成功, False: 失败
    """
    config = ACTION_CONFIGS['knob']
    gripper_width = int(knob_diameter * 1000000) + config['gripper_offset']
    
    print("="*70)
    print("动作类型: Knob (旋转旋钮)")
    print("="*70)
    print(f"目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
    print(f"旋转: {rotation_angle}° ({rotation_direction}), 夹持宽度: {gripper_width/1000:.1f}mm")
    
    # 步骤1: 夹爪打开到合适宽度
    print(f"\n步骤1: 夹爪打开到 {gripper_width/1000:.1f}mm...")
    piper.GripperCtrl(gripper_width + 5000, 1000, 0x01, 0)  # 比旋钮大一点
    time.sleep(0.8)
    
    # 步骤2: 移动到旋钮上方
    print(f"\n步骤2: 移动到旋钮上方 {config['approach_offset']*100:.0f}cm...")
    targetT_above = np.eye(4)
    targetT_above[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    targetT_above[0, 3] = target_pos[0]
    targetT_above[1, 3] = target_pos[1]
    targetT_above[2, 3] = target_pos[2] + config['approach_offset']
    
    joints_above = piper_arm.inverse_kinematics(targetT_above)
    if not joints_above:
        print("❌ 上方位置IK失败")
        return False
    
    joints_above.append((gripper_width + 5000) / 1000000)
    if not control_arm(joints_above, NORMAL_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    
    # 步骤3: 下降到旋钮高度
    print("\n步骤3: 下降到旋钮高度...")
    targetT_knob = targetT_above.copy()
    targetT_knob[2, 3] = target_pos[2]
    
    joints_knob = piper_arm.inverse_kinematics(targetT_knob)
    if not joints_knob:
        print("❌ 旋钮位置IK失败")
        return False
    
    joints_knob.append(gripper_width / 1000000)
    if not control_arm(joints_knob, APPROACH_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    
    # 步骤4: 旋转夹爪
    direction_sign = 1 if rotation_direction == 'cw' else -1
    total_rotation = rotation_angle * direction_sign
    
    # 检查是否需要分段旋转
    if abs(total_rotation) > config['max_single_rotation']:
        print(f"\n步骤4: 分段旋转 (总角度{rotation_angle}°)...")
        segments = int(np.ceil(abs(total_rotation) / config['max_single_rotation']))
        angle_per_segment = total_rotation / segments
        
        for i in range(segments):
            print(f"  旋转段 {i+1}/{segments}: {angle_per_segment:.1f}°")
            joints_knob[5] += angle_per_segment * PI / 180
            if not control_arm(joints_knob, config['rotation_speed'], use_moveit):
                return False
            time.sleep(config['hold_time'])
    else:
        print(f"\n步骤4: 旋转夹爪 {rotation_angle}° ({rotation_direction})...")
        joints_knob[5] += total_rotation * PI / 180
        if not control_arm(joints_knob, config['rotation_speed'], use_moveit):
            return False
        time.sleep(config['hold_time'])
    
    # 步骤5: 松开夹爪
    print("\n步骤5: 松开夹爪...")
    joints_knob[6] = (gripper_width + 5000) / 1000000
    control_arm_sdk(joints_knob, 10)
    time.sleep(0.5)
    
    # 步骤6: 上升
    print("\n步骤6: 上升...")
    if not control_arm(joints_above, APPROACH_SPEED, use_moveit):
        return False
    time.sleep(0.5)
    
    # 步骤7: 返回安全位置
    print("\n步骤7: 返回安全位置...")
    joints_safe = [0, 0, -0.4, 0, 0, 0, 40000 / 1000000]
    if not control_arm(joints_safe, FAST_SPEED, use_moveit):
        return False
    
    print("="*70)
    print("✓✓✓ Knob 操作完成！✓✓✓")
    print("="*70)
    return True


def execute_action(action_type, target_pos, piper_arm, params=None, use_moveit=True):
    """
    统一的按钮操作接口
    
    参数:
        action_type: 'toggle' / 'plugin' / 'push' / 'knob'
        target_pos: 目标位置 [x, y, z] (基座坐标系)
        piper_arm: 机械臂对象
        params: 动作特定参数字典
        use_moveit: 是否使用 MoveIt
    
    返回:
        success: True/False
    """
    action_map = {
        'toggle': action_toggle,
        'plugin': action_plugin,
        'push': action_push,
        'knob': action_knob
    }
    
    if action_type not in action_map:
        print(f"❌ 未知动作类型: {action_type}")
        print(f"   支持的类型: {list(action_map.keys())}")
        return False
    
    # 执行对应动作
    try:
        if params is None:
            params = {}
        params['use_moveit'] = use_moveit
        return action_map[action_type](target_pos, piper_arm, **params)
    except Exception as e:
        print(f"❌ 动作执行失败: {e}")
        import traceback
        traceback.print_exc()
        return False


# ========================================
# 抓取流程（7步，使用 MoveIt）
# ========================================

def move_and_grasp(object_center, joints, piper_arm, 
                   gripper_close_value=None, 
                   rotation_angle=None, 
                   rotation_direction=None,
                   use_moveit=True):
    """
    7步抓取流程（预抓取 → 接近 → 抓取 → 抬起 → 旋转 → 返回）
    
    参数:
        object_center: 目标物体中心坐标（相机坐标系）
        joints: 当前关节角度
        piper_arm: 机械臂对象
        gripper_close_value: 夹爪闭合值(0.001mm单位)
        rotation_angle: 旋转角度(度)
        rotation_direction: 旋转方向(1=右旋,-1=左旋)
        use_moveit: 是否使用 MoveIt
    """
    # 使用默认值
    if gripper_close_value is None:
        gripper_close_value = GRIPPER_CLOSE_VALUE
    if rotation_angle is None:
        rotation_angle = ROTATION_ANGLE
    if rotation_direction is None:
        rotation_direction = ROTATION_DIRECTION
    
    print("="*70)
    print("开始抓取流程 [{}模式]".format("MoveIt" if use_moveit else "SDK"))
    print("="*70)
    print(f"物体位置(相机): ({object_center[0]:.3f}, {object_center[1]:.3f}, {object_center[2]:.3f})")
    print(f"抓取参数: 夹爪={gripper_close_value/1000:.1f}mm, 旋转={'右' if rotation_direction==1 else '左'}{rotation_angle}°")
    
    # ===== 坐标变换：相机坐标系 → 基坐标系 =====
    base_T_link6 = piper_arm.forward_kinematics(joints)
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera
    
    base_ob_center = base_T_link6 @ link6_T_cam @ np.array([object_center[0], object_center[1], object_center[2], 1])
    print(f"物体位置(基座): ({base_ob_center[0]:.3f}, {base_ob_center[1]:.3f}, {base_ob_center[2]:.3f})")
    
    # 发布可视化标记
    pub = rospy.Publisher('/target_point_under_based', Marker, queue_size=10)
    publish_sphere_marker(pub, base_ob_center, frame_id="arm_base", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)
    
    # 构建目标变换矩阵（夹爪朝下）
    targetT = np.array([[0, 0, 1, 0], 
                        [0, 1, 0, 0], 
                        [-1, 0, 0, 0], 
                        [0, 0, 0, 1]], dtype=float)
    targetT[0, 3] = base_ob_center[0]
    targetT[1, 3] = base_ob_center[1]
    targetT[2, 3] = base_ob_center[2]
    
    # 逆运动学求解（物体位置）
    joints_grasp = piper_arm.inverse_kinematics(targetT)
    if not joints_grasp:
        print("❌ 逆运动学求解失败")
        return False
    
    print(f"IK解 (度): [{', '.join([f'{j*180/PI:.1f}' for j in joints_grasp])}]")
    
    # ===== 步骤1: 夹爪完全打开 =====
    print("\n步骤1: 夹爪完全打开 (70mm)...")
    piper.GripperCtrl(70000, 1000, 0x01, 0)
    time.sleep(1.0)
    print("  ✓ 夹爪已打开")
    
    # ===== 步骤2: 移动到预抓取位置（物体上方） =====
    print(f"\n步骤2: 移动到预抓取位置 (物体上方{PRE_GRASP_OFFSET*100:.0f}cm)...")
    targetT_pre = targetT.copy()
    targetT_pre[0, 3] -= PRE_GRASP_OFFSET  # 沿夹爪z轴后退
    
    joints_pre = piper_arm.inverse_kinematics(targetT_pre)
    if not joints_pre:
        print("❌ 预抓取位置IK失败")
        return False
    
    joints_pre.append(0.07)  # 夹爪保持打开
    if not control_arm(joints_pre, NORMAL_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    print("  ✓ 已到达预抓取位置")
    
    # ===== 步骤3: 缓慢接近物体 =====
    print("\n步骤3: 缓慢下降到物体位置...")
    joints_grasp.append(0.07)  # 夹爪保持打开
    if not control_arm(joints_grasp, APPROACH_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    print("  ✓ 已到达物体位置")
    
    # ===== 步骤4: 闭合夹爪抓取 =====
    print(f"\n步骤4: 闭合夹爪抓取 (70mm → {gripper_close_value/1000:.1f}mm)...")
    joints_grasp[6] = gripper_close_value / 1000000  # 转换为米
    if not control_arm(joints_grasp, APPROACH_SPEED, use_moveit):
        return False
    time.sleep(1.5)
    print(f"  ✓ 夹爪已闭合到 {gripper_close_value/1000:.1f}mm (物体已抓取)")
    
    # ===== 步骤5: 抬起物体 =====
    print(f"\n步骤5: 抬起物体 (向上{LIFT_HEIGHT*100:.0f}cm)...")
    targetT_lift = targetT.copy()
    targetT_lift[2, 3] += LIFT_HEIGHT  # 沿z轴向上
    
    joints_lift = piper_arm.inverse_kinematics(targetT_lift)
    if not joints_lift:
        print("⚠️  抬起位置IK失败，跳过抬起步骤")
        joints_lift = joints_grasp[:6]
    
    joints_lift.append(gripper_close_value / 1000000)
    if not control_arm(joints_lift, APPROACH_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    print("  ✓ 物体已抬起")
    
    # ===== 步骤6: 旋转夹爪 =====
    actual_rotation = rotation_angle * rotation_direction
    print(f"\n步骤6: 旋转夹爪 {'右旋' if rotation_direction == 1 else '左旋'} {rotation_angle}度...")
    joints_lift[5] += actual_rotation * PI / 180
    if not control_arm(joints_lift, NORMAL_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    print("  ✓ 旋转完成")
    
    # ===== 步骤7: 返回安全位置 =====
    print("\n步骤7: 返回安全位置...")
    joints_safe = [0, 0, -0.4, 0, 0, joints_lift[5], gripper_close_value / 1000000]
    if not control_arm(joints_safe, FAST_SPEED, use_moveit):
        return False
    time.sleep(1.0)
    
    print("="*70)
    print("✓✓✓ 抓取任务完成！✓✓✓")
    print("="*70)
    
    return True


# ========================================
# 主程序
# ========================================

if __name__ == "__main__":
    print("="*70)
    print("Piper 视觉抓取程序 - MoveIt 增强版")
    print("="*70)
    print(f"\n📊 MoveIt 状态: {'✓ 已加载' if MOVEIT_AVAILABLE else '✗ 未加载(使用SDK模式)'}")
    print(f"\n⚙️  默认参数:")
    print(f"  夹爪闭合值: {GRIPPER_CLOSE_VALUE/1000:.1f}mm")
    print(f"  旋转: {'右' if ROTATION_DIRECTION==1 else '左'}{ROTATION_ANGLE}°")
    print(f"  预抓取偏移: {PRE_GRASP_OFFSET*100:.0f}cm")
    print(f"  抬起高度: {LIFT_HEIGHT*100:.0f}cm")
    
    # 用户自定义参数
    use_custom = input("\n是否使用自定义参数? (y/n, 默认n): ").strip().lower()
    
    custom_gripper = GRIPPER_CLOSE_VALUE
    custom_angle = ROTATION_ANGLE
    custom_direction = ROTATION_DIRECTION
    use_moveit = MOVEIT_AVAILABLE
    
    if use_custom == 'y':
        try:
            val = input(f"夹爪闭合值(0-70000, 默认{GRIPPER_CLOSE_VALUE}): ").strip()
            if val:
                custom_gripper = max(0, min(70000, int(val)))
            
            val = input(f"旋转角度(度, 默认{ROTATION_ANGLE}): ").strip()
            if val:
                custom_angle = float(val)
            
            val = input(f"旋转方向(1=右旋, -1=左旋, 默认{ROTATION_DIRECTION}): ").strip()
            if val and int(val) in [1, -1]:
                custom_direction = int(val)
            
            if MOVEIT_AVAILABLE:
                val = input("使用MoveIt? (y/n, 默认y): ").strip().lower()
                use_moveit = (val != 'n')
        except:
            print("⚠️  输入无效，使用默认值")
    
    print("\n最终参数:")
    print(f"  夹爪: {custom_gripper/1000:.1f}mm")
    print(f"  旋转: {'右' if custom_direction==1 else '左'}{custom_angle}°")
    print(f"  模式: {'MoveIt' if use_moveit else 'SDK'}")
    print("="*70)
    
    # ===== 初始化硬件 =====
    print("\n初始化机械臂...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    piper.GripperCtrl(70000, 1000, 0x01, 0)
    print("  ✓ 硬件初始化完成")
    
    # 回零位
    joints_zero = [0, 0, 0, 0, 0, 0, 0]
    control_arm_sdk(joints_zero, 100)
    time.sleep(2)
    
    # ===== 初始化 ROS =====
    print("\n初始化ROS节点...")
    rospy.init_node('vision_grasp_moveit_node', anonymous=True)
    
    # ===== 初始化 MoveIt =====
    move_group = None
    if use_moveit and MOVEIT_AVAILABLE:
        move_group = initialize_moveit()
        if move_group is None:
            print("⚠️  MoveIt初始化失败，将使用SDK模式")
            use_moveit = False
    
    # ===== 初始化 Piper Arm =====
    piper_arm = PiperArm()
    
    # ===== 订阅视觉检测结果 =====
    sub = rospy.Subscriber('/object_point',
                          PointStamped,
                          object_point_callback,
                          queue_size=10,
                          tcp_nodelay=True)
    
    print("\n✓ 系统就绪，等待视觉检测结果...")
    print("  订阅话题: /object_point")
    print("="*70)
    
    # ===== 主循环 =====
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if receive_object_center:
            # 读取当前关节角度
            msg = piper.GetArmJointMsgs()
            joints = [
                msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_6 * 1e-3 * PI / 180.0,
            ]
            
            # 执行抓取
            if move_and_grasp(object_center, joints, piper_arm, 
                            custom_gripper, custom_angle, custom_direction, use_moveit):
                print("\n程序结束")
                break
            
            receive_object_center = False
        
        rate.sleep()
    
    # ===== 清理资源 =====
    if MOVEIT_AVAILABLE:
        moveit_commander.roscpp_shutdown()




