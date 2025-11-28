#!/usr/bin/env python3
from piper_sdk import *
import rospy
import time
import sys
import numpy as np
import math
from piper_arm import PiperArm
from utils.utils_piper import read_joints
from utils.utils_piper import enable_fun
from utils.utils_ros import publish_tf, publish_sphere_marker, publish_trajectory
from utils.utils_math import quaternion_to_rotation_matrix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path

# MoveIt imports - with error handling
MOVEIT_AVAILABLE = False
try:
    import moveit_commander
    import moveit_msgs.msg
    import geometry_msgs.msg
    from moveit_commander.conversions import pose_to_list
    from moveit_msgs.msg import DisplayTrajectory
    from nav_msgs.msg import Path as NavPath
    from geometry_msgs.msg import PoseStamped
    MOVEIT_AVAILABLE = True
    print("✓ MoveIt 模块加载成功")
except ImportError as e:
    print(f"⚠️  MoveIt 模块加载失败: {e}")
    print("  将使用原始SDK控制模式")
    MOVEIT_AVAILABLE = False

PI = math.pi
factor = 1000 * 180 / PI
receive_object_center = False
object_center = []
simulation = True

# 用户可自定义参数
GRIPPER_CLOSE_VALUE = 40000  # 夹爪闭合值(单位:0.001mm) 默认40mm
ROTATION_ANGLE = 90  # 旋转角度(度) 默认90度
ROTATION_DIRECTION = 1  # 旋转方向: 1=右旋(顺时针), -1=左旋(逆时针)

# 抓取动作参数 (新增)
PRE_GRASP_OFFSET = 0.0  # 预抓取位置偏移(米) - 在物体上方10cm
LIFT_HEIGHT = 0.00       # 抬起高度(米) - 抓取后向上抬起5cm
APPROACH_SPEED = 20      # 接近物体速度 (1-100) - 慢速接近避免碰撞

# MoveIt 配置参数
USE_MOVEIT = True  # ⚠️ 是否使用MoveIt进行轨迹规划 (True=MoveIt, False=SDK直接控制)
                     # 建议: 如果SDK模式工作正常,保持False; 需要避障功能时改为True
PLANNING_TIME = 5.0  # 规划时间(秒)
PLANNING_ATTEMPTS = 10  # 规划尝试次数
VELOCITY_SCALING = 1.0  # 速度缩放因子 (0.0-1.0) - 改为最大速度
ACCELERATION_SCALING = 1.0  # 加速度缩放因子 (0.0-1.0) - 改为最大加速度

# 轨迹执行频率控制 (新增)
TRAJECTORY_PUBLISH_RATE = 50   # 轨迹发布到RViz的频率 (Hz)
TRAJECTORY_EXECUTE_RATE = 60   # SDK执行轨迹的频率 (Hz) - 建议50Hz
TRAJECTORY_SAMPLE_POINTS = 15  # 轨迹采样点数


def control_arm(joints, speed=2):
    """控制机械臂运动（原始SDK方式）
    
    Args:
        joints: 关节角度列表 [rad], 如果长度>6则最后一个是夹爪位置[m]
        speed: 运动速度
    """
    position = joints

    joint_0 = int(position[0] * factor)
    joint_1 = int(position[1] * factor)
    joint_2 = int(position[2] * factor)
    joint_3 = int(position[3] * factor)
    joint_4 = int(position[4] * factor)
    joint_5 = int(position[5] * factor)

    if (joint_4 < -70000) :
        joint_4 = -70000

    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    if len(joints) > 6:
        joint_6 = round(position[6] * 1000 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

    print(piper.GetArmStatus())
    print(position)


def control_arm_moveit(move_group, joints, gripper_value=None):
    """使用MoveIt控制机械臂运动（优化版：频率控制+轨迹可视化）
    
    Args:
        move_group: MoveIt的MoveGroupCommander对象
        joints: 关节角度列表 [rad]
        gripper_value: 夹爪值(0.001mm单位), None=不控制夹爪
    
    Returns:
        True: 规划并执行成功
        False: 失败
    """
    try:
        global display_trajectory_publisher, end_effector_path_publisher
        
        # 1. 清除旧状态并设置新目标
        move_group.clear_pose_targets()
        move_group.stop()
        
        # 设置关节目标
        joint_goal = move_group.get_current_joint_values()
        for i in range(min(6, len(joints))):
            joint_goal[i] = joints[i]
        move_group.set_joint_value_target(joint_goal)
        
        # 2. MoveIt规划
        print("  [MoveIt] 规划轨迹...")
        plan = move_group.plan()
        
        # 检查规划结果 (兼容不同MoveIt版本)
        if isinstance(plan, tuple):
            success, trajectory = plan[0], plan[1]
        else:
            success = plan.joint_trajectory.points != []
            trajectory = plan
        
        if not success:
            print("  ❌ 规划失败")
            return False
        
        # 提取轨迹点
        if isinstance(plan, tuple):
            traj_points = plan[1].joint_trajectory.points
        else:
            traj_points = plan.joint_trajectory.points
        
        if len(traj_points) == 0:
            print("  ❌ 轨迹为空")
            return False
        
        print(f"  ✓ 规划成功 (轨迹点: {len(traj_points)})")
        
        # 3. 发布关节轨迹到RViz可视化
        if display_trajectory_publisher is not None:
            try:
                display_msg = DisplayTrajectory()
                display_msg.trajectory_start = move_group.get_current_state()
                display_msg.trajectory.append(trajectory if isinstance(plan, tuple) else plan)
                display_trajectory_publisher.publish(display_msg)
                print(f"  ✓ 关节轨迹已发布 (话题: /move_group/display_planned_path)")
            except Exception as e:
                print(f"  ⚠️ 轨迹发布失败: {e}")
        
        # 4. 计算并发布末端执行器路径
        if end_effector_path_publisher is not None:
            try:
                path_msg = NavPath()
                path_msg.header.frame_id = "arm_base"
                path_msg.header.stamp = rospy.Time.now()
                
                # 为每个轨迹点计算末端位姿
                for point in traj_points[::max(1, len(traj_points)//20)]:  # 采样20个点
                    robot_state = move_group.get_current_state()
                    robot_state.joint_state.position = point.positions
                    
                    pose = move_group.get_current_pose().pose
                    pose_stamped = PoseStamped()
                    pose_stamped.header = path_msg.header
                    pose_stamped.pose = pose
                    path_msg.poses.append(pose_stamped)
                
                end_effector_path_publisher.publish(path_msg)
                print(f"  ✓ 末端轨迹已发布 (点数: {len(path_msg.poses)}, 话题: /end_effector_path)")
            except Exception as e:
                print(f"  ⚠️ 末端轨迹发布失败: {e}")
        
        # 5. SDK执行轨迹（采样并使用ROS Rate控制频率）
        sample_indices = np.linspace(0, len(traj_points)-1, 
                                    min(TRAJECTORY_SAMPLE_POINTS, len(traj_points)), 
                                    dtype=int)
        sample_points = [traj_points[i] for i in sample_indices]
        
        print(f"  [SDK] 执行轨迹 (采样点: {len(sample_points)}, 频率: {TRAJECTORY_EXECUTE_RATE}Hz)")
        piper.MotionCtrl_2(0x01, 0x01, 5, 0x00)
        
        # 使用ROS Rate控制执行频率
        rate = rospy.Rate(TRAJECTORY_EXECUTE_RATE)
        for idx, point in enumerate(sample_points):
            joint_values = list(point.positions[:6])
            
            # 转换为SDK格式
            j0 = int(joint_values[0] * factor)
            j1 = int(joint_values[1] * factor)
            j2 = int(joint_values[2] * factor)
            j3 = int(joint_values[3] * factor)
            j4 = int(joint_values[4] * factor)
            j5 = int(joint_values[5] * factor)
            
            if j4 < -70000:
                j4 = -70000
            
            piper.JointCtrl(j0, j1, j2, j3, j4, j5)
            
            # 按照指定频率执行
            if idx < len(sample_points) - 1:
                rate.sleep()
            
            if idx % 5 == 0:
                print(f"    进度: {idx+1}/{len(sample_points)}")
        
        # 6. 到达最终位置并等待稳定
        print("  到达最终目标位置...")
        final_point = traj_points[-1]
        final_joints = list(final_point.positions[:6])
        
        j0 = int(final_joints[0] * factor)
        j1 = int(final_joints[1] * factor)
        j2 = int(final_joints[2] * factor)
        j3 = int(final_joints[3] * factor)
        j4 = int(final_joints[4] * factor)
        j5 = int(final_joints[5] * factor)
        
        if j4 < -70000:
            j4 = -70000
        
        # 发送最终位置
        piper.JointCtrl(j0, j1, j2, j3, j4, j5)
        rospy.sleep(0.5)
        
        print(f"  ✓ 已到达目标位置")
        
        # 7. 控制夹爪（夹爪不通过MoveIt控制）
        if gripper_value is not None:
            piper.GripperCtrl(gripper_value, 1000, 0x01, 0)
        
        print("  ✓ SDK执行完成")
        return True
        
    except Exception as e:
        print(f"  ❌ MoveIt控制失败: {e}")
        import traceback
        traceback.print_exc()
        return False



def control_arm_moveit_pose(move_group, target_pose, gripper_value=None):
    """使用MoveIt通过目标位姿控制机械臂
    
    Args:
        move_group: MoveIt的MoveGroupCommander对象
        target_pose: geometry_msgs/Pose对象或4x4变换矩阵
        gripper_value: 夹爪值(0.001mm单位), None=不控制夹爪
    
    Returns:
        True: 规划并执行成功
        False: 失败
    """
    try:
        # 如果是numpy矩阵，转换为Pose
        if isinstance(target_pose, np.ndarray):
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = target_pose[0, 3]
            pose_goal.position.y = target_pose[1, 3]
            pose_goal.position.z = target_pose[2, 3]
            
            # 从旋转矩阵计算四元数
            from tf.transformations import quaternion_from_matrix
            quat = quaternion_from_matrix(target_pose)
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]
        else:
            pose_goal = target_pose
        
        move_group.set_pose_target(pose_goal)
        
        print("MoveIt笛卡尔规划中...")
        plan = move_group.plan()
        
        # 检查规划是否成功
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
        else:
            success = plan.joint_trajectory.points != []
            trajectory = plan
        
        if not success:
            print("❌ MoveIt笛卡尔规划失败")
            return False
        
        print("✓ MoveIt规划成功，执行轨迹...")
        execute_result = move_group.execute(trajectory, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        
        # 控制夹爪
        if gripper_value is not None:
            piper.GripperCtrl(gripper_value, 1000, 0x01, 0)
        
        print("✓ MoveIt执行完成")
        return True
        
    except Exception as e:
        print(f"❌ MoveIt笛卡尔控制失败: {e}")
        return False

def object_point_callback(msg):
    # print("Receive visual detection result", msg.point.x, msg.point.y, msg.point.z)
    if(np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z)):
        return
    global receive_object_center, object_center
    receive_object_center = True
    object_center = [msg.point.x, msg.point.y, msg.point.z]


def move_and_grasp(object_center, joints, piper_arm, move_group=None, gripper_close_value=None, rotation_angle=None, rotation_direction=None):
    """移动并抓取物体（支持MoveIt规划）
    
    Args:
        object_center: 目标物体中心坐标
        joints: 当前关节角度
        piper_arm: 机械臂对象
        move_group: MoveIt的MoveGroupCommander对象, None=使用原始SDK控制
        gripper_close_value: 夹爪闭合值(0.001mm单位), None=使用全局默认值
        rotation_angle: 旋转角度(度), None=使用全局默认值
        rotation_direction: 旋转方向(1=右旋,-1=左旋), None=使用全局默认值
    """
    # 使用传入参数或全局默认值
    if gripper_close_value is None:
        gripper_close_value = GRIPPER_CLOSE_VALUE
    if rotation_angle is None:
        rotation_angle = ROTATION_ANGLE
    if rotation_direction is None:
        rotation_direction = ROTATION_DIRECTION
    
    use_moveit = USE_MOVEIT and move_group is not None
    
    print("="*70)
    print(f"抓取模式: {'MoveIt轨迹规划' if use_moveit else '原始SDK点到点'}")
    print("="*70)
    print("prepare to grasp point under camera frame", object_center[0], object_center[1], object_center[2])
    print(f"抓取前: 夹爪完全打开 (70mm)")
    print(f"抓取后: 夹爪闭合到 {gripper_close_value/1000:.1f}mm")
    print(f"旋转设置: {'右旋' if rotation_direction == 1 else '左旋'} {rotation_angle}度")
    
    # 步骤1: 先确保夹爪完全打开 (准备抓取)
    print("\n步骤1: 夹爪完全打开...")
    piper.GripperCtrl(70000, 1000, 0x01, 0)  # 70000 = 70mm 完全打开
    time.sleep(0.8)  # 减少等待时间

    # transfer point from camera frame to base_link frame
    base_T_link6 = piper_arm.forward_kinematics(joints)
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera

    base_ob_center = base_T_link6 @ link6_T_cam @ np.array([object_center[0], object_center[1], object_center[2], 1])

    # publish target object center
    print("point under base frame", base_ob_center)
    pub = rospy.Publisher('/target_point_under_based', Marker, queue_size=10)
    publish_sphere_marker(pub, base_ob_center, frame_id="arm_base", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)

    targetT = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
    targetT[0, 3] = base_ob_center[0]
    targetT[1, 3] = base_ob_center[1]
    targetT[2, 3] = base_ob_center[2]

    # inverse kinematics
    joints = piper_arm.inverse_kinematics(targetT)
    joints_array = np.array(joints)
    print("base ob center", base_ob_center)
    if not joints :
        print("ik fail")
        return False
    print("Planned ik[degree]:", joints_array / PI * 180)

    # 步骤2: 移动到物体上方预抓取位置 (夹爪保持打开 70mm)
    print(f"\n步骤2: 移动到物体上方 {PRE_GRASP_OFFSET*100:.0f}cm (预抓取位置, 夹爪保持打开 70mm)...")
    
    # 计算预抓取位置 (沿着夹爪z轴方向后退)
    pre_grasp_targetT = targetT.copy()
    pre_grasp_targetT[0, 3] -= PRE_GRASP_OFFSET * targetT[0, 2]  # 沿夹爪z轴后退
    pre_grasp_targetT[1, 3] -= PRE_GRASP_OFFSET * targetT[1, 2]
    pre_grasp_targetT[2, 3] -= PRE_GRASP_OFFSET * targetT[2, 2]
    
    joints_pre_grasp = piper_arm.inverse_kinematics(pre_grasp_targetT)
    if not joints_pre_grasp:
        print("❌ 预抓取位置逆运动学失败，使用原位置")
        joints_pre_grasp = joints
    
    if use_moveit:
        if not control_arm_moveit(move_group, joints_pre_grasp, gripper_value=70000):
            print("❌ MoveIt移动失败，尝试使用原始SDK...")
            joints_pre_grasp.append(0.07)
            control_arm(joints_pre_grasp, 80)
    else:
        joints_pre_grasp.append(0.07)
        control_arm(joints_pre_grasp, 80)

    print("  ⏱️  等待到达预抓取位置...")
    time.sleep(1.5)
    print("  ✓ 已到达预抓取位置")
    
    # 步骤3: 缓慢下降到物体位置 (接近抓取)
    print(f"\n步骤3: 缓慢下降到物体位置 (夹爪保持打开)...")
    if use_moveit:
        if not control_arm_moveit(move_group, joints, gripper_value=70000):
            print("❌ MoveIt移动失败，尝试使用原始SDK...")
            joints.append(0.07)
            control_arm(joints, APPROACH_SPEED)  # 使用配置的接近速度
    else:
        joints.append(0.07)
        control_arm(joints, APPROACH_SPEED)  # 慢速接近物体

    print("  ⏱️  等待到达抓取位置...")
    time.sleep(1.0)
    print("  ✓ 已到达抓取位置")
    
    # 步骤4: 闭合夹爪抓取物体
    print(f"\n步骤4: 闭合夹爪抓取 (从70mm闭合到{gripper_close_value/1000:.1f}mm)...")
    piper.GripperCtrl(gripper_close_value, 1000, 0x01, 0)
    time.sleep(1.0)
    print(f"✓ 夹爪已闭合到 {gripper_close_value/1000:.1f}mm (物体已抓取)")
    
    # 步骤5: 抬起物体 (沿z轴上升)
    print(f"\n步骤5: 抬起物体 {LIFT_HEIGHT*100:.0f}cm...")
    
    lift_targetT = targetT.copy()
    lift_targetT[0, 3] -= LIFT_HEIGHT * targetT[0, 2]
    lift_targetT[1, 3] -= LIFT_HEIGHT * targetT[1, 2]
    lift_targetT[2, 3] -= LIFT_HEIGHT * targetT[2, 2]
    
    joints_lift = piper_arm.inverse_kinematics(lift_targetT)
    if not joints_lift:
        print("⚠️  抬起位置逆运动学失败，跳过抬起")
        joints_lift = joints
    
    if use_moveit:
        if not control_arm_moveit(move_group, joints_lift, gripper_value=gripper_close_value):
            joints_lift.append(gripper_close_value / 1000000)
            control_arm(joints_lift, 40)
    else:
        joints_lift.append(gripper_close_value / 1000000)
        control_arm(joints_lift, 40)
    
    time.sleep(0.8)
    print("✓ 物体已抬起")
    
    # 步骤6: 旋转夹爪 (带物体旋转)
    actual_rotation = rotation_angle * rotation_direction
    print(f"\n步骤6: 旋转夹爪 {'右旋' if rotation_direction == 1 else '左旋'} {rotation_angle}度 (保持夹爪闭合)...")
    
    # 更新关节角度（加上旋转）
    joints_rotated = joints_lift[:6] if len(joints_lift) > 6 else joints_lift[:]
    joints_rotated[5] += actual_rotation * PI / 180  # 在当前角度基础上旋转
    
    if use_moveit:
        if not control_arm_moveit(move_group, joints_rotated, gripper_value=gripper_close_value):
            print("❌ MoveIt旋转失败，尝试使用原始SDK...")
            joints_rotated.append(gripper_close_value / 1000000)
            control_arm(joints_rotated, 60)  # 提高速度到60
    else:
        joints_rotated.append(gripper_close_value / 1000000)
        control_arm(joints_rotated, 60)  # 提高速度到60
    
    time.sleep(1.0)  # 减少等待时间
    print(f"✓ 旋转完成")
    
    # 步骤7: 返回安全位置 (保持夹爪闭合和旋转状态)
    print(f"\n步骤7: 返回安全位置 (保持夹爪闭合{gripper_close_value/1000:.1f}mm和旋转状态)...")
    joints_safe = [0, 0, -0.4, 0, 0, joints_rotated[5]]  # 保持旋转角度
    
    if use_moveit:
        if not control_arm_moveit(move_group, joints_safe, gripper_value=gripper_close_value):
            print("❌ MoveIt返回失败，尝试使用原始SDK...")
            joints_safe.append(gripper_close_value / 1000000)
            control_arm(joints_safe, 80)  # 提高速度到80
    
    time.sleep(1.0)  # 减少等待时间
    print("="*70)
    print("✓✓✓ 抓取任务完成！✓✓✓")
    print("="*70)

    return True



if __name__ == "__main__":
    # 用户可在此处自定义参数
    print("="*70)
    print("Piper 视觉抓取程序 (支持MoveIt轨迹规划)")
    print("="*70)
    print("\n⚙️  MoveIt配置:")
    print(f"  使用MoveIt: {USE_MOVEIT}")
    print(f"  规划时间: {PLANNING_TIME}秒")
    print(f"  规划尝试次数: {PLANNING_ATTEMPTS}")
    print(f"  速度缩放: {VELOCITY_SCALING}")
    print(f"  加速度缩放: {ACCELERATION_SCALING}")
    print("\n⚠️  用户自定义参数:")
    print(f"  夹爪闭合值: {GRIPPER_CLOSE_VALUE} (0.001mm) = {GRIPPER_CLOSE_VALUE/1000:.1f}mm")
    print(f"  旋转角度: {ROTATION_ANGLE}度")
    print(f"  旋转方向: {'右旋(顺时针)' if ROTATION_DIRECTION == 1 else '左旋(逆时针)'}")
    print("\n💡 修改方法: 编辑文件顶部的全局变量")
    print("  USE_MOVEIT = True/False  # 启用/禁用MoveIt")
    print("  GRIPPER_CLOSE_VALUE = 40000  # 40mm")
    print("  ROTATION_ANGLE = 90  # 90度")
    print("  ROTATION_DIRECTION = 1  # 1=右旋, -1=左旋")
    print("="*70)
    
    # 允许用户临时修改参数
    use_custom = input("\n是否使用自定义参数? (y/n, 默认n): ").strip().lower()
    
    custom_gripper = GRIPPER_CLOSE_VALUE
    custom_angle = ROTATION_ANGLE
    custom_direction = ROTATION_DIRECTION
    use_moveit = USE_MOVEIT
    
    if use_custom == 'y':
        try:
            # MoveIt开关
            val = input(f"是否使用MoveIt? (y/n, 默认{'y' if USE_MOVEIT else 'n'}): ").strip().lower()
            if val in ['y', 'n']:
                use_moveit = (val == 'y')
            
            val = input(f"输入夹爪闭合值(0.001mm单位, 0-70000, 默认{GRIPPER_CLOSE_VALUE}): ").strip()
            if val:
                custom_gripper = int(val)
                if not (0 <= custom_gripper <= 70000):
                    print(f"⚠️  值超出范围，使用默认值 {GRIPPER_CLOSE_VALUE}")
                    custom_gripper = GRIPPER_CLOSE_VALUE
            
            val = input(f"输入旋转角度(度, 默认{ROTATION_ANGLE}): ").strip()
            if val:
                custom_angle = float(val)
            
            dir_input = input(f"输入旋转方向(1=右旋, -1=左旋, 默认{ROTATION_DIRECTION}): ").strip()
            if dir_input:
                custom_direction = int(dir_input)
                if custom_direction not in [1, -1]:
                    print(f"⚠️  方向无效，使用默认值 {ROTATION_DIRECTION}")
                    custom_direction = ROTATION_DIRECTION
        except:
            print("⚠️  输入无效，使用默认值")
    
    print("\n" + "="*70)
    print("最终使用参数:")
    print(f"  MoveIt规划: {'启用' if use_moveit else '禁用'}")
    print(f"  夹爪闭合值: {custom_gripper} = {custom_gripper/1000:.1f}mm")
    print(f"  旋转: {'右旋' if custom_direction == 1 else '左旋'} {custom_angle}度")
    print("="*70)
    
    # 初始化ROS节点
    rospy.init_node('vision_grasp_moveit_node', anonymous=True)
    
    # 初始化MoveIt (如果启用)
    move_group = None
    if use_moveit:
        if not MOVEIT_AVAILABLE:
            print("\n⚠️  MoveIt 不可用，自动切换到原始SDK模式")
            use_moveit = False
        else:
            try:
                print("\n初始化MoveIt...")
                moveit_commander.roscpp_initialize(sys.argv)
                robot = moveit_commander.RobotCommander()
                scene = moveit_commander.PlanningSceneInterface()
                
                # 初始化轨迹可视化发布器
                global display_trajectory_publisher, end_effector_path_publisher
                display_trajectory_publisher = rospy.Publisher(
                    '/move_group/display_planned_path',
                    DisplayTrajectory,
                    queue_size=20
                )
                end_effector_path_publisher = rospy.Publisher(
                    '/end_effector_path',
                    NavPath,
                    queue_size=10
                )
                print(f"  ✓ RViz轨迹可视化已启用")
                print(f"  ✓ 末端路径发布器已启用 (频率: {TRAJECTORY_PUBLISH_RATE}Hz)")
                
                # 创建MoveGroup (根据SRDF配置，机械臂组名为 "arm")
                group_name = "arm"
                move_group = moveit_commander.MoveGroupCommander(group_name)
                
                # 配置规划参数
                move_group.set_planning_time(PLANNING_TIME)
                move_group.set_num_planning_attempts(PLANNING_ATTEMPTS)
                move_group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
                move_group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)
                
                # 设置参考坐标系
                move_group.set_pose_reference_frame("arm_base")
                
                print(f"✓ MoveIt初始化成功")
                print(f"  规划组: {group_name}")
                print(f"  参考坐标系: {move_group.get_pose_reference_frame()}")
                print(f"  末端执行器: {move_group.get_end_effector_link()}")
                print(f"  关节数量: {len(move_group.get_active_joints())}")
                
            except Exception as e:
                print(f"⚠️  MoveIt初始化失败: {e}")
                print("  将使用原始SDK控制模式")
                move_group = None
                use_moveit = False
    
    # 初始化机械臂SDK
    print("\n初始化机械臂SDK...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    piper.GripperCtrl(70000, 1000, 0x01, 0)  # 初始化: 完全打开
    print("✓ 机械臂SDK初始化成功")

    # 设置初始位置
    joints = [0, 0, 0, 0, 0, 0, 0]
    control_arm(joints, 100)
    time.sleep(2)

    # 初始化PiperArm
    piper_arm = PiperArm()
    
    # 订阅视觉检测结果
    sub = rospy.Subscriber('/object_point',
                           PointStamped,
                           object_point_callback,
                           queue_size=10,
                           tcp_nodelay=True)

    print("\n等待视觉检测结果...")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (receive_object_center):
            # 获取当前关节角度
            msg = piper.GetArmJointMsgs()

            theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
            theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
            theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
            theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
            theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
            theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

            joints = [theta1, theta2, theta3, theta4, theta5, theta6]

            # 执行抓取（传入MoveGroup对象）
            if move_and_grasp(object_center, joints, piper_arm, move_group, 
                            custom_gripper, custom_angle, custom_direction):
                break
            receive_object_center = False

        rate.sleep()
    
    # 清理
    if use_moveit and move_group is not None:
        moveit_commander.roscpp_shutdown()
    
    print("\n程序结束")







