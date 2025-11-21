#!/usr/bin/env python3
"""
机械臂前向按压操作程序
夹爪闭合状态下，沿夹爪z轴方向对目标物体实现前向按压
支持MoveIt轨迹规划（自动检测，失败时回退到SDK模式）
"""
from piper_sdk import *
import time
import math
import numpy as np
from piper_arm import PiperArm
from utils.utils_piper import enable_fun

# 尝试导入MoveIt (如果失败则使用SDK模式)
MOVEIT_AVAILABLE = False
display_trajectory_publisher = None
end_effector_path_publisher = None
try:
    import rospy
    import moveit_commander
    from moveit_msgs.msg import DisplayTrajectory
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    MOVEIT_AVAILABLE = True
    print("✓ MoveIt已加载，将使用MoveIt轨迹规划")
except ImportError as e:
    print(f"⚠️  MoveIt导入失败: {e}")
    print("   将使用SDK模式（直接控制）")

PI = math.pi
factor = 1000 * 180 / PI

# ========== MoveIt配置 ==========
USE_MOVEIT = True  # 是否使用MoveIt（需要MOVEIT_AVAILABLE=True）
VELOCITY_SCALING = 1.0      # MoveIt速度缩放 (0.1-1.0)
ACCELERATION_SCALING = 1.0  # MoveIt加速度缩放 (0.1-1.0)

# 轨迹执行频率控制
TRAJECTORY_PUBLISH_RATE = 10   # 轨迹发布到RViz的频率 (Hz)
TRAJECTORY_EXECUTE_RATE = 50   # SDK执行轨迹的频率 (Hz) - 建议50Hz
TRAJECTORY_SAMPLE_POINTS = 20  # 轨迹采样点数
# ================================

# ========== 用户自定义参数 ==========
# 目标位置 (基座坐标系, 单位:米)
TARGET_X = 0.25  # 目标物体x坐标 (表面位置)
TARGET_Y = 0.0   # 目标物体y坐标
TARGET_Z = 0.20  # 目标物体z坐标

# 按压参数（沿夹爪z轴方向）
PRESS_DISTANCE_BEFORE = 0.05  # 目标前方停留距离 (米) 默认5cm，沿夹爪z轴后退
PRESS_DEPTH = 0.02            # 按压深度 (米) 默认2cm，沿夹爪z轴前进
PRESS_DURATION = 2.0          # 按压持续时间 (秒)

# 夹爪状态 (单位:0.001mm, 0=闭合, 70000=完全打开)
GRIPPER_CLOSE_VALUE = 0  # 按压时夹爪闭合

# 运动速度宏定义 (1-100, 数值越大越快)
SPEED_ZERO = 100              # 回零位速度
SPEED_TO_START = 50           # 移动到目标前方的速度
SPEED_APPROACH = 100           # 接近物体表面的速度
SPEED_PRESS = 100              # 按压时的速度
SPEED_RETREAT = 100            # 后退的速度
# ====================================


def control_arm(piper, joints, speed=20):
    """控制机械臂运动"""
    position = joints
    
    joint_0 = int(position[0] * factor)
    joint_1 = int(position[1] * factor)
    joint_2 = int(position[2] * factor)
    joint_3 = int(position[3] * factor)
    joint_4 = int(position[4] * factor)
    joint_5 = int(position[5] * factor)
    
    if joint_4 < -70000:
        joint_4 = -70000
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    
    if len(joints) > 6:
        joint_6 = round(position[6] * 1000 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    
    print(f"关节角度 (度): [{position[0]*180/PI:.1f}, {position[1]*180/PI:.1f}, {position[2]*180/PI:.1f}, {position[3]*180/PI:.1f}, {position[4]*180/PI:.1f}, {position[5]*180/PI:.1f}]")
    
    # 等待运动完成
    time.sleep(2)
    return True


def control_arm_moveit(piper, move_group, joints, speed=5, gripper_value=None):
    """使用MoveIt规划轨迹，然后用SDK执行
    
    Args:
        piper: PiperInterface对象
        move_group: MoveIt move_group对象
        joints: 目标关节角度 [j0,j1,j2,j3,j4,j5], 单位:弧度
        speed: SDK执行速度 (1-100)
        gripper_value: 夹爪位置 (0.001mm单位), None=不控制
    
    Returns:
        True: 成功, False: 失败
    """
    try:
        global display_trajectory_publisher, end_effector_path_publisher
        
        # 1. 清除旧状态并设置新目标
        move_group.clear_pose_targets()
        move_group.stop()
        move_group.set_joint_value_target(joints)
        move_group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
        move_group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)
        
        # 2. MoveIt规划
        print("  [MoveIt] 规划轨迹...")
        plan = move_group.plan()
        
        # 检查规划结果
        if isinstance(plan, tuple):
            success, trajectory = plan[0], plan[1]
        else:
            success, trajectory = True, plan
        
        if not success or not trajectory.joint_trajectory.points:
            print("  ❌ 规划失败")
            return False
        
        traj_points = trajectory.joint_trajectory.points
        print(f"  ✓ 规划成功 (轨迹点: {len(traj_points)})")
        
        # 3. 发布关节轨迹到RViz可视化
        if display_trajectory_publisher is not None:
            display_msg = DisplayTrajectory()
            display_msg.trajectory_start = move_group.get_current_state()
            display_msg.trajectory.append(trajectory)
            display_trajectory_publisher.publish(display_msg)
            print(f"  ✓ 关节轨迹已发布 (话题: /move_group/display_planned_path)")
        
        # 4. 计算并发布末端执行器路径
        if end_effector_path_publisher is not None:
            path_msg = Path()
            path_msg.header.frame_id = "dummy_link"  # 或 "base_link"
            path_msg.header.stamp = rospy.Time.now()
            
            # 为每个轨迹点计算末端位姿
            for point in traj_points:
                robot_state = move_group.get_current_state()
                robot_state.joint_state.position = point.positions
                
                # 获取末端执行器位姿
                pose = move_group.get_current_pose().pose
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose = pose
                path_msg.poses.append(pose_stamped)
            
            end_effector_path_publisher.publish(path_msg)
            print(f"  ✓ 末端轨迹已发布 (点数: {len(path_msg.poses)}, 话题: /end_effector_path)")
        
        # 5. SDK执行轨迹（采样）
        sample_indices = np.linspace(0, len(traj_points)-1, min(TRAJECTORY_SAMPLE_POINTS, len(traj_points)), dtype=int)
        sample_points = [traj_points[i] for i in sample_indices]
        
        print(f"  [SDK] 执行轨迹 (采样点: {len(sample_points)}, 速度: {speed}, 频率: {TRAJECTORY_EXECUTE_RATE}Hz)")
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        
        # 使用ROS Rate控制执行频率
        rate = rospy.Rate(TRAJECTORY_EXECUTE_RATE)
        for idx, point in enumerate(sample_points):
            joints_int = [int(point.positions[i] * factor) for i in range(6)]
            joints_int[4] = max(-70000, joints_int[4])  # 限制joint4范围
            piper.JointCtrl(*joints_int)
            
            # 按照指定频率执行
            if idx < len(sample_points) - 1:  # 最后一个点不sleep
                rate.sleep()
        
        # 6. 到达最终位置并等待稳定
        final_joints = [int(traj_points[-1].positions[i] * factor) for i in range(6)]
        final_joints[4] = max(-70000, final_joints[4])
        piper.JointCtrl(*final_joints)
        rospy.sleep(0.5)  # 等待到达
        
        # 7. 控制夹爪
        if gripper_value is not None:
            piper.GripperCtrl(gripper_value, 1000, 0x01, 0)
        
        print(f"  ✓ 执行完成")
        return True
        
    except Exception as e:
        print(f"  ❌ 执行失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def xyz_to_joints(piper_arm, x, y, z):
    """将XYZ坐标转换为关节角度（垂直向下姿态）"""
    targetT = np.array([
        [0,  0,  1, x],
        [0,  1,  0, y],
        [-1, 0,  0, z],
        [0,  0,  0, 1]
    ], dtype=float)
    
    return piper_arm.inverse_kinematics(targetT)


def move_to_position(piper, piper_arm, move_group, x, y, z, speed, use_moveit=False):
    """移动到指定位置"""
    joints = xyz_to_joints(piper_arm, x, y, z)
    if not joints:
        print(f"❌ 逆运动学求解失败: ({x:.3f}, {y:.3f}, {z:.3f})")
        return False
    
    if use_moveit and move_group:
        return control_arm_moveit(piper, move_group, joints, speed)
    else:
        return control_arm(piper, joints, speed)


def press_action(piper, piper_arm, target_x, target_y, target_z, height_above, press_depth, press_duration, gripper_value, move_group=None):
    """执行前向按压动作（沿夹爪z轴方向）"""
    use_moveit = MOVEIT_AVAILABLE and USE_MOVEIT and move_group is not None
    print(f"\n{'='*50}\n按压操作 {'[MoveIt]' if use_moveit else '[SDK]'}\n{'='*50}")
    
    # 1. 夹爪闭合
    print(f"\n1. 夹爪闭合 {gripper_value/1000:.1f}mm")
    piper.GripperCtrl(gripper_value, 1000, 0x01, 0)
    time.sleep(1)
    
    # 2. 移动到A点（目标前方）
    start_x = target_x - height_above
    print(f"\n2. 移到A点 ({start_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    if not move_to_position(piper, piper_arm, move_group, start_x, target_y, target_z, SPEED_TO_START, use_moveit):
        return False
    print(f"   保持1秒...")
    time.sleep(1)
    
    # 3. 按压（前进到按压位置并保持）
    press_x = target_x + press_depth
    print(f"\n3. 按压 {(height_above + press_depth)*1000:.1f}mm → ({press_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    if not move_to_position(piper, piper_arm, move_group, press_x, target_y, target_z, SPEED_PRESS, use_moveit):
        return False
    print(f"   保持 {press_duration}秒...")
    time.sleep(press_duration)
    
    # 4. 后退到A点
    print(f"\n4. 后退到A点 ({start_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    if not move_to_position(piper, piper_arm, move_group, start_x, target_y, target_z, SPEED_RETREAT, use_moveit):
        return False
    
    print(f"\n{'='*50}\n✓ 按压完成\n{'='*50}")
    return True


def main():
    print("="*50)
    print("Piper 前向按压程序")
    print("="*50)
    
    # 显示默认参数
    print(f"\n📍 默认参数:")
    print(f"  目标: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})m")
    print(f"  前方距离: {PRESS_DISTANCE_BEFORE*1000:.0f}mm, 按压深度: {PRESS_DEPTH*1000:.0f}mm")
    print(f"  按压时长: {PRESS_DURATION:.0f}秒, 夹爪: {GRIPPER_CLOSE_VALUE/1000:.0f}mm")
    print(f"  速度: 回零={SPEED_ZERO}, 移动={SPEED_TO_START}, 按压={SPEED_PRESS}")
    
    # 询问是否使用自定义参数
    use_custom = input("\n使用自定义参数? (y/n, 默认n): ").strip().lower()
    target_x, target_y, target_z = TARGET_X, TARGET_Y, TARGET_Z
    distance_before, press_depth, press_duration = PRESS_DISTANCE_BEFORE, PRESS_DEPTH, PRESS_DURATION
    gripper_value = GRIPPER_CLOSE_VALUE
    
    if use_custom == 'y':
        try:
            val = input(f"  x (默认{TARGET_X}): ").strip()
            if val: target_x = float(val)
            val = input(f"  y (默认{TARGET_Y}): ").strip()
            if val: target_y = float(val)
            val = input(f"  z (默认{TARGET_Z}): ").strip()
            if val: target_z = float(val)
            val = input(f"  前方距离/m (默认{PRESS_DISTANCE_BEFORE}): ").strip()
            if val: distance_before = float(val)
            val = input(f"  按压深度/m (默认{PRESS_DEPTH}): ").strip()
            if val: press_depth = float(val)
            val = input(f"  按压时长/s (默认{PRESS_DURATION}): ").strip()
            if val: press_duration = float(val)
            val = input(f"  夹爪值 (默认{GRIPPER_CLOSE_VALUE}): ").strip()
            if val: gripper_value = max(0, min(70000, int(val)))
        except Exception as e:
            print(f"⚠️ 输入无效，使用默认值: {e}")
    
    print(f"\n最终参数: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})m")
    print(f"  前方{distance_before*1000:.0f}mm → 表面 → 压入{press_depth*1000:.0f}mm (持续{press_duration:.0f}s) → 后退")
    
    # 初始化机械臂
    print("\n初始化机械臂...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper)
    piper_arm = PiperArm()
    
    # 初始化MoveIt
    move_group = None
    if MOVEIT_AVAILABLE and USE_MOVEIT:
        try:
            print("初始化MoveIt...")
            import os
            # 获取项目根目录（move_a_to_b.py 所在目录）
            project_root = os.path.dirname(os.path.abspath(__file__))
            piper_ros_path = os.path.join(project_root, "piper_ros")
            if os.path.exists(piper_ros_path):
                current_path = os.environ.get('ROS_PACKAGE_PATH', '')
                src_path = os.path.join(piper_ros_path, 'src')
                if src_path not in current_path:
                    os.environ['ROS_PACKAGE_PATH'] = f"{src_path}:{current_path}"
            
            try:
                rospy.init_node('piper_press_moveit', anonymous=True, disable_signals=True)
            except rospy.exceptions.ROSException as e:
                if "node with name" not in str(e):
                    raise
            
            moveit_commander.roscpp_initialize([])
            robot = moveit_commander.RobotCommander()
            group_names = robot.get_group_names()
            
            # 初始化轨迹可视化发布器
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
            print(f"  ✓ RViz轨迹可视化已启用")
            print(f"  ✓ 末端路径发布器已启用 (频率: {TRAJECTORY_PUBLISH_RATE}Hz)")
            
            planning_group = None
            for name in ['arm', 'piper_arm', 'manipulator']:
                if name in group_names:
                    planning_group = name
                    break
            
            if planning_group:
                move_group = moveit_commander.MoveGroupCommander(planning_group)
                move_group.set_planning_time(5.0)
                move_group.stop()
                move_group.clear_pose_targets()
                print(f"  ✓ MoveIt就绪 (组: {planning_group})")
            else:
                print(f"  ⚠️ 未找到规划组，使用SDK模式")
        except Exception as e:
            print(f"  ⚠️ MoveIt初始化失败: {e}")
            move_group = None
    
    # 回零位
    print("\n回零位...")
    piper.MotionCtrl_2(0x01, 0x01, SPEED_ZERO, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    piper.GripperCtrl(70000, 1000, 0x01, 0)
    time.sleep(2)
    
    # 执行按压
    if not press_action(piper, piper_arm, target_x, target_y, target_z, 
                        distance_before, press_depth, press_duration, gripper_value, move_group=move_group):
        print("❌ 按压失败")
        return
    
    # 询问是否回零位
    if input("\n回到零位? (y/n, 默认y): ").strip().lower() != 'n':
        print("回零位...")
        piper.MotionCtrl_2(0x01, 0x01, SPEED_ZERO, 0x00)
        piper.JointCtrl(0, 0, 0, 0, 0, 0)
        piper.GripperCtrl(0, 1000, 0x01, 0)
        time.sleep(2)
        print("✓ 已回零位")
    
    # 清理MoveIt
    if MOVEIT_AVAILABLE and move_group:
        try:
            move_group.stop()
            move_group.clear_pose_targets()
            moveit_commander.roscpp_shutdown()
            print("\n✓ MoveIt已关闭")
        except:
            pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
        if MOVEIT_AVAILABLE:
            try:
                moveit_commander.roscpp_shutdown()
                time.sleep(0.5)
            except:
                pass
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        if MOVEIT_AVAILABLE:
            try:
                moveit_commander.roscpp_shutdown()
                time.sleep(0.5)
            except:
                pass
