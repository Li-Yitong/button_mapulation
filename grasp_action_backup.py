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

PI = math.pi
factor = 1000 * 180 / PI
receive_object_center = False
object_center = []
simulation = True

# 用户可自定义参数
GRIPPER_CLOSE_VALUE = 40000  # 夹爪闭合值(单位:0.001mm) 默认40mm
ROTATION_ANGLE = 90  # 旋转角度(度) 默认90度
ROTATION_DIRECTION = 1  # 旋转方向: 1=右旋(顺时针), -1=左旋(逆时针)


def control_arm(joints, speed=2):

    # joints [rad]

    position = joints

    joint_0 = int(position[0] * factor)
    joint_1 = int(position[1] * factor)
    joint_2 = int(position[2] * factor)
    joint_3 = int(position[3] * factor)
    joint_4 = int(position[4] * factor)
    joint_5 = int(position[5] * factor)

    if (joint_4 < -70000) :
        joint_4 = -70000

    # piper.MotionCtrl_1()
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    if len(joints) > 6:
        joint_6 = round(position[6] * 1000 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

    print(piper.GetArmStatus())
    print(position)

def object_point_callback(msg):
    # print("Receive visual detection result", msg.point.x, msg.point.y, msg.point.z)
    if(np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z)):
        return
    global receive_object_center, object_center
    receive_object_center = True
    object_center = [msg.point.x, msg.point.y, msg.point.z]


def move_and_grasp(object_center, joints, piper_arm, gripper_close_value=None, rotation_angle=None, rotation_direction=None):
    """移动并抓取物体
    Args:
        object_center: 目标物体中心坐标
        joints: 当前关节角度
        piper_arm: 机械臂对象
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
    
    print("prepare to grasp point under camera frame", object_center[0], object_center[1], object_center[2])
    print(f"抓取前: 夹爪完全打开 (70mm)")
    print(f"抓取后: 夹爪闭合到 {gripper_close_value/1000:.1f}mm")
    print(f"旋转设置: {'右旋' if rotation_direction == 1 else '左旋'} {rotation_angle}度")
    
    # 步骤1: 先确保夹爪完全打开 (准备抓取)
    print("\n步骤1: 夹爪完全打开...")
    piper.GripperCtrl(70000, 1000, 0x01, 0)  # 70000 = 70mm 完全打开
    time.sleep(1)

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
    print("Planed ik[degree]:", joints_array / PI * 180)

    # time_now = rospy.Time.now()
    # publish_tf(piper_arm, joints, time_now)

    # 步骤2: 移动到目标位置，夹爪保持完全打开状态 (70mm)
    joints.append(0.07)  # 70mm = 0.07m 完全打开
    print("\n步骤2: 移动到目标位置 (夹爪保持打开 70mm)...")
    control_arm(joints, 20)
    time.sleep(10)
    
    # 步骤3: 闭合夹爪抓取物体 (闭合到用户自定义值)
    print(f"\n步骤3: 闭合夹爪抓取 (从70mm闭合到{gripper_close_value/1000:.1f}mm)...")
    joints[6] = gripper_close_value / 1000000  # 转换为米
    control_arm(joints, 20)
    time.sleep(2)
    print(f"✓ 夹爪已闭合到 {gripper_close_value/1000:.1f}mm (物体已抓取)")
    
    # 步骤4: 旋转夹爪 (带物体旋转)
    actual_rotation = rotation_angle * rotation_direction
    print(f"\n步骤4: 旋转夹爪 {'右旋' if rotation_direction == 1 else '左旋'} {rotation_angle}度 (保持夹爪闭合)...")
    joints[5] += actual_rotation * PI / 180  # 在当前角度基础上旋转
    control_arm(joints, 20)
    time.sleep(2)
    print(f"✓ 旋转完成")
    
    # 步骤5: 返回安全位置 (保持夹爪闭合和旋转状态)
    print(f"\n步骤5: 返回安全位置 (保持夹爪闭合{gripper_close_value/1000:.1f}mm和旋转状态)...")
    joints_safe = [0, 0, -0.4, 0, 0, joints[5], joints[6]]  # 保持旋转角度和夹爪状态
    control_arm(joints_safe, 20)
    time.sleep(2)
    print("✓ 抓取任务完成！")

    return True



if __name__ == "__main__":
    # 用户可在此处自定义参数
    print("="*60)
    print("Piper 视觉抓取程序")
    print("="*60)
    print("\n⚠️  用户自定义参数:")
    print(f"  夹爪闭合值: {GRIPPER_CLOSE_VALUE} (0.001mm) = {GRIPPER_CLOSE_VALUE/1000:.1f}mm")
    print(f"  旋转角度: {ROTATION_ANGLE}度")
    print(f"  旋转方向: {'右旋(顺时针)' if ROTATION_DIRECTION == 1 else '左旋(逆时针)'}")
    print("\n💡 修改方法: 编辑文件顶部的全局变量")
    print("  GRIPPER_CLOSE_VALUE = 40000  # 40mm")
    print("  ROTATION_ANGLE = 90  # 90度")
    print("  ROTATION_DIRECTION = 1  # 1=右旋, -1=左旋")
    print("="*60)
    
    # 允许用户临时修改参数
    use_custom = input("\n是否使用自定义参数? (y/n, 默认n): ").strip().lower()
    
    custom_gripper = GRIPPER_CLOSE_VALUE
    custom_angle = ROTATION_ANGLE
    custom_direction = ROTATION_DIRECTION
    
    if use_custom == 'y':
        try:
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
    
    print("\n最终使用参数:")
    print(f"  夹爪闭合值: {custom_gripper} = {custom_gripper/1000:.1f}mm")
    print(f"  旋转: {'右旋' if custom_direction == 1 else '左旋'} {custom_angle}度")
    print("="*60)
    
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    piper.GripperCtrl(70000, 1000, 0x01, 0)  # 初始化: 完全打开

    # 设置初始位置
    joints = [0, 0, 0, 0, 0, 0, 0]
    control_arm(joints, 100)
    time.sleep(2)


    # 初始化节点
    rospy.init_node('vison_grasp_node', anonymous=True)

    piper_arm = PiperArm()
    sub = rospy.Subscriber('/object_point',
                           PointStamped,
                           object_point_callback,
                           queue_size=10,
                           tcp_nodelay=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # time_now = rospy.Time.now()
        # publish_tf(piper_arm, joints, time_now)
        if (receive_object_center):
            msg = piper.GetArmJointMsgs()

            theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
            theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
            theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
            theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
            theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
            theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

            joints = [theta1, theta2, theta3, theta4, theta5, theta6]

            if move_and_grasp(object_center, joints, piper_arm, custom_gripper, custom_angle, custom_direction):
                break
            receive_object_center = False

        rate.sleep()







