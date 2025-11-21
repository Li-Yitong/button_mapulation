#!/usr/bin/env python3
"""
机械臂前向按压操作程序
夹爪闭合状态下，沿夹爪z轴方向对目标物体实现前向按压
"""
from piper_sdk import *
import time
import math
import numpy as np
from piper_arm import PiperArm
from utils.utils_piper import enable_fun

PI = math.pi
factor = 1000 * 180 / PI

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
SPEED_TO_START = 100           # 移动到目标前方的速度
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
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)  # 使用传入的速度参数
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    
    if len(joints) > 6:
        joint_6 = round(position[6] * 1000 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    
    print(f"关节角度 (度): [{position[0]*180/PI:.1f}, {position[1]*180/PI:.1f}, {position[2]*180/PI:.1f}, {position[3]*180/PI:.1f}, {position[4]*180/PI:.1f}, {position[5]*180/PI:.1f}]")


def xyz_to_joints(piper_arm, x, y, z, orientation='default'):
    """
    将基座坐标系的xyz坐标转换为关节角度
    
    Args:
        piper_arm: PiperArm对象
        x, y, z: 目标位置坐标 (米)
        orientation: 末端姿态 'default'=垂直向下
    
    Returns:
        joints: 关节角度列表 [j0,j1,j2,j3,j4,j5], 单位:弧度
        None: 逆运动学求解失败
    """
    # 构造目标变换矩阵 (末端姿态: 垂直向下)
    if orientation == 'default':
        # Z轴向下, X轴向前, Y轴向左
        targetT = np.array([
            [0,  0,  1, x],
            [0,  1,  0, y],
            [-1, 0,  0, z],
            [0,  0,  0, 1]
        ], dtype=float)
    else:
        targetT = np.array([
            [0,  0,  1, x],
            [0,  1,  0, y],
            [-1, 0,  0, z],
            [0,  0,  0, 1]
        ], dtype=float)
    
    # 逆运动学求解
    joints = piper_arm.inverse_kinematics(targetT)
    
    if not joints:
        return None
    
    return joints


def move_to_position(piper, piper_arm, x, y, z, speed=20, gripper_pos=None):
    """
    移动到指定的xyz位置
    
    Args:
        x, y, z: 目标位置 (基座坐标系, 单位:米)
        speed: 运动速度
        gripper_pos: 夹爪位置 (0.001mm单位), None=不改变
    
    Returns:
        True: 成功
        False: 失败
    """
    print(f"\n目标位置: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m")
    
    # 逆运动学求解
    joints = xyz_to_joints(piper_arm, x, y, z)
    
    if joints is None:
        print("❌ 逆运动学求解失败！位置可能超出工作空间")
        return False
    
    joints_deg = np.array(joints) * 180 / PI
    print(f"关节角度 (度): [{joints_deg[0]:.1f}, {joints_deg[1]:.1f}, {joints_deg[2]:.1f}, {joints_deg[3]:.1f}, {joints_deg[4]:.1f}, {joints_deg[5]:.1f}]")
    
    # 添加夹爪控制
    if gripper_pos is not None:
        joints.append(gripper_pos / 1000000)  # 转换为米
    
    # 执行运动
    control_arm(piper, joints, speed)
    
    return True


def press_action(piper, piper_arm, target_x, target_y, target_z, height_above, press_depth, press_duration, gripper_value):
    """执行前向按压动作（沿夹爪z轴方向）
    Args:
        target_x, target_y, target_z: 目标物体表面坐标 (米)
        height_above: 目标前方停留距离 (米) - 沿夹爪z轴后退
        press_depth: 按压深度 (米) - 沿夹爪z轴前进
        press_duration: 按压持续时间 (秒)
        gripper_value: 夹爪闭合值 (0.001mm单位)
    
    说明：
        夹爪z轴指向前方（夹爪朝向），按压是沿这个方向前进
        在默认垂直向下姿态时，夹爪z轴 = 基座x轴正方向
    """
    print("\n" + "="*70)
    print("开始执行前向按压操作 (沿夹爪z轴)")
    print("="*70)
    
    # 步骤1: 夹爪闭合
    print(f"\n步骤1: 夹爪闭合到 {gripper_value/1000:.1f}mm")
    piper.GripperCtrl(gripper_value, 1000, 0x01, 0)
    time.sleep(1)
    
    # 步骤2: 移动到目标前方（沿夹爪z轴后退一段距离）
    # 对于默认垂直向下姿态，夹爪z轴对应基座x轴正方向
    # 所以后退 = x方向减小
    start_x = target_x - height_above
    print(f"\n步骤2: 移动到目标前方 ({start_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    print(f"  (沿夹爪z轴后退 {height_above*1000:.1f}mm, 速度={SPEED_TO_START})")
    if not move_to_position(piper, piper_arm, start_x, target_y, target_z, speed=SPEED_TO_START, gripper_pos=gripper_value):
        print("❌ 移动到目标前方失败")
        return False
    print("✓ 已到达目标前方")
    time.sleep(2)
    
    # 步骤3: 前进到物体表面（沿夹爪z轴前进）
    print(f"\n步骤3: 沿夹爪z轴前进到物体表面 ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    print(f"  (前进 {height_above*1000:.1f}mm, 速度={SPEED_APPROACH})")
    if not move_to_position(piper, piper_arm, target_x, target_y, target_z, speed=SPEED_APPROACH, gripper_pos=gripper_value):
        print("❌ 前进到物体表面失败")
        return False
    print("✓ 已接触物体表面")
    time.sleep(1)
    
    # 步骤4: 按压 - 继续沿夹爪z轴前进
    press_x = target_x + press_depth
    print(f"\n步骤4: 按压前进 {press_depth*1000:.1f}mm 到 x={press_x:.3f}m")
    print(f"  (沿夹爪z轴方向压入, 速度={SPEED_PRESS})")
    if not move_to_position(piper, piper_arm, press_x, target_y, target_z, speed=SPEED_PRESS, gripper_pos=gripper_value):
        print("❌ 按压失败")
        return False
    print(f"✓ 按压中... (持续 {press_duration}秒)")
    time.sleep(press_duration)
    
    # 步骤5: 后退到前方位置
    print(f"\n步骤5: 沿夹爪z轴后退到前方 x={start_x:.3f}m")
    print(f"  (后退 {(press_depth + height_above)*1000:.1f}mm, 速度={SPEED_RETREAT})")
    if not move_to_position(piper, piper_arm, start_x, target_y, target_z, speed=SPEED_RETREAT, gripper_pos=gripper_value):
        print("❌ 后退失败")
        return False
    print("✓ 已后退")
    time.sleep(1)
    
    print("\n" + "="*70)
    print("✓ 前向按压操作完成！")
    print("="*70)
    return True


def main():
    print("="*70)
    print("Piper 机械臂前向按压操作程序 (沿夹爪z轴)")
    print("="*70)
    
    # 显示默认参数
    print("\n📍 默认参数:")
    print(f"  目标位置: x={TARGET_X:.3f}m, y={TARGET_Y:.3f}m, z={TARGET_Z:.3f}m")
    print(f"  前方距离: {PRESS_DISTANCE_BEFORE*1000:.1f}mm (沿夹爪z轴后退)")
    print(f"  按压深度: {PRESS_DEPTH*1000:.1f}mm (沿夹爪z轴前进)")
    print(f"  按压时长: {PRESS_DURATION:.1f}秒")
    print(f"  夹爪状态: {GRIPPER_CLOSE_VALUE} (0.001mm) = {GRIPPER_CLOSE_VALUE/1000:.1f}mm (闭合)")
    print(f"\n⚡ 运动速度 (1-100):")
    print(f"  回零位: {SPEED_ZERO}")
    print(f"  移动到前方: {SPEED_TO_START}")
    print(f"  接近表面: {SPEED_APPROACH}")
    print(f"  按压: {SPEED_PRESS}")
    print(f"  后退: {SPEED_RETREAT}")
    print("\n💡 说明:")
    print("  - 夹爪z轴 = 夹爪朝向方向")
    print("  - 默认垂直向下姿态时，夹爪z轴 = 基座x轴正方向")
    print("  - 前向按压 = 沿夹爪z轴前进")
    print("\n💡 修改方法: 编辑文件顶部的全局变量")
    print("="*70)
    
    # 询问是否使用自定义参数
    use_custom = input("\n是否使用自定义参数? (y/n, 默认n): ").strip().lower()
    
    # 初始化变量
    target_x = TARGET_X
    target_y = TARGET_Y
    target_z = TARGET_Z
    distance_before = PRESS_DISTANCE_BEFORE
    press_depth = PRESS_DEPTH
    press_duration = PRESS_DURATION
    gripper_value = GRIPPER_CLOSE_VALUE
    
    if use_custom == 'y':
        try:
            print("\n输入目标物体位置 (单位:米):")
            val = input(f"  x-表面位置 (默认{TARGET_X}): ").strip()
            if val: target_x = float(val)
            val = input(f"  y (默认{TARGET_Y}): ").strip()
            if val: target_y = float(val)
            val = input(f"  z (默认{TARGET_Z}): ").strip()
            if val: target_z = float(val)
            
            print("\n输入按压参数:")
            val = input(f"  前方停留距离/m (默认{PRESS_DISTANCE_BEFORE}): ").strip()
            if val: distance_before = float(val)
            val = input(f"  按压深度/m (默认{PRESS_DEPTH}): ").strip()
            if val: press_depth = float(val)
            val = input(f"  按压持续时间/秒 (默认{PRESS_DURATION}): ").strip()
            if val: press_duration = float(val)
            
            val = input(f"\n夹爪闭合值 (0-70000, 0.001mm单位, 默认{GRIPPER_CLOSE_VALUE}): ").strip()
            if val:
                gripper_value = int(val)
                if not (0 <= gripper_value <= 70000):
                    print(f"⚠️  超出范围，使用默认值 {GRIPPER_CLOSE_VALUE}")
                    gripper_value = GRIPPER_CLOSE_VALUE
        except Exception as e:
            print(f"⚠️  输入无效，使用默认值: {e}")
    
    # 显示最终参数
    print("\n" + "="*70)
    print("最终参数:")
    print(f"  目标位置: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) m")
    print(f"  前方距离: {distance_before*1000:.1f}mm")
    print(f"  按压深度: {press_depth*1000:.1f}mm")
    print(f"  按压时长: {press_duration:.1f}秒")
    print(f"  夹爪: {gripper_value} = {gripper_value/1000:.1f}mm")
    print("\n前向按压流程 (沿夹爪z轴):")
    print(f"  1. 夹爪闭合 → {gripper_value/1000:.1f}mm")
    print(f"  2. 移动到前方 → x={target_x - distance_before:.3f}m (后退{distance_before*1000:.1f}mm)")
    print(f"  3. 前进到表面 → x={target_x:.3f}m (前进{distance_before*1000:.1f}mm)")
    print(f"  4. 按压前进 → x={target_x + press_depth:.3f}m (压入{press_depth*1000:.1f}mm, 持续{press_duration:.1f}秒)")
    print(f"  5. 后退到前方 → x={target_x - distance_before:.3f}m (后退{(distance_before + press_depth)*1000:.1f}mm)")
    print("="*70)
    
    # 初始化机械臂
    print("\n正在初始化机械臂...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper)
    
    # 初始化PiperArm用于逆运动学
    piper_arm = PiperArm()
    
    # 回零位
    print("\n先回到零位...")
    piper.MotionCtrl_2(0x01, 0x01, SPEED_ZERO, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    piper.GripperCtrl(70000, 1000, 0x01, 0)  # 先打开夹爪
    time.sleep(2)
    
    # 执行前向按压动作
    if not press_action(piper, piper_arm, target_x, target_y, target_z, 
                        distance_before, press_depth, press_duration, gripper_value):
        print("❌ 前向按压操作失败")
        return
    
    # 询问是否回零位
    back_zero = input("\n是否回到零位? (y/n, 默认y): ").strip().lower()
    if back_zero != 'n':
        print("\n回到零位...")
        joints_zero = [0, 0, 0, 0, 0, 0]
        piper.MotionCtrl_2(0x01, 0x01, SPEED_ZERO, 0x00)
        piper.JointCtrl(0, 0, 0, 0, 0, 0)
        piper.GripperCtrl(0, 1000, 0x01, 0)  # 夹爪闭合
        time.sleep(2)
        print("✓ 已回到零位")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
