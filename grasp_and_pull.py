#!/usr/bin/env python3
"""
机械臂抓取并拔出操作程序
1. 张开夹爪
2. 移动到目标位置
3. 闭合夹爪到指定程度
4. 沿夹爪z轴方向拔出指定距离
5. 回零位
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
TARGET_X = 0.25  # 目标物体x坐标
TARGET_Y = 0.0   # 目标物体y坐标
TARGET_Z = 0.20  # 目标物体z坐标

# 夹爪参数
GRIPPER_OPEN_VALUE = 70000   # 张开值 (0.001mm单位, 70000=70mm完全打开)
GRIPPER_CLOSE_VALUE = 40000  # 闭合值 (0.001mm单位, 40000=40mm, 0=完全闭合)

# 拔出参数（沿夹爪z轴方向）
PULL_OUT_DISTANCE = 0.10  # 拔出距离 (米) 默认10cm，沿夹爪z轴后退
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


def grasp_and_pull_action(piper, piper_arm, target_x, target_y, target_z, 
                          gripper_open, gripper_close, pull_distance):
    """执行抓取并拔出动作
    
    Args:
        target_x, target_y, target_z: 目标位置坐标 (米)
        gripper_open: 夹爪张开值 (0.001mm单位)
        gripper_close: 夹爪闭合值 (0.001mm单位)
        pull_distance: 拔出距离 (米) - 沿夹爪z轴后退
    
    说明：
        夹爪z轴指向前方（夹爪朝向）
        在默认垂直向下姿态时，夹爪z轴 = 基座x轴正方向
        拔出 = 沿夹爪z轴后退 = 基座x轴负方向
    """
    print("\n" + "="*70)
    print("开始执行抓取并拔出操作")
    print("="*70)
    
    # 步骤1: 张开夹爪
    print(f"\n步骤1: 张开夹爪到 {gripper_open/1000:.1f}mm")
    piper.GripperCtrl(gripper_open, 1000, 0x01, 0)
    time.sleep(1.5)
    print("✓ 夹爪已张开")
    
    # 步骤2: 移动到目标位置
    print(f"\n步骤2: 移动到目标位置 ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    if not move_to_position(piper, piper_arm, target_x, target_y, target_z, 
                           speed=20, gripper_pos=gripper_open):
        print("❌ 移动到目标位置失败")
        return False
    print("✓ 已到达目标位置")
    time.sleep(2)
    
    # 步骤3: 闭合夹爪
    print(f"\n步骤3: 闭合夹爪到 {gripper_close/1000:.1f}mm")
    print(f"  (从 {gripper_open/1000:.1f}mm 闭合到 {gripper_close/1000:.1f}mm)")
    piper.GripperCtrl(gripper_close, 1000, 0x01, 0)
    time.sleep(2)
    print("✓ 夹爪已闭合，抓取完成")
    time.sleep(1)
    
    # 步骤4: 沿夹爪z轴拔出（后退）
    # 对于默认垂直向下姿态，夹爪z轴 = 基座x轴正方向
    # 拔出（后退）= x方向减小
    pull_x = target_x - pull_distance
    print(f"\n步骤4: 沿夹爪z轴拔出 {pull_distance*1000:.1f}mm")
    print(f"  从 x={target_x:.3f}m 后退到 x={pull_x:.3f}m")
    if not move_to_position(piper, piper_arm, pull_x, target_y, target_z, 
                           speed=15, gripper_pos=gripper_close):
        print("❌ 拔出失败")
        return False
    print("✓ 拔出完成")
    time.sleep(2)
    
    print("\n" + "="*70)
    print("✓ 抓取并拔出操作完成！")
    print("="*70)
    return True


def main():
    print("="*70)
    print("Piper 机械臂抓取并拔出操作程序")
    print("="*70)
    
    # 显示默认参数
    print("\n📍 默认参数:")
    print(f"  目标位置: x={TARGET_X:.3f}m, y={TARGET_Y:.3f}m, z={TARGET_Z:.3f}m")
    print(f"  夹爪张开: {GRIPPER_OPEN_VALUE} (0.001mm) = {GRIPPER_OPEN_VALUE/1000:.1f}mm")
    print(f"  夹爪闭合: {GRIPPER_CLOSE_VALUE} (0.001mm) = {GRIPPER_CLOSE_VALUE/1000:.1f}mm")
    print(f"  拔出距离: {PULL_OUT_DISTANCE*1000:.1f}mm (沿夹爪z轴后退)")
    print("\n💡 说明:")
    print("  - 夹爪值: 0=完全闭合, 70000=完全打开(70mm)")
    print("  - 夹爪z轴 = 夹爪朝向方向")
    print("  - 默认垂直向下姿态时，夹爪z轴 = 基座x轴正方向")
    print("  - 拔出 = 沿夹爪z轴后退")
    print("\n💡 修改方法: 编辑文件顶部的全局变量")
    print("="*70)
    
    # 询问是否使用自定义参数
    use_custom = input("\n是否使用自定义参数? (y/n, 默认n): ").strip().lower()
    
    # 初始化变量
    target_x = TARGET_X
    target_y = TARGET_Y
    target_z = TARGET_Z
    gripper_open = GRIPPER_OPEN_VALUE
    gripper_close = GRIPPER_CLOSE_VALUE
    pull_distance = PULL_OUT_DISTANCE
    
    if use_custom == 'y':
        try:
            print("\n输入目标位置 (单位:米):")
            val = input(f"  x (默认{TARGET_X}): ").strip()
            if val: target_x = float(val)
            val = input(f"  y (默认{TARGET_Y}): ").strip()
            if val: target_y = float(val)
            val = input(f"  z (默认{TARGET_Z}): ").strip()
            if val: target_z = float(val)
            
            print("\n输入夹爪参数 (单位:0.001mm, 范围0-70000):")
            val = input(f"  张开值 (默认{GRIPPER_OPEN_VALUE}): ").strip()
            if val:
                gripper_open = int(val)
                if not (0 <= gripper_open <= 70000):
                    print(f"⚠️  超出范围，使用默认值 {GRIPPER_OPEN_VALUE}")
                    gripper_open = GRIPPER_OPEN_VALUE
            
            val = input(f"  闭合值 (默认{GRIPPER_CLOSE_VALUE}): ").strip()
            if val:
                gripper_close = int(val)
                if not (0 <= gripper_close <= 70000):
                    print(f"⚠️  超出范围，使用默认值 {GRIPPER_CLOSE_VALUE}")
                    gripper_close = GRIPPER_CLOSE_VALUE
            
            if gripper_close >= gripper_open:
                print("⚠️  警告: 闭合值应小于张开值")
                print(f"     自动调整: 张开={gripper_open}, 闭合={int(gripper_open*0.6)}")
                gripper_close = int(gripper_open * 0.6)
            
            print("\n输入拔出参数:")
            val = input(f"  拔出距离/m (默认{PULL_OUT_DISTANCE}): ").strip()
            if val: pull_distance = float(val)
            
        except Exception as e:
            print(f"⚠️  输入无效，使用默认值: {e}")
    
    # 显示最终参数
    print("\n" + "="*70)
    print("最终参数:")
    print(f"  目标位置: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) m")
    print(f"  夹爪张开: {gripper_open} = {gripper_open/1000:.1f}mm")
    print(f"  夹爪闭合: {gripper_close} = {gripper_close/1000:.1f}mm")
    print(f"  拔出距离: {pull_distance*1000:.1f}mm")
    print("\n操作流程:")
    print(f"  1. 张开夹爪 → {gripper_open/1000:.1f}mm")
    print(f"  2. 移动到目标 → ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    print(f"  3. 闭合夹爪 → {gripper_close/1000:.1f}mm (抓取)")
    print(f"  4. 沿夹爪z轴拔出 → 后退{pull_distance*1000:.1f}mm 到 x={target_x-pull_distance:.3f}m")
    print(f"  5. 回零位 → (0, 0, 0)")
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
    piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(2)
    
    # 执行抓取并拔出动作
    if not grasp_and_pull_action(piper, piper_arm, target_x, target_y, target_z,
                                 gripper_open, gripper_close, pull_distance):
        print("❌ 操作失败")
        return
    
    # 回零位
    print("\n回到零位...")
    piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    piper.GripperCtrl(0, 1000, 0x01, 0)  # 夹爪闭合
    time.sleep(2)
    print("✓ 已回到零位")
    
    print("\n" + "="*70)
    print("✓✓✓ 全部操作完成！✓✓✓")
    print("="*70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()