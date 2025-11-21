#!/usr/bin/env python3
from piper_sdk import *
import time
import math

PI = math.pi
factor = 1000 * 180 / PI

# 全局变量：记录当前夹爪状态
current_gripper_position = None  # None表示未初始化

def control_arm(piper, joints, speed=20, control_gripper=True):
    """控制机械臂运动
    Args:
        control_gripper: True=控制夹爪, False=保持夹爪当前状态
    """
    global current_gripper_position
    
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
    
    # 只有明确要求控制夹爪时才发送夹爪命令
    if control_gripper and len(joints) > 6:
        joint_6 = round(position[6] * 1000 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        current_gripper_position = position[6]  # 记录当前夹爪位置

def set_gripper(piper, position):
    """单独设置夹爪位置
    Args:
        position: 夹爪位置 (单位:米) 0.00-0.07
    """
    global current_gripper_position
    
    if 0 <= position <= 0.07:
        joint_6 = round(position * 1000 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        current_gripper_position = position
        print(f"夹爪设置到 {position*1000:.1f}mm")
    else:
        print(f"⚠️  夹爪位置 {position} 超出范围 (0.00-0.07)")

def get_gripper_position():
    """获取当前夹爪位置"""
    global current_gripper_position
    return current_gripper_position

def go_to_zero(piper, speed=50, close_gripper=True):
    """回零位
    Args:
        close_gripper: True=夹爪闭合, False=夹爪保持当前状态
    """
    print("回到零位...")
    if close_gripper:
        joints = [0, 0, 0, 0, 0, 0, 0.00]  # 夹爪闭合 (0m = 0mm)
        print("  夹爪状态: 闭合")
        control_arm(piper, joints, speed, control_gripper=True)
    else:
        joints = [0, 0, 0, 0, 0, 0]  # 不控制夹爪
        print("  夹爪状态: 保持当前")
        control_arm(piper, joints, speed, control_gripper=False)
    time.sleep(3)

def go_to_ready_pose(piper, speed=30, gripper_position=None):
    """移动到准备姿态：机械臂向前伸展
    Args:
        gripper_position: 夹爪位置 (单位:米)
                         None = 保持当前状态
                         0.00 = 闭合
                         0.07 = 打开70mm
    """
    print("移动到准备姿态（向前伸展）...")
    # 机械臂向前伸展的姿态
    joints = [0, -0.5, -0.8, 0, -0.5, 0]
    
    if gripper_position is not None:
        joints.append(gripper_position)
        print(f"  夹爪位置: {gripper_position*1000:.1f}mm")
        control_arm(piper, joints, speed, control_gripper=True)
    else:
        print(f"  夹爪位置: 保持当前")
        control_arm(piper, joints, speed, control_gripper=False)
    
    time.sleep(3)
    print("准备姿态就绪")

def action1_rotate_gripper(piper, rotation_angle, direction):
    """动作1: 控制夹爪旋转（夹爪保持当前状态）"""
    direction_text = "右旋" if direction == 1 else "左旋"
    actual_angle = rotation_angle if direction == 1 else -rotation_angle
    print(f"\n执行动作1: 夹爪{direction_text} {rotation_angle * 180 / PI:.1f} 度")
    
    # 移动到准备姿态（夹爪保持当前状态）
    go_to_ready_pose(piper, gripper_position=None)
    
    # 旋转夹爪（保持夹爪开合状态）
    joints = [0, -0.5, -0.8, 0, -0.5, 0]
    joints[5] = actual_angle
    control_arm(piper, joints, 20, control_gripper=False)  # 不控制夹爪
    time.sleep(2)
    
    # 旋转回来（保持夹爪开合状态）
    joints[5] = 0
    control_arm(piper, joints, 20, control_gripper=False)  # 不控制夹爪
    time.sleep(2)
    
    print("动作1完成")

def action2_close_and_forward(piper, forward_distance):
    """动作2: 夹爪闭合过程中向前移动"""
    print(f"\n执行动作2: 夹爪闭合并前进 {forward_distance:.3f} 米")
    
    # 先移动到准备姿态（夹爪打开）
    print("步骤1: 移动到准备姿态，夹爪打开...")
    set_gripper(piper, 0.07)  # 先打开夹爪
    time.sleep(0.5)
    go_to_ready_pose(piper, gripper_position=None)  # 移动到准备姿态，保持夹爪打开
    
    # 从准备姿态开始，逐步闭合夹爪并前进
    base_joint2 = -0.8
    steps = 10
    print("步骤2: 开始渐进闭合并前进...")
    for i in range(steps + 1):
        # 夹爪从 0.07m(70mm) 逐渐闭合到 0.02m(20mm)
        gripper_pos = 0.07 - (0.05 * (i / steps))  # 70mm → 20mm
        joints = [0, -0.5, base_joint2 - (forward_distance * 2) * (i / steps), 0, -0.5, 0, gripper_pos]
        control_arm(piper, joints, 20, control_gripper=True)  # 同时控制机械臂和夹爪
        time.sleep(0.3)
    
    print(f"步骤3: 夹爪已闭合到 {gripper_pos*1000:.1f}mm")
    time.sleep(1)
    
    # 打开夹爪
    print("步骤4: 打开夹爪...")
    set_gripper(piper, 0.07)  # 单独控制夹爪打开
    time.sleep(1)
    
    print("动作2完成")

def action3_close_move_back_forth(piper, move_distance):
    """动作3: 夹爪闭合后前后移动（夹爪保持闭合状态）"""
    print(f"\n执行动作3: 夹爪闭合后前后移动 {move_distance:.3f} 米")
    
    # 先移动到准备姿态（夹爪打开）
    print("步骤1: 移动到准备姿态，夹爪打开...")
    set_gripper(piper, 0.07)  # 先打开夹爪
    time.sleep(0.5)
    go_to_ready_pose(piper, gripper_position=None)  # 移动到准备姿态
    
    # 从准备姿态开始
    joints = [0, -0.5, -0.8, 0, -0.5, 0]
    
    # 向前移动（夹爪保持打开）
    print("步骤2: 向前移动（夹爪保持打开）...")
    joints[2] -= move_distance * 2
    control_arm(piper, joints, 20, control_gripper=False)  # 不改变夹爪
    time.sleep(2)
    
    # 闭合夹爪
    print("步骤3: 闭合夹爪到 20mm...")
    set_gripper(piper, 0.02)  # 单独控制夹爪闭合
    time.sleep(1)
    
    # 向后移动（夹爪保持闭合！）
    print("步骤4: 向后移动（夹爪保持闭合）...")
    joints[2] += move_distance * 2
    control_arm(piper, joints, 20, control_gripper=False)  # 不改变夹爪状态
    time.sleep(2)
    
    # 再向前移动（夹爪保持闭合！）
    print("步骤5: 再次向前移动（夹爪保持闭合）...")
    joints[2] -= move_distance * 2
    control_arm(piper, joints, 20, control_gripper=False)  # 不改变夹爪状态
    time.sleep(2)
    
    # 打开夹爪
    print("步骤6: 打开夹爪...")
    set_gripper(piper, 0.07)  # 单独控制夹爪打开
    time.sleep(1)
    
    print("动作3完成")

def enable_fun(piper):
    """使能函数"""
    enable_flag = False
    timeout = 5
    start_time = time.time()
    elapsed_time_flag = False
    while not enable_flag:
        elapsed_time = time.time() - start_time
        enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)
        if elapsed_time > timeout:
            print("使能超时，退出程序")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
    if elapsed_time_flag:
        print("程序自动使能超时，退出程序")
        exit(0)

def main():
    # 初始化
    print("="*60)
    print("Piper 夹爪控制演示程序 (改进版)")
    print("="*60)
    print("\n⚠️  夹爪控制说明：")
    print("  - 机械臂移动/旋转时，夹爪保持当前状态")
    print("  - 只有明确发送夹爪命令时，夹爪才会改变")
    print("  - joints[6] 单位是 米(m)")
    print("  - 0.00m = 完全闭合 (0mm)")
    print("  - 0.07m = 完全打开 (70mm)")
    print("="*60)
    
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper)
    
    # 回到零位（夹爪闭合）
    go_to_zero(piper, close_gripper=True)
    
    while True:
        print("\n" + "="*60)
        current_pos = get_gripper_position()
        if current_pos is not None:
            print(f"当前夹爪位置: {current_pos*1000:.1f}mm")
        print("请选择动作:")
        print("1. 夹爪旋转 (夹爪保持当前状态)")
        print("2. 夹爪闭合并前进")
        print("3. 夹爪闭合后前后移动 (展示状态保持)")
        print("4. 手动控制夹爪位置")
        print("5. 测试：移动机械臂但保持夹爪状态")
        print("0. 退出程序")
        print("="*60)
        
        choice = input("\n输入选项 (0-5): ").strip()
        
        if choice == '0':
            print("\n退出程序...")
            go_to_zero(piper, close_gripper=True)
            break
            
        elif choice == '1':
            print("\n请选择旋转方向:")
            print("1. 右旋")
            print("2. 左旋")
            dir_input = input("输入方向 (1/2，默认1): ").strip()
            if dir_input == '2':
                direction = -1
                dir_text = "左旋"
            else:
                direction = 1
                dir_text = "右旋"
            
            angle_input = input(f"输入{dir_text}角度(度，默认90): ").strip()
            if angle_input == '':
                angle = 90
            else:
                try:
                    angle = float(angle_input)
                except:
                    print("无效输入，使用默认值90度")
                    angle = 90
            
            action1_rotate_gripper(piper, angle * PI / 180, direction)
            
            # 询问是否回零位
            back = input("\n回到零位？(y/n，默认n): ").strip().lower()
            if back == 'y':
                close_g = input("回零时闭合夹爪？(y/n，默认y): ").strip().lower()
                go_to_zero(piper, close_gripper=(close_g != 'n'))
            
        elif choice == '2':
            dist_input = input("输入前进距离(米，默认0.05): ").strip()
            if dist_input == '':
                distance = 0.05
            else:
                try:
                    distance = float(dist_input)
                except:
                    print("无效输入，使用默认值0.05米")
                    distance = 0.05
            
            action2_close_and_forward(piper, distance)
            
            # 询问是否回零位
            back = input("\n回到零位？(y/n，默认n): ").strip().lower()
            if back == 'y':
                go_to_zero(piper, close_gripper=True)
            
        elif choice == '3':
            dist_input = input("输入移动距离(米，默认0.03): ").strip()
            if dist_input == '':
                distance = 0.03
            else:
                try:
                    distance = float(dist_input)
                except:
                    print("无效输入，使用默认值0.03米")
                    distance = 0.03
            
            action3_close_move_back_forth(piper, distance)
            
            # 询问是否回零位
            back = input("\n回到零位？(y/n，默认n): ").strip().lower()
            if back == 'y':
                go_to_zero(piper, close_gripper=True)
        
        elif choice == '4':
            print("\n手动控制夹爪:")
            print("  0.00m = 完全闭合 (0mm)")
            print("  0.02m = 轻微张开 (20mm)")
            print("  0.07m = 完全打开 (70mm)")
            pos_input = input("输入夹爪位置(米，0.00-0.07): ").strip()
            try:
                pos = float(pos_input)
                if 0 <= pos <= 0.07:
                    set_gripper(piper, pos)
                    time.sleep(2)
                    print("✓ 完成")
                else:
                    print("✗ 位置必须在 0.00-0.07 之间")
            except:
                print("✗ 无效输入")
        
        elif choice == '5':
            print("\n测试：移动机械臂但保持夹爪状态")
            print("1. 先设置夹爪到某个位置")
            print("2. 然后移动机械臂")
            print("3. 观察夹爪是否保持不变")
            
            # 设置夹爪
            pos_input = input("\n设置夹爪位置(米，0.00-0.07，默认0.035): ").strip()
            if pos_input == '':
                pos = 0.035
            else:
                try:
                    pos = float(pos_input)
                except:
                    pos = 0.035
            
            print(f"设置夹爪到 {pos*1000:.1f}mm...")
            set_gripper(piper, pos)
            time.sleep(2)
            
            # 移动机械臂（不改变夹爪）
            print("\n移动到准备姿态（夹爪保持当前状态）...")
            go_to_ready_pose(piper, gripper_position=None)
            
            print("\n回零位（夹爪保持当前状态）...")
            go_to_zero(piper, close_gripper=False)
            
            current_pos = get_gripper_position()
            if current_pos is not None:
                print(f"\n✓ 测试完成！夹爪应该保持在 {current_pos*1000:.1f}mm")
            else:
                print(f"\n✓ 测试完成！夹爪应该保持在 {pos*1000:.1f}mm")
            
        else:
            print("无效选项，请重新选择")
    
    print("\n程序结束")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
