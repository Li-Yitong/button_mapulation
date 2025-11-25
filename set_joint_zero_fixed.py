#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
设置关节电机零点位置 - 修复版
将手动设置的位置作为零位
"""
import time
import sys
sys.path.insert(0, '/home/robot/button/V4.0/project2')
from piper_sdk import C_PiperInterface_V2

def enable_arm_safe(piper, max_attempts=10):
    """安全使能机械臂"""
    for i in range(max_attempts):
        try:
            piper.EnableArm(7)  # 使能所有关节
            time.sleep(0.1)
            # 检查是否成功
            status = piper.GetArmLowSpdInfoMsgs()
            if status:
                print("✓ 机械臂使能成功")
                return True
        except:
            pass
        time.sleep(0.5)
    print("⚠️ 机械臂使能失败，但继续执行")
    return False

if __name__ == "__main__":
    print("="*70)
    print("设置关节电机零点位置")
    print("Set Joint Motor Zero Position")
    print("="*70)
    print("\n⚠️  警告: 设置零点前会对指定的电机失能")
    print("⚠️  Warning: Motor will be disabled before setting zero position")
    print("⚠️  请确保机械臂处于安全位置，并准备好支撑！")
    print("⚠️  Ensure the arm is in a safe position and ready for support!")
    print("\n输入 'q' 可以随时退出程序")
    print("Enter 'q' to exit at any time\n")
    
    # 连接机械臂
    try:
        piper = C_PiperInterface_V2("can0")
        piper.ConnectPort()
        time.sleep(0.2)
        print("✓ 连接成功\n")
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        sys.exit(1)
    
    # 尝试使能机械臂
    enable_arm_safe(piper)
    time.sleep(0.5)
    
    # 设置控制模式
    try:
        piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        time.sleep(0.1)
    except:
        pass
    
    mode = -1
    while True:
        # 模式选择
        if mode == -1:
            print("\n" + "="*70)
            print("Step 1: 请选择设置模式")
            print("Step 1: Select setting mode")
            print("-"*70)
            print("  0 - 指定单个电机 (Single motor)")
            print("  1 - 顺序设置所有电机 (Sequential all motors)")
            print("  q - 退出 (Quit)")
            print("="*70)
            mode = input("> ").strip()
            
            if mode == '0':
                mode = 0
            elif mode == '1':
                mode = 1
            elif mode == 'q':
                print("\n退出程序")
                break
            else:
                print("✗ 无效输入，请重新选择")
                mode = -1
        
        # 单电机设置
        elif mode == 0:
            print("\n" + "="*70)
            print("Step 2: 输入需要设置零点的电机序号 (1~6)")
            print("        7代表所有电机同时设置")
            print("Step 2: Enter motor number (1~6), 7 for all motors")
            print("="*70)
            
            motor_input = input("> ").strip()
            if motor_input == 'q':
                mode = -1
                continue
            
            try:
                motor_num = int(motor_input)
                if motor_num < 1 or motor_num > 7:
                    print("✗ 输入超出范围 (1-7)")
                    continue
            except:
                print("✗ 请输入整数")
                continue
            
            # 失能电机
            print(f"\n⚠️  正在失能第 {motor_num} 号电机...")
            piper.DisableArm(motor_num)
            time.sleep(0.5)
            
            print(f"✓ 第 {motor_num} 号电机已失能")
            print(f"\n{'='*70}")
            print("👉 请手动移动机械臂到期望的零点位置")
            print("👉 Please manually move the arm to desired zero position")
            print(f"{'='*70}")
            
            # 等待用户确认
            print(f"\nStep 3: 位置调整完成后，按回车设置零点")
            print(f"Step 3: Press Enter to set zero position when ready")
            user_input = input("(按回车继续 / Press Enter) ")
            
            if user_input.strip() == 'q':
                # 重新使能
                piper.EnableArm(motor_num)
                mode = -1
                continue
            
            # 设置零点
            print(f"\n正在设置第 {motor_num} 号电机零点...")
            piper.JointConfig(motor_num, 0xAE)  # 0xAE = 设置零点命令
            time.sleep(0.5)
            
            # 重新使能
            piper.EnableArm(motor_num)
            time.sleep(0.3)
            
            # 重置控制模式
            try:
                piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
                time.sleep(0.1)
            except:
                pass
            
            print(f"✓ 第 {motor_num} 号电机零点设置成功！")
            print(f"\n提示: 现在的位置已经被设置为零点")
            print(f"Tip: Current position is now set as zero\n")
        
        # 顺序设置
        elif mode == 1:
            print("\n" + "="*70)
            print("Step 2: 输入从第几号电机开始设置 (1~6)")
            print("Step 2: Enter starting motor number (1~6)")
            print("="*70)
            
            start_input = input("> ").strip()
            if start_input == 'q':
                mode = -1
                continue
            
            try:
                start_motor = int(start_input)
                if start_motor < 1 or start_motor > 6:
                    print("✗ 输入超出范围 (1-6)")
                    continue
            except:
                print("✗ 请输入整数")
                continue
            
            # 依次设置每个电机
            for motor_num in range(start_motor, 7):
                print(f"\n{'='*70}")
                print(f"正在设置第 {motor_num} 号电机")
                print(f"Setting motor {motor_num}")
                print(f"{'='*70}")
                
                # 失能电机
                print(f"⚠️  失能第 {motor_num} 号电机...")
                piper.DisableArm(motor_num)
                time.sleep(0.5)
                
                print(f"✓ 第 {motor_num} 号电机已失能")
                print(f"\n👉 请手动调整第 {motor_num} 号电机到零点位置")
                print(f"👉 Manually adjust motor {motor_num} to zero position")
                
                # 等待用户确认
                user_input = input(f"\n按回车设置零点 (或输入 'q' 退出) / Press Enter: ")
                
                if user_input.strip() == 'q':
                    piper.EnableArm(motor_num)
                    mode = -1
                    break
                
                # 设置零点
                print(f"正在设置第 {motor_num} 号电机零点...")
                piper.JointConfig(motor_num, 0xAE)
                time.sleep(0.5)
                
                # 重新使能
                piper.EnableArm(motor_num)
                time.sleep(0.3)
                
                print(f"✓ 第 {motor_num} 号电机零点设置完成！\n")
            
            if mode == 1:  # 如果没有被中断
                print(f"\n{'='*70}")
                print("✓ 所有电机零点设置完成！")
                print("✓ All motors zero position set successfully!")
                print(f"{'='*70}\n")
                mode = -1
    
    print("\n程序结束")
    print("Program ended")
