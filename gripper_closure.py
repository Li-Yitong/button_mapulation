#!/usr/bin/env python3
from piper_sdk import *
import time

def enable_gripper(piper):
    """使能机械臂"""
    enable_flag = False
    timeout = 5
    start_time = time.time()
    
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
            exit(0)
        time.sleep(1)
    
    print("使能成功")

def open_gripper(piper):
    """完全打开夹爪（张开到最大行程）"""
    print("打开夹爪到最大行程...")
    # 根据官方文档：gripper_angle 单位是 0.001mm
    # 数值越大，张开距离越大
    max_range = 70000  # 70mm (根据配置的 max_range=70)
    result = piper.GripperCtrl(max_range, 1000, 0x01, 0)
    print(f"命令返回: {result}")
    time.sleep(2)
    print("夹爪已打开")

def close_gripper(piper, position=0):
    """设置夹爪位置
    position: 0-70000 (单位:0.001mm)
              0 = 完全闭合
              70000 = 完全打开(70mm)
    """
    print(f"设置夹爪到位置 {position} (0.001mm 单位)...")
    mm_value = position / 1000
    print(f"张开距离: {mm_value:.1f}mm")
    result = piper.GripperCtrl(position, 1000, 0x01, 0)
    print(f"命令返回: {result}")
    time.sleep(2)
    print("夹爪位置已设置")

def set_gripper_position(piper, position):
    """设置夹爪到指定位置
    position: 0-70000 (单位:0.001mm)
              0 = 完全闭合
              70000 = 完全打开(70mm)
    """
    print(f"设置夹爪到位置 {position} (0.001mm 单位)...")
    mm_value = position / 1000
    print(f"张开距离: {mm_value:.1f}mm")
    result = piper.GripperCtrl(position, 1000, 0x01, 0)
    print(f"命令返回: {result}")
    time.sleep(2)
    print(f"夹爪位置已设置")

if __name__ == "__main__":
    # 初始化
    print("="*60)
    print("Piper 夹爪控制程序")
    print("="*60)
    
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    
    # 重要：配置夹爪参数（第一次使用或夹爪无响应时必须执行）
    print("\n配置夹爪参数...")
    try:
        # GripperTeachingPendantParamConfig(max_load_config, max_range_config)
        piper.GripperTeachingPendantParamConfig(100, 70)
        piper.ArmParamEnquiryAndConfig(4)
        time.sleep(1)
        print("✓ 夹爪参数配置完成")
    except Exception as e:
        print(f"配置警告: {e}")
        print("继续运行...")
    
    # 读取夹爪反馈
    print("\n夹爪反馈信息:")
    feedback = piper.GetGripperTeachingPendantParamFeedback()
    print(feedback)
    
    piper.EnableArm(7)
    enable_gripper(piper)
    
    # 默认状态：夹爪完全闭合
    print("\n设置默认状态：夹爪完全闭合...")
    print("⚠️ 官方文档说明：")
    print("   - gripper_angle 单位是 0.001mm")
    print("   - 0 = 完全闭合 (0mm 张开)")
    print("   - 70000 = 完全打开 (70mm 张开)")
    for i in range(3):
        print(f"  第{i+1}次发送闭合命令 (0)...")
        piper.GripperCtrl(0, 1000, 0x01, 0)  # 0 = 完全闭合
        time.sleep(1.5)
    print("✓ 夹爪已完全闭合")
    
    while True:
        print("\n" + "="*60)
        print("请选择操作:")
        print("⚠️  单位: 0.001mm | 0=闭合, 70000=打开(70mm)")
        print("1. 完全闭合 (0mm) ← 默认状态")
        print("2. 轻微张开 (10mm = 10000)")
        print("3. 小幅张开 (20mm = 20000)")
        print("4. 中度张开 (35mm = 35000)")
        print("5. 大幅张开 (50mm = 50000)")
        print("6. 完全打开 (70mm = 70000)")
        print("7. 自定义位置 (输入0-70000, 单位:0.001mm)")
        print("8. 强制完全闭合（连续3次 0）")
        print("0. 退出程序")
        print("="*60)
        
        choice = input("\n输入选项 (0-8): ").strip()
        
        if choice == '0':
            print("\n退出程序...")
            print("退出前完全闭合夹爪...")
            piper.GripperCtrl(0, 1000, 0x01, 0)  # 0 = 完全闭合
            break
            
        elif choice == '1':
            print("发送完全闭合命令 (0)...")
            piper.GripperCtrl(0, 1000, 0x01, 0)  # 0 = 完全闭合
            time.sleep(2)
            
        elif choice == '2':
            close_gripper(piper, 10000)  # 10mm
            
        elif choice == '3':
            close_gripper(piper, 20000)  # 20mm
            
        elif choice == '4':
            close_gripper(piper, 35000)  # 35mm
            
        elif choice == '5':
            close_gripper(piper, 50000)  # 50mm
            
        elif choice == '6':
            print("发送完全打开命令 (70000)...")
            piper.GripperCtrl(70000, 1000, 0x01, 0)  # 70mm = 完全打开
            time.sleep(2)
            
        elif choice == '7':
            try:
                position = int(input("输入夹爪位置 (0-70000, 单位:0.001mm): ").strip())
                if 0 <= position <= 70000:
                    set_gripper_position(piper, position)
                else:
                    print("无效位置，必须在0-70000之间")
            except:
                print("无效输入")
        
        elif choice == '8':
            print("强制完全闭合（连续发送3次命令 0）...")
            for i in range(3):
                print(f"  第{i+1}次闭合命令 (0)...")
                piper.GripperCtrl(0, 1000, 0x01, 0)
                time.sleep(1.5)
            print("✓ 强制闭合完成")
                
        else:
            print("无效选项，请重新选择")
    
    print("\n程序结束")
