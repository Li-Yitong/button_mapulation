#!/usr/bin/env python3
"""
Piper 夹爪精确控制程序
详细说明夹爪位置的计算和控制方法
"""
from piper_sdk import *
import time

def enable_gripper(piper):
    """使能机械臂"""
    print("正在使能机械臂...")
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
    
    print("✓ 使能成功\n")

def gripper_control_by_ratio(piper, ratio):
    """
    通过百分比控制夹爪
    ratio: 0.0 到 1.0
           0.0 = 完全打开
           1.0 = 完全闭合
    """
    if not (0.0 <= ratio <= 1.0):
        print(f"错误：ratio 必须在 0.0-1.0 之间，当前值: {ratio}")
        return False
    
    # 计算夹爪位置
    # 在 grasp_action.py 中：joint_6 = round(position[6] * 1000 * 1000)
    # position[6] 范围通常是 0.0 到 0.10
    position = ratio * 0.10  # 将百分比转换为 0.0-0.10 的范围
    gripper_value = round(position * 1000 * 1000)
    
    print(f"百分比: {ratio*100:.1f}%")
    print(f"位置值: {position:.4f}")
    print(f"发送值: {gripper_value}")
    
    result = piper.GripperCtrl(gripper_value, 1000, 0x01, 0)
    print(f"命令返回: {result}\n")
    time.sleep(2)
    return True

def gripper_control_by_position(piper, position):
    """
    直接通过位置值控制夹爪
    position: 0 到 100000
              0 = 完全打开
              100000 = 完全闭合
    """
    if not (0 <= position <= 100000):
        print(f"错误：position 必须在 0-100000 之间，当前值: {position}")
        return False
    
    percentage = (position / 100000) * 100
    print(f"位置值: {position}")
    print(f"闭合程度: {percentage:.1f}%")
    
    result = piper.GripperCtrl(position, 1000, 0x01, 0)
    print(f"命令返回: {result}\n")
    time.sleep(2)
    return True

def gripper_control_by_meter(piper, meter_value):
    """
    通过米值控制夹爪（与 grasp_action.py 保持一致）
    meter_value: 0.0 到 0.10
                 0.0 = 完全打开
                 0.10 = 完全闭合（最大夹持力）
    """
    if not (0.0 <= meter_value <= 0.10):
        print(f"错误：meter_value 必须在 0.0-0.10 之间，当前值: {meter_value}")
        return False
    
    # 这是 grasp_action.py 中使用的计算方法
    gripper_value = round(meter_value * 1000 * 1000)
    percentage = (meter_value / 0.10) * 100
    
    print(f"米值: {meter_value:.4f}")
    print(f"闭合程度: {percentage:.1f}%")
    print(f"发送值: {gripper_value}")
    
    result = piper.GripperCtrl(gripper_value, 1000, 0x01, 0)
    print(f"命令返回: {result}\n")
    time.sleep(2)
    return True

def print_gripper_info():
    """打印夹爪控制信息"""
    print("\n" + "="*70)
    print("Piper 夹爪控制详解")
    print("="*70)
    print("\n【控制方法说明】")
    print("1. GripperCtrl(position, speed, enable, block)")
    print("   - position: 0-100000 (0=完全打开, 100000=最大夹持力)")
    print("   - speed: 1000 (固定速度)")
    print("   - enable: 0x01 (使能)")
    print("   - block: 0 (非阻塞)")
    print("\n【位置计算公式】")
    print("   在 grasp_action.py 中：")
    print("   gripper_position = round(meter_value * 1000 * 1000)")
    print("   例如：0.06 米 → 60000")
    print("         0.10 米 → 100000")
    print("\n【常用位置参考】")
    print("   0      - 完全打开")
    print("   20000  - 轻微闭合 (20%)")
    print("   40000  - 半开半闭 (40%)")
    print("   60000  - 抓取常用 (60%) ← grasp_action.py 使用")
    print("   80000  - 较紧夹持 (80%)")
    print("   100000 - 最大夹持力 (100%) ← grasp_action.py 的 0.10")
    print("="*70 + "\n")

def main():
    print_gripper_info()
    
    # 初始化
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_gripper(piper)
    
    while True:
        print("="*70)
        print("请选择控制方式:")
        print("1. 百分比控制 (0-100%)")
        print("2. 位置值控制 (0-100000)")
        print("3. 米值控制 (0.0-0.10 米) ← 与 grasp_action.py 一致")
        print("4. 预设位置")
        print("5. 查看控制信息")
        print("0. 退出程序")
        print("="*70)
        
        choice = input("\n输入选项 (0-5): ").strip()
        
        if choice == '0':
            print("\n退出程序，打开夹爪...")
            piper.GripperCtrl(0, 1000, 0x01, 0)
            break
            
        elif choice == '1':
            try:
                percent = float(input("输入百分比 (0-100): ").strip())
                ratio = percent / 100.0
                gripper_control_by_ratio(piper, ratio)
            except:
                print("无效输入\n")
                
        elif choice == '2':
            try:
                position = int(input("输入位置值 (0-100000): ").strip())
                gripper_control_by_position(piper, position)
            except:
                print("无效输入\n")
                
        elif choice == '3':
            try:
                meter = float(input("输入米值 (0.0-0.10): ").strip())
                gripper_control_by_meter(piper, meter)
            except:
                print("无效输入\n")
                
        elif choice == '4':
            print("\n预设位置:")
            print("1. 完全打开 (0)")
            print("2. 20% 闭合 (20000)")
            print("3. 40% 闭合 (40000)")
            print("4. 60% 闭合 (60000) ← grasp_action.py 使用")
            print("5. 80% 闭合 (80000)")
            print("6. 完全闭合 (100000)")
            preset = input("选择 (1-6): ").strip()
            
            preset_positions = {
                '1': 0,
                '2': 20000,
                '3': 40000,
                '4': 60000,
                '5': 80000,
                '6': 100000
            }
            
            if preset in preset_positions:
                gripper_control_by_position(piper, preset_positions[preset])
            else:
                print("无效选项\n")
                
        elif choice == '5':
            print_gripper_info()
            
        else:
            print("无效选项\n")
    
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
