#!/usr/bin/env python3
"""
Piper 夹爪诊断程序
检查夹爪连接、状态和控制是否正常
"""
from piper_sdk import *
import time

def diagnose_gripper():
    print("="*70)
    print("Piper 夹爪诊断程序")
    print("="*70)
    
    # 步骤1: 连接机械臂
    print("\n【步骤1】连接机械臂...")
    try:
        piper = C_PiperInterface_V2("can0")
        piper.ConnectPort()
        print("✓ 连接成功")
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        return None
    
    # 步骤2: 检查机械臂状态
    print("\n【步骤2】检查机械臂状态...")
    try:
        status = piper.GetArmStatus()
        print("机械臂状态:")
        print(status)
    except Exception as e:
        print(f"✗ 获取状态失败: {e}")
    
    # 步骤3: 检查关节消息
    print("\n【步骤3】检查关节消息...")
    try:
        msg = piper.GetArmJointMsgs()
        print("关节状态:")
        print(f"  Joint 1: {msg.joint_state.joint_1 * 1e-3}°")
        print(f"  Joint 2: {msg.joint_state.joint_2 * 1e-3}°")
        print(f"  Joint 3: {msg.joint_state.joint_3 * 1e-3}°")
        print(f"  Joint 4: {msg.joint_state.joint_4 * 1e-3}°")
        print(f"  Joint 5: {msg.joint_state.joint_5 * 1e-3}°")
        print(f"  Joint 6: {msg.joint_state.joint_6 * 1e-3}°")
    except Exception as e:
        print(f"✗ 获取关节消息失败: {e}")
    
    # 步骤4: 检查电机状态
    print("\n【步骤4】检查电机状态...")
    try:
        low_spd_info = piper.GetArmLowSpdInfoMsgs()
        motors = [
            low_spd_info.motor_1,
            low_spd_info.motor_2,
            low_spd_info.motor_3,
            low_spd_info.motor_4,
            low_spd_info.motor_5,
            low_spd_info.motor_6
        ]
        
        for i, motor in enumerate(motors, 1):
            enable_status = motor.foc_status.driver_enable_status
            print(f"  电机 {i}: {'✓ 已使能' if enable_status else '✗ 未使能'}")
    except Exception as e:
        print(f"✗ 获取电机状态失败: {e}")
    
    # 步骤5: 尝试使能机械臂
    print("\n【步骤5】尝试使能机械臂...")
    try:
        result = piper.EnableArm(7)
        print(f"EnableArm(7) 返回: {result}")
        time.sleep(1)
        
        # 再次检查使能状态
        low_spd_info = piper.GetArmLowSpdInfoMsgs()
        all_enabled = (
            low_spd_info.motor_1.foc_status.driver_enable_status and
            low_spd_info.motor_2.foc_status.driver_enable_status and
            low_spd_info.motor_3.foc_status.driver_enable_status and
            low_spd_info.motor_4.foc_status.driver_enable_status and
            low_spd_info.motor_5.foc_status.driver_enable_status and
            low_spd_info.motor_6.foc_status.driver_enable_status
        )
        
        if all_enabled:
            print("✓ 所有电机已使能")
        else:
            print("✗ 部分电机未使能")
            
    except Exception as e:
        print(f"✗ 使能失败: {e}")
    
    # 步骤6: 测试夹爪控制
    print("\n【步骤6】测试夹爪控制...")
    print("尝试5种不同的控制方式...\n")
    
    test_methods = [
        ("方法1: GripperCtrl(0, 1000, 0x01, 0)", lambda: piper.GripperCtrl(0, 1000, 0x01, 0)),
        ("方法2: GripperCtrl(50000, 1000, 0x01, 0)", lambda: piper.GripperCtrl(50000, 1000, 0x01, 0)),
        ("方法3: GripperCtrl(100000, 1000, 0x01, 0)", lambda: piper.GripperCtrl(100000, 1000, 0x01, 0)),
        ("方法4: GripperCtrl(0, 500, 0x01, 0) [慢速]", lambda: piper.GripperCtrl(0, 500, 0x01, 0)),
        ("方法5: GripperCtrl(100000, 2000, 0x01, 0) [快速]", lambda: piper.GripperCtrl(100000, 2000, 0x01, 0)),
    ]
    
    for i, (desc, func) in enumerate(test_methods, 1):
        print(f"测试 {i}/5: {desc}")
        try:
            result = func()
            print(f"  返回值: {result}")
            time.sleep(2)
            print("  ✓ 命令执行成功")
        except Exception as e:
            print(f"  ✗ 命令执行失败: {e}")
        print()
    
    # 步骤7: 检查夹爪是否有独立的使能
    print("\n【步骤7】检查API中是否有夹爪专用函数...")
    gripper_methods = [attr for attr in dir(piper) if 'gripper' in attr.lower() or 'grip' in attr.lower()]
    if gripper_methods:
        print("发现可能相关的方法:")
        for method in gripper_methods:
            print(f"  - {method}")
    else:
        print("  未发现夹爪专用方法")
    
    print("\n" + "="*70)
    print("诊断完成")
    print("="*70)
    print("\n【建议检查项】")
    print("1. 夹爪电源是否连接")
    print("2. 夹爪是否正确安装到机械臂上")
    print("3. 夹爪与机械臂的通信线是否连接正常")
    print("4. 查看终端输出中的返回值和错误信息")
    print("5. 手动轻推夹爪，检查是否有机械卡死")
    print("6. 检查夹爪指示灯状态（如果有）")
    print("7. 查看 Piper SDK 文档确认夹爪型号和控制方法")
    print("="*70)
    
    return piper

if __name__ == "__main__":
    try:
        piper = diagnose_gripper()
        
        print("\n是否要进入交互测试模式? (y/n): ", end="")
        choice = input().strip().lower()
        
        if choice == 'y' and piper:
            print("\n进入交互测试模式...")
            print("您可以手动输入夹爪位置值进行测试")
            print("输入 'q' 退出\n")
            
            while True:
                try:
                    cmd = input("输入夹爪位置 (0-100000) 或 'q' 退出: ").strip()
                    if cmd.lower() == 'q':
                        break
                    
                    position = int(cmd)
                    if 0 <= position <= 100000:
                        print(f"发送命令: GripperCtrl({position}, 1000, 0x01, 0)")
                        result = piper.GripperCtrl(position, 1000, 0x01, 0)
                        print(f"返回值: {result}")
                        time.sleep(2)
                    else:
                        print("位置值必须在 0-100000 之间")
                except ValueError:
                    print("无效输入，请输入数字")
                except Exception as e:
                    print(f"错误: {e}")
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
