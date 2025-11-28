#!/usr/bin/env python3
"""
Piper 夹爪测试程序 - 基于官方文档
验证闭合控制是否正确
"""
from piper_sdk import C_PiperInterface_V2
import time

def enable_piper_arm(piper, timeout=10):
    """使能机械臂的所有电机"""
    print("\n【2】使能机械臂...")
    enable_flag = False
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
        
        if elapsed_time > timeout:
            print("✗ 使能超时")
            return False
        time.sleep(1)
    
    print("✓ 机械臂已使能")
    return True

def test_gripper_official():
    print("="*70)
    print("Piper 夹爪测试 - 官方文档版本")
    print("="*70)
    
    # 初始化
    print("\n【1】初始化连接...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    print("✓ 连接成功")
    
    # 使能机械臂
    if not enable_piper_arm(piper):
        return
    
    # 配置夹爪参数
    print("\n【3】配置夹爪参数...")
    try:
        piper.GripperTeachingPendantParamConfig(100, 70)
        piper.ArmParamEnquiryAndConfig(4)
        time.sleep(1)
        print("✓ 配置完成")
        
        feedback = piper.GetGripperTeachingPendantParamFeedback()
        print(f"夹爪反馈: {feedback}")
    except Exception as e:
        print(f"⚠️  配置警告: {e}")
    
    # 初始化夹爪（官方推荐流程）
    print("\n【4】初始化夹爪（禁用并清除错误）...")
    piper.GripperCtrl(0, 1000, 0x02, 0)  # 禁用并清除错误
    time.sleep(0.1)
    print("✓ 错误已清除")
    
    print("\n【5】启用夹爪...")
    piper.GripperCtrl(0, 1000, 0x01, 0)  # 启用
    time.sleep(0.1)
    print("✓ 夹爪已启用")
    
    # 测试序列
    print("\n" + "="*70)
    print("开始测试序列（按 Ctrl+C 随时终止）")
    print("="*70)
    
    test_sequence = [
        (0, "完全闭合 (0mm 张开)"),
        (10000, "轻微张开 (10mm)"),
        (20000, "小幅张开 (20mm)"),
        (35000, "中度张开 (35mm)"),
        (50000, "大幅张开 (50mm)"),
        (70000, "完全打开 (70mm)"),
        (0, "返回闭合 (0mm)"),
    ]
    
    try:
        for i, (angle, description) in enumerate(test_sequence, 1):
            print(f"\n【测试 {i}/{len(test_sequence)}】{description}")
            print(f"  发送命令: GripperCtrl({angle}, 1000, 0x01, 0)")
            
            # 连续发送3次确保执行
            for j in range(3):
                result = piper.GripperCtrl(angle, 1000, 0x01, 0)
                if j == 0:
                    print(f"  返回值: {result}")
                time.sleep(0.3)
            
            print(f"  ✓ 命令已发送，等待执行...")
            time.sleep(3)
            print(f"  ✓ 完成")
            
            # 询问用户
            response = input("\n  夹爪是否正确移动到目标位置？(y/n/q退出): ").strip().lower()
            if response == 'q':
                break
            elif response == 'n':
                print("  ⚠️  位置不正确，请检查硬件连接和配置")
        
        # 最终闭合
        print("\n【结束】将夹爪闭合到安全位置...")
        for i in range(3):
            piper.GripperCtrl(0, 1000, 0x01, 0)
            time.sleep(0.3)
        print("✓ 测试完成")
        
    except KeyboardInterrupt:
        print("\n\n✗ 测试被用户中断")
        print("正在安全关闭...")
        piper.GripperCtrl(0, 1000, 0x01, 0)
        time.sleep(1)
    
    print("\n" + "="*70)
    print("测试结束")
    print("="*70)

if __name__ == "__main__":
    test_gripper_official()
