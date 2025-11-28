#!/usr/bin/env python3
"""
Piper 夹爪零点设置教程
可以自定义设置夹爪的零点位置
"""
from piper_sdk import C_PiperInterface_V2
import time

def set_gripper_zero_official():
    """
    官方标准方法：将当前位置设为零点
    
    使用场景：
    1. 夹爪行程数据异常时
    2. 首次使用夹爪时
    3. 更换夹爪后
    """
    print("="*70)
    print("方法1: 官方标准零点设置")
    print("="*70)
    
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    
    print("\n步骤1: 禁用夹爪")
    piper.GripperCtrl(0, 1000, 0x00, 0)  # gripper_code=0x00 禁用
    time.sleep(1.5)
    
    print("步骤2: 将当前位置设为零点")
    piper.GripperCtrl(0, 1000, 0x00, 0xAE)  # set_zero=0xAE 设置零点
    print("✓ 零点已设置为当前位置")
    
    print("\n注意：此方法会将夹爪**当前所在位置**设为零点")
    print("如果夹爪当前是张开的，零点就是张开位置！")

def set_gripper_zero_custom():
    """
    自定义方法：先移动夹爪到期望的零点位置，再设置零点
    
    推荐使用：可以精确控制零点位置
    """
    print("\n" + "="*70)
    print("方法2: 自定义零点设置（推荐）")
    print("="*70)
    
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    
    # 使能机械臂
    print("\n步骤1: 使能机械臂...")
    piper.EnableArm(7)
    timeout = 10
    start_time = time.time()
    enable_flag = False
    
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
            return
        time.sleep(1)
    
    print("✓ 使能成功")
    
    # 初始化夹爪
    print("\n步骤2: 初始化夹爪...")
    piper.GripperCtrl(0, 1000, 0x02, 0)  # 禁用并清除错误
    time.sleep(0.5)
    piper.GripperCtrl(0, 1000, 0x01, 0)  # 启用夹爪
    time.sleep(0.5)
    print("✓ 夹爪已初始化")
    
    # 让用户选择零点位置
    print("\n" + "="*70)
    print("零点位置选择：")
    print("1. 完全闭合位置作为零点（推荐）")
    print("2. 当前位置作为零点")
    print("3. 自定义位置作为零点")
    print("="*70)
    
    choice = input("\n请选择 (1-3): ").strip()
    
    if choice == '1':
        print("\n步骤3: 移动夹爪到完全闭合位置...")
        # 先尝试闭合到最小位置
        for i in range(3):
            piper.GripperCtrl(0, 1000, 0x01, 0)
            time.sleep(0.5)
        print("✓ 夹爪已移动到完全闭合位置")
        
    elif choice == '2':
        print("\n步骤3: 使用当前位置...")
        print("✓ 保持当前位置")
        
    elif choice == '3':
        try:
            position = int(input("\n输入目标位置 (0-70000, 单位:0.001mm): ").strip())
            if 0 <= position <= 70000:
                print(f"\n步骤3: 移动夹爪到位置 {position} ({position/1000}mm)...")
                for i in range(3):
                    piper.GripperCtrl(position, 1000, 0x01, 0)
                    time.sleep(0.5)
                print(f"✓ 夹爪已移动到位置 {position/1000}mm")
            else:
                print("✗ 无效位置")
                return
        except:
            print("✗ 无效输入")
            return
    else:
        print("✗ 无效选择")
        return
    
    # 确认设置零点
    confirm = input("\n确认将当前位置设为零点？(y/n): ").strip().lower()
    if confirm != 'y':
        print("✗ 取消设置")
        return
    
    # 设置零点
    print("\n步骤4: 设置零点...")
    piper.GripperCtrl(0, 1000, 0x00, 0)  # 先禁用
    time.sleep(1.5)
    piper.GripperCtrl(0, 1000, 0x00, 0xAE)  # 设置零点
    time.sleep(1)
    print("✓ 零点设置完成！")
    
    # 验证零点
    print("\n步骤5: 验证零点...")
    print("现在会让夹爪移动到零点位置（发送 GripperCtrl(0, ...)）")
    time.sleep(2)
    
    piper.GripperCtrl(0, 1000, 0x01, 0)  # 启用并移动到零点
    time.sleep(2)
    print("✓ 夹爪应该已回到零点位置")
    
    print("\n" + "="*70)
    print("零点设置完成！")
    print("="*70)

def test_gripper_after_zero():
    """
    设置零点后的测试
    """
    print("\n" + "="*70)
    print("零点设置后测试")
    print("="*70)
    
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    
    # 初始化
    print("\n初始化夹爪...")
    piper.GripperCtrl(0, 1000, 0x02, 0)
    time.sleep(0.5)
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.5)
    
    # 测试序列
    test_positions = [
        (0, "零点位置"),
        (10000, "10mm"),
        (20000, "20mm"),
        (35000, "35mm"),
        (50000, "50mm"),
        (0, "返回零点"),
    ]
    
    print("\n开始测试...")
    for position, description in test_positions:
        print(f"\n移动到: {description} (position={position})")
        for i in range(3):
            piper.GripperCtrl(position, 1000, 0x01, 0)
            time.sleep(0.3)
        time.sleep(2)
        
        response = input("位置正确？(y/n): ").strip().lower()
        if response != 'y':
            print("⚠️  位置异常，可能需要重新设置零点")
            break
    
    print("\n测试完成")

def main():
    print("="*70)
    print("Piper 夹爪零点设置工具")
    print("="*70)
    print("\n选择操作：")
    print("1. 使用官方标准方法设置零点（将当前位置设为零点）")
    print("2. 使用自定义方法设置零点（推荐，可选择零点位置）")
    print("3. 测试当前零点设置")
    print("4. 查看零点设置说明")
    print("="*70)
    
    choice = input("\n请选择 (1-4): ").strip()
    
    if choice == '1':
        set_gripper_zero_official()
    elif choice == '2':
        set_gripper_zero_custom()
    elif choice == '3':
        test_gripper_after_zero()
    elif choice == '4':
        show_zero_explanation()
    else:
        print("无效选择")

def show_zero_explanation():
    """显示零点设置说明"""
    print("\n" + "="*70)
    print("夹爪零点设置详细说明")
    print("="*70)
    
    print("""
📌 什么是零点？

零点是夹爪位置的参考基准点。当你发送 GripperCtrl(0, ...) 时，
夹爪会移动到这个零点位置。

📌 零点的作用：

1. 定义夹爪的起始参考位置
2. 所有位置命令都是相对于零点的距离
3. 例如：
   - GripperCtrl(0, ...)     → 移动到零点
   - GripperCtrl(10000, ...) → 从零点张开 10mm
   - GripperCtrl(50000, ...) → 从零点张开 50mm

📌 设置零点的方法：

方法1: 官方标准方法（简单但不灵活）
    piper.GripperCtrl(0, 1000, 0x00, 0)    # 禁用
    time.sleep(1.5)
    piper.GripperCtrl(0, 1000, 0x00, 0xAE) # 设置当前位置为零点
    
    ⚠️  注意：这会把夹爪**当前位置**设为零点！
    如果夹爪当前是张开的，零点就会是张开状态。

方法2: 自定义方法（推荐）
    步骤1: 移动夹爪到期望的零点位置
           piper.GripperCtrl(desired_position, 1000, 0x01, 0)
    
    步骤2: 禁用夹爪
           piper.GripperCtrl(0, 1000, 0x00, 0)
    
    步骤3: 设置零点
           piper.GripperCtrl(0, 1000, 0x00, 0xAE)

📌 推荐的零点位置：

✅ 推荐：将完全闭合位置设为零点
   - 优点：逻辑清晰，0 表示闭合
   - 方法：先让夹爪完全闭合，再设置零点

⚠️  不推荐：将打开位置设为零点
   - 缺点：容易混淆，0 表示打开

📌 何时需要重新设置零点？

1. 首次使用夹爪时
2. 夹爪行程数据异常时
3. 更换夹爪后
4. 夹爪位置反馈不准确时

📌 参数说明：

GripperCtrl(gripper_angle, gripper_effort, gripper_code, set_zero)

set_zero 参数：
  - 0x00: 正常模式（不设置零点）
  - 0xAE: 将当前位置设为零点

gripper_code 参数：
  - 0x00: 禁用夹爪
  - 0x01: 启用夹爪
  - 0x02: 禁用并清除错误
  - 0x03: 启用并清除错误

📌 设置零点后的验证：

1. 发送 GripperCtrl(0, 1000, 0x01, 0)
2. 观察夹爪是否回到你设置的零点位置
3. 如果位置不对，重新设置零点
""")

if __name__ == "__main__":
    main()
