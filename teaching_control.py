#!/usr/bin/env python3
"""
Piper 机械臂控制参数教学脚本
演示各种参数如何控制机械臂的不同运动
"""
from piper_sdk import *
import time
import math

PI = math.pi
factor = 1000 * 180 / PI

def enable_fun(piper):
    """使能函数"""
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
            print("使能超时")
            return False
        time.sleep(1)
    return True

def show_header():
    """显示标题"""
    print("\n" + "="*80)
    print(" "*20 + "Piper 机械臂控制参数教学程序")
    print("="*80)

def lesson_1_joint_control(piper):
    """课程1: 关节角度控制 - JointCtrl()"""
    print("\n" + "="*80)
    print("课程1: 关节角度控制 - JointCtrl()")
    print("="*80)
    print("\n📖 知识点:")
    print("  - 机械臂有6个关节 (joint_0 到 joint_5)")
    print("  - 每个关节可以独立旋转")
    print("  - 参数单位: 0.001度 (需要乘以1000)")
    print("  - 正数 = 正向旋转, 负数 = 反向旋转")
    print("\n关节说明:")
    print("  joint_0: 底座旋转 (水平转动)")
    print("  joint_1: 肩部俯仰")
    print("  joint_2: 肘部俯仰")
    print("  joint_3: 腕部旋转1")
    print("  joint_4: 腕部旋转2")
    print("  joint_5: 末端旋转")
    
    input("\n按Enter继续演示...")
    
    # 演示1: 回零位
    print("\n演示1: 所有关节回到零位")
    print("代码: piper.JointCtrl(0, 0, 0, 0, 0, 0)")
    piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(3)
    
    # 演示2: 单个关节控制
    print("\n演示2: 只旋转joint_0 (底座) 45度")
    angle = 45
    joint_0 = int(angle * 1000)  # 转换为0.001度
    print(f"代码: piper.JointCtrl({joint_0}, 0, 0, 0, 0, 0)")
    print(f"说明: 45度 * 1000 = {joint_0} (0.001度单位)")
    piper.JointCtrl(joint_0, 0, 0, 0, 0, 0)
    time.sleep(3)
    
    # 演示3: 多个关节组合
    print("\n演示3: 组合控制多个关节")
    joints_deg = [0, -30, -45, 0, -20, 0]
    joints_raw = [int(j * 1000) for j in joints_deg]
    print(f"目标角度(度): {joints_deg}")
    print(f"发送参数: {joints_raw}")
    print(f"代码: piper.JointCtrl({joints_raw[0]}, {joints_raw[1]}, {joints_raw[2]}, {joints_raw[3]}, {joints_raw[4]}, {joints_raw[5]})")
    piper.JointCtrl(joints_raw[0], joints_raw[1], joints_raw[2], joints_raw[3], joints_raw[4], joints_raw[5])
    time.sleep(3)
    
    # 回零
    print("\n回到零位...")
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(2)

def lesson_2_motion_control(piper):
    """课程2: 运动模式控制 - MotionCtrl_2()"""
    print("\n" + "="*80)
    print("课程2: 运动模式控制 - MotionCtrl_2()")
    print("="*80)
    print("\n📖 知识点:")
    print("  - MotionCtrl_2(enable, mode, speed, roughly_stop)")
    print("  - enable: 0x00=关闭, 0x01=开启")
    print("  - mode: 0x00=关节空间, 0x01=笛卡尔空间")
    print("  - speed: 运动速度 (1-100)")
    print("  - roughly_stop: 0x00=精确停止, 0x01=大致停止")
    
    input("\n按Enter继续演示...")
    
    # 演示1: 不同速度
    print("\n演示1: 相同运动，不同速度")
    target_joints = [int(30 * 1000), 0, 0, 0, 0, 0]
    
    print("\n  a) 慢速运动 (speed=10)")
    print("     代码: piper.MotionCtrl_2(0x01, 0x01, 10, 0x00)")
    piper.MotionCtrl_2(0x01, 0x01, 10, 0x00)
    piper.JointCtrl(target_joints[0], 0, 0, 0, 0, 0)
    time.sleep(4)
    
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(3)
    
    print("\n  b) 快速运动 (speed=50)")
    print("     代码: piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)")
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.JointCtrl(target_joints[0], 0, 0, 0, 0, 0)
    time.sleep(2)
    
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(2)
    
    # 演示2: 运动模式
    print("\n演示2: 运动模式对比")
    print("  - 0x00=关节空间: 每个关节独立运动到目标")
    print("  - 0x01=笛卡尔空间: 末端沿直线轨迹运动")
    print("  (本演示仅展示参数，实际效果需要配合轨迹规划)")
    
    piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)

def lesson_3_gripper_control(piper):
    """课程3: 夹爪控制 - GripperCtrl()"""
    print("\n" + "="*80)
    print("课程3: 夹爪控制 - GripperCtrl()")
    print("="*80)
    print("\n📖 知识点:")
    print("  - GripperCtrl(gripper_angle, gripper_effort, gripper_code, set_zero)")
    print("  - gripper_angle: 张开距离 (单位:0.001mm)")
    print("      * 0 = 完全闭合 (0mm)")
    print("      * 70000 = 完全打开 (70mm)")
    print("  - gripper_effort: 夹持力矩 (单位:0.001N·m, 通常1000)")
    print("  - gripper_code:")
    print("      * 0x00 = 禁用夹爪")
    print("      * 0x01 = 启用夹爪")
    print("      * 0x02 = 禁用并清除错误")
    print("      * 0x03 = 启用并清除错误")
    print("  - set_zero:")
    print("      * 0x00 = 正常模式")
    print("      * 0xAE = 设置当前位置为零点")
    
    input("\n按Enter继续演示...")
    
    # 演示1: 完全打开
    print("\n演示1: 夹爪完全打开 (70mm)")
    print("代码: piper.GripperCtrl(70000, 1000, 0x01, 0x00)")
    print("说明: 70mm * 1000 = 70000 (0.001mm单位)")
    piper.GripperCtrl(70000, 1000, 0x01, 0x00)
    time.sleep(2)
    
    # 演示2: 半开
    print("\n演示2: 夹爪半开 (35mm)")
    print("代码: piper.GripperCtrl(35000, 1000, 0x01, 0x00)")
    piper.GripperCtrl(35000, 1000, 0x01, 0x00)
    time.sleep(2)
    
    # 演示3: 完全闭合
    print("\n演示3: 夹爪完全闭合 (0mm)")
    print("代码: piper.GripperCtrl(0, 1000, 0x01, 0x00)")
    piper.GripperCtrl(0, 1000, 0x01, 0x00)
    time.sleep(2)
    
    # 演示4: 不同力矩
    print("\n演示4: 不同夹持力矩")
    print("  a) 小力矩 (500)")
    print("     代码: piper.GripperCtrl(30000, 500, 0x01, 0x00)")
    piper.GripperCtrl(30000, 500, 0x01, 0x00)
    time.sleep(2)
    
    print("  b) 大力矩 (2000)")
    print("     代码: piper.GripperCtrl(30000, 2000, 0x01, 0x00)")
    piper.GripperCtrl(30000, 2000, 0x01, 0x00)
    time.sleep(2)

def lesson_4_combined_control(piper):
    """课程4: 组合控制 - 机械臂+夹爪"""
    print("\n" + "="*80)
    print("课程4: 组合控制 - 同时控制机械臂和夹爪")
    print("="*80)
    print("\n📖 知识点:")
    print("  - 可以先设置运动模式，再同时控制关节和夹爪")
    print("  - 通过joints数组传递7个参数: [6个关节 + 夹爪]")
    print("  - 夹爪参数需要从米转换: gripper_m * 1000000")
    
    input("\n按Enter继续演示...")
    
    # 演示1: 机械臂移动 + 夹爪打开
    print("\n演示1: 机械臂伸展 + 夹爪打开")
    joints_deg = [0, -30, -40, 0, -30, 0]
    gripper_m = 0.07  # 70mm = 0.07m
    
    print(f"关节角度(度): {joints_deg}")
    print(f"夹爪位置: {gripper_m}m = {gripper_m*1000}mm")
    
    piper.MotionCtrl_2(0x01, 0x01, 20, 0x00)
    joints_raw = [int(j * 1000) for j in joints_deg]
    piper.JointCtrl(joints_raw[0], joints_raw[1], joints_raw[2], joints_raw[3], joints_raw[4], joints_raw[5])
    piper.GripperCtrl(int(gripper_m * 1000000), 1000, 0x01, 0x00)
    time.sleep(3)
    
    # 演示2: 保持姿态 + 闭合夹爪
    print("\n演示2: 保持当前姿态 + 闭合夹爪")
    gripper_m = 0.00  # 完全闭合
    print(f"夹爪位置: {gripper_m}m = {gripper_m*1000}mm")
    piper.GripperCtrl(int(gripper_m * 1000000), 1000, 0x01, 0x00)
    time.sleep(2)
    
    # 回零
    print("\n回到零位...")
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    piper.GripperCtrl(0, 1000, 0x01, 0x00)
    time.sleep(2)

def lesson_5_practical_example(piper):
    """课程5: 实战案例 - 抓取动作"""
    print("\n" + "="*80)
    print("课程5: 实战案例 - 完整抓取流程")
    print("="*80)
    print("\n📖 流程:")
    print("  1. 夹爪打开")
    print("  2. 移动到物体上方")
    print("  3. 下降到物体位置")
    print("  4. 闭合夹爪抓取")
    print("  5. 抬起物体")
    print("  6. 返回安全位置")
    
    input("\n按Enter开始演示...")
    
    # 步骤1
    print("\n步骤1: 夹爪打开到70mm")
    print("  piper.GripperCtrl(70000, 1000, 0x01, 0x00)")
    piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
    piper.GripperCtrl(70000, 1000, 0x01, 0x00)
    time.sleep(2)
    
    # 步骤2
    print("\n步骤2: 移动到物体上方")
    print("  关节角度: [0, -20, -30, 0, -25, 0]度")
    piper.JointCtrl(0, -20000, -30000, 0, -25000, 0)
    time.sleep(3)
    
    # 步骤3
    print("\n步骤3: 下降到物体位置")
    print("  关节角度: [0, -20, -50, 0, -40, 0]度")
    piper.JointCtrl(0, -20000, -50000, 0, -40000, 0)
    time.sleep(3)
    
    # 步骤4
    print("\n步骤4: 闭合夹爪到40mm (抓取)")
    print("  piper.GripperCtrl(40000, 1000, 0x01, 0x00)")
    piper.GripperCtrl(40000, 1000, 0x01, 0x00)
    time.sleep(2)
    
    # 步骤5
    print("\n步骤5: 抬起物体")
    print("  关节角度: [0, -20, -30, 0, -25, 0]度")
    piper.JointCtrl(0, -20000, -30000, 0, -25000, 0)
    time.sleep(3)
    
    # 步骤6
    print("\n步骤6: 旋转90度")
    print("  关节角度: [90, -20, -30, 0, -25, 0]度")
    piper.JointCtrl(90000, -20000, -30000, 0, -25000, 0)
    time.sleep(3)
    
    # 步骤7
    print("\n步骤7: 返回安全位置并释放")
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(2)
    piper.GripperCtrl(70000, 1000, 0x01, 0x00)
    time.sleep(2)
    
    print("\n✓ 抓取流程演示完成！")

def lesson_6_parameter_summary(piper):
    """课程6: 参数总结表"""
    print("\n" + "="*80)
    print("课程6: 参数总结表")
    print("="*80)
    
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│ 1. JointCtrl(j0, j1, j2, j3, j4, j5) - 关节角度控制                │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│ 参数: j0~j5 (6个关节角度)                                          │")
    print("│ 单位: 0.001度                                                       │")
    print("│ 范围: 根据机械臂型号不同                                           │")
    print("│ 转换: 角度(度) * 1000 = 参数值                                     │")
    print("│ 示例: 45度 → 45000                                                 │")
    print("└─────────────────────────────────────────────────────────────────────┘")
    
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│ 2. MotionCtrl_2(enable, mode, speed, roughly_stop) - 运动控制      │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│ enable: 0x00=关闭, 0x01=开启                                        │")
    print("│ mode: 0x00=关节空间, 0x01=笛卡尔空间                               │")
    print("│ speed: 1~100 (速度等级)                                             │")
    print("│ roughly_stop: 0x00=精确, 0x01=大致                                  │")
    print("│ 常用: piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)                     │")
    print("└─────────────────────────────────────────────────────────────────────┘")
    
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│ 3. GripperCtrl(angle, effort, code, set_zero) - 夹爪控制           │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│ angle: 张开距离                                                      │")
    print("│   - 单位: 0.001mm                                                   │")
    print("│   - 0 = 完全闭合                                                    │")
    print("│   - 70000 = 完全打开(70mm)                                          │")
    print("│ effort: 夹持力矩 (单位:0.001N·m, 常用1000)                          │")
    print("│ code: 0x00=禁用, 0x01=启用, 0x02/0x03=清除错误                      │")
    print("│ set_zero: 0x00=正常, 0xAE=设置零点                                  │")
    print("│ 常用: piper.GripperCtrl(40000, 1000, 0x01, 0x00)                   │")
    print("└─────────────────────────────────────────────────────────────────────┘")
    
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│ 4. 单位转换速查表                                                   │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│ 关节角度: 度 × 1000 → 参数值                                        │")
    print("│   例: 90° → 90000                                                   │")
    print("│                                                                      │")
    print("│ 夹爪位置: mm × 1000 → 参数值                                        │")
    print("│   例: 50mm → 50000                                                  │")
    print("│                                                                      │")
    print("│ 夹爪位置(米): m × 1000000 → 参数值                                  │")
    print("│   例: 0.05m → 50000                                                 │")
    print("└─────────────────────────────────────────────────────────────────────┘")
    
    print("\n┌─────────────────────────────────────────────────────────────────────┐")
    print("│ 5. 典型控制流程                                                      │")
    print("├─────────────────────────────────────────────────────────────────────┤")
    print("│ 1) 初始化: piper.EnableArm(7)                                       │")
    print("│ 2) 设置运动模式: piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)          │")
    print("│ 3) 控制关节: piper.JointCtrl(...)                                   │")
    print("│ 4) 控制夹爪: piper.GripperCtrl(...)                                 │")
    print("│ 5) 等待运动完成: time.sleep(...)                                    │")
    print("└─────────────────────────────────────────────────────────────────────┘")

def main():
    """主程序"""
    show_header()
    
    print("\n正在初始化机械臂...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    
    if not enable_fun(piper):
        print("❌ 使能失败")
        return
    
    print("✓ 机械臂初始化成功")
    
    while True:
        print("\n" + "="*80)
        print("📚 教学课程目录:")
        print("="*80)
        print("1. 关节角度控制 - JointCtrl()")
        print("2. 运动模式控制 - MotionCtrl_2()")
        print("3. 夹爪控制 - GripperCtrl()")
        print("4. 组合控制 - 机械臂+夹爪")
        print("5. 实战案例 - 完整抓取流程")
        print("6. 参数总结表")
        print("0. 退出程序")
        print("="*80)
        
        choice = input("\n选择课程 (0-6): ").strip()
        
        if choice == '0':
            print("\n正在返回零位...")
            piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
            piper.JointCtrl(0, 0, 0, 0, 0, 0)
            piper.GripperCtrl(0, 1000, 0x01, 0x00)
            time.sleep(2)
            print("退出程序")
            break
            
        elif choice == '1':
            lesson_1_joint_control(piper)
            
        elif choice == '2':
            lesson_2_motion_control(piper)
            
        elif choice == '3':
            lesson_3_gripper_control(piper)
            
        elif choice == '4':
            lesson_4_combined_control(piper)
            
        elif choice == '5':
            lesson_5_practical_example(piper)
            
        elif choice == '6':
            lesson_6_parameter_summary(piper)
            input("\n按Enter返回主菜单...")
            
        else:
            print("❌ 无效选择")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
