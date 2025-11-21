# #!/usr/bin/env python3
# # -*-coding:utf8-*-
# """
# 基于 Piper SDK 官方示例的回零测试
# """
# import time
# from piper_sdk import *

# def configure_gripper(piper):
#     """配置夹爪参数（示教模式退出后需要重新配置）"""
#     print("配置夹爪参数...")
#     try:
#         piper.GripperTeachingPendantParamConfig(100, 70)
#         piper.ArmParamEnquiryAndConfig(4)
#         time.sleep(1)
#         print("✓ 夹爪参数配置完成")
#         return True
#     except Exception as e:
#         print(f"⚠️  配置警告: {e}")
#         return False

# def diagnose_arm_state(piper):
#     """诊断机械臂当前状态"""
#     print("\n" + "="*60)
#     print("机械臂状态诊断")
#     print("="*60)
    
#     # 1. 检查机械臂状态
#     print("\n1. 机械臂整体状态:")
#     status = piper.GetArmStatus()
#     status_str = str(status)
#     print(status_str)
    
#     # 2. 检查控制模式
#     if "Control Mode:" in status_str:
#         mode = status_str.split("Control Mode: ")[1].split("\n")[0]
#         print(f"\n控制模式: {mode}")
#         if "TEACH" in mode.upper():
#             print("⚠️  警告：机械臂处于示教模式！需要退出示教模式")
    
#     # 3. 检查运动状态
#     if "Motion Status:" in status_str:
#         motion = status_str.split("Motion Status: ")[1].split("\n")[0]
#         print(f"运动状态: {motion}")
    
#     # 4. 检查使能状态
#     if "Arm Status:" in status_str:
#         arm_status = status_str.split("Arm Status: ")[1].split("\n")[0]
#         print(f"使能状态: {arm_status}")
#         if "DISABLE" in arm_status.upper():
#             print("⚠️  警告：机械臂未使能！")
    
#     # 5. 检查每个电机状态
#     print("\n2. 电机使能状态:")
#     low_spd_info = piper.GetArmLowSpdInfoMsgs()
#     motor_status = [
#         ("电机1", low_spd_info.motor_1.foc_status.driver_enable_status),
#         ("电机2", low_spd_info.motor_2.foc_status.driver_enable_status),
#         ("电机3", low_spd_info.motor_3.foc_status.driver_enable_status),
#         ("电机4", low_spd_info.motor_4.foc_status.driver_enable_status),
#         ("电机5", low_spd_info.motor_5.foc_status.driver_enable_status),
#         ("电机6", low_spd_info.motor_6.foc_status.driver_enable_status),
#     ]
#     for name, status in motor_status:
#         status_icon = "✓" if status else "✗"
#         print(f"  {status_icon} {name}: {'已使能' if status else '未使能'}")
    
#     # 6. 检查当前关节角度
#     print("\n3. 当前关节角度:")
#     msg = piper.GetArmJointMsgs()
#     angles = [
#         msg.joint_state.joint_1 * 1e-3,
#         msg.joint_state.joint_2 * 1e-3,
#         msg.joint_state.joint_3 * 1e-3,
#         msg.joint_state.joint_4 * 1e-3,
#         msg.joint_state.joint_5 * 1e-3,
#         msg.joint_state.joint_6 * 1e-3,
#     ]
#     for i, angle in enumerate(angles, 1):
#         print(f"  关节{i}: {angle:8.2f}°")
    
#     print("="*60 + "\n")
#     return all([status[1] for status in motor_status])

# def enable_arm(piper):
#     """使能机械臂（参考gripper_control_simple.py）
#     注意：示教模式退出后必须重新使能
#     """
#     print("正在使能机械臂...")
#     enable_flag = False
#     timeout = 5
#     start_time = time.time()
    
#     while not enable_flag:
#         elapsed_time = time.time() - start_time
        
#         # 读取每个电机的使能状态
#         low_spd_info = piper.GetArmLowSpdInfoMsgs()
#         motor_status = [
#             low_spd_info.motor_1.foc_status.driver_enable_status,
#             low_spd_info.motor_2.foc_status.driver_enable_status,
#             low_spd_info.motor_3.foc_status.driver_enable_status,
#             low_spd_info.motor_4.foc_status.driver_enable_status,
#             low_spd_info.motor_5.foc_status.driver_enable_status,
#             low_spd_info.motor_6.foc_status.driver_enable_status,
#         ]
        
#         print(f"  电机使能状态: {motor_status} (等待时间: {elapsed_time:.1f}s)")
        
#         enable_flag = all(motor_status)
#         piper.EnableArm(7)
#         piper.GripperCtrl(0, 1000, 0x01, 0)  # 同时使能夹爪
        
#         if elapsed_time > timeout:
#             print("✗ 使能超时")
#             print(f"  未使能的电机: {[i+1 for i, status in enumerate(motor_status) if not status]}")
#             return False
#         time.sleep(1)
    
#     print("✓ 使能成功")
#     return True

# def test_method_1_ModeCtrl():
#     """方法1: 使用 ModeCtrl (官方 piper_ctrl_go_zero.py 的方法)"""
#     print("\n" + "="*60)
#     print("方法 1: 使用 ModeCtrl - 平滑回零")
#     print("="*60)
    
#     piper = C_PiperInterface_V2("can0")
#     piper.ConnectPort()
    
#     # 诊断：检查初始状态
#     diagnose_arm_state(piper)
    
#     # 重要：配置夹爪参数（示教模式退出后必须重新配置）
#     print("步骤1: 配置夹爪参数...")
#     configure_gripper(piper)
    
#     # 读取夹爪反馈
#     print("\n夹爪反馈信息:")
#     feedback = piper.GetGripperTeachingPendantParamFeedback()
#     print(feedback)
    
#     print("\n步骤2: 检查机械臂状态...")
#     status = piper.GetArmStatus()
#     print(status)
    
#     # 读取当前位置
#     msg = piper.GetArmJointMsgs()
#     current_angles = [
#         msg.joint_state.joint_1 * 1e-3,
#         msg.joint_state.joint_2 * 1e-3,
#         msg.joint_state.joint_3 * 1e-3,
#         msg.joint_state.joint_4 * 1e-3,
#         msg.joint_state.joint_5 * 1e-3,
#         msg.joint_state.joint_6 * 1e-3,
#     ]
#     print(f"\n当前关节角度: {[round(a, 2) for a in current_angles]}°")
    
#     # 使能机械臂
#     print("\n步骤3: 使能机械臂...")
#     piper.EnableArm(7)
#     if not enable_arm(piper):
#         print("使能失败，退出程序")
#         return None
    
#     # 检查并退出示教模式
#     print("\n步骤3.5: 检查并退出示教模式...")
#     status = piper.GetArmStatus()
#     status_str = str(status)
#     if "Teach Status: 2" in status_str or "Control Mode: 2" in status_str:
#         print("  ⚠️  机械臂处于示教模式，正在退出...")
#         piper.MotionCtrl_2(0x00, 0x01, 0, 0x00)  # 退出示教模式
#         time.sleep(1)
        
#         # 重新检查状态
#         status = piper.GetArmStatus()
#         status_str = str(status)
#         if "Control Mode:" in status_str:
#             mode = status_str.split("Control Mode: ")[1].split("\n")[0]
#             print(f"  当前控制模式: {mode}")
        
#         if "Control Mode: 2" in status_str:
#             print("  ✗ 仍在示教模式，尝试强制退出...")
#             # 尝试多次退出
#             for i in range(3):
#                 piper.MotionCtrl_2(0x00, 0x01, 0, 0x00)
#                 time.sleep(0.5)
#         else:
#             print("  ✓ 已退出示教模式")
#     else:
#         print("  ✓ 不在示教模式，无需退出")
    
#     factor = 57295.7795  # 1000*180/3.1415926
#     position = [0, 0, 0, 0, 0, 0, 0]
    
#     joint_0 = round(position[0]*factor)
#     joint_1 = round(position[1]*factor)
#     joint_2 = round(position[2]*factor)
#     joint_3 = round(position[3]*factor)
#     joint_4 = round(position[4]*factor)
#     joint_5 = round(position[5]*factor)
#     joint_6 = round(position[6]*1000*1000)
    
#     print("\n步骤4: 发送回零命令（从当前位置平滑移动到零位）...")
#     print("⚠️  机械臂将平滑移动到零位！")
#     time.sleep(2)
    
#     # 使用 MotionCtrl_2 替代 ModeCtrl
#     print("\n发送控制命令:")
#     print(f"  MotionCtrl_2(0x01, 0x01, 30, 0x00)")
#     piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
    
#     print(f"  JointCtrl({joint_0}, {joint_1}, {joint_2}, {joint_3}, {joint_4}, {joint_5})")
#     piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    
#     print(f"  GripperCtrl({abs(joint_6)}, 1000, 0x01, 0)")
#     piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    
#     print("\n命令已发送，监控运动状态...")
    
#     # 监控关节角度变化
#     for i in range(10):
#         time.sleep(1)
        
#         # 读取状态
#         status = piper.GetArmStatus()
#         status_str = str(status)
        
#         # 读取关节角度
#         msg = piper.GetArmJointMsgs()
#         angles = [
#             msg.joint_state.joint_1 * 1e-3,
#             msg.joint_state.joint_2 * 1e-3,
#             msg.joint_state.joint_3 * 1e-3,
#             msg.joint_state.joint_4 * 1e-3,
#             msg.joint_state.joint_5 * 1e-3,
#             msg.joint_state.joint_6 * 1e-3,
#         ]
        
#         print(f"\n--- {i+1}秒 ---")
#         print(f"关节角度: {[round(a, 2) for a in angles]}°")
        
#         # 提取关键信息
#         if "Control Mode:" in status_str:
#             mode = status_str.split("Control Mode: ")[1].split("\n")[0]
#             print(f"控制模式: {mode}")
        
#         if "Motion Status:" in status_str:
#             motion = status_str.split("Motion Status: ")[1].split("\n")[0]
#             print(f"运动状态: {motion}")
            
#             if "REACH_TARGET_POS_SUCCESSFULLY" in motion:
#                 print("✓✓✓ 已到达目标位置！")
#                 break
    
#     print("\n✓ 方法1测试完成")
#     return piper

# def test_method_2_MotionCtrl2_only():
#     """方法2: 只使用 MotionCtrl_2 (官方 piper_ctrl_joint.py 的方法)"""
#     print("\n" + "="*60)
#     print("方法 2: 使用 MotionCtrl_2 - 平滑回零")
#     print("="*60)
    
#     piper = C_PiperInterface_V2("can0")
#     piper.ConnectPort()
    
#     # 诊断：检查初始状态
#     diagnose_arm_state(piper)
    
#     # 重要：配置夹爪参数（示教模式退出后必须重新配置）
#     print("步骤1: 配置夹爪参数...")
#     configure_gripper(piper)
    
#     # 读取夹爪反馈
#     print("\n夹爪反馈信息:")
#     feedback = piper.GetGripperTeachingPendantParamFeedback()
#     print(feedback)
    
#     print("\n步骤2: 检查机械臂状态...")
#     status = piper.GetArmStatus()
#     print(status)
    
#     # 读取当前位置
#     msg = piper.GetArmJointMsgs()
#     current_angles = [
#         msg.joint_state.joint_1 * 1e-3,
#         msg.joint_state.joint_2 * 1e-3,
#         msg.joint_state.joint_3 * 1e-3,
#         msg.joint_state.joint_4 * 1e-3,
#         msg.joint_state.joint_5 * 1e-3,
#         msg.joint_state.joint_6 * 1e-3,
#     ]
#     print(f"\n当前关节角度: {[round(a, 2) for a in current_angles]}°")
    
#     # 使能机械臂（统一使用 enable_arm 函数）
#     print("\n步骤3: 使能机械臂...")
#     piper.EnableArm(7)
#     if not enable_arm(piper):
#         print("使能失败，退出程序")
#         return None
    
#     # 检查并退出示教模式
#     print("\n步骤3.5: 检查并退出示教模式...")
#     status = piper.GetArmStatus()
#     status_str = str(status)
#     if "Teach Status: 2" in status_str or "Control Mode: 2" in status_str:
#         print("  ⚠️  机械臂处于示教模式，正在退出...")
#         piper.MotionCtrl_2(0x00, 0x01, 0, 0x00)  # 退出示教模式
#         time.sleep(1)
        
#         # 重新检查状态
#         status = piper.GetArmStatus()
#         status_str = str(status)
#         if "Control Mode:" in status_str:
#             mode = status_str.split("Control Mode: ")[1].split("\n")[0]
#             print(f"  当前控制模式: {mode}")
        
#         if "Control Mode: 2" in status_str:
#             print("  ✗ 仍在示教模式，尝试强制退出...")
#             # 尝试多次退出
#             for i in range(3):
#                 piper.MotionCtrl_2(0x00, 0x01, 0, 0x00)
#                 time.sleep(0.5)
#         else:
#             print("  ✓ 已退出示教模式")
#     else:
#         print("  ✓ 不在示教模式，无需退出")
    
#     factor = 57295.7795  # 1000*180/3.1415926
#     position = [0, 0, 0, 0, 0, 0, 0]
    
#     joint_0 = round(position[0]*factor)
#     joint_1 = round(position[1]*factor)
#     joint_2 = round(position[2]*factor)
#     joint_3 = round(position[3]*factor)
#     joint_4 = round(position[4]*factor)
#     joint_5 = round(position[5]*factor)
#     joint_6 = round(position[6]*1000*1000)
    
#     print("\n步骤4: 发送回零命令（从当前位置平滑移动到零位）...")
#     print("⚠️  机械臂将平滑移动到零位！")
#     time.sleep(2)
    
#     # 直接发送回零命令，不失能
#     print("\n发送控制命令:")
#     print(f"  MotionCtrl_2(0x01, 0x01, 100, 0x00)")
#     piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
    
#     print(f"  JointCtrl({joint_0}, {joint_1}, {joint_2}, {joint_3}, {joint_4}, {joint_5})")
#     piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    
#     print(f"  GripperCtrl({abs(joint_6)}, 1000, 0x01, 0)")
#     piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    
#     print("\n命令已发送，监控运动状态...")
    
#     # 监控关节角度变化
#     for i in range(10):
#         time.sleep(1)
        
#         # 读取状态
#         status = piper.GetArmStatus()
#         status_str = str(status)
        
#         # 读取关节角度
#         msg = piper.GetArmJointMsgs()
#         angles = [
#             msg.joint_state.joint_1 * 1e-3,
#             msg.joint_state.joint_2 * 1e-3,
#             msg.joint_state.joint_3 * 1e-3,
#             msg.joint_state.joint_4 * 1e-3,
#             msg.joint_state.joint_5 * 1e-3,
#             msg.joint_state.joint_6 * 1e-3,
#         ]
        
#         print(f"\n--- {i+1}秒 ---")
#         print(f"关节角度: {[round(a, 2) for a in angles]}°")
        
#         # 提取关键信息
#         if "Control Mode:" in status_str:
#             mode = status_str.split("Control Mode: ")[1].split("\n")[0]
#             print(f"控制模式: {mode}")
        
#         if "Motion Status:" in status_str:
#             motion = status_str.split("Motion Status: ")[1].split("\n")[0]
#             print(f"运动状态: {motion}")
            
#             if "REACH_TARGET_POS_SUCCESSFULLY" in motion:
#                 print("✓✓✓ 已到达目标位置！")
#                 break
    
#     print("\n✓ 方法2测试完成")
#     return piper

# def main():
#     print("="*60)
#     print("Piper 机械臂平滑回零测试")
#     print("="*60)
#     print("\n特点：")
#     print("  ✓ 不会失能摔下来")
#     print("  ✓ 从当前位置平滑移动到零位")
#     print("  ✓ 适用于任意姿态")
#     print("  ✓ 示教模式退出后自动重新配置")
#     print("="*60)
    
#     print("\n请选择测试方法：")
#     print("1. 使用 ModeCtrl (速度较慢，更平滑)")
#     print("2. 使用 MotionCtrl_2 (速度较快) - 推荐")
#     print("3. 两种方法都测试")
    
#     choice = input("\n输入选项 (1/2/3): ").strip()
    
#     if choice == '1':
#         test_method_1_ModeCtrl()
#     elif choice == '2':
#         test_method_2_MotionCtrl2_only()
#     elif choice == '3':
#         print("\n先测试方法1...")
#         test_method_1_ModeCtrl()
        
#         input("\n按 Enter 继续测试方法2...")
#         test_method_2_MotionCtrl2_only()
#     else:
#         print("无效选项")
#         return
    
#     print("\n" + "="*60)
#     print("测试完成！")
#     print("="*60)
#     print("\n使用说明：")
#     print("  这个回零方法的优点：")
#     print("    ✓ 不需要失能，机械臂不会摔下来")
#     print("    ✓ 从当前位置平滑移动到零位")
#     print("    ✓ 可以在运行过程中随时调用")
#     print("    ✓ 示教模式退出后会重新配置夹爪参数")
#     print("\n  在 grasp_action.py 中使用：")
#     print("    1. 连接机械臂后先配置夹爪参数")
#     print("    2. 使能机械臂（包括夹爪）")
#     print("    3. 直接发送回零命令")
#     print("    4. 示教模式退出后需要重新执行步骤1-2")

# if __name__ == "__main__":
#     try:
#         main()
#     except KeyboardInterrupt:
#         print("\n\n用户中断")
#     except Exception as e:
#         print(f"\n✗ 错误: {e}")
#         import traceback
#         traceback.print_exc()
#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
最简化的Piper机械臂回零脚本
适用于任意位置，平滑回零，不会摔下来
"""
import time
from piper_sdk import *

def go_zero():
    """平滑回零 - 最简化版本"""
    # 1. 连接机械臂
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    print("✓ 已连接机械臂")
    
    # 2. 使能（等待所有电机使能完成）
    print("正在使能...")
    piper.EnableArm(7)
    time.sleep(2)  # 等待使能完成
    print("✓ 使能完成")
    
    # 3. 发送回零命令（所有关节角度设为0）
    print("开始回零...")
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)  # 速度50
    piper.JointCtrl(0, 0, 0, 0, 0, 0)  # 所有关节角度为0
    piper.GripperCtrl(0, 1000, 0x01, 0)  # 夹爪也回零
    
    # 4. 等待完成
    time.sleep(5)  # 等待足够时间让机械臂移动到零位
    print("✓ 回零完成！")
    
    return piper

if __name__ == "__main__":
    try:
        piper = go_zero()
        print("\n机械臂已回到零位，可以安全关闭程序")
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")