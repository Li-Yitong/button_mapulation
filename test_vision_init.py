#!/usr/bin/env python3
"""
测试 vision_button_action.py 的硬件初始化
"""
import sys

# 模拟导入检查
print("="*70)
print("测试硬件初始化逻辑")
print("="*70)

print("\n[1/4] 检查导入...")
try:
    from piper_sdk import C_PiperInterface_V2
    from piper_arm import PiperArm
    print("  ✓ 导入成功")
except Exception as e:
    print(f"  ✗ 导入失败: {e}")
    sys.exit(1)

print("\n[2/4] 测试 Piper SDK 初始化...")
try:
    piper = C_PiperInterface_V2("can0")
    print(f"  ✓ 创建实例成功: {type(piper)}")
    
    # 测试连接
    result = piper.ConnectPort()
    print(f"  ✓ ConnectPort 返回: {result}")
    
    # 测试使能
    result = piper.EnableArm(7)
    print(f"  ✓ EnableArm 返回: {result}")
    
    # 测试获取关节信息
    msg = piper.GetArmJointMsgs()
    print(f"  ✓ GetArmJointMsgs 成功: {type(msg)}")
    print(f"    关节1: {msg.joint_state.joint_1}")
    print(f"    关节6: {msg.joint_state.joint_6}")
    
except Exception as e:
    print(f"  ✗ 失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n[3/4] 测试 PiperArm 初始化...")
try:
    piper_arm = PiperArm()
    print(f"  ✓ 创建实例成功: {type(piper_arm)}")
    
    # 检查标定参数
    print(f"  ✓ link6_q_camera: {piper_arm.link6_q_camera}")
    print(f"  ✓ link6_t_camera: {piper_arm.link6_t_camera}")
    
except Exception as e:
    print(f"  ✗ 失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n[4/4] 测试 button_actions 更新...")
try:
    import button_actions
    
    # 更新全局对象
    button_actions.piper = piper
    button_actions.piper_arm = piper_arm
    
    print(f"  ✓ button_actions.piper: {type(button_actions.piper)}")
    print(f"  ✓ button_actions.piper_arm: {type(button_actions.piper_arm)}")
    
    # 测试是否能访问
    msg = button_actions.piper.GetArmJointMsgs()
    print(f"  ✓ 通过 button_actions.piper 访问成功")
    
except Exception as e:
    print(f"  ✗ 失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "="*70)
print("✓✓✓ 所有测试通过！")
print("="*70)
