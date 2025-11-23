#!/usr/bin/env python3
"""
MoveIt2 交互式手动测试
允许用户逐步测试 MoveIt2 功能
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def print_header(text):
    print("\n" + "="*70)
    print(f"  {text}")
    print("="*70)

def print_step(text):
    print(f"\n{text}")

def wait_confirm(prompt="继续? (Enter 继续, q 退出): "):
    """等待用户确认"""
    response = input(prompt)
    if response.lower() == 'q':
        print("\n测试已取消")
        sys.exit(0)
    return True

# 导入模块
print_header("MoveIt2 交互式手动测试")
print("\n正在加载模块...")

import button_actions
import copy

# 初始化 ROS2 用于 MoveIt2
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup as MoveGroupAction

print("✓ 模块加载成功")

# 1. 检查机器人连接
print_step("[1/7] 检查机器人连接")
if button_actions.piper is None:
    print("  正在连接机器人...")
    from piper_sdk import C_PiperInterface
    
    button_actions.piper = C_PiperInterface()
    button_actions.piper.ConnectPort()
    print("  ✓ 机器人已连接")
    
    print("  正在使能机器人...")
    button_actions.piper.EnableArm(7, 0x01)
    time.sleep(2)
    print("  ✓ 机器人已使能")
else:
    print("  ✓ 机器人已连接")

# 2. 读取当前位置
print_step("[2/7] 读取当前关节角度")
initial_joints = button_actions.get_current_joints()
print("  当前关节角度 (弧度):")
for i, angle in enumerate(initial_joints):
    print(f"    joint{i+1}: {angle:8.5f} rad ({angle*180/3.14159:7.2f}°)")

# 3. 初始化 MoveIt2
print_step("[3/7] 初始化 MoveIt2")

if not button_actions.MOVEIT_AVAILABLE:
    print("  ❌ MoveIt2 模块不可用")
    sys.exit(1)

if button_actions.moveit_node is None:
    print("  正在初始化 ROS2 节点...")
    try:
        rclpy.init()
        node = rclpy.create_node('manual_test_node')
        print("  ✓ ROS2 节点创建成功")
        
        print("  正在创建 Action Client...")
        client = ActionClient(node, MoveGroupAction, '/move_action')
        print("  ✓ Action Client 创建成功")
        
        print("  正在连接 MoveIt2 服务器...")
        if not client.wait_for_server(timeout_sec=5.0):
            print("  ❌ MoveIt2 服务器未响应")
            print("  请确保 MoveIt2 已启动: ./start_moveit2.sh --background")
            sys.exit(1)
        print("  ✓ MoveIt2 服务器已连接")
        
        # 初始化 button_actions 中的 MoveIt2
        button_actions.moveit_node = node
        button_actions.move_group = client
        print("  ✓ MoveIt2 初始化完成")
    except Exception as e:
        print(f"  ❌ 初始化失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
else:
    print("  ✓ MoveIt2 已经初始化")

# 4. 测试 SDK 控制
print_step("[4/7] 测试 SDK 控制")
print("  将进行小幅运动测试 (joint1 +3°)")
print("  ⚠️  机器人将开始运动！")
wait_confirm()

test_joints_sdk = copy.copy(initial_joints)
test_joints_sdk[0] += 0.05  # 约 2.8°

print(f"  目标: joint1 = {test_joints_sdk[0]:.5f} rad ({test_joints_sdk[0]*180/3.14159:.2f}°)")
print("  正在执行 SDK 控制...")

start_time = time.time()
success_sdk = button_actions.control_arm_sdk(test_joints_sdk, speed=30, gripper_value=None)
elapsed_sdk = time.time() - start_time

if success_sdk:
    print(f"  ✓ SDK 控制成功 (用时: {elapsed_sdk:.2f}s)")
    time.sleep(0.5)
    
    # 验证位置
    new_joints = button_actions.get_current_joints()
    error = abs(new_joints[0] - test_joints_sdk[0])
    print(f"  位置误差: {error:.5f} rad ({error*180/3.14159:.3f}°)")
else:
    print(f"  ❌ SDK 控制失败")

time.sleep(1)

# 5. 返回初始位置
print_step("[5/7] 返回初始位置")
wait_confirm()

print("  正在返回...")
button_actions.control_arm_sdk(initial_joints, speed=30, gripper_value=None)
print("  ✓ 已返回")
time.sleep(1)

# 6. 测试 MoveIt2 控制
print_step("[6/7] 测试 MoveIt2 控制")
print("  将使用 MoveIt2 进行小幅运动测试 (joint1 -3°)")
print("  ⚠️  机器人将开始运动！")
wait_confirm()

test_joints_moveit = copy.copy(initial_joints)
test_joints_moveit[0] -= 0.05  # 约 -2.8°

print(f"  目标: joint1 = {test_joints_moveit[0]:.5f} rad ({test_joints_moveit[0]*180/3.14159:.2f}°)")
print("  正在执行 MoveIt2 控制...")
print("  (这可能需要几秒钟进行规划...)")

start_time = time.time()
success_moveit = button_actions.control_arm_moveit(
    test_joints_moveit, 
    speed=30, 
    gripper_value=None
)
elapsed_moveit = time.time() - start_time

if success_moveit:
    print(f"  ✓ MoveIt2 控制成功 (用时: {elapsed_moveit:.2f}s)")
    time.sleep(0.5)
    
    # 验证位置
    new_joints = button_actions.get_current_joints()
    error = abs(new_joints[0] - test_joints_moveit[0])
    print(f"  位置误差: {error:.5f} rad ({error*180/3.14159:.3f}°)")
else:
    print(f"  ❌ MoveIt2 控制失败（可能回退到 SDK）")

time.sleep(1)

# 7. 返回初始位置
print_step("[7/7] 最终返回初始位置")
wait_confirm()

print("  正在返回...")
button_actions.control_arm_sdk(initial_joints, speed=30, gripper_value=None)
print("  ✓ 已返回")
time.sleep(0.5)

# 总结
print_header("测试总结")

print("\n性能对比:")
print(f"  SDK 模式:    {elapsed_sdk:.2f}s")
if success_moveit:
    print(f"  MoveIt2 模式: {elapsed_moveit:.2f}s")
    ratio = elapsed_moveit / elapsed_sdk if elapsed_sdk > 0 else 0
    print(f"  时间比值:    {ratio:.2f}x")

print("\n结果:")
if success_sdk:
    print("  ✓ SDK 控制: 正常")
else:
    print("  ❌ SDK 控制: 失败")

if success_moveit:
    print("  ✓ MoveIt2 控制: 正常")
else:
    print("  ⚠️  MoveIt2 控制: 失败或回退")

if success_sdk and success_moveit:
    print("\n🎉 所有测试通过！MoveIt2 集成工作正常")
elif success_sdk:
    print("\n✓ SDK 控制正常")
    print("⚠️  MoveIt2 需要检查")
else:
    print("\n❌ 测试失败")

print("\n" + "="*70)
print("  测试完成")
print("="*70)

# 清理
if button_actions.moveit_node is not None:
    try:
        button_actions.moveit_node.destroy_node()
        rclpy.shutdown()
    except:
        pass
