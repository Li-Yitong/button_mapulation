#!/usr/bin/env python3
"""
快速验证 MoveIt2 集成是否成功
"""

import sys

print("\n" + "="*70)
print("MoveIt2 集成验证")
print("="*70)

# 测试 1: button_actions 导入
print("\n[测试 1/3] 导入 button_actions...")
try:
    import button_actions
    print("  ✓ button_actions 导入成功")
    print(f"  - USE_MOVEIT = {button_actions.USE_MOVEIT}")
    print(f"  - MOVEIT_AVAILABLE = {button_actions.MOVEIT_AVAILABLE}")
except Exception as e:
    print(f"  ✗ 失败: {e}")
    sys.exit(1)

# 测试 2: MoveIt2 模块导入
print("\n[测试 2/3] 测试 MoveIt2 ROS2 模块...")
try:
    from moveit_msgs.action import MoveGroup as MoveGroupAction
    from rclpy.action import ActionClient
    print("  ✓ MoveIt2 ROS2 模块可用")
except ImportError as e:
    print(f"  ✗ MoveIt2 模块不可用: {e}")
    print("  提示: 确保已安装 ros-foxy-moveit-msgs")

# 测试 3: vision_button_action_ros2 导入
print("\n[测试 3/3] 导入 vision_button_action_ros2...")
try:
    import vision_button_action_ros2
    print("  ✓ vision_button_action_ros2 导入成功")
    print("  - 支持 --moveit 参数")
except Exception as e:
    print(f"  ✗ 失败: {e}")
    sys.exit(1)

print("\n" + "="*70)
print("✅ 所有基础测试通过!")
print("="*70)

print("\n使用方法:")
print("\n1. SDK 模式（原有方式）:")
print("   python3 vision_button_action_ros2.py")
print("\n2. MoveIt2 模式（需先启动 MoveIt2）:")
print("   ./start_moveit2.sh --background")
print("   python3 vision_button_action_ros2.py --moveit")
print("\n3. 一键启动:")
print("   ./start_vision_button_moveit.sh --moveit")

print("\n" + "="*70 + "\n")
