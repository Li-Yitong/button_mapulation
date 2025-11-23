#!/usr/bin/env python3
"""
MoveIt2 状态诊断脚本
快速检查 button_actions.py 中的 MoveIt2 配置
"""
import sys

print("="*70)
print("MoveIt2 状态诊断")
print("="*70)

# 导入 button_actions
try:
    import button_actions
    print("✓ button_actions 模块导入成功")
except ImportError as e:
    print(f"✗ button_actions 导入失败: {e}")
    sys.exit(1)

# 检查 USE_MOVEIT 标志
print(f"\n[配置检查]")
print(f"  USE_MOVEIT = {button_actions.USE_MOVEIT}")
print(f"  MOVEIT_AVAILABLE = {button_actions.MOVEIT_AVAILABLE}")

# 检查全局对象
print(f"\n[对象检查]")
print(f"  piper = {button_actions.piper}")
print(f"  piper_arm = {button_actions.piper_arm}")
print(f"  move_group = {button_actions.move_group}")
print(f"  moveit_node = {button_actions.moveit_node}")

# 分析状态
print(f"\n[状态分析]")
if button_actions.USE_MOVEIT:
    if button_actions.move_group is not None:
        print("  ✓ MoveIt2 已启用且已连接")
        print("  → 将使用 MoveIt2 进行路径规划")
    else:
        print("  ⚠️ MoveIt2 已启用但 move_group 为 None")
        print("  → 需要初始化 move_group")
        print("  → 当前会回退到 SDK 模式")
else:
    print("  ⚠️ MoveIt2 未启用")
    print("  → 将使用 SDK 直接控制")
    print("  → 要启用 MoveIt2，启动时添加 --moveit 参数")

# 检查笛卡尔规划函数
print(f"\n[函数检查]")
try:
    import inspect
    source = inspect.getsource(button_actions.move_along_end_effector_z)
    
    # 检查是否包含 MoveIt 笛卡尔规划代码
    has_moveit_code = "compute_cartesian_path" in source
    has_fallback = "inverse_kinematics" in source
    
    print(f"  move_along_end_effector_z():")
    print(f"    - 包含 MoveIt2 笛卡尔规划: {'✓' if has_moveit_code else '✗'}")
    print(f"    - 包含 IK 回退机制: {'✓' if has_fallback else '✗'}")
    
except Exception as e:
    print(f"  ✗ 无法检查函数: {e}")

# 推荐操作
print(f"\n[推荐操作]")
print("="*70)

if not button_actions.USE_MOVEIT:
    print("❌ 问题: MoveIt2 未启用")
    print("\n解决方案:")
    print("  1. 确保 MoveIt2 正在运行:")
    print("     ./start_moveit2.sh --background")
    print("")
    print("  2. 使用 --moveit 参数启动 vision_button_action_ros2.py:")
    print("     python3 vision_button_action_ros2.py --moveit")
    print("")
    print("  3. 或使用启动脚本（已修复）:")
    print("     ./start_vision_button_ros2.sh --moveit")
    
elif button_actions.move_group is None:
    print("⚠️ 问题: move_group 未初始化")
    print("\n解决方案:")
    print("  1. 检查 MoveIt2 action server 是否运行:")
    print("     ros2 action list | grep move_action")
    print("")
    print("  2. 如果没有，启动 MoveIt2:")
    print("     ./start_moveit2.sh --background")
    print("")
    print("  3. 重启 vision_button_action_ros2.py:")
    print("     pkill -f vision_button_action_ros2")
    print("     python3 vision_button_action_ros2.py --moveit")
    
else:
    print("✅ MoveIt2 配置正确！")
    print("\n当前配置:")
    print("  - MoveIt2 已启用")
    print("  - move_group 已初始化")
    print("  - 笛卡尔规划可用")
    print("\n执行流程:")
    print("  1. 尝试 MoveIt2 笛卡尔路径规划")
    print("  2. 如果失败，自动回退到 IK")
    print("  3. 如果 IK 也失败，报告错误")

print("="*70)
