#!/usr/bin/env python3
"""
MoveIt2 实时测试脚本
测试在真实机器人上的 control_arm_moveit() 函数

测试流程:
1. 检查系统环境
2. 初始化机器人
3. 测试简单运动
4. 测试 MoveIt2 规划
5. 对比 SDK 和 MoveIt2 性能
"""

import sys
import time
import os

# 添加路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def print_section(title):
    """打印分节标题"""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)


def check_environment():
    """检查系统环境"""
    print_section("1. 环境检查")
    
    checks_passed = 0
    checks_total = 0
    
    # 检查 ROS2
    checks_total += 1
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro == 'foxy':
        print(f"  ✓ ROS2 环境: {ros_distro}")
        checks_passed += 1
    else:
        print(f"  ❌ ROS2 环境未设置 (当前: {ros_distro})")
        print("     请运行: source /opt/ros/foxy/setup.bash")
    
    # 检查 Python 导入
    checks_total += 1
    try:
        import rclpy
        print("  ✓ rclpy 已安装")
        checks_passed += 1
    except ImportError:
        print("  ❌ rclpy 未安装")
    
    checks_total += 1
    try:
        import button_actions
        print("  ✓ button_actions 可导入")
        checks_passed += 1
    except ImportError as e:
        print(f"  ❌ button_actions 导入失败: {e}")
    
    # 检查 MoveIt2
    checks_total += 1
    try:
        from moveit_msgs.action import MoveGroup
        print("  ✓ moveit_msgs 已安装")
        checks_passed += 1
    except ImportError:
        print("  ❌ moveit_msgs 未安装")
    
    # 检查 ROS2 节点
    checks_total += 1
    import subprocess
    result = subprocess.run(
        ['bash', '-c', 'source /opt/ros/foxy/setup.bash && ros2 node list'],
        capture_output=True, text=True
    )
    if '/robot_state_publisher' in result.stdout:
        print("  ✓ robot_state_publisher 运行中")
        checks_passed += 1
    else:
        print("  ⚠️  robot_state_publisher 未运行")
    
    # 检查 MoveIt2 action
    checks_total += 1
    result = subprocess.run(
        ['bash', '-c', 'source /opt/ros/foxy/setup.bash && ros2 action list'],
        capture_output=True, text=True
    )
    if '/move_action' in result.stdout:
        print("  ✓ MoveIt2 action server 运行中")
        checks_passed += 1
    else:
        print("  ⚠️  MoveIt2 action server 未运行")
        print("     启动: ./start_moveit2.sh --background")
    
    print(f"\n  环境检查: {checks_passed}/{checks_total} 通过")
    
    return checks_passed >= 4  # 至少 4 项必须通过


def initialize_robot():
    """初始化机器人连接"""
    print_section("2. 初始化机器人")
    
    try:
        import button_actions
        
        # 检查是否已初始化
        if button_actions.piper is not None:
            print("  ✓ 机器人已连接")
            return True
        
        print("  正在连接机器人...")
        # button_actions 会在导入时自动初始化
        time.sleep(1)
        
        if button_actions.piper is not None:
            print("  ✓ 机器人连接成功")
            return True
        else:
            print("  ❌ 机器人连接失败")
            print("     请检查机器人是否通电并连接")
            return False
            
    except Exception as e:
        print(f"  ❌ 初始化失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_get_current_joints():
    """测试获取当前关节角度"""
    print_section("3. 测试获取关节状态")
    
    try:
        import button_actions
        
        print("  正在读取关节角度...")
        joints = button_actions.get_current_joints()
        
        print(f"  ✓ 当前关节角度 (弧度):")
        for i, angle in enumerate(joints):
            print(f"    joint{i+1}: {angle:8.5f} rad ({angle*180/3.14159:.2f}°)")
        
        return True, joints
        
    except Exception as e:
        print(f"  ❌ 读取失败: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def test_sdk_control(current_joints):
    """测试 SDK 控制模式"""
    print_section("4. 测试 SDK 控制")
    
    try:
        import button_actions
        import copy
        
        # 创建一个小幅度的测试运动
        target_joints = copy.copy(current_joints)
        target_joints[0] += 0.05  # joint1 移动 0.05 弧度 (约 2.8 度)
        
        print("  测试运动:")
        print(f"    起始: joint1 = {current_joints[0]:.5f} rad")
        print(f"    目标: joint1 = {target_joints[0]:.5f} rad")
        print(f"    变化: +0.05 rad (约 +2.8°)")
        
        print("\n  执行 SDK 控制...")
        start_time = time.time()
        
        success = button_actions.control_arm_sdk(
            target_joints,
            speed=30,  # 较慢速度
            gripper_value=None
        )
        
        elapsed = time.time() - start_time
        
        if success:
            print(f"  ✓ SDK 控制成功 (用时: {elapsed:.2f}s)")
            
            # 验证位置
            time.sleep(0.5)
            new_joints = button_actions.get_current_joints()
            error = abs(new_joints[0] - target_joints[0])
            print(f"  位置误差: {error:.5f} rad ({error*180/3.14159:.3f}°)")
            
            return True
        else:
            print(f"  ❌ SDK 控制失败")
            return False
            
    except Exception as e:
        print(f"  ❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_moveit2_control(current_joints):
    """测试 MoveIt2 控制模式"""
    print_section("5. 测试 MoveIt2 控制")
    
    try:
        import button_actions
        import copy
        
        # 检查 MoveIt2 是否可用
        if not button_actions.MOVEIT_AVAILABLE:
            print("  ❌ MoveIt2 不可用")
            print("     button_actions.MOVEIT_AVAILABLE = False")
            return False
        
        if button_actions.move_group is None:
            print("  ⚠️  MoveIt2 未初始化")
            print("     需要使用 --moveit 参数启动应用")
            print("     或手动调用 initialize_moveit()")
            return False
        
        # 创建测试运动（与 SDK 测试相反方向）
        target_joints = copy.copy(current_joints)
        target_joints[0] -= 0.05  # joint1 移动 -0.05 弧度
        
        print("  测试运动:")
        print(f"    起始: joint1 = {current_joints[0]:.5f} rad")
        print(f"    目标: joint1 = {target_joints[0]:.5f} rad")
        print(f"    变化: -0.05 rad (约 -2.8°)")
        
        print("\n  执行 MoveIt2 控制...")
        start_time = time.time()
        
        success = button_actions.control_arm_moveit(
            target_joints,
            speed=30,
            gripper_value=None
        )
        
        elapsed = time.time() - start_time
        
        if success:
            print(f"  ✓ MoveIt2 控制成功 (用时: {elapsed:.2f}s)")
            
            # 验证位置
            time.sleep(0.5)
            new_joints = button_actions.get_current_joints()
            error = abs(new_joints[0] - target_joints[0])
            print(f"  位置误差: {error:.5f} rad ({error*180/3.14159:.3f}°)")
            
            return True
        else:
            print(f"  ❌ MoveIt2 控制失败（已回退到 SDK 模式）")
            return False
            
    except Exception as e:
        print(f"  ❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_comparison(current_joints):
    """对比 SDK 和 MoveIt2 性能"""
    print_section("6. 性能对比测试")
    
    try:
        import button_actions
        import copy
        
        if not button_actions.MOVEIT_AVAILABLE or button_actions.move_group is None:
            print("  ⚠️  MoveIt2 不可用，跳过对比测试")
            return True
        
        # 定义测试运动
        test_movements = [
            ("小幅运动", [0.05, 0, 0, 0, 0, 0]),
            ("多关节运动", [0.05, 0.03, -0.02, 0, 0, 0]),
        ]
        
        results = []
        
        for name, delta in test_movements:
            print(f"\n  测试: {name}")
            print(f"  关节变化: {delta}")
            
            # 计算目标位置
            target_joints = copy.copy(current_joints)
            for i in range(len(delta)):
                target_joints[i] += delta[i]
            
            # 测试 SDK
            print(f"\n  → SDK 模式:")
            start = time.time()
            success_sdk = button_actions.control_arm_sdk(
                target_joints, speed=50, gripper_value=None
            )
            time_sdk = time.time() - start
            
            if success_sdk:
                print(f"    ✓ 成功 (用时: {time_sdk:.2f}s)")
            else:
                print(f"    ❌ 失败")
            
            time.sleep(1)
            
            # 恢复到起始位置
            button_actions.control_arm_sdk(
                current_joints, speed=50, gripper_value=None
            )
            time.sleep(1)
            
            # 测试 MoveIt2
            print(f"\n  → MoveIt2 模式:")
            start = time.time()
            success_moveit = button_actions.control_arm_moveit(
                target_joints, speed=50, gripper_value=None
            )
            time_moveit = time.time() - start
            
            if success_moveit:
                print(f"    ✓ 成功 (用时: {time_moveit:.2f}s)")
            else:
                print(f"    ❌ 失败（回退到 SDK）")
            
            time.sleep(1)
            
            # 恢复到起始位置
            button_actions.control_arm_sdk(
                current_joints, speed=50, gripper_value=None
            )
            time.sleep(1)
            
            # 记录结果
            results.append({
                'name': name,
                'sdk_time': time_sdk if success_sdk else None,
                'moveit_time': time_moveit if success_moveit else None,
                'sdk_success': success_sdk,
                'moveit_success': success_moveit
            })
        
        # 打印对比结果
        print("\n  " + "-"*70)
        print("  对比结果:")
        print("  " + "-"*70)
        
        for result in results:
            print(f"\n  {result['name']}:")
            if result['sdk_success']:
                print(f"    SDK:     {result['sdk_time']:.2f}s")
            else:
                print(f"    SDK:     失败")
            
            if result['moveit_success']:
                print(f"    MoveIt2: {result['moveit_time']:.2f}s")
                if result['sdk_success'] and result['sdk_time']:
                    ratio = result['moveit_time'] / result['sdk_time']
                    print(f"    比值:    {ratio:.2f}x")
            else:
                print(f"    MoveIt2: 失败")
        
        return True
        
    except Exception as e:
        print(f"  ❌ 对比测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_safety_return(initial_joints):
    """安全返回到初始位置"""
    print_section("7. 返回初始位置")
    
    try:
        import button_actions
        
        print("  正在返回初始位置...")
        success = button_actions.control_arm_sdk(
            initial_joints,
            speed=30,
            gripper_value=None
        )
        
        if success:
            print("  ✓ 已返回初始位置")
            time.sleep(1)
            
            # 验证
            final_joints = button_actions.get_current_joints()
            max_error = max([abs(final_joints[i] - initial_joints[i]) for i in range(6)])
            print(f"  位置误差: {max_error:.5f} rad ({max_error*180/3.14159:.3f}°)")
        else:
            print("  ⚠️  返回失败，请手动检查")
        
        return success
        
    except Exception as e:
        print(f"  ❌ 返回失败: {e}")
        return False


def main():
    """主测试流程"""
    print("="*70)
    print("  MoveIt2 实时测试 - control_arm_moveit()")
    print("="*70)
    
    # 1. 环境检查
    if not check_environment():
        print("\n❌ 环境检查未通过，请解决上述问题后重试")
        return 1
    
    # 2. 初始化机器人
    if not initialize_robot():
        print("\n❌ 机器人初始化失败")
        return 1
    
    # 3. 获取初始状态
    success, initial_joints = test_get_current_joints()
    if not success:
        print("\n❌ 无法读取关节状态")
        return 1
    
    print("\n⚠️  警告: 即将开始运动测试")
    print("    机器人将进行小幅度运动 (约 ±3°)")
    print("    请确保机器人周围安全")
    
    response = input("\n继续测试? (y/N): ")
    if response.lower() != 'y':
        print("测试已取消")
        return 0
    
    try:
        # 4. SDK 测试
        success_sdk = test_sdk_control(initial_joints)
        time.sleep(2)
        
        # 返回初始位置
        test_safety_return(initial_joints)
        time.sleep(1)
        
        # 5. MoveIt2 测试
        success_moveit = test_moveit2_control(initial_joints)
        time.sleep(2)
        
        # 返回初始位置
        test_safety_return(initial_joints)
        time.sleep(1)
        
        # 6. 性能对比（可选）
        if success_sdk and success_moveit:
            response = input("\n执行性能对比测试? (y/N): ")
            if response.lower() == 'y':
                test_comparison(initial_joints)
        
    finally:
        # 7. 安全返回
        test_safety_return(initial_joints)
    
    # 总结
    print_section("测试总结")
    
    if success_sdk:
        print("  ✓ SDK 控制: 通过")
    else:
        print("  ❌ SDK 控制: 失败")
    
    if success_moveit:
        print("  ✓ MoveIt2 控制: 通过")
    else:
        print("  ⚠️  MoveIt2 控制: 未测试或失败")
    
    if success_sdk and success_moveit:
        print("\n🎉 所有测试通过！MoveIt2 集成工作正常")
        return 0
    elif success_sdk:
        print("\n✓ SDK 控制正常")
        print("⚠️  MoveIt2 需要检查（可能未启动或未初始化）")
        return 0
    else:
        print("\n❌ 测试失败，请检查系统状态")
        return 1


if __name__ == "__main__":
    sys.exit(main())
