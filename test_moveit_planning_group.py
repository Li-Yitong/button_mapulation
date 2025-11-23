#!/usr/bin/env python3
"""
快速测试：验证MoveIt2规划组名称修复
不需要启动完整的按钮控制，只测试MoveIt2规划
"""
import os
import sys
import time

# 检测ROS2环境
ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
if ros_distro == 'unknown':
    print("❌ 未检测到ROS2环境！")
    print("   请先运行: source /opt/ros/foxy/setup.bash")
    sys.exit(1)

print(f"✓ ROS2 ({ros_distro}) 环境已加载")

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from moveit_msgs.action import MoveGroup as MoveGroupAction
    from moveit_msgs.msg import Constraints, JointConstraint, RobotState
    from sensor_msgs.msg import JointState
    print("✓ ROS2/MoveIt2模块导入成功")
except ImportError as e:
    print(f"❌ 模块导入失败: {e}")
    sys.exit(1)

class MoveItTester(Node):
    def __init__(self):
        super().__init__('moveit_tester')
        self.action_client = ActionClient(self, MoveGroupAction, '/move_action')
        print("⏳ 等待MoveIt2 Action Server...")
        
        if self.action_client.wait_for_server(timeout_sec=5.0):
            print("✓ MoveIt2 Action Server已连接")
        else:
            print("❌ MoveIt2 Action Server连接超时！")
            print("   请先启动: ./start_moveit2_clean.sh")
            sys.exit(1)
    
    def test_planning_group(self, group_name):
        """测试规划组名称是否有效"""
        print(f"\n{'='*60}")
        print(f"测试规划组: '{group_name}'")
        print(f"{'='*60}")
        
        # 构建简单的规划请求
        goal_msg = MoveGroupAction.Goal()
        goal_msg.request.group_name = group_name
        
        # 设置起始状态（零位）
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = JointState()
        goal_msg.request.start_state.joint_state.name = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        goal_msg.request.start_state.joint_state.position = [0.0] * 6
        goal_msg.request.start_state.is_diff = False
        
        # 设置目标约束（小幅移动）
        constraints = Constraints()
        for i in range(6):
            jc = JointConstraint()
            jc.joint_name = f'joint{i+1}'
            jc.position = 0.1 if i == 1 else 0.0  # 仅移动joint2
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.request.planner_id = "RRTConnect"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 3.0
        
        goal_msg.planning_options.plan_only = True
        
        print(f"📤 发送规划请求 (group_name='{group_name}')...")
        future = self.action_client.send_goal_async(goal_msg)
        
        # 等待goal被接受
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            print("❌ 请求超时")
            return False
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            print("❌ 请求被拒绝")
            return False
        
        print("✓ 请求已接受，等待规划结果...")
        
        # 等待规划结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        
        if not result_future.done():
            print("❌ 规划超时")
            return False
        
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            print(f"✅ 规划成功！")
            print(f"   - 错误码: {result.result.error_code.val}")
            print(f"   - 轨迹点数: {len(result.result.planned_trajectory.joint_trajectory.points)}")
            return True
        else:
            print(f"❌ 规划失败")
            print(f"   - 错误码: {result.result.error_code.val}")
            if result.result.error_code.val == 99999:
                print(f"   - 原因: 规划组 '{group_name}' 不存在或配置错误")
            return False

def main():
    print("\n" + "="*60)
    print("MoveIt2规划组名称测试工具")
    print("="*60)
    
    rclpy.init()
    tester = MoveItTester()
    
    # 测试错误的规划组名称
    print("\n[测试1] 使用错误的规划组名称 'piper_arm' (旧代码)")
    success_old = tester.test_planning_group('piper_arm')
    
    time.sleep(1)
    
    # 测试正确的规划组名称
    print("\n[测试2] 使用正确的规划组名称 'arm' (修复后)")
    success_new = tester.test_planning_group('arm')
    
    # 总结
    print("\n" + "="*60)
    print("测试总结")
    print("="*60)
    print(f"'piper_arm' (旧代码):  {'✅ 成功' if success_old else '❌ 失败'}")
    print(f"'arm' (修复后):        {'✅ 成功' if success_new else '❌ 失败'}")
    
    if success_new and not success_old:
        print("\n✅ 修复验证成功！规划组名称已正确更新为 'arm'")
    elif success_old and success_new:
        print("\n⚠️  两个名称都能工作，但建议使用 'arm' (与SRDF一致)")
    elif not success_new:
        print("\n❌ 修复验证失败！请检查SRDF配置和MoveIt2状态")
    
    print("="*60)
    
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
