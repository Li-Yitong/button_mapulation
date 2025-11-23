#!/usr/bin/env python3
"""
测试frame名称修复后的MoveIt2规划
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import WorkspaceParameters, Constraints, JointConstraint
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import time

def main():
    rclpy.init()
    node = Node('test_frame_fix')
    
    print("=" * 60)
    print("测试frame名称修复 (arm_base → base_link)")
    print("=" * 60)
    
    # 创建action client
    client = ActionClient(node, MoveGroup, '/move_action')
    
    print("\n⏳ 等待MoveIt2 action server...")
    if not client.wait_for_server(timeout_sec=5.0):
        print("❌ Action server未启动")
        return
    
    print("✅ Action server已连接")
    
    # 创建goal
    goal = MoveGroup.Goal()
    
    # 使用正确的frame名称: base_link
    goal.request.workspace_parameters = WorkspaceParameters()
    goal.request.workspace_parameters.header = Header()
    goal.request.workspace_parameters.header.frame_id = "base_link"  # 修复后的frame名称
    goal.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
    goal.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
    
    # 设置目标：joint2从当前位置移动0.1弧度
    goal.request.group_name = "arm"
    goal.request.num_planning_attempts = 1
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.5
    goal.request.max_acceleration_scaling_factor = 0.5
    
    # 目标约束
    goal.request.goal_constraints = [Constraints()]
    joint_constraint = JointConstraint()
    joint_constraint.joint_name = "joint2"
    joint_constraint.position = 0.1  # 目标位置
    joint_constraint.tolerance_above = 0.1
    joint_constraint.tolerance_below = 0.1
    joint_constraint.weight = 1.0
    goal.request.goal_constraints[0].joint_constraints = [joint_constraint]
    
    # 只规划不执行
    goal.planning_options.plan_only = True
    
    print("\n📤 发送目标 (frame_id='base_link')...")
    future = client.send_goal_async(goal)
    
    # 等待goal acceptance
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if not future.done():
        print("❌ Goal发送超时")
        return
        
    goal_handle = future.result()
    if not goal_handle.accepted:
        print("❌ Goal被拒绝")
        return
    
    print("✅ Goal已接受")
    
    # 等待规划结果
    print("\n⏳ 等待规划结果 (最多30秒)...")
    result_future = goal_handle.get_result_async()
    
    start_time = time.time()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
    elapsed = time.time() - start_time
    
    if not result_future.done():
        print(f"❌ 规划超时 ({elapsed:.1f}秒)")
        return
    
    result = result_future.result()
    
    print(f"\n⏱️  规划耗时: {elapsed:.2f}秒")
    print(f"📊 错误代码: {result.result.error_code.val}")
    
    if result.result.error_code.val == 1:  # SUCCESS
        print("✅ 规划成功！")
        print(f"   轨迹点数: {len(result.result.planned_trajectory.joint_trajectory.points)}")
    else:
        print(f"❌ 规划失败: {result.result.error_code.val}")
    
    print("\n" + "=" * 60)
    
    client.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
