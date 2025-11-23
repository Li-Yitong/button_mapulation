#!/usr/bin/env python3
"""
测试MoveIt2规划功能 - 使用button_actions.py的实际代码路径
但用简单的目标位置（从零位到一个小角度）
"""

import sys
sys.path.insert(0, '/home/robot/button/V4.0/project2')

# 导入必需模块
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, WorkspaceParameters
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

def main():
    print("=" * 60)
    print("MoveIt2规划测试 - 简单目标")
    print("=" * 60)
    
    # 初始化ROS2
    rclpy.init()
    node = Node('test_simple_plan')
    client = ActionClient(node, MoveGroupAction, '/move_action')
    
    print("等待MoveIt2 action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        print("❌ MoveIt2服务器不可用")
        return
    print("✓ MoveIt2服务器就绪\n")
    
    # 创建规划请求 - 从零位到简单目标
    goal = MoveGroupAction.Goal()
    
    # 1. workspace
    goal.request.workspace_parameters = WorkspaceParameters()
    goal.request.workspace_parameters.header = Header()
    goal.request.workspace_parameters.header.frame_id = "base_link"  # Fixed: 使用正确的frame名称
    goal.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
    goal.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
    
    # 2. 基本参数
    goal.request.group_name = 'arm'
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.5
    goal.request.max_acceleration_scaling_factor = 0.5
    
    # 3. 起始状态（零位）
    goal.request.start_state = RobotState()
    goal.request.start_state.joint_state = JointState()
    goal.request.start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    goal.request.start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal.request.start_state.is_diff = False
    
    # 4. 目标约束（只移动joint2到0.2弧度）
    constraints = Constraints()
    target_positions = [0.0, 0.2, 0.0, 0.0, 0.0, 0.0]  # 简单目标
    
    for name, pos in zip(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], 
                         target_positions):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = 0.1
        jc.tolerance_below = 0.1
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    
    goal.request.goal_constraints = [constraints]
    
    # 5. planning options
    goal.planning_options.plan_only = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    
    print(f"发送规划请求:")
    print(f"  起始: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
    print(f"  目标: {target_positions}")
    print(f"  plan_only: True")
    print()
    
    # 发送goal
    send_goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)
    
    if not send_goal_future.done():
        print("❌ Goal发送超时")
        return
    
    goal_handle = send_goal_future.result()
    if not goal_handle or not goal_handle.accepted:
        print("❌ Goal被拒绝")
        return
    
    print("✓ Goal已接受，等待规划结果...\n")
    
    # 等待结果
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
    
    if not result_future.done():
        print("❌ 规划超时(10秒)")
        return
    
    result = result_future.result()
    if result and result.result.error_code.val == 1:  # SUCCESS
        print("✓ 规划成功!")
        if result.result.planned_trajectory.joint_trajectory.points:
            print(f"  轨迹点数: {len(result.result.planned_trajectory.joint_trajectory.points)}")
            print(f"  规划时间: {result.result.planning_time}秒")
    else:
        error_code = result.result.error_code.val if result else "None"
        print(f"❌ 规划失败 (错误码: {error_code})")
    
    # 清理
    client.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
