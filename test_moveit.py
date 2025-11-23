#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, WorkspaceParameters
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import time

def main():
    rclpy.init()
    node = Node('test_moveit_client')
    client = ActionClient(node, MoveGroup, '/move_action')
    
    print("Waiting for server...")
    if not client.wait_for_server(timeout_sec=10.0):
        print("ERROR: Server not available")
        return
    print("OK: Server ready")
    
    goal = MoveGroup.Goal()
    goal.request.workspace_parameters = WorkspaceParameters()
    goal.request.workspace_parameters.header = Header()
    goal.request.workspace_parameters.header.frame_id = "arm_base"
    goal.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
    goal.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
    goal.request.group_name = 'arm'
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.5
    goal.request.max_acceleration_scaling_factor = 0.5
    goal.request.start_state = RobotState()
    goal.request.start_state.joint_state = JointState()
    goal.request.start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    goal.request.start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal.request.start_state.is_diff = False
    
    constraints = Constraints()
    for name, pos in zip(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], 
                         [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    goal.request.goal_constraints = [constraints]
    goal.planning_options.plan_only = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    
    print("Sending goal...")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    
    if future.done():
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            print("OK: Goal accepted")
        else:
            print("ERROR: Goal rejected")
    else:
        print("ERROR: Timeout waiting for goal acceptance")
    
    # 正确清理：先销毁client，再销毁node
    client.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
