#!/usr/bin/env python3#!/usr/bin/env python3#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import rclpy""""""

from rclpy.node import Node

from rclpy.action import ActionClient简单测试MoveIt2 action client简单的 MoveIt2 测试脚本

from moveit_msgs.action import MoveGroup

from moveit_msgs.msg import Constraints, JointConstraint, RobotState, WorkspaceParameters"""验证 move_group 是否正常运行

from sensor_msgs.msg import JointState

from geometry_msgs.msg import Vector3import rclpy"""

from std_msgs.msg import Header

import timefrom rclpy.node import Node



def main():from rclpy.action import ActionClientimport rclpy

    rclpy.init()

    node = Node('test_moveit_client')from moveit_msgs.action import MoveGroupfrom rclpy.node import Node

    

    client = ActionClient(node, MoveGroup, '/move_action')from moveit_msgs.msg import Constraints, JointConstraint, RobotState, WorkspaceParametersfrom moveit_msgs.action import MoveGroup

    

    print("Waiting for action server...")from sensor_msgs.msg import JointStatefrom rclpy.action import ActionClient

    timeout = 10.0

    start = time.time()from geometry_msgs.msg import Vector3import sys

    while not client.server_is_ready():

        rclpy.spin_once(node, timeout_sec=0.1)from std_msgs.msg import Header

        if time.time() - start > timeout:

            print("ERROR: Server not ready!")import time

            return

    class MoveIt2SimpleTester(Node):

    print("OK: Server is ready")

    def main():    """简单的 MoveIt2 测试节点"""

    goal = MoveGroup.Goal()

        rclpy.init()    

    goal.request.workspace_parameters = WorkspaceParameters()

    goal.request.workspace_parameters.header = Header()    node = Node('test_moveit_client')    def __init__(self):

    goal.request.workspace_parameters.header.frame_id = "arm_base"

    goal.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)            super().__init__('moveit2_simple_tester')

    goal.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)

        # 创建action client        self.get_logger().info('初始化 MoveIt2 测试节点...')

    goal.request.group_name = 'arm'

    goal.request.num_planning_attempts = 5    client = ActionClient(node, MoveGroup, '/move_action')        

    goal.request.allowed_planning_time = 5.0

    goal.request.max_velocity_scaling_factor = 0.5            # 创建 MoveGroup action client

    goal.request.max_acceleration_scaling_factor = 0.5

        print("等待action server...")        self._action_client = ActionClient(

    goal.request.start_state = RobotState()

    goal.request.start_state.joint_state = JointState()    timeout = 10.0            self,

    goal.request.start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    goal.request.start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    start = time.time()            MoveGroup,

    goal.request.start_state.is_diff = False

        while not client.server_is_ready():            '/move_action'

    constraints = Constraints()

    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']        rclpy.spin_once(node, timeout_sec=0.1)        )

    target_positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]

            if time.time() - start > timeout:        

    for i, (name, pos) in enumerate(zip(joint_names, target_positions)):

        jc = JointConstraint()            print("Server不可用!")    def test_connection(self):

        jc.joint_name = name

        jc.position = pos            return        """测试与 move_group 的连接"""

        jc.tolerance_above = 0.01

        jc.tolerance_below = 0.01            self.get_logger().info('等待 move_group action server...')

        jc.weight = 1.0

        constraints.joint_constraints.append(jc)    print("✓ Server已就绪")        

    

    goal.request.goal_constraints = [constraints]            if self._action_client.wait_for_server(timeout_sec=10.0):

    

    goal.planning_options.plan_only = True    # 创建一个简单的goal            self.get_logger().info('✅ 成功连接到 move_group action server!')

    goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        goal = MoveGroup.Goal()            return True

    print("\nSending goal...")

    print(f"  group: {goal.request.group_name}")            else:

    print(f"  plan_only: {goal.planning_options.plan_only}")

    print(f"  target: joint2 = {target_positions[1]}")    # Workspace parameters            self.get_logger().error('❌ 无法连接到 move_group action server')

    

    future = client.send_goal_async(goal)    goal.request.workspace_parameters = WorkspaceParameters()            return False

    

    print("Waiting for goal acceptance...")    goal.request.workspace_parameters.header = Header()

    timeout = 10.0

    start = time.time()    goal.request.workspace_parameters.header.frame_id = "arm_base"

    spin_count = 0

    while not future.done():    goal.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)def main(args=None):

        rclpy.spin_once(node, timeout_sec=0.01)

        spin_count += 1    goal.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)    """主函数"""

        if spin_count % 100 == 0:

            elapsed = time.time() - start        rclpy.init(args=args)

            print(f"  Spinning... (count={spin_count}, time={elapsed:.1f}s)")

        if time.time() - start > timeout:    # 基本参数    

            print(f"ERROR: Timeout after {spin_count} spins")

            node.destroy_node()    goal.request.group_name = 'arm'    print('\n' + '='*60)

            rclpy.shutdown()

            return    goal.request.num_planning_attempts = 5    print('MoveIt2 Simple Connection Test')

    

    print(f"OK: Got response after {spin_count} spins")    goal.request.allowed_planning_time = 5.0    print('='*60 + '\n')

    

    goal_handle = future.result()    goal.request.max_velocity_scaling_factor = 0.5    

    if not goal_handle:

        print("ERROR: goal_handle is None")    goal.request.max_acceleration_scaling_factor = 0.5    tester = MoveIt2SimpleTester()

    elif not goal_handle.accepted:

        print(f"ERROR: Goal rejected with status: {goal_handle.status}")        

    else:

        print("OK: Goal accepted!")    # 起始状态    try:

        

        print("Waiting for planning result...")    goal.request.start_state = RobotState()        # 测试连接

        result_future = goal_handle.get_result_async()

        start = time.time()    goal.request.start_state.joint_state = JointState()        print('测试与 move_group 的连接...')

        spin_count = 0

        while not result_future.done():    goal.request.start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']        if tester.test_connection():

            rclpy.spin_once(node, timeout_sec=0.01)

            spin_count += 1    goal.request.start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]            print('\n✅ MoveIt2 运行正常!\n')

            if spin_count % 100 == 0:

                elapsed = time.time() - start    goal.request.start_state.is_diff = False            print('move_group 节点已成功启动并可以接收规划请求。')

                print(f"  Waiting for result... (count={spin_count}, time={elapsed:.1f}s)")

            if time.time() - start > 30.0:                result = 0

                print(f"ERROR: Result timeout")

                break    # 目标约束        else:

        

        if result_future.done():    constraints = Constraints()            print('\n❌ MoveIt2 连接失败\n')

            result = result_future.result()

            if result:    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']            print('请确保已启动: ros2 launch piper_with_gripper_moveit demo_foxy.launch.py')

                print(f"OK: Planning complete! Error code: {result.result.error_code.val}")

                if result.result.planned_trajectory:    target_positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]  # 简单的目标            result = 1

                    points = len(result.result.planned_trajectory.joint_trajectory.points)

                    print(f"  Trajectory points: {points}")            

            else:

                print("ERROR: Result is None")    for i, (name, pos) in enumerate(zip(joint_names, target_positions)):        print('='*60 + '\n')

    

    node.destroy_node()        jc = JointConstraint()        

    rclpy.shutdown()

    print("\nTest completed")        jc.joint_name = name    except KeyboardInterrupt:



if __name__ == '__main__':        jc.position = pos        tester.get_logger().info('测试被用户中断')

    main()

        jc.tolerance_above = 0.01        result = 0

        jc.tolerance_below = 0.01    finally:

        jc.weight = 1.0        tester.destroy_node()

        constraints.joint_constraints.append(jc)        rclpy.shutdown()

        

    goal.request.goal_constraints = [constraints]    return result

    

    # Planning options

    goal.planning_options.plan_only = Trueif __name__ == '__main__':

    goal.planning_options.planning_scene_diff.robot_state.is_diff = True    sys.exit(main())

    
    print("\n发送goal...")
    print(f"  group: {goal.request.group_name}")
    print(f"  plan_only: {goal.planning_options.plan_only}")
    print(f"  目标: joint2 = {target_positions[1]}")
    
    future = client.send_goal_async(goal)
    
    print("等待goal被接受...")
    timeout = 10.0
    start = time.time()
    spin_count = 0
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.01)
        spin_count += 1
        if spin_count % 100 == 0:
            elapsed = time.time() - start
            print(f"  Spinning... ({spin_count} times, {elapsed:.1f}s)")
        if time.time() - start > timeout:
            print(f"❌ 超时 (spin了{spin_count}次)")
            node.destroy_node()
            rclpy.shutdown()
            return
    
    print(f"✓ Goal响应收到 (spin了{spin_count}次)")
    
    goal_handle = future.result()
    if not goal_handle:
        print("❌ goal_handle is None")
    elif not goal_handle.accepted:
        print(f"❌ Goal被拒绝: {goal_handle.status}")
    else:
        print("✓ Goal已接受!")
        
        # 等待结果
        print("等待规划结果...")
        result_future = goal_handle.get_result_async()
        start = time.time()
        spin_count = 0
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.01)
            spin_count += 1
            if spin_count % 100 == 0:
                elapsed = time.time() - start
                print(f"  等待结果... ({spin_count} times, {elapsed:.1f}s)")
            if time.time() - start > 30.0:
                print(f"❌ 等待结果超时")
                break
        
        if result_future.done():
            result = result_future.result()
            if result:
                print(f"✓ 规划完成! 错误码: {result.result.error_code.val}")
                if result.result.planned_trajectory:
                    points = len(result.result.planned_trajectory.joint_trajectory.points)
                    print(f"  轨迹点数: {points}")
            else:
                print("❌ Result is None")
    
    node.destroy_node()
    rclpy.shutdown()
    print("\n完成")

if __name__ == '__main__':
    main()
