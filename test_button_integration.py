#!/usr/bin/env python3
"""
测试button_actions.py的MoveIt2集成
简化版：只测试初始化和一次规划
"""
import sys
import os
sys.path.insert(0, '/home/robot/button/V4.0/project2')

# 导入button_actions的全局变量和函数
from button_actions import *

def test_initialization():
    """测试初始化"""
    global piper, moveit_node, move_group, joint_state_publisher
    
    print("="*60)
    print("测试button_actions.py的MoveIt2集成")
    print("="*60)
    
    # 初始化硬件
    print("\n1️⃣ 初始化硬件...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    
    for i in range(7):
        piper.EnableArm(i + 1)
        time.sleep(0.1)
    
    piper.GripperCtrl(70000, 1000, 0x01, 0)
    print("  ✓ 硬件初始化完成")
    
    # 初始化ROS2
    print("\n2️⃣ 初始化ROS2...")
    if MOVEIT_AVAILABLE:
        rclpy.init()
        print("  ✓ ROS2已初始化")
        
        # 创建节点
        moveit_node = Node('test_button_actions_node')
        print("  ✓ ROS2节点已创建")
        
        # 启动joint_states发布器
        from sensor_msgs.msg import JointState
        joint_state_publisher = moveit_node.create_publisher(JointState, '/joint_states', 10)
        joint_state_timer = moveit_node.create_timer(0.1, publish_joint_states_callback)
        print("  ✓ joint_states发布器已启动")
        
        # 等待一下让joint_states开始发布
        print("  ⏳ 等待joint_states发布...")
        for _ in range(10):
            rclpy.spin_once(moveit_node, timeout_sec=0.1)
        
        # 创建action client
        move_group = ActionClient(moveit_node, MoveGroupAction, '/move_action')
        print("  ⏳ 等待MoveIt2 action server...")
        
        timeout = 10.0
        start_time = time.time()
        while not move_group.server_is_ready():
            rclpy.spin_once(moveit_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                print("  ❌ MoveIt2 action server未响应")
                return False
        
        print("  ✓ MoveIt2 action client已连接")
        return True
    else:
        print("  ❌ MoveIt2不可用")
        return False

def test_simple_planning():
    """测试简单规划"""
    print("\n3️⃣ 测试MoveIt2规划...")
    
    try:
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.msg import WorkspaceParameters
        from std_msgs.msg import Header
        from geometry_msgs.msg import Vector3
        
        goal = MoveGroupAction.Goal()
        
        # 设置workspace
        goal.request.workspace_parameters = WorkspaceParameters()
        goal.request.workspace_parameters.header = Header()
        goal.request.workspace_parameters.header.frame_id = "base_link"
        goal.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        goal.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        
        # 设置目标
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 5.0
        
        # 简单目标：joint2移动0.1弧度
        goal.request.goal_constraints = [Constraints()]
        jc = JointConstraint()
        jc.joint_name = "joint2"
        jc.position = 0.1
        jc.tolerance_above = 0.1
        jc.tolerance_below = 0.1
        jc.weight = 1.0
        goal.request.goal_constraints[0].joint_constraints = [jc]
        
        # 只规划
        goal.planning_options.plan_only = True
        
        print("  📤 发送规划目标...")
        future = move_group.send_goal_async(goal)
        
        # 等待接受
        rclpy.spin_until_future_complete(moveit_node, future, timeout_sec=5.0)
        if not future.done():
            print("  ❌ Goal发送超时")
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("  ❌ Goal被拒绝")
            return False
        
        print("  ✅ Goal已接受")
        
        # 等待结果
        print("  ⏳ 等待规划结果...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(moveit_node, result_future, timeout_sec=30.0)
        
        if not result_future.done():
            print("  ❌ 规划超时")
            return False
        
        result = result_future.result()
        if result.result.error_code.val == 1:
            print(f"  ✅ 规划成功！轨迹点数: {len(result.result.planned_trajectory.joint_trajectory.points)}")
            return True
        else:
            print(f"  ❌ 规划失败，错误代码: {result.result.error_code.val}")
            return False
            
    except Exception as e:
        print(f"  ❌ 规划测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    try:
        # 测试初始化
        if not test_initialization():
            print("\n❌ 初始化失败")
            return 1
        
        # 测试规划
        if not test_simple_planning():
            print("\n❌ 规划测试失败")
            return 1
        
        print("\n" + "="*60)
        print("✅ 所有测试通过！button_actions.py已准备好")
        print("="*60)
        return 0
        
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # 清理
        if MOVEIT_AVAILABLE and moveit_node:
            moveit_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())
