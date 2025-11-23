#!/usr/bin/env python3
"""
测试 MoveIt2 集成
验证能否通过 MoveIt2 控制 Piper 机械臂
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
import sys

class MoveIt2TestNode(Node):
    def __init__(self):
        super().__init__('moveit2_test_node')
        
        self.get_logger().info("="*70)
        self.get_logger().info("MoveIt2 集成测试")
        self.get_logger().info("="*70)
        
        # 创建 MoveGroup action 客户端
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        self.get_logger().info("等待 MoveGroup action 服务器...")
        if self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("✓ MoveGroup action 服务器已连接")
        else:
            self.get_logger().error("✗ MoveGroup action 服务器未响应")
            self.get_logger().error("  请确保 MoveIt2 已启动:")
            self.get_logger().error("  ros2 launch piper_with_gripper_moveit demo.launch.py")
            sys.exit(1)
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_state = None
        self.get_logger().info("等待关节状态...")
    
    def joint_state_callback(self, msg):
        """接收关节状态"""
        if self.current_joint_state is None:
            self.get_logger().info(f"✓ 接收到关节状态: {len(msg.name)} 个关节")
        self.current_joint_state = msg
    
    def send_goal(self, joint_values):
        """发送目标关节角度到 MoveIt2"""
        if self.current_joint_state is None:
            self.get_logger().error("✗ 尚未接收到关节状态")
            return False
        
        self.get_logger().info("\n发送运动规划请求...")
        self.get_logger().info(f"  目标关节角度: {joint_values}")
        
        # 创建运动规划请求
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # 设置目标关节约束
        goal_msg.request.goal_constraints.append(Constraints())
        
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        for i, (name, value) in enumerate(zip(joint_names, joint_values)):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = value
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            goal_msg.request.goal_constraints[0].joint_constraints.append(constraint)
        
        # 发送目标
        self.get_logger().info("  发送 action goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("✗ Goal 被拒绝")
            return False
        
        self.get_logger().info("✓ Goal 已接受，等待结果...")
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("✓✓✓ 运动规划成功！✓✓✓")
            return True
        else:
            self.get_logger().error(f"✗ 运动规划失败，错误码: {result.result.error_code.val}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveIt2TestNode()
    
    # 等待接收关节状态
    import time
    timeout = 5.0
    start_time = time.time()
    while node.current_joint_state is None and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if node.current_joint_state is None:
        node.get_logger().error("✗ 未接收到关节状态")
        node.get_logger().error("  请确保机械臂硬件已连接并启动")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # 测试：移动到零位
    print("\n" + "="*70)
    print("测试 1: 移动到零位")
    print("="*70)
    success = node.send_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    if success:
        print("\n" + "="*70)
        print("✓✓✓ MoveIt2 集成测试成功！✓✓✓")
        print("="*70)
        print("\n现在您可以:")
        print("  1. 在 button_actions.py 中使用 MoveIt2")
        print("  2. 启动完整的视觉按钮操作系统:")
        print("     ./start_vision_button_ros2.sh --moveit --rviz")
    else:
        print("\n" + "="*70)
        print("✗ MoveIt2 集成测试失败")
        print("="*70)
        print("请检查:")
        print("  1. MoveIt2 是否正确启动")
        print("  2. 机械臂硬件是否连接")
        print("  3. 查看终端日志获取更多信息")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
