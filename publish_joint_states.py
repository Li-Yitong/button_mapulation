#!/usr/bin/env python3
"""
手动发布joint_states来测试MoveIt2
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('manual_joint_state_pub')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        self.get_logger().info('开始发布joint_states...')
        
    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # 发布所有关节的零位
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.velocity = []
        msg.effort = []
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = JointStatePublisher()
    
    print("=" * 60)
    print("手动发布joint_states (所有关节零位)")
    print("按Ctrl+C停止")
    print("=" * 60)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n停止发布")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
