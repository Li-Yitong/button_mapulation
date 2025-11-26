#!/usr/bin/env python3
"""测试 MoveIt2 连接"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup

def main():
    print("初始化 ROS2...")
    rclpy.init()
    
    node = Node('test_moveit_connection')
    print("✓ 节点已创建")
    
    # 测试 /move_group action
    print("创建 Action Client: /move_group")
    client = ActionClient(node, MoveGroup, '/move_group')
    print("✓ Action Client 已创建")
    
    print("等待 action server...")
    timeout = 10.0
    import time
    start = time.time()
    
    while not client.server_is_ready():
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.time() - start
        if elapsed > timeout:
            print(f"✗ 超时 ({timeout}s)")
            print("MoveIt2 服务未运行!")
            node.destroy_node()
            rclpy.shutdown()
            return False
        
        if int(elapsed) % 2 == 0 and int(elapsed) > 0:
            print(f"  等待中... {elapsed:.0f}s")
    
    print("✓ MoveIt2 action server 已连接!")
    
    # 清理
    client.destroy()
    node.destroy_node()
    rclpy.shutdown()
    return True

if __name__ == '__main__':
    success = main()
    exit(0 if success else 1)
