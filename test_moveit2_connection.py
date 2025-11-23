#!/usr/bin/env python3
"""
简单的 MoveIt2 连接测试
不需要机器人运动，只检查 MoveIt2 是否可以连接
"""

import sys
import os
import time

def test_moveit2_connection():
    """测试 MoveIt2 连接"""
    print("="*70)
    print("  MoveIt2 连接测试")
    print("="*70)
    
    # 1. 检查 ROS2 环境
    print("\n[1/6] 检查 ROS2 环境...")
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print(f"  ✓ ROS_DISTRO = {ros_distro}")
    else:
        print("  ❌ ROS_DISTRO 未设置")
        print("     请运行: source /opt/ros/foxy/setup.bash")
        return False
    
    # 2. 导入 ROS2
    print("\n[2/6] 导入 rclpy...")
    try:
        import rclpy
        print("  ✓ rclpy 导入成功")
    except ImportError as e:
        print(f"  ❌ rclpy 导入失败: {e}")
        return False
    
    # 3. 导入 MoveIt2 消息
    print("\n[3/6] 导入 MoveIt2 消息...")
    try:
        from moveit_msgs.action import MoveGroup as MoveGroupAction
        from moveit_msgs.msg import Constraints, JointConstraint
        print("  ✓ moveit_msgs 导入成功")
    except ImportError as e:
        print(f"  ❌ moveit_msgs 导入失败: {e}")
        return False
    
    # 4. 初始化 ROS2
    print("\n[4/6] 初始化 ROS2 节点...")
    try:
        rclpy.init()
        node = rclpy.create_node('moveit2_test_node')
        print("  ✓ 节点创建成功")
    except Exception as e:
        print(f"  ❌ 节点创建失败: {e}")
        return False
    
    # 5. 创建 Action Client
    print("\n[5/6] 创建 MoveIt2 Action Client...")
    try:
        from rclpy.action import ActionClient
        
        action_client = ActionClient(
            node,
            MoveGroupAction,
            '/move_action'
        )
        print("  ✓ Action Client 创建成功")
        
        # 6. 检查服务器连接
        print("\n[6/6] 等待 MoveIt2 Action Server...")
        print("  (超时时间: 5 秒)")
        
        server_available = action_client.wait_for_server(timeout_sec=5.0)
        
        if server_available:
            print("  ✓ MoveIt2 Action Server 已连接！")
            print("\n" + "="*70)
            print("  🎉 MoveIt2 连接测试通过！")
            print("="*70)
            print("\n可以使用的功能:")
            print("  - control_arm_moveit() 函数")
            print("  - MoveIt2 运动规划")
            print("  - 轨迹执行")
            
            # 清理
            node.destroy_node()
            rclpy.shutdown()
            return True
        else:
            print("  ❌ MoveIt2 Action Server 未响应")
            print("\n可能的原因:")
            print("  1. MoveIt2 未启动")
            print("  2. Action server 名称不匹配")
            print("  3. 网络/通信问题")
            print("\n解决方案:")
            print("  启动 MoveIt2:")
            print("    cd /home/robot/button/V4.0/project2")
            print("    ./start_moveit2.sh --background")
            print("\n  检查 action 列表:")
            print("    ros2 action list")
            
            # 清理
            node.destroy_node()
            rclpy.shutdown()
            return False
            
    except Exception as e:
        print(f"  ❌ Action Client 创建失败: {e}")
        import traceback
        traceback.print_exc()
        
        # 清理
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass
        
        return False


def main():
    """主函数"""
    success = test_moveit2_connection()
    
    if success:
        print("\n下一步:")
        print("  1. 测试完整功能: python3 test_moveit2_realtime.py")
        print("  2. 启动视觉系统: ./start_vision_button_moveit.sh --moveit")
        return 0
    else:
        print("\n请先解决上述问题，然后重新测试")
        return 1


if __name__ == "__main__":
    sys.exit(main())
