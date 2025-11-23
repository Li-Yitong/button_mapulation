#!/usr/bin/env python3
"""
Demo 1: 读取机械臂状态 - ROS2 版本
测试机械臂连接和状态读取
"""
import rclpy
from rclpy.node import Node
from piper_sdk import C_PiperInterface_V2
import time

class PiperStatusReader(Node):
    def __init__(self):
        super().__init__('piper_status_reader')
        
        # 初始化机械臂
        self.get_logger().info("正在连接机械臂...")
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        
        # 创建定时器，1Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("✓ 机械臂已连接，开始读取状态...")
    
    def timer_callback(self):
        """读取并显示机械臂状态"""
        try:
            # 读取关节状态
            msg = self.piper.GetArmJointMsgs()
            
            # 读取末端位姿（修正方法名）
            end_pose = self.piper.GetArmEndPoseMsgs()
            
            self.get_logger().info("="*60)
            self.get_logger().info("关节角度 (度):")
            self.get_logger().info(f"  J1: {msg.joint_state.joint_1 * 1e-3:.2f}°")
            self.get_logger().info(f"  J2: {msg.joint_state.joint_2 * 1e-3:.2f}°")
            self.get_logger().info(f"  J3: {msg.joint_state.joint_3 * 1e-3:.2f}°")
            self.get_logger().info(f"  J4: {msg.joint_state.joint_4 * 1e-3:.2f}°")
            self.get_logger().info(f"  J5: {msg.joint_state.joint_5 * 1e-3:.2f}°")
            self.get_logger().info(f"  J6: {msg.joint_state.joint_6 * 1e-3:.2f}°")
            
            if hasattr(msg.joint_state, 'joint_7'):
                gripper = msg.joint_state.joint_7 * 1e-3
                self.get_logger().info(f"  夹爪: {gripper:.4f}m")
            
            self.get_logger().info(f"\n末端位姿:")
            self.get_logger().info(f"  X: {end_pose.end_pose.X_axis:.4f}m")
            self.get_logger().info(f"  Y: {end_pose.end_pose.Y_axis:.4f}m")
            self.get_logger().info(f"  Z: {end_pose.end_pose.Z_axis:.4f}m")
            self.get_logger().info(f"  Rx: {end_pose.end_pose.RX_axis:.4f}rad")
            self.get_logger().info(f"  Ry: {end_pose.end_pose.RY_axis:.4f}rad")
            self.get_logger().info(f"  Rz: {end_pose.end_pose.RZ_axis:.4f}rad")
            
        except Exception as e:
            self.get_logger().error(f"读取状态失败: {e}")

def main(args=None):
    print("\n" + "="*60)
    print("Demo 1: 机械臂状态读取")
    print("="*60)
    print("功能: 实时读取机械臂关节角度和末端位姿")
    print("按 Ctrl+C 退出")
    print("="*60 + "\n")
    
    rclpy.init(args=args)
    node = PiperStatusReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n程序退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
