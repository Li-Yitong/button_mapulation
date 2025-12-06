#!/usr/bin/env python3
"""
Piper TF Publisher - ROS2 版本
发布机械臂各关节和末端的TF坐标变换
"""
import rclpy
from rclpy.node import Node
from piper_sdk import C_PiperInterface_V2
import numpy as np
import math
from piper_arm import PiperArm
from utils.utils_ros import publish_tf_ros2

PI = math.pi


class PiperTFPublisher(Node):
    """Piper机械臂TF发布节点"""
    
    def __init__(self):
        super().__init__('piper_tf_publisher')
        
        self.get_logger().info("="*70)
        self.get_logger().info("Piper TF Publisher - ROS2 版本")
        self.get_logger().info("="*70)
        
        # 声明参数：是否发布相机TF（手眼标定时设为False）
        self.declare_parameter('publish_camera_tf', True)
        self.publish_camera = self.get_parameter('publish_camera_tf').get_parameter_value().bool_value
        
        if not self.publish_camera:
            self.get_logger().warn("⚠️  相机TF发布已禁用（手眼标定模式）")
        
        # 初始化 Piper SDK
        try:
            self.piper = C_PiperInterface_V2()
            self.piper.ConnectPort()
            self.get_logger().info("✓ Piper SDK 初始化成功")
        except Exception as e:
            self.get_logger().error(f"✗ Piper SDK 初始化失败: {e}")
            raise
        
        # 初始化运动学模块
        try:
            self.piper_arm = PiperArm()
            self.get_logger().info("✓ PiperArm 运动学模块初始化成功")
        except Exception as e:
            self.get_logger().error(f"✗ PiperArm 初始化失败: {e}")
            raise
        
        # 创建定时器（10Hz）
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("✓ TF 发布器已启动 (10Hz)")
        self.get_logger().info("="*70)
    
    def timer_callback(self):
        """定时器回调：读取关节角度并发布TF"""
        try:
            # 读取当前关节角度
            msg = self.piper.GetArmJointMsgs()
            
            # 转换为弧度
            theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
            theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
            theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
            theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
            theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
            theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0
            
            joints = [theta1, theta2, theta3, theta4, theta5, theta6]
            
            # 发布TF变换（根据参数决定是否发布相机TF）
            time_now = self.get_clock().now().to_msg()
            publish_tf_ros2(self, self.piper_arm, joints, time_now, publish_camera=self.publish_camera)
            
        except Exception as e:
            self.get_logger().error(f"TF发布错误: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PiperTFPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断，退出程序")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
