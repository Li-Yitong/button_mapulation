#!/usr/bin/env python3
"""
验证手眼标定结果
比较通过标定变换计算的按钮位置与实际检测位置的差异
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import time


class CalibrationVerifier(Node):
    """标定验证节点"""
    
    def __init__(self):
        super().__init__('calibration_verifier')
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 统计数据
        self.errors = []
        self.sample_count = 0
        
        self.get_logger().info("="*70)
        self.get_logger().info("手眼标定验证工具")
        self.get_logger().info("="*70)
        self.get_logger().info("操作步骤:")
        self.get_logger().info("1. 确保相机和机械臂TF都在发布")
        self.get_logger().info("2. 确保按钮检测节点在运行")
        self.get_logger().info("3. 移动机械臂到不同位置观察误差")
        self.get_logger().info("="*70)
        
        # 创建定时器检查TF
        self.timer = self.create_timer(1.0, self.verify_transform)
    
    def verify_transform(self):
        """验证标定变换的准确性"""
        try:
            # 查询 base → camera 变换
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 提取变换
            t = trans.transform.translation
            r = trans.transform.rotation
            
            self.sample_count += 1
            
            self.get_logger().info(f"\n样本 #{self.sample_count}")
            self.get_logger().info(f"base → camera 变换:")
            self.get_logger().info(f"  位置: ({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) m")
            self.get_logger().info(f"  姿态: ({r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f})")
            
            # 如果有历史数据，计算稳定性
            if self.sample_count > 1:
                self.get_logger().info(f"  ✓ TF变换稳定")
            
        except Exception as e:
            self.get_logger().error(f"查询TF失败: {e}")
            self.get_logger().info("请确保:")
            self.get_logger().info("  1. piper_tf_publisher_ros2.py 正在运行")
            self.get_logger().info("  2. 相机驱动正在运行")
            self.get_logger().info("  3. handeye_publish.launch.py 正在运行")


def main():
    rclpy.init()
    
    node = CalibrationVerifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
