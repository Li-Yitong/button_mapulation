#!/usr/bin/env python3
"""
Demo 2: 使能和失能机械臂 - ROS2 版本
测试机械臂的使能控制
"""
import rclpy
from rclpy.node import Node
from piper_sdk import C_PiperInterface_V2
import time

class PiperEnableDemo(Node):
    def __init__(self):
        super().__init__('piper_enable_demo')
        
        # 初始化机械臂
        self.get_logger().info("正在连接机械臂...")
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        
        self.get_logger().info("✓ 机械臂已连接")
    
    def enable_arm(self):
        """使能机械臂"""
        self.get_logger().info("正在使能机械臂...")
        
        # 使能所有关节（1-6）和夹爪（7）
        result = self.piper.EnableArm(7)
        
        # 等待使能完成
        time.sleep(2)
        
        # 检查使能状态
        status = self.piper.GetArmLowSpdInfoMsgs()
        
        if status.motor_1.foc_status.driver_enable_status == 1:
            self.get_logger().info("✓ 机械臂已使能")
            return True
        else:
            self.get_logger().error("✗ 机械臂使能失败")
            return False
    
    def disable_arm(self):
        """失能机械臂"""
        self.get_logger().info("正在失能机械臂...")
        
        result = self.piper.DisableArm(7)
        time.sleep(1)
        
        self.get_logger().info("✓ 机械臂已失能")
        return True
    
    def demo_sequence(self):
        """演示使能/失能流程"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("开始演示使能/失能流程")
        self.get_logger().info("="*60)
        
        # 1. 使能机械臂
        self.get_logger().info("\n[步骤 1] 使能机械臂")
        if not self.enable_arm():
            return False
        
        self.get_logger().info("\n机械臂已使能，等待 3 秒...")
        time.sleep(3)
        
        # 2. 失能机械臂
        self.get_logger().info("\n[步骤 2] 失能机械臂")
        self.disable_arm()
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("演示完成！")
        self.get_logger().info("="*60)
        
        return True

def main(args=None):
    print("\n" + "="*60)
    print("Demo 2: 机械臂使能/失能控制")
    print("="*60)
    print("功能: 测试机械臂的使能和失能功能")
    print("="*60 + "\n")
    
    rclpy.init(args=args)
    node = PiperEnableDemo()
    
    try:
        # 运行演示序列
        node.demo_sequence()
        
    except KeyboardInterrupt:
        print("\n程序中断")
    except Exception as e:
        node.get_logger().error(f"发生错误: {e}")
    finally:
        # 确保失能机械臂
        try:
            node.disable_arm()
        except:
            pass
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
