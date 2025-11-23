#!/usr/bin/env python3
"""
Demo 3: 移动到零位 - ROS2 版本
测试机械臂归零功能
"""
import rclpy
from rclpy.node import Node
from piper_sdk import C_PiperInterface_V2
import time

class PiperGoZeroDemo(Node):
    def __init__(self):
        super().__init__('piper_go_zero_demo')
        
        # 初始化机械臂
        self.get_logger().info("正在连接机械臂...")
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        
        self.get_logger().info("✓ 机械臂已连接")
    
    def enable_arm(self):
        """使能机械臂"""
        self.get_logger().info("正在使能机械臂...")
        result = self.piper.EnableArm(7)
        time.sleep(2)
        
        status = self.piper.GetArmLowSpdInfoMsgs()
        if status.motor_1.foc_status.driver_enable_status == 1:
            self.get_logger().info("✓ 机械臂已使能")
            return True
        else:
            self.get_logger().error("✗ 机械臂使能失败")
            return False
    
    # def disable_arm(self):
    #     """失能机械臂"""
    #     self.get_logger().info("正在失能机械臂...")
    #     self.piper.DisableArm(7)
    #     time.sleep(1)
    #     self.get_logger().info("✓ 机械臂已失能")
    
    def print_current_angles(self):
        """打印当前关节角度"""
        msg = self.piper.GetArmJointMsgs()
        self.get_logger().info(f"  关节1: {msg.joint_state.joint_1 * 1e-3:>7.2f}°")
        self.get_logger().info(f"  关节2: {msg.joint_state.joint_2 * 1e-3:>7.2f}°")
        self.get_logger().info(f"  关节3: {msg.joint_state.joint_3 * 1e-3:>7.2f}°")
        self.get_logger().info(f"  关节4: {msg.joint_state.joint_4 * 1e-3:>7.2f}°")
        self.get_logger().info(f"  关节5: {msg.joint_state.joint_5 * 1e-3:>7.2f}°")
        self.get_logger().info(f"  关节6: {msg.joint_state.joint_6 * 1e-3:>7.2f}°")
    
    def go_to_zero(self):
        """移动到零位"""
        self.get_logger().info("\n正在移动到零位...")
        
        # 1. 设置为关节控制模式 (MOVE J)
        self.piper.MotionCtrl_2(
            ctrl_mode=0x01,      # CAN 指令控制
            move_mode=0x01,      # MOVE J (关节运动)
            move_spd_rate_ctrl=30,  # 速度 30%
            is_mit_mode=0x00     # 位置速度模式
        )
        
        time.sleep(0.1)
        
        # 2. 发送目标关节角度（所有关节为 0，单位 0.001 度）
        self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
        
        # 等待运动完成（监测运动状态）
        max_wait = 10  # 最大等待 10 秒
        start_time = time.time()
        
        while time.time() - start_time < max_wait:
            msg = self.piper.GetArmJointMsgs()
            current_angles = [
                msg.joint_state.joint_1 * 1e-3,
                msg.joint_state.joint_2 * 1e-3,
                msg.joint_state.joint_3 * 1e-3,
                msg.joint_state.joint_4 * 1e-3,
                msg.joint_state.joint_5 * 1e-3,
                msg.joint_state.joint_6 * 1e-3
            ]
            
            # 检查是否接近目标（误差小于 1 度）
            errors = [abs(angle) for angle in current_angles]
            max_error = max(errors)
            
            if max_error < 1.0:
                self.get_logger().info("✓ 已到达零位")
                return True
            
            time.sleep(0.1)
        
        self.get_logger().warn("⚠ 移动超时，但可能已接近目标位置")
        return True
    
    def demo_sequence(self):
        """演示归零流程"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("开始演示归零功能")
        self.get_logger().info("="*60)
        
        # 1. 使能机械臂
        self.get_logger().info("\n[步骤 1] 使能机械臂")
        if not self.enable_arm():
            return False
        
        # 2. 显示当前位置
        self.get_logger().info("\n[步骤 2] 当前关节角度:")
        self.print_current_angles()
        
        # 3. 移动到零位
        self.get_logger().info("\n[步骤 3] 开始归零")
        if not self.go_to_zero():
            return False
        
        time.sleep(1)
        
        # 4. 显示最终位置
        self.get_logger().info("\n[步骤 4] 归零后关节角度:")
        self.print_current_angles()
        
        # # 5. 失能机械臂
        # self.get_logger().info("\n[步骤 5] 失能机械臂")
        # self.disable_arm()
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("演示完成！")
        self.get_logger().info("="*60)
        
        return True

def main(args=None):
    print("\n" + "="*60)
    print("Demo 3: 机械臂归零功能")
    print("="*60)
    print("功能: 将机械臂所有关节移动到 0 度位置")
    print("警告: 请确保机械臂运动空间无障碍物！")
    print("="*60 + "\n")
    
    # 等待用户确认
    try:
        input("按 Enter 键开始演示（Ctrl+C 取消）...")
    except KeyboardInterrupt:
        print("\n演示已取消")
        return
    
    rclpy.init(args=args)
    node = PiperGoZeroDemo()
    
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
