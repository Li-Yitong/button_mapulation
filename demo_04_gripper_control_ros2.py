#!/usr/bin/env python3
"""
Demo 4: 夹爪控制 - ROS2 版本
测试夹爪的开合功能
"""
import rclpy
from rclpy.node import Node
from piper_sdk import C_PiperInterface_V2
import time

class PiperGripperDemo(Node):
    def __init__(self):
        super().__init__('piper_gripper_demo')
        
        # 初始化机械臂
        self.get_logger().info("正在连接机械臂...")
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        
        self.get_logger().info("✓ 机械臂已连接")
    
    def enable_arm(self):
        """使能机械臂（包括夹爪）"""
        self.get_logger().info("正在使能机械臂和夹爪...")
        result = self.piper.EnableArm(7)  # 7 表示包括夹爪
        time.sleep(2)
        
        status = self.piper.GetArmLowSpdInfoMsgs()
        if status.motor_1.foc_status.driver_enable_status == 1:
            self.get_logger().info("✓ 机械臂和夹爪已使能")
            return True
        else:
            self.get_logger().error("✗ 使能失败")
            return False
    
    def disable_arm(self):
        """失能机械臂"""
        self.get_logger().info("正在失能机械臂...")
        self.piper.DisableArm(7)
        time.sleep(1)
        self.get_logger().info("✓ 机械臂已失能")
    
    def get_gripper_position(self):
        """获取夹爪位置"""
        gripper_msg = self.piper.GetArmGripperMsgs()
        # 返回夹爪角度，单位转换：原始值 * 0.001mm
        return gripper_msg.gripper_state.grippers_angle * 0.001
    
    def go_to_zero(self):
        """机械臂所有关节回零"""
        self.get_logger().info("正在移动机械臂到零位...")
        
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
                self.get_logger().info("✓ 机械臂已到达零位")
                return True
            
            time.sleep(0.1)
        
        self.get_logger().warn("⚠ 移动超时，但可能已接近零位")
        return True
    
    def set_gripper_zero(self):
        """设置当前位置为零点"""
        self.get_logger().info("正在设置夹爪零点...")
        
        # 设置当前位置为零点（set_zero=0xAE）
        self.piper.GripperCtrl(0, 1000, 0x01, 0xAE)
        time.sleep(1)
        
        self.get_logger().info("✓ 夹爪零点设置完成")
    
    def set_gripper_position(self, position_mm):
        """设置夹爪到指定位置（单位：mm）"""
        # 转换为 SDK 单位（0.001mm）
        position_raw = int(position_mm * 1000)
        
        # 限制范围 0-70mm
        if position_raw < 0:
            position_raw = 0
            self.get_logger().warn(f"位置超出范围，已限制为 0mm")
        elif position_raw > 70000:
            position_raw = 70000
            self.get_logger().warn(f"位置超出范围，已限制为 70mm")
        
        self.piper.GripperCtrl(position_raw, 1000, 0x01, 0)
        time.sleep(1.5)
        
        current_pos = self.get_gripper_position()
        self.get_logger().info(f"目标: {position_mm:.1f}mm, 实际: {current_pos:.3f}mm")
    
    def demo_sequence(self):
        """演示夹爪控制流程"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("开始演示夹爪控制功能")
        self.get_logger().info("="*60)
        
        # 1. 使能机械臂
        self.get_logger().info("\n[步骤 1] 使能机械臂和夹爪")
        if not self.enable_arm():
            return False
        
        # 2. 机械臂回零
        self.get_logger().info("\n[步骤 2] 机械臂回零")
        self.go_to_zero()
        time.sleep(1)
        
        # 3. 设置夹爪零点
        self.get_logger().info("\n[步骤 3] 设置夹爪零点")
        self.get_logger().info("提示: 当前夹爪位置将被设为 0mm")
        self.set_gripper_zero()
        
        # 4. 显示当前位置
        current_pos = self.get_gripper_position()
        self.get_logger().info(f"\n当前夹爪位置: {current_pos:.3f}mm")
        
        # 5. 循环接受用户输入
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("进入交互模式")
        self.get_logger().info("="*60)
        print("\n提示:")
        print("  - 输入张开值（0-70mm）来控制夹爪")
        print("  - 按 Ctrl+C 退出程序")
        print("  - 范围: 0mm (完全关闭) ~ 70mm (完全打开)")
        print("")
        
        while True:
            try:
                # 读取用户输入
                user_input = input("请输入目标位置 (mm): ").strip()
                
                if not user_input:
                    continue
                
                try:
                    position = float(user_input)
                    self.set_gripper_position(position)
                except ValueError:
                    print("错误: 请输入有效的数字")
                    
            except EOFError:
                # 处理 Ctrl+D
                break
        
        return True

def main(args=None):
    print("\n" + "="*60)
    print("Demo 4: 夹爪交互控制")
    print("="*60)
    print("功能: 设置零点后，通过输入数值控制夹爪张开")
    print("夹爪范围: 0mm (完全关闭) ~ 70mm (完全打开)")
    print("警告: 确保夹爪附近无障碍物！")
    print("注意: 按 Ctrl+C 退出程序，机械臂保持使能状态")
    print("="*60 + "\n")
    
    # 等待用户确认
    try:
        input("按 Enter 键开始演示（Ctrl+C 取消）...")
    except KeyboardInterrupt:
        print("\n演示已取消")
        return
    
    rclpy.init(args=args)
    node = PiperGripperDemo()
    
    try:
        # 运行演示序列
        node.demo_sequence()
        
    except KeyboardInterrupt:
        print("\n\n程序退出")
        print("机械臂保持使能状态")
    except Exception as e:
        node.get_logger().error(f"发生错误: {e}")
    finally:
        # 保持使能状态，不自动失能
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
