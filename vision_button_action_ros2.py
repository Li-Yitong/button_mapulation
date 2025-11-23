#!/usr/bin/env python3
"""
视觉按钮操作整合器 - ROS2 版本
整合视觉检测 + button_actions.py 的动作执行
"""
import rclpy
from rclpy.node import Node
import sys
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker

# 导入必需的库
from piper_sdk import C_PiperInterface_V2
from piper_arm import PiperArm
import math

# 导入 button_actions.py 的功能
import button_actions
from button_actions import (
    action_toggle, action_plugin, action_push, action_knob,
    PI, factor
)


class VisionButtonActionNode(Node):
    """视觉按钮操作节点"""
    
    def __init__(self, enable_moveit=False):
        super().__init__('vision_button_action_ros2')
        
        self.get_logger().info("="*70)
        self.get_logger().info("视觉按钮操作整合器 - ROS2 版本")
        if enable_moveit:
            self.get_logger().info("🤖 MoveIt2 模式已启用")
        else:
            self.get_logger().info("🔧 SDK 直接控制模式")
        self.get_logger().info("="*70)
        
        # 保存 MoveIt 启用标志
        self.enable_moveit = enable_moveit
        
        # 全局变量
        self.receive_button_center = False
        self.button_center = None
        self.button_type = None
        
        # 初始化硬件
        if not self.initialize_hardware():
            self.get_logger().error("硬件初始化失败，退出程序")
            raise RuntimeError("Hardware initialization failed")
        
        # 创建订阅器
        self.sub_object_point = self.create_subscription(
            PointStamped,
            '/object_point',
            self.object_point_callback,
            10
        )
        
        self.sub_button_type = self.create_subscription(
            String,
            '/button_type',
            self.button_type_callback,
            10
        )
        
        # 创建发布器（用于可视化）
        self.pub_target_marker = self.create_publisher(
            Marker,
            '/target_button_base',
            10
        )
        
        self.get_logger().info("✓ ROS2 节点已初始化")
        self.get_logger().info("✓ 等待接收按钮信息...")
        self.get_logger().info("  订阅话题:")
        self.get_logger().info("    - /object_point (按钮3D位置)")
        self.get_logger().info("    - /button_type (按钮类型)")
        self.get_logger().info("="*70)
        
        # 创建定时器检查是否可以执行动作
        self.timer = self.create_timer(0.1, self.check_and_execute)
    
    def initialize_hardware(self):
        """初始化机械臂硬件"""
        self.get_logger().info("\n初始化硬件...")
        
        # 1. 初始化 Piper SDK
        self.get_logger().info("[1/2] 初始化 Piper SDK...")
        try:
            self.piper = C_PiperInterface_V2("can0")
            self.piper.ConnectPort()
            self.piper.EnableArm(7)
            
            from utils.utils_piper import enable_fun
            enable_fun(piper=self.piper)
            
            self.get_logger().info("  ✓ Piper SDK 初始化成功")
        except Exception as e:
            self.get_logger().error(f"  ✗ Piper SDK 初始化失败: {e}")
            return False
        
        # 2. 初始化运动学模块
        self.get_logger().info("[2/2] 初始化 PiperArm...")
        try:
            self.piper_arm = PiperArm()
            self.get_logger().info("  ✓ PiperArm 初始化成功")
        except Exception as e:
            self.get_logger().error(f"  ✗ PiperArm 初始化失败: {e}")
            return False
        
        # 3. 更新 button_actions 的全局对象
        button_actions.piper = self.piper
        button_actions.piper_arm = self.piper_arm
        button_actions.moveit_node = self  # 传递 ROS2 节点
        
        # 4. 初始化 MoveIt2 (如果启用)
        if self.enable_moveit:
            self.get_logger().info("[3/3] 初始化 MoveIt2...")
            if not self.initialize_moveit():
                self.get_logger().warn("  ⚠️ MoveIt2 初始化失败，将使用 SDK 模式")
                button_actions.USE_MOVEIT = False
            else:
                self.get_logger().info("  ✓ MoveIt2 初始化成功")
                button_actions.USE_MOVEIT = True
        else:
            button_actions.USE_MOVEIT = False
        
        self.get_logger().info("✓ 硬件初始化完成")
        return True
    
    def initialize_moveit(self):
        """初始化 MoveIt2 action client"""
        try:
            # 导入 MoveIt2 相关模块
            from moveit_msgs.action import MoveGroup as MoveGroupAction
            from rclpy.action import ActionClient
            
            # 创建 MoveGroup action client
            self._moveit_action_client = ActionClient(
                self,
                MoveGroupAction,
                '/move_action'
            )
            
            # 等待 move_group action server
            self.get_logger().info("  等待 move_group action server...")
            if not self._moveit_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("  ✗ move_group action server 未响应")
                return False
            
            # 传递 action client 给 button_actions
            button_actions.move_group = self._moveit_action_client
            
            self.get_logger().info("  ✓ MoveIt2 action client 已连接")
            return True
            
        except Exception as e:
            self.get_logger().error(f"  ✗ MoveIt2 初始化错误: {e}")
            return False
    
    def object_point_callback(self, msg):
        """接收按钮的 3D 位置（相机坐标系）"""
        if np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z):
            return
        
        self.button_center = [msg.point.x, msg.point.y, msg.point.z]
        self.receive_button_center = True
        
        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info(f"接收到按钮位置 (相机坐标系):")
        self.get_logger().info(f"  X: {self.button_center[0]:.4f} m")
        self.get_logger().info(f"  Y: {self.button_center[1]:.4f} m")
        self.get_logger().info(f"  Z: {self.button_center[2]:.4f} m")
        self.get_logger().info(f"{'='*70}\n")
    
    def button_type_callback(self, msg):
        """接收按钮类型"""
        self.button_type = msg.data.lower()
        self.get_logger().info(f"接收到按钮类型: {self.button_type}")
    
    def get_current_joints(self):
        """获取当前关节角度"""
        msg = self.piper.GetArmJointMsgs()
        
        theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
        theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
        theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
        theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
        theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
        theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0
        
        return [theta1, theta2, theta3, theta4, theta5, theta6]
    
    def transform_camera_to_base(self, button_center_camera, current_joints):
        """将相机坐标系的按钮位置转换到基座坐标系"""
        from utils.utils_math import quaternion_to_rotation_matrix
        
        # 末端到基座的变换
        base_T_link6 = self.piper_arm.forward_kinematics(current_joints)
        
        # 相机到末端的变换
        link6_T_cam = np.eye(4)
        link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(self.piper_arm.link6_q_camera)
        link6_T_cam[:3, 3] = self.piper_arm.link6_t_camera
        
        # 按钮在相机坐标系的齐次坐标
        button_cam_homogeneous = np.array([
            button_center_camera[0],
            button_center_camera[1],
            button_center_camera[2],
            1.0
        ])
        
        # 转换到基座坐标系
        button_base = base_T_link6 @ link6_T_cam @ button_cam_homogeneous
        
        return button_base
    
    def apply_tcp_offset(self, button_base, current_joints, tcp_offset_local):
        """应用夹爪自身坐标系的TCP偏移"""
        # 获取末端姿态
        base_T_link6 = self.piper_arm.forward_kinematics(current_joints)
        
        # 提取旋转矩阵
        R_base_link6 = base_T_link6[:3, :3]
        
        # 将夹爪坐标系的偏移量转换到基座坐标系
        offset_base = R_base_link6 @ np.array(tcp_offset_local)
        
        # 应用偏移
        target_base = button_base.copy()
        target_base[:3] = button_base[:3] - offset_base
        
        return target_base
    
    def publish_marker(self, position, frame_id="arm_base"):
        """发布可视化标记"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_button"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.pub_target_marker.publish(marker)
    
    def execute_button_action(self, button_center_camera, button_type_str):
        """执行按钮操作"""
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("开始执行按钮操作")
        self.get_logger().info("="*70)
        
        # 1. 获取当前关节角度
        self.get_logger().info("[步骤 1/5] 获取当前关节角度...")
        current_joints = self.get_current_joints()
        self.get_logger().info(f"  当前关节角度 (度): {np.array(current_joints) * 180 / PI}")
        
        # 2. 坐标转换
        self.get_logger().info("[步骤 2/5] 坐标转换: 相机坐标系 → 基座坐标系...")
        button_base = self.transform_camera_to_base(button_center_camera, current_joints)
        self.get_logger().info(f"  相机坐标系: ({button_center_camera[0]:.4f}, {button_center_camera[1]:.4f}, {button_center_camera[2]:.4f})")
        self.get_logger().info(f"  基座坐标系: ({button_base[0]:.4f}, {button_base[1]:.4f}, {button_base[2]:.4f})")
        
        # 发布可视化标记
        self.publish_marker(button_base[:3])
        
        # 3. 应用TCP偏移补偿
        self.get_logger().info("[步骤 3/5] 应用TCP偏移补偿...")
        TCP_OFFSET_LOCAL = np.array([-0.018, 0.007, 0.063])
        target_base = self.apply_tcp_offset(button_base, current_joints, TCP_OFFSET_LOCAL)
        
        self.get_logger().info(f"  按钮位置 (基座系): ({button_base[0]:.4f}, {button_base[1]:.4f}, {button_base[2]:.4f})")
        self.get_logger().info(f"  TCP偏移 (夹爪系): ({TCP_OFFSET_LOCAL[0]:.4f}, {TCP_OFFSET_LOCAL[1]:.4f}, {TCP_OFFSET_LOCAL[2]:.4f})")
        self.get_logger().info(f"  目标位置 (基座系): ({target_base[0]:.4f}, {target_base[1]:.4f}, {target_base[2]:.4f})")
        
        # 4. 更新 button_actions 的目标位置
        self.get_logger().info("[步骤 4/5] 更新 button_actions 目标位置...")
        button_actions.TARGET_X = target_base[0]
        button_actions.TARGET_Y = target_base[1]
        button_actions.TARGET_Z = target_base[2]
        
        self.get_logger().info(f"  ✓ 已更新 TARGET_X = {button_actions.TARGET_X:.4f}")
        self.get_logger().info(f"  ✓ 已更新 TARGET_Y = {button_actions.TARGET_Y:.4f}")
        self.get_logger().info(f"  ✓ 已更新 TARGET_Z = {button_actions.TARGET_Z:.4f}")
        
        # 5. 根据按钮类型调用对应的动作函数
        self.get_logger().info(f"[步骤 5/5] 执行动作: {button_type_str.upper()}...")
        
        action_map = {
            'toggle': action_toggle,
            'plugin': action_plugin,
            'push': action_push,
            'knob': action_knob
        }
        
        if button_type_str not in action_map:
            self.get_logger().error(f"未知的按钮类型 '{button_type_str}'")
            self.get_logger().error(f"支持的类型: {list(action_map.keys())}")
            return False
        
        # 调用动作函数
        try:
            success = action_map[button_type_str]()
            
            if success:
                self.get_logger().info("\n" + "="*70)
                self.get_logger().info("✓✓✓ 按钮操作执行成功！✓✓✓")
                self.get_logger().info("="*70 + "\n")
            else:
                self.get_logger().warn("\n" + "="*70)
                self.get_logger().warn("✗✗✗ 按钮操作执行失败 ✗✗✗")
                self.get_logger().warn("="*70 + "\n")
            
            return success
        
        except Exception as e:
            self.get_logger().error(f"动作执行错误: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def check_and_execute(self):
        """定时器回调：检查是否可以执行动作"""
        # 检查是否同时接收到位置和类型
        if self.receive_button_center and self.button_type is not None:
            self.get_logger().info(f"\n{'='*70}")
            self.get_logger().info(f"准备执行操作:")
            self.get_logger().info(f"  按钮类型: {self.button_type}")
            self.get_logger().info(f"  按钮位置: {self.button_center}")
            self.get_logger().info(f"{'='*70}")
            
            # 执行动作
            success = self.execute_button_action(self.button_center, self.button_type)
            
            # 重置标志
            self.receive_button_center = False
            self.button_type = None
            self.button_center = None
            
            if success:
                self.get_logger().info("等待下一个按钮...")
            else:
                self.get_logger().warn("操作失败，等待下一个按钮...")


def main(args=None):
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='视觉按钮操作整合器')
    parser.add_argument('--moveit', action='store_true', 
                       help='启用 MoveIt2 规划（需要先启动 MoveIt2）')
    cli_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    try:
        node = VisionButtonActionNode(enable_moveit=cli_args.moveit)
        
        print("\n" + "="*70)
        if cli_args.moveit:
            print("🤖 MoveIt2 模式已启用")
            print("   - 使用碰撞检测和路径规划")
            print("   - 需要先启动: ./start_moveit2.sh --background")
        else:
            print("🔧 SDK 直接控制模式")
            print("   - 使用参数: --moveit 启用 MoveIt2")
        print("提示: 在 realsense_yolo_button_interactive_ros2 窗口中:")
        print("  1. 点击选择按钮")
        print("  2. 按 ENTER 确认")
        print("="*70 + "\n")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断，退出程序")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
