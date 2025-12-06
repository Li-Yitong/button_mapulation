#!/usr/bin/env python3
"""
手动手眼标定工具 - 兼容 ROS2 Foxy
不依赖 easy_handeye2 GUI，直接采集样本并计算
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import cv2
import yaml
from pathlib import Path
import time

class ManualHandeyeCalibration(Node):
    def __init__(self):
        super().__init__('manual_handeye_calibration')
        
        # 参数
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('robot_effector_frame', 'link6')
        self.declare_parameter('tracking_base_frame', 'camera_color_optical_frame')
        self.declare_parameter('tracking_marker_frame', 'checkerboard')
        self.declare_parameter('calibration_name', 'piper_realsense_handeye')
        
        self.robot_base = self.get_parameter('robot_base_frame').value
        self.robot_effector = self.get_parameter('robot_effector_frame').value
        self.camera_frame = self.get_parameter('tracking_base_frame').value
        self.marker_frame = self.get_parameter('tracking_marker_frame').value
        self.calib_name = self.get_parameter('calibration_name').value
        
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 让TF listener先接收数据
        self.get_logger().info("等待TF数据...")
        for _ in range(30):  # 约3秒
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 样本存储
        self.robot_poses = []  # base → effector
        self.marker_poses = []  # camera → marker
        
        self.get_logger().info("="*70)
        self.get_logger().info("手动手眼标定工具 (Eye-in-Hand)")
        self.get_logger().info("="*70)
        self.get_logger().info(f"  机器人: {self.robot_base} → {self.robot_effector}")
        self.get_logger().info(f"  相机:   {self.camera_frame} → {self.marker_frame}")
        self.get_logger().info("="*70)
        self.get_logger().info("")
        self.get_logger().info("操作说明：")
        self.get_logger().info("  1. 移动机械臂到不同姿态（确保棋盘格在视野内）")
        self.get_logger().info("  2. 输入 's' + Enter 采集当前样本")
        self.get_logger().info("  3. 采集 12-20 个样本后，输入 'c' + Enter 计算标定")
        self.get_logger().info("  4. 输入 'q' + Enter 退出")
        self.get_logger().info("="*70)
        
    def get_transform_matrix(self, parent_frame, child_frame):
        """获取 TF 变换并转换为 4x4 矩阵"""
        try:
            # 多次 spin 以刷新 TF buffer，再查询最新可用数据
            target_time = rclpy.time.Time()  # Time(0) 等价于最新可用
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.2)
                if self.tf_buffer.can_transform(
                    parent_frame,
                    child_frame,
                    target_time,
                    timeout=rclpy.duration.Duration(seconds=0.2),
                ):
                    break
                else:
                    self.get_logger().error(
                        f"TF 查询失败: {parent_frame} → {child_frame} 无可用数据"
                    )
                    return None, False

            trans = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                target_time,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            
            # 构建 4x4 变换矩阵
            T = np.eye(4)
            t = trans.transform.translation
            q = trans.transform.rotation
            
            # 平移
            T[0, 3] = t.x
            T[1, 3] = t.y
            T[2, 3] = t.z
            
            # 旋转（四元数 → 旋转矩阵）
            R = self.quaternion_to_rotation_matrix([q.x, q.y, q.z, q.w])
            T[:3, :3] = R
            
            return T, True
            
        except Exception as e:
            self.get_logger().error(f"TF 查询失败: {e}")
            return None, False
    
    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """四元数转旋转矩阵 [x, y, z, w]"""
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
    
    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """旋转矩阵转四元数 [w, x, y, z]"""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        return np.array([w, x, y, z])
    
    def take_sample(self):
        """采集一个标定样本"""
        self.get_logger().info("正在采集样本...")
        
        # 1. 获取机器人姿态 (base → effector)
        T_base_effector, success1 = self.get_transform_matrix(
            self.robot_base, self.robot_effector
        )
        
        # 2. 获取标记姿态 (camera → marker)
        T_camera_marker, success2 = self.get_transform_matrix(
            self.camera_frame, self.marker_frame
        )
        
        if not (success1 and success2):
            self.get_logger().error("❌ 采集失败！请确保：")
            self.get_logger().error("   - 机械臂 TF 发布器正在运行")
            self.get_logger().error("   - 棋盘格在相机视野内")
            return False
        
        self.robot_poses.append(T_base_effector)
        self.marker_poses.append(T_camera_marker)
        
        self.get_logger().info(f"✓ 样本 #{len(self.robot_poses)} 采集成功")
        self.get_logger().info(f"  机器人位置: [{T_base_effector[0,3]:.3f}, {T_base_effector[1,3]:.3f}, {T_base_effector[2,3]:.3f}]")
        self.get_logger().info(f"  棋盘格距离: {T_camera_marker[2,3]:.3f} m")
        
        return True
    
    def compute_calibration(self):
        """计算手眼标定"""
        n = len(self.robot_poses)
        
        if n < 3:
            self.get_logger().error(f"❌ 样本不足！当前 {n} 个，至少需要 3 个")
            return False
        
        self.get_logger().info("="*70)
        self.get_logger().info(f"开始计算标定 (使用 {n} 个样本)...")
        self.get_logger().info("="*70)
        
        # 准备 OpenCV 格式的数据
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []
        
        for i in range(n):
            # 机器人姿态 (base → gripper，需要求逆得到 gripper → base)
            T_base_gripper = self.robot_poses[i]
            T_gripper_base = np.linalg.inv(T_base_gripper)
            R_gripper2base.append(T_gripper_base[:3, :3])
            t_gripper2base.append(T_gripper_base[:3, 3].reshape(3, 1))
            
            # 标记姿态 (camera → target)
            T_cam_target = self.marker_poses[i]
            R_target2cam.append(T_cam_target[:3, :3])
            t_target2cam.append(T_cam_target[:3, 3].reshape(3, 1))
        
        # 调用 OpenCV 手眼标定（Tsai-Lenz 方法）
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        # 转换为四元数
        q_cam2gripper = self.rotation_matrix_to_quaternion(R_cam2gripper)
        
        self.get_logger().info("✓ 标定计算完成！")
        self.get_logger().info("="*70)
        self.get_logger().info("标定结果 (link6 → camera_color_optical_frame):")
        self.get_logger().info(f"  平移: [{t_cam2gripper[0,0]:.6f}, {t_cam2gripper[1,0]:.6f}, {t_cam2gripper[2,0]:.6f}]")
        self.get_logger().info(f"  四元数: [{q_cam2gripper[0]:.6f}, {q_cam2gripper[1]:.6f}, {q_cam2gripper[2]:.6f}, {q_cam2gripper[3]:.6f}]")
        self.get_logger().info("="*70)
        
        # 保存结果
        self.save_calibration(t_cam2gripper.flatten(), q_cam2gripper)
        
        return True
    
    def save_calibration(self, translation, quaternion):
        """保存标定结果到 YAML 文件"""
        calib_dir = Path.home() / '.ros' / 'easy_handeye'
        calib_dir.mkdir(parents=True, exist_ok=True)
        calib_file = calib_dir / f'{self.calib_name}.yaml'
        
        # 构建 YAML 数据（兼容 easy_handeye2 格式）
        # 注意：OpenCV 输出的是 gripper → camera，但我们需要存储 camera → gripper 的逆
        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = self.quaternion_to_rotation_matrix(quaternion[[1,2,3,0]])  # w,x,y,z → x,y,z,w
        T_cam2gripper[:3, 3] = translation
        
        # 求逆得到 gripper → camera
        T_gripper2cam = np.linalg.inv(T_cam2gripper)
        t = T_gripper2cam[:3, 3]
        q = self.rotation_matrix_to_quaternion(T_gripper2cam[:3, :3])
        
        data = {
            'calibration': {
                'translation': {
                    'x': float(t[0]),
                    'y': float(t[1]),
                    'z': float(t[2])
                },
                'rotation': {
                    'x': float(q[1]),  # w,x,y,z → x,y,z,w
                    'y': float(q[2]),
                    'z': float(q[3]),
                    'w': float(q[0])
                }
            }
        }
        
        with open(calib_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        self.get_logger().info(f"✓ 标定结果已保存到: {calib_file}")
        self.get_logger().info("")
        self.get_logger().info("下一步：")
        self.get_logger().info(f"  python3 read_calibration_result.py")
        self.get_logger().info("  （复制输出的代码到 piper_arm.py）")
    
    def run(self):
        """主循环"""
        import sys, select
        
        while rclpy.ok():
            # 非阻塞输入检查
            if select.select([sys.stdin], [], [], 0.1)[0]:
                cmd = sys.stdin.readline().strip().lower()
                
                if cmd == 's':
                    self.take_sample()
                elif cmd == 'c':
                    self.compute_calibration()
                elif cmd == 'q':
                    self.get_logger().info("退出标定工具")
                    break
                else:
                    self.get_logger().warn(f"未知命令: {cmd}")
            
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = ManualHandeyeCalibration()
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
