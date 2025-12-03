#!/usr/bin/env python3
"""
AprilTag面板检测脚本
功能：检测AprilTag面板，并计算其相对于相机、夹爪、基座的位置(xyz)和姿态(rpy)
"""
import cv2
import numpy as np
import pyrealsense2 as rs
from dt_apriltags import Detector
import time
import math
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 导入 PiperArm 和 SDK
from piper_arm import PiperArm
from piper_sdk import C_PiperInterface_V2

class AprilTagPanelDetector:
    def __init__(self):
        """初始化AprilTag检测器和RealSense相机"""
        print("=" * 60)
        print("AprilTag面板检测系统")
        print("=" * 60)
        
        # 🔧 初始化 Piper SDK 和运动学
        print("\n[1/4] 初始化机械臂...")
        try:
            self.piper = C_PiperInterface_V2()
            self.piper.ConnectPort()
            self.piper_arm = PiperArm()
            
            # 从 piper_arm 获取手眼标定参数
            # link6 (夹爪) 到 camera 的变换
            link6_q_camera = self.piper_arm.link6_q_camera  # [w, x, y, z]
            link6_t_camera = self.piper_arm.link6_t_camera  # [x, y, z]
            
            # 将四元数转换为旋转矩阵
            self.link6_R_camera = self.quaternion_to_rotation_matrix(link6_q_camera)
            self.link6_t_camera = np.array(link6_t_camera)
            
            # 构建 link6_T_camera 变换矩阵 (4x4)
            self.link6_T_camera = np.eye(4)
            self.link6_T_camera[:3, :3] = self.link6_R_camera
            self.link6_T_camera[:3, 3] = self.link6_t_camera
            
            print("  ✓ 机械臂连接成功")
            print("  ✓ 已加载手眼标定参数 (link6_T_camera):")
            print(self.link6_T_camera)
        except Exception as e:
            print(f"  ✗ 机械臂初始化失败: {e}")
            print("  ⚠️  将使用默认手眼标定矩阵（功能受限）")
            self.piper = None
            self.piper_arm = None
            # 使用默认矩阵
            self.link6_T_camera = np.eye(4)
            self.link6_T_camera[2, 3] = 0.05
        
        # 初始化AprilTag检测器
        # 支持的标签家族: 'tag36h11', 'tag25h9', 'tag16h5', 'tagCircle21h7', 'tagStandard41h12'
        self.tag_family = 'tag25h9'  # 改为tag36h11以提高检测率
        print(f"\n[2/4] 初始化AprilTag检测器 (家族: {self.tag_family})...")
        try:
            self.detector = Detector(
                families=self.tag_family,
                nthreads=4,
                quad_decimate=1.0,      # 小标签使用全分辨率
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.4,  # 提高锐化帮助小标签检测
                debug=0
            )
            print("  ✓ AprilTag检测器初始化成功")
        except Exception as e:
            print(f"  ✗ AprilTag检测器初始化失败: {e}")
            raise
        
        # 初始化RealSense相机
        print("\n[3/4] 初始化RealSense相机...")
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 配置流
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # 启动管道
            profile = self.pipeline.start(self.config)
            
            # 优化相机设置
            device = profile.get_device()
            color_sensor = device.first_color_sensor()
            
            # 设置自动曝光和白平衡
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
            color_sensor.set_option(rs.option.enable_auto_white_balance, 1)
            
            # 尝试增加锐度
            try:
                color_sensor.set_option(rs.option.sharpness, 100)
                print("  ✓ 相机锐度已优化")
            except:
                pass
            
            # 创建对齐器（对齐深度到彩色）
            self.align = rs.align(rs.stream.color)
            
            # 预热相机
            for _ in range(30):
                self.pipeline.wait_for_frames()
            
            # 获取相机内参
            color_stream = profile.get_stream(rs.stream.color)
            self.color_intrin = color_stream.as_video_stream_profile().intrinsics
            
            # 构建相机内参矩阵
            self.camera_matrix = np.array([
                [self.color_intrin.fx, 0, self.color_intrin.ppx],
                [0, self.color_intrin.fy, self.color_intrin.ppy],
                [0, 0, 1]
            ], dtype=np.float64)
            
            # 畸变系数
            self.dist_coeffs = np.array(self.color_intrin.coeffs, dtype=np.float64)
            
            print("  ✓ RealSense相机初始化成功")
            print(f"    分辨率: 640x480")
            print(f"    内参 fx={self.color_intrin.fx:.2f}, fy={self.color_intrin.fy:.2f}")
            print(f"    主点 cx={self.color_intrin.ppx:.2f}, cy={self.color_intrin.ppy:.2f}")
        except Exception as e:
            print(f"  ✗ RealSense相机初始化失败: {e}")
            raise
        
        # AprilTag尺寸（米）
        self.tag_size = 0.025  # 🔧 默认2.5cm，根据实际标签调整
        
        print("\n" + "=" * 60)
        print("✓ 初始化完成！")
        print("=" * 60)
        print("\n操作说明:")
        print("  [q/ESC] - 退出程序")
        print("  [s]     - 更改AprilTag尺寸")
        print("  [空格]  - 暂停/继续")
        print(f"\n当前AprilTag尺寸: {self.tag_size*1000:.1f}mm")
        print("=" * 60 + "\n")
    
    def quaternion_to_rotation_matrix(self, q):
        """
        将四元数转换为旋转矩阵
        q: [w, x, y, z]
        """
        w, x, y, z = q
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
            [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
            [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
        return R
    
    def rotation_matrix_to_euler_angles(self, R):
        """
        将旋转矩阵转换为欧拉角 (roll, pitch, yaw) 单位：弧度
        使用ZYX顺序 (Yaw-Pitch-Roll)
        """
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        
        singular = sy < 1e-6
        
        if not singular:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0
        
        return roll, pitch, yaw
    
    def get_current_joints(self):
        """获取当前关节角度（弧度）"""
        if self.piper is None:
            return None
        
        try:
            msg = self.piper.GetArmJointMsgs()
            PI = math.pi
            joints = [
                msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
                msg.joint_state.joint_6 * 1e-3 * PI / 180.0,
            ]
            return joints
        except Exception as e:
            print(f"⚠️  读取关节角度失败: {e}")
            return None
    
    def transform_camera_to_gripper(self, camera_T_tag):
        """
        将AprilTag从相机坐标系转换到夹爪坐标系
        
        Args:
            camera_T_tag: 4x4变换矩阵，AprilTag相对于相机
            
        Returns:
            gripper_T_tag: 4x4变换矩阵，AprilTag相对于夹爪
        """
        # link6_T_tag = link6_T_camera @ camera_T_tag
        gripper_T_tag = self.link6_T_camera @ camera_T_tag
        return gripper_T_tag
    
    def transform_camera_to_base(self, camera_T_tag, current_joints):
        """
        将AprilTag从相机坐标系转换到基座坐标系
        
        Args:
            camera_T_tag: 4x4变换矩阵，AprilTag相对于相机
            current_joints: 当前关节角度列表（弧度）
            
        Returns:
            base_T_tag: 4x4变换矩阵，AprilTag相对于基座
        """
        if self.piper_arm is None or current_joints is None:
            return None
        
        try:
            # 计算正向运动学：base_T_link6
            base_T_link6 = self.piper_arm.forward_kinematics(current_joints)
            
            # base_T_tag = base_T_link6 @ link6_T_camera @ camera_T_tag
            base_T_tag = base_T_link6 @ self.link6_T_camera @ camera_T_tag
            
            return base_T_tag
        except Exception as e:
            print(f"⚠️  坐标转换失败: {e}")
            return None
    
    def detect_apriltags(self, gray_image):
        """检测AprilTag标签（带图像增强）"""
        # 图像预处理以提高小标签检测
        # 1. 自适应直方图均衡化（CLAHE）- 提高对比度
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray_image)
        
        # 2. 轻微锐化
        kernel = np.array([[-1,-1,-1],
                          [-1, 9,-1],
                          [-1,-1,-1]])
        enhanced = cv2.filter2D(enhanced, -1, kernel)
        
        # 使用dt-apriltags库进行检测
        detections = self.detector.detect(
            enhanced,
            estimate_tag_pose=True,
            camera_params=[
                self.color_intrin.fx,
                self.color_intrin.fy,
                self.color_intrin.ppx,
                self.color_intrin.ppy
            ],
            tag_size=self.tag_size
        )
        return detections
    
    def draw_detection(self, image, detection):
        """在图像上绘制检测结果"""
        # 绘制边框
        corners = detection.corners.astype(int)
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(image, pt1, pt2, (0, 255, 0), 2)
        
        # 绘制中心点
        center = detection.center.astype(int)
        cv2.circle(image, tuple(center), 5, (0, 0, 255), -1)
        
        # 绘制ID
        cv2.putText(image, f"ID: {detection.tag_id}", 
                    (center[0] - 20, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return image
    
    def draw_axis(self, image, rvec, tvec):
        """绘制3D坐标轴"""
        axis_length = self.tag_size
        axis = np.float32([
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length],
            [0, 0, 0]
        ]).reshape(-1, 3)
        
        # 投影到图像平面
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        imgpts = imgpts.astype(int)
        
        origin = tuple(imgpts[3].ravel())
        # X轴 - 红色
        image = cv2.line(image, origin, tuple(imgpts[0].ravel()), (0, 0, 255), 3)
        # Y轴 - 绿色
        image = cv2.line(image, origin, tuple(imgpts[1].ravel()), (0, 255, 0), 3)
        # Z轴 - 蓝色
        image = cv2.line(image, origin, tuple(imgpts[2].ravel()), (255, 0, 0), 3)
        
        return image
    
    def display_pose_info(self, image, detections, current_joints=None):
        """在图像上显示位姿信息（相机、夹爪、基座三个坐标系）"""
        y_offset = 30
        
        # 标题
        cv2.putText(image, "AprilTag Pose: Camera/Gripper/Base", (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 28
        
        # 标签数量
        cv2.putText(image, f"Tags: {len(detections)}", (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 22
        
        # 显示每个标签的位姿
        for i, det in enumerate(detections):
            if det.pose_t is None or det.pose_R is None:
                continue
            
            # ========== 相机坐标系下的位姿 ==========
            x_cam, y_cam, z_cam = det.pose_t.flatten()
            roll_cam, pitch_cam, yaw_cam = self.rotation_matrix_to_euler_angles(det.pose_R)
            roll_cam_deg = math.degrees(roll_cam)
            pitch_cam_deg = math.degrees(pitch_cam)
            yaw_cam_deg = math.degrees(yaw_cam)
            
            # ========== 夹爪坐标系下的位姿 ==========
            # 构建相机坐标系下的4x4变换矩阵
            camera_T_tag = np.eye(4)
            camera_T_tag[:3, :3] = det.pose_R
            camera_T_tag[:3, 3] = det.pose_t.flatten()
            
            # 转换到夹爪坐标系
            gripper_T_tag = self.transform_camera_to_gripper(camera_T_tag)
            
            # 提取夹爪坐标系下的位置和姿态
            x_grip, y_grip, z_grip = gripper_T_tag[:3, 3]
            R_grip = gripper_T_tag[:3, :3]
            roll_grip, pitch_grip, yaw_grip = self.rotation_matrix_to_euler_angles(R_grip)
            roll_grip_deg = math.degrees(roll_grip)
            pitch_grip_deg = math.degrees(pitch_grip)
            yaw_grip_deg = math.degrees(yaw_grip)
            
            # 转换到基座坐标系
            base_T_tag = self.transform_camera_to_base(camera_T_tag, current_joints)
            if base_T_tag is not None:
                x_base, y_base, z_base = base_T_tag[:3, 3]
                R_base = base_T_tag[:3, :3]
                roll_base, pitch_base, yaw_base = self.rotation_matrix_to_euler_angles(R_base)
                roll_base_deg = math.degrees(roll_base)
                pitch_base_deg = math.degrees(pitch_base)
                yaw_base_deg = math.degrees(yaw_base)
            else:
                x_base = y_base = z_base = 0
                roll_base_deg = pitch_base_deg = yaw_base_deg = 0
            
            # ========== 显示信息 ==========
            cv2.putText(image, f"--- Tag ID {det.tag_id} ---", (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            y_offset += 18
            
            # 相机坐标系
            cv2.putText(image, f"[Camera]", (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 255), 1)
            y_offset += 15
            
            cv2.putText(image, f"X:{x_cam:+.3f} Y:{y_cam:+.3f} Z:{z_cam:+.3f}m", (12, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            y_offset += 13
            
            cv2.putText(image, f"R:{roll_cam_deg:+.1f} P:{pitch_cam_deg:+.1f} Y:{yaw_cam_deg:+.1f}deg", (12, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            y_offset += 16
            
            # 夹爪坐标系
            cv2.putText(image, f"[Gripper]", (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
            y_offset += 15
            
            cv2.putText(image, f"X:{x_grip:+.3f} Y:{y_grip:+.3f} Z:{z_grip:+.3f}m", (12, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 255, 200), 1)
            y_offset += 13
            
            cv2.putText(image, f"R:{roll_grip_deg:+.1f} P:{pitch_grip_deg:+.1f} Y:{yaw_grip_deg:+.1f}deg", (12, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 255, 200), 1)
            y_offset += 16
            
            # 基座坐标系
            if base_T_tag is not None:
                cv2.putText(image, f"[Base]", (10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 150, 100), 1)
                y_offset += 15
                
                cv2.putText(image, f"X:{x_base:+.3f} Y:{y_base:+.3f} Z:{z_base:+.3f}m", (12, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 150), 1)
                y_offset += 13
                
                cv2.putText(image, f"R:{roll_base_deg:+.1f} P:{pitch_base_deg:+.1f} Y:{yaw_base_deg:+.1f}deg", (12, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 150), 1)
                y_offset += 18
            else:
                cv2.putText(image, f"[Base] N/A", (10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
                y_offset += 18
        
        return image
    
    def run(self):
        """主循环"""
        is_paused = False
        paused_frame = None
        
        try:
            while True:
                # 获取帧
                if not is_paused:
                    frames = self.pipeline.wait_for_frames()
                    aligned_frames = self.align.process(frames)
                    
                    color_frame = aligned_frames.get_color_frame()
                    depth_frame = aligned_frames.get_depth_frame()
                    
                    if not color_frame or not depth_frame:
                        continue
                    
                    # 转换为numpy数组
                    color_image = np.asanyarray(color_frame.get_data())
                    paused_frame = color_image.copy()
                else:
                    color_image = paused_frame.copy()
                
                # 转换为灰度图
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                
                # 检测AprilTag
                start_time = time.time()
                detections = self.detect_apriltags(gray)
                detect_time = (time.time() - start_time) * 1000
                
                # 获取当前关节角度（用于基座坐标系转换）
                current_joints = self.get_current_joints()
                
                # 可视化
                vis_image = color_image.copy()
                
                # 绘制检测结果
                for det in detections:
                    vis_image = self.draw_detection(vis_image, det)
                    
                    # 绘制坐标轴
                    if det.pose_t is not None and det.pose_R is not None:
                        # 转换为OpenCV格式
                        rvec, _ = cv2.Rodrigues(det.pose_R)
                        tvec = det.pose_t
                        vis_image = self.draw_axis(vis_image, rvec, tvec)
                
                # 显示位姿信息（包含当前关节角度）
                vis_image = self.display_pose_info(vis_image, detections, current_joints)
                
                # 显示检测时间
                cv2.putText(vis_image, f"Detection: {detect_time:.1f}ms", 
                            (10, vis_image.shape[0] - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 显示标签尺寸
                cv2.putText(vis_image, f"Tag size: {self.tag_size*1000:.1f}mm", 
                            (10, vis_image.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 显示暂停状态
                if is_paused:
                    cv2.putText(vis_image, "PAUSED", (vis_image.shape[1] - 120, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # 显示图像
                cv2.imshow('AprilTag Panel Detection', vis_image)
                
                # 在控制台输出位姿信息
                if detections and not is_paused:
                    # 获取当前关节角度
                    current_joints = self.get_current_joints()
                    
                    print("\n" + "=" * 80)
                    print(f"检测到 {len(detections)} 个AprilTag标签")
                    print("=" * 80)
                    for det in detections:
                        if det.pose_t is None or det.pose_R is None:
                            continue
                        
                        # 相机坐标系
                        x_cam, y_cam, z_cam = det.pose_t.flatten()
                        roll_cam, pitch_cam, yaw_cam = self.rotation_matrix_to_euler_angles(det.pose_R)
                        
                        # 夹爪坐标系
                        camera_T_tag = np.eye(4)
                        camera_T_tag[:3, :3] = det.pose_R
                        camera_T_tag[:3, 3] = det.pose_t.flatten()
                        gripper_T_tag = self.transform_camera_to_gripper(camera_T_tag)
                        x_grip, y_grip, z_grip = gripper_T_tag[:3, 3]
                        R_grip = gripper_T_tag[:3, :3]
                        roll_grip, pitch_grip, yaw_grip = self.rotation_matrix_to_euler_angles(R_grip)
                        
                        # 基座坐标系
                        base_T_tag = self.transform_camera_to_base(camera_T_tag, current_joints)
                        
                        print(f"\n[Tag ID: {det.tag_id}]")
                        print(f"  📷 相机坐标系 (Camera Frame):")
                        print(f"     位置 (米): X={x_cam:+.4f}, Y={y_cam:+.4f}, Z={z_cam:+.4f}")
                        print(f"     姿态 (度): Roll={math.degrees(roll_cam):+.2f}, Pitch={math.degrees(pitch_cam):+.2f}, Yaw={math.degrees(yaw_cam):+.2f}")
                        
                        print(f"  🤖 夹爪坐标系 (Gripper Frame):")
                        print(f"     位置 (米): X={x_grip:+.4f}, Y={y_grip:+.4f}, Z={z_grip:+.4f}")
                        print(f"     姿态 (度): Roll={math.degrees(roll_grip):+.2f}, Pitch={math.degrees(pitch_grip):+.2f}, Yaw={math.degrees(yaw_grip):+.2f}")
                        
                        if base_T_tag is not None:
                            x_base, y_base, z_base = base_T_tag[:3, 3]
                            R_base = base_T_tag[:3, :3]
                            roll_base, pitch_base, yaw_base = self.rotation_matrix_to_euler_angles(R_base)
                            print(f"  🏠 基座坐标系 (Base Frame):")
                            print(f"     位置 (米): X={x_base:+.4f}, Y={y_base:+.4f}, Z={z_base:+.4f}")
                            print(f"     姿态 (度): Roll={math.degrees(roll_base):+.2f}, Pitch={math.degrees(pitch_base):+.2f}, Yaw={math.degrees(yaw_base):+.2f}")
                        else:
                            print(f"  🏠 基座坐标系: 不可用 (需要机械臂连接)")
                        
                        print(f"  置信度: {det.decision_margin:.2f}, 汉明距离: {det.hamming}")
                    print("=" * 80)
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == 27:  # q或ESC退出
                    print("\n退出程序...")
                    break
                elif key == ord(' '):  # 空格键暂停/继续
                    is_paused = not is_paused
                    if is_paused:
                        print("\n⏸  已暂停")
                    else:
                        print("\n▶  继续")
                elif key == ord('s'):  # 更改标签尺寸
                    print(f"\n当前AprilTag尺寸: {self.tag_size*1000:.1f}mm")
                    try:
                        new_size = float(input("请输入新的标签尺寸(mm): ")) / 1000.0
                        if new_size > 0:
                            self.tag_size = new_size
                            print(f"✓ 标签尺寸已更新为: {self.tag_size*1000:.1f}mm")
                        else:
                            print("✗ 无效尺寸")
                    except:
                        print("✗ 输入无效")
        
        except KeyboardInterrupt:
            print("\n检测到Ctrl+C，退出程序...")
        
        finally:
            # 清理资源
            self.pipeline.stop()
            cv2.destroyAllWindows()
            if self.piper is not None:
                try:
                    self.piper.DisconnectPort()
                except:
                    pass
            print("\n✓ 资源已清理")

def main():
    """主函数"""
    try:
        detector = AprilTagPanelDetector()
        detector.run()
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
