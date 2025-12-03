#!/usr/bin/env python3
"""
交互式按钮检测器 - ROS2 AprilTag版本
使用 pyrealsense2 直接读取相机（高性能）+ ROS2 发布结果
集成AprilTag实时检测，发布夹爪坐标系欧拉角
同步YOLO检测，实时性优化
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import cv2
from ultralytics import YOLO
import numpy as np
import pyrealsense2 as rs
import time
import math

# 导入坐标转换所需的模块
from piper_sdk import C_PiperInterface_V2
from piper_arm import PiperArm

# 导入AprilTag检测器
from dt_apriltags import Detector

PI = math.pi

# ========================================
# 性能调优参数
# ========================================
DETECTION_SKIP_FRAMES = 0  # 🔧 异步模式：每帧都放入队列，检测线程自动处理最新帧
YOLO_CONF_THRESHOLD = 0.4  # 🔧 降低阈值提高召回率（小图像需要）
YOLO_SCALE_FACTOR = 0.1    # 🔧 🚀 极限模式：640x480 → 64x48 (100倍加速!)
UI_REFRESH_RATE = 30       # 🔧 UI刷新率（Hz），独立于检测频率

# ========================================
# 全局变量
# ========================================
all_detections = []
selected_button_index = -1
selected_button_locked = False
selected_box_signature = None

# 鼠标位置
mouse_x, mouse_y = 0, 0

# 当前帧数据
current_depth_data = None
current_color_data = None
current_depth_intrin = None

# 交互控制
is_paused = False
paused_frame = None
paused_detections = []

# 帧计数器
frame_counter = 0

# FPS统计
last_fps_time = time.time()
fps_counter = 0
current_fps = 0.0
last_detect_time_ms = 0.0  # 最近一次检测耗时




# ========================================
# 鼠标回调函数
# ========================================
def mouse_callback(event, x, y, flags, param):
    """处理鼠标事件，允许用户点击选择按钮"""
    global selected_button_index, mouse_x, mouse_y, all_detections
    global current_depth_data, current_color_data, current_depth_intrin
    global selected_button_locked, is_paused, paused_detections
    global selected_box_signature
    
    mouse_x, mouse_y = x, y
    
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"\n{'='*70}")
        print(f"[鼠标点击] 位置: ({x}, {y})")
        print(f"[检测状态] 当前检测到 {len(all_detections)} 个按钮")
        
        detections_to_check = paused_detections if is_paused else list(all_detections)
        
        found = False
        for idx, det in enumerate(detections_to_check):
            x1, y1, x2, y2, class_name, conf, center_3d = det
            
            print(f"  检查按钮 #{idx}: 类型={class_name}, 框=[{x1},{y1},{x2},{y2}]", end="")
            
            if x1 <= x <= x2 and y1 <= y <= y2:
                print(" → ✓ 匹配!")
                found = True
                selected_button_index = idx
                selected_button_locked = True
                
                # 立即刷新显示
                if current_color_data is not None:
                    instant_display = visualize_detections(current_color_data, detections_to_check, selected_button_index)
                    cv2.imshow('detection', instant_display)
                    cv2.waitKey(1)
                
                if center_3d is None and current_depth_data is not None:
                    # 简化的3D点提取
                    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                    depth = current_depth_data[cy, cx] * 0.001  # mm -> m
                    if depth > 0:
                        center_3d = np.array([
                            (cx - current_depth_intrin.ppx) * depth / current_depth_intrin.fx,
                            (cy - current_depth_intrin.ppy) * depth / current_depth_intrin.fy,
                            depth
                        ])
                    if is_paused:
                        paused_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                    if idx < len(all_detections):
                        all_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                
                remember_selected_detection(det)
                
                print(f"\n{'='*70}")
                print(f"✓✓✓ 已选择按钮 #{idx} 【已锁定】✓✓✓")
                print(f"  类型: {class_name}")
                print(f"  置信度: {conf:.2f}")
                print(f"  检测框: [{x1}, {y1}, {x2}, {y2}]")
                print(f"  2D中心: ({int((x1+x2)/2)}, {int((y1+y2)/2)})")
                if center_3d is not None:
                    print(f"  [相机坐标系] XYZ: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f}) m")
                    
                    # 转换到基座坐标系
                    if param is not None and hasattr(param, 'piper_arm') and param.piper_arm is not None:
                        base_3d = transform_button_camera_to_base(center_3d, param.piper, param.piper_arm)
                        if base_3d is not None:
                            print(f"  [基座坐标系] XYZ: ({base_3d[0]:.3f}, {base_3d[1]:.3f}, {base_3d[2]:.3f}) m")
                        else:
                            print(f"  [基座坐标系] 转换失败（无法获取关节角度）")
                    else:
                        print(f"  [基座坐标系] N/A (Piper SDK未初始化)")
                print(f"  提示: 按 ENTER 确认 | 按 ESC 取消选择")
                print(f"{'='*70}")
                break
            else:
                print(" → ✗ 不匹配")
        
        if not found:
            print(f"\n  ✗✗✗ 点击位置 ({x}, {y}) 不在任何按钮内")
            print(f"{'='*70}")


# ========================================
# YOLO 检测
# ========================================
def YOLODetection(model, color_img, conf_threshold=0.5):
    """YOLO 检测 - 固定编号顺序"""
    results = model(color_img, verbose=False)
    target_boxes = []
    
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            class_name = model.names[cls_id]
            
            if conf >= conf_threshold:
                target_boxes.append((x1, y1, x2, y2, class_name, conf))
    
    target_boxes.sort(key=lambda box: (box[1] // 100, box[0]))
    return target_boxes


def _make_signature_from_detection(det):
    """根据检测框生成唯一签名"""
    x1, y1, x2, y2, class_name, *_ = det
    center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    return {"class": class_name, "center": center, "bbox": (x1, y1, x2, y2)}


def remember_selected_detection(det):
    """记录当前选中的检测框特征"""
    global selected_box_signature
    selected_box_signature = _make_signature_from_detection(det)


def sync_selection_with_detections():
    """在检测结果更新后，重新关联已选中的按钮"""
    global selected_button_index, selected_box_signature

    if selected_box_signature is None or not all_detections:
        return

    best_idx = -1
    best_dist = float('inf')
    target_class = selected_box_signature["class"]
    target_center = selected_box_signature["center"]

    for idx, det in enumerate(all_detections):
        x1, y1, x2, y2, class_name, *_ = det
        if class_name != target_class:
            continue
        center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        dist = np.linalg.norm(np.array(center) - np.array(target_center))
        if dist < best_dist:
            best_dist = dist
            best_idx = idx

    if best_idx != -1 and best_dist < 80:
        selected_button_index = best_idx
    else:
        selected_button_index = -1
        selected_box_signature = None


# ========================================
# 坐标转换：相机 → 基座（按钮专用）
# ========================================
def transform_button_camera_to_base(button_camera, piper, piper_arm):
    """
    将相机坐标系的按钮位置转换到基座坐标系
    
    Args:
        button_camera: [x, y, z] 或 np.array 相机坐标系
        piper: Piper SDK实例
        piper_arm: PiperArm实例
    
    Returns:
        button_base: [x, y, z] 基座坐标系，失败返回None
    """
    try:
        from utils.utils_math import quaternion_to_rotation_matrix
        
        # 获取当前关节角度
        msg = piper.GetArmJointMsgs()
        current_joints = [
            msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_6 * 1e-3 * PI / 180.0,
        ]
        
        # 基座到link6的变换（正向运动学）
        base_T_link6 = piper_arm.forward_kinematics(current_joints)
        
        # link6到相机的变换（手眼标定）
        link6_T_cam = np.eye(4)
        link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
        link6_T_cam[:3, 3] = piper_arm.link6_t_camera
        
        # 按钮在相机坐标系的齐次坐标
        button_cam_h = np.array([button_camera[0], button_camera[1], button_camera[2], 1.0])
        
        # 转换到基座坐标系
        button_base = base_T_link6 @ link6_T_cam @ button_cam_h
        
        return button_base[:3]
    except Exception as e:
        return None





# ========================================
# 可视化
# ========================================
def visualize_detections(color_img, detections, selected_idx, apriltag_pose_info=None):
    """可视化检测结果（包含AprilTag位姿信息）"""
    annotated = color_img.copy()
    
    for idx, det in enumerate(detections):
        x1, y1, x2, y2, class_name, conf, center_3d = det
        
        if idx == selected_idx:
            color = (0, 255, 0)
            thickness = 4
        else:
            color = (255, 0, 0)
            thickness = 2
        
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
        
        label = f"#{idx} {class_name} {conf:.2f}"
        cv2.putText(annotated, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if center_3d is not None:
            coord_text = f"({center_3d[0]:.2f}, {center_3d[1]:.2f}, {center_3d[2]:.2f})"
            cv2.putText(annotated, coord_text, (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    # FPS信息
    global current_fps
    cv2.putText(annotated, f"FPS:{current_fps:.0f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # 显示AprilTag位姿信息（左侧）
    if apriltag_pose_info is not None and len(apriltag_pose_info) > 0:
        y_offset = 60  # 从FPS下方开始
        x_start = 10  # 左侧起始位置
        
        # 计算需要的背景高度（每个Tag约178行，包含新增的Gripper in Base）
        info_height = len(apriltag_pose_info) * 178
        
        # 半透明背景
        overlay = annotated.copy()
        cv2.rectangle(overlay, (5, y_offset - 5), (430, y_offset + info_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated)
        
        # 标题
        cv2.putText(annotated, "=== AprilTag Pose ===", (x_start, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        y_offset += 22
        
        for info in apriltag_pose_info:
            tag_id = info['tag_id']
            
            # Tag ID
            cv2.putText(annotated, f"--- Tag ID {tag_id} ---", (x_start, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)
            y_offset += 18
            
            # AprilTag在相机坐标系
            cv2.putText(annotated, "[Tag in Camera]", (x_start, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 100), 1)
            y_offset += 15
            
            x_cam = info['camera']['position'][0]
            y_cam = info['camera']['position'][1]
            z_cam = info['camera']['position'][2]
            cv2.putText(annotated, f"Pos: X{x_cam:+.3f} Y{y_cam:+.3f} Z{z_cam:+.3f}m", (x_start + 5, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            y_offset += 13
            
            roll = info['camera']['rpy'][0]
            pitch = info['camera']['rpy'][1]
            yaw = info['camera']['rpy'][2]
            cv2.putText(annotated, f"RPY: R{roll:+.1f} P{pitch:+.1f} Y{yaw:+.1f}deg", (x_start + 5, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            y_offset += 16
            
            # 相机坐标系（实际上和Tag Frame一样，可以省略）
            # cv2.putText(annotated, "[Camera]", (x_start, y_offset),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 255), 1)
            # y_offset += 15
            
            # AprilTag在夹爪坐标系
            cv2.putText(annotated, "[Tag in Gripper]", (x_start, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
            y_offset += 15
            
            x_grip = info['gripper']['position'][0]
            y_grip = info['gripper']['position'][1]
            z_grip = info['gripper']['position'][2]
            cv2.putText(annotated, f"Pos: X{x_grip:+.3f} Y{y_grip:+.3f} Z{z_grip:+.3f}m", (x_start + 5, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 255, 200), 1)
            y_offset += 13
            
            roll_grip = info['gripper']['rpy'][0]
            pitch_grip = info['gripper']['rpy'][1]
            yaw_grip = info['gripper']['rpy'][2]
            cv2.putText(annotated, f"RPY: R{roll_grip:+.1f} P{pitch_grip:+.1f} Y{yaw_grip:+.1f}deg", (x_start + 5, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 255, 200), 1)
            y_offset += 16
            
            # 当前夹爪姿态（基座坐标系）
            if info.get('gripper_in_base_rpy') is not None:
                cv2.putText(annotated, "[Current Gripper Pose]", (x_start, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 255, 255), 1)
                y_offset += 15
                
                roll_gb = info['gripper_in_base_rpy'][0]
                pitch_gb = info['gripper_in_base_rpy'][1]
                yaw_gb = info['gripper_in_base_rpy'][2]
                cv2.putText(annotated, f"RPY: R{roll_gb:+.1f} P{pitch_gb:+.1f} Y{yaw_gb:+.1f}deg (in Base)", (x_start + 5, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 255, 255), 1)
                y_offset += 18
            
            # AprilTag在基座坐标系
            if info.get('base') is not None:
                cv2.putText(annotated, "[Tag in Base]", (x_start, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 150, 100), 1)
                y_offset += 15
                
                x_base = info['base']['position'][0]
                y_base = info['base']['position'][1]
                z_base = info['base']['position'][2]
                cv2.putText(annotated, f"Pos: X{x_base:+.3f} Y{y_base:+.3f} Z{z_base:+.3f}m", (x_start + 5, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 150), 1)
                y_offset += 13
                
                roll_base = info['base']['rpy'][0]
                pitch_base = info['base']['rpy'][1]
                yaw_base = info['base']['rpy'][2]
                cv2.putText(annotated, f"RPY: R{roll_base:+.1f} P{pitch_base:+.1f} Y{yaw_base:+.1f}deg", (x_start + 5, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 150), 1)
                y_offset += 18
            else:
                cv2.putText(annotated, "[Base] N/A", (x_start, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
                y_offset += 18
    
    return annotated


# ========================================
# ROS2 节点（仅用于发布结果）
# ========================================
class ButtonDetectorNode(Node):
    """按钮检测节点 - 直接读取相机版本"""
    def __init__(self):
        super().__init__('button_detector_ros2_direct')
        self.get_logger().info("="*70)
        self.get_logger().info("交互式按钮检测器 - ROS2 AprilTag版本")
        self.get_logger().info("="*70)
        
        # 初始化 Piper SDK
        try:
            self.piper = C_PiperInterface_V2("can0")
            self.piper.ConnectPort()
            self.piper_arm = PiperArm()
            self.get_logger().info("✓ Piper SDK 初始化成功")
        except Exception as e:
            self.get_logger().warn(f"⚠️  Piper SDK 初始化失败: {e}")
            self.piper = None
            self.piper_arm = None
        
        # 加载 YOLO 模型
        self.model = YOLO('yolo_button.pt')
        self.get_logger().info("✓ YOLO 模型加载成功")
        
        # 创建发布器
        self.point_pub = self.create_publisher(PointStamped, '/object_point', 10)  # 相机系（兼容）
        self.point_base_pub = self.create_publisher(PointStamped, '/object_point_base', 10)  # 基座系（新增）
        self.type_pub = self.create_publisher(String, '/button_type', 10)
        self.marker_pub = self.create_publisher(Marker, '/object_center_marker', 10)
        self.normal_pub = self.create_publisher(Vector3, '/button_normal', 10)  # 夹爪系欧拉角（兼容）
        self.normal_base_pub = self.create_publisher(Vector3, '/button_normal_base', 10)  # 基座系欧拉角（新增）
        
        self.get_logger().info("✓ ROS2发布器已创建")
        self.get_logger().info("  - /object_point (相机系，兼容)")
        self.get_logger().info("  - /object_point_base (基座系，推荐)")
        self.get_logger().info("  - /button_normal (夹爪系，兼容)")
        self.get_logger().info("  - /button_normal_base (基座系，推荐)")
        self.get_logger().info("="*70)
    
    def publish_result(self, center_3d, class_name):
        """发布检测结果到ROS2话题"""
        # 发布点坐标
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "camera"
        point_msg.point.x = center_3d[0]
        point_msg.point.y = center_3d[1]
        point_msg.point.z = center_3d[2]
        self.point_pub.publish(point_msg)
        
        # 发布按钮类型
        type_msg = String()
        type_msg.data = class_name
        self.type_pub.publish(type_msg)
        
        # 发布可视化Marker
        marker = Marker()
        marker.header.frame_id = "camera"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "button_center"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = center_3d[0]
        marker.pose.position.y = center_3d[1]
        marker.pose.position.z = center_3d[2]
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ButtonDetectorNode()

    # 配置 RealSense（先初始化主程序的相机）
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 🔍 自动查询并选择最佳分辨率
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        node.get_logger().error("未找到RealSense设备！")
        return
    
    dev = devices[0]
    node.get_logger().info(f"✓ 检测到设备: {dev.get_info(rs.camera_info.name)}")
    
    # 查询深度和彩色传感器支持的分辨率
    depth_sensor = dev.first_depth_sensor()
    color_sensor = None
    for sensor in dev.query_sensors():
        if sensor.is_color_sensor():
            color_sensor = sensor
            break
    
    # 获取深度流支持的分辨率
    depth_profiles = depth_sensor.get_stream_profiles()
    depth_resolutions = set()
    for profile in depth_profiles:
        if profile.stream_type() == rs.stream.depth:
            video_profile = profile.as_video_stream_profile()
            fps = video_profile.fps()
            if fps == 30:  # 只考虑30fps
                depth_resolutions.add((video_profile.width(), video_profile.height()))
    
    # 获取彩色流支持的分辨率
    color_resolutions = set()
    if color_sensor:
        color_profiles = color_sensor.get_stream_profiles()
        for profile in color_profiles:
            if profile.stream_type() == rs.stream.color:
                video_profile = profile.as_video_stream_profile()
                fps = video_profile.fps()
                if fps == 30 and profile.format() == rs.format.bgr8:
                    color_resolutions.add((video_profile.width(), video_profile.height()))
    
    # 找到深度和彩色都支持的分辨率（按优先级）
    preferred_resolutions = [(848, 480), (640, 480), (1280, 720)]
    selected_width, selected_height = 640, 480  # 默认值
    
    for pref_w, pref_h in preferred_resolutions:
        if (pref_w, pref_h) in depth_resolutions and (pref_w, pref_h) in color_resolutions:
            selected_width, selected_height = pref_w, pref_h
            node.get_logger().info(f"✓ 选择分辨率: {selected_width}x{selected_height} (深度+彩色都支持)")
            break
    
    # 配置相机流
    config.enable_stream(rs.stream.depth, selected_width, selected_height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, selected_width, selected_height, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    node.get_logger().info("✓ RealSense相机启动成功（直接读取模式）")

    # AprilTag检测器实例
    # ⚠️ 关键参数：tag_size必须与实际AprilTag尺寸一致！
    APRILTAG_SIZE = 0.025  # 单位：米（25mm）⚠️ 请确认实际尺寸！
    apriltag_detector = Detector(
        families='tag25h9',
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.4,
        debug=0
    )
    node.get_logger().info("✓ AprilTag检测器初始化成功")
    node.get_logger().warn(f"⚠️  AprilTag尺寸设置: {APRILTAG_SIZE*1000:.0f}mm - 请确认实际尺寸是否一致！")
    
    # 相机内参（从RealSense获取）
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    
    # 🔥 关键修复：AprilTag检测用彩色图，应使用彩色流的内参！
    color_intrin_obj = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    depth_intrin_obj = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    
    # AprilTag检测使用彩色相机内参
    camera_params = [color_intrin_obj.fx, color_intrin_obj.fy, color_intrin_obj.ppx, color_intrin_obj.ppy]
    
    node.get_logger().info(f"✓ 相机内参已加载")
    node.get_logger().info(f"  彩色相机: fx={color_intrin_obj.fx:.1f}, fy={color_intrin_obj.fy:.1f}")
    node.get_logger().info(f"  深度相机: fx={depth_intrin_obj.fx:.1f}, fy={depth_intrin_obj.fy:.1f}")
    
    # 手眼标定参数（从piper_arm.py加载）
    if node.piper_arm is not None:
        from utils.utils_math import quaternion_to_rotation_matrix
        # 构建link6到camera的变换矩阵
        link6_T_camera = np.eye(4)
        link6_T_camera[:3, :3] = quaternion_to_rotation_matrix(node.piper_arm.link6_q_camera)
        link6_T_camera[:3, 3] = node.piper_arm.link6_t_camera
        
        # 打印手眼标定参数（调试用）
        node.get_logger().info(f"✓ 手眼标定矩阵已加载")
        node.get_logger().info(f"  link6_t_camera (平移): {node.piper_arm.link6_t_camera}")
        node.get_logger().info(f"  link6_q_camera (四元数): {node.piper_arm.link6_q_camera}")
    else:
        link6_T_camera = np.eye(4)
        link6_T_camera[2, 3] = 0.05
        node.get_logger().warn(f"⚠️  使用默认手眼标定参数")
    
    # 创建 OpenCV 窗口
    cv2.namedWindow('detection', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('detection', mouse_callback, node)

    global all_detections, selected_button_index, selected_button_locked
    global current_depth_data, current_color_data, current_depth_intrin
    global frame_counter, last_fps_time, fps_counter, current_fps
    global is_paused, paused_frame, paused_detections
    global selected_box_signature

    import signal
    import sys
    
    # 注册Ctrl+C信号处理器
    def signal_handler(sig, frame):
        print("\n\n收到退出信号 (Ctrl+C)，正在退出...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        node.get_logger().info("✓ 开始检测...")
        node.get_logger().info("  提示: 按 Ctrl+C 或 'q' 键退出")
        
        while rclpy.ok():
            # FPS统计
            fps_counter += 1
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                current_fps = fps_counter / (current_time - last_fps_time)
                fps_counter = 0
                last_fps_time = current_time
            
            # 获取相机帧（直接读取，无延迟）
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            aligned_depth = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not (aligned_depth and color_frame):
                continue
            
            depth_data = np.asanyarray(aligned_depth.get_data())
            color_data = np.asanyarray(color_frame.get_data())
            depth_intrin = aligned_depth.profile.as_video_stream_profile().intrinsics
            
            current_depth_data = depth_data
            current_color_data = color_data
            current_depth_intrin = depth_intrin
            
            # 暂停模式
            if is_paused:
                if paused_frame is not None:
                    display_img = visualize_detections(paused_frame, paused_detections, selected_button_index, None)
                    cv2.putText(display_img, "PAUSED - Click to Select", (10, 100), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                    cv2.imshow('detection', display_img)
                key = cv2.waitKey(1) & 0xFF
                if key == 32:  # SPACE
                    is_paused = False
                    node.get_logger().info("▶️  画面继续")
                    paused_frame = None
                    paused_detections = []
                elif key == 27:  # ESC
                    selected_button_index = -1
                    selected_button_locked = False
                    selected_box_signature = None
                elif key == ord('q'):
                    break
                continue

            # YOLO同步检测
            color_data_small = cv2.resize(color_data, None, fx=YOLO_SCALE_FACTOR, fy=YOLO_SCALE_FACTOR, interpolation=cv2.INTER_NEAREST)
            target_boxes_small = YOLODetection(node.model, color_data_small, conf_threshold=YOLO_CONF_THRESHOLD)
            all_detections = []
            for x1, y1, x2, y2, class_name, conf in target_boxes_small:
                all_detections.append((
                    int(x1 / YOLO_SCALE_FACTOR), int(y1 / YOLO_SCALE_FACTOR),
                    int(x2 / YOLO_SCALE_FACTOR), int(y2 / YOLO_SCALE_FACTOR),
                    class_name, conf, None
                ))

            # AprilTag检测与多坐标系转换
            gray = cv2.cvtColor(color_data, cv2.COLOR_BGR2GRAY)
            apriltag_detections = apriltag_detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=APRILTAG_SIZE  # 使用全局设置的标签尺寸
            )
            
            # 绘制AprilTag检测结果（边框和坐标轴）
            for det in apriltag_detections:
                # 绘制边框
                corners = det.corners.astype(int)
                for i in range(4):
                    pt1 = tuple(corners[i])
                    pt2 = tuple(corners[(i + 1) % 4])
                    cv2.line(color_data, pt1, pt2, (0, 255, 0), 2)
                
                # 绘制中心点
                center = det.center.astype(int)
                cv2.circle(color_data, tuple(center), 5, (0, 0, 255), -1)
                
                # 绘制ID
                cv2.putText(color_data, f"ID: {det.tag_id}", 
                            (center[0] - 20, center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # 绘制3D坐标轴
                if det.pose_t is not None and det.pose_R is not None:
                    # 构建坐标轴端点（3D）
                    axis_length = 0.025  # 25mm坐标轴长度
                    axis_3d = np.float32([
                        [axis_length, 0, 0],  # X轴 - 红色
                        [0, axis_length, 0],  # Y轴 - 绿色
                        [0, 0, axis_length],  # Z轴 - 蓝色
                        [0, 0, 0]             # 原点
                    ]).reshape(-1, 3)
                    
                    # 投影到图像平面
                    rvec, _ = cv2.Rodrigues(det.pose_R)
                    tvec = det.pose_t
                    
                    # 🔥 修复：使用彩色相机内参矩阵（AprilTag检测在彩色图上）
                    camera_matrix = np.array([
                        [color_intrin_obj.fx, 0, color_intrin_obj.ppx],
                        [0, color_intrin_obj.fy, color_intrin_obj.ppy],
                        [0, 0, 1]
                    ], dtype=np.float64)
                    
                    dist_coeffs = np.zeros(5, dtype=np.float64)
                    
                    imgpts, _ = cv2.projectPoints(axis_3d, rvec, tvec, camera_matrix, dist_coeffs)
                    imgpts = imgpts.astype(int)
                    
                    origin = tuple(imgpts[3].ravel())
                    # X轴 - 红色
                    cv2.line(color_data, origin, tuple(imgpts[0].ravel()), (0, 0, 255), 3)
                    # Y轴 - 绿色
                    cv2.line(color_data, origin, tuple(imgpts[1].ravel()), (0, 255, 0), 3)
                    # Z轴 - 蓝色
                    cv2.line(color_data, origin, tuple(imgpts[2].ravel()), (255, 0, 0), 3)
            
            # 准备AprilTag位姿信息用于UI显示
            apriltag_pose_info = []
            
            # 获取当前关节角度（用于基座坐标系转换）
            current_joints = None
            if node.piper is not None:
                try:
                    msg = node.piper.GetArmJointMsgs()
                    current_joints = [
                        msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
                        msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
                        msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
                        msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
                        msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
                        msg.joint_state.joint_6 * 1e-3 * PI / 180.0,
                    ]
                except:
                    pass
            
            for det in apriltag_detections:
                if det.pose_t is not None and det.pose_R is not None:
                    # 构建相机到tag的变换矩阵
                    camera_T_tag = np.eye(4)
                    camera_T_tag[:3, :3] = det.pose_R
                    camera_T_tag[:3, 3] = det.pose_t.flatten()
                    
                    # ========== 相机坐标系 ==========
                    x_cam, y_cam, z_cam = det.pose_t.flatten()
                    R_cam = det.pose_R
                    
                    # 🔍 调试：打印AprilTag原始检测数据（每30帧一次）
                    if frame_counter % 30 == 0:
                        node.get_logger().info(f"[AprilTag原始数据] ID:{det.tag_id}")
                        node.get_logger().info(f"  pose_t (平移): [{x_cam:.4f}, {y_cam:.4f}, {z_cam:.4f}] m")
                        node.get_logger().info(f"  pose_R (旋转):\n{R_cam}")
                    
                    # 欧拉角转换函数
                    def rotation_matrix_to_euler(R):
                        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
                        singular = sy < 1e-6
                        if not singular:
                            roll = np.arctan2(R[2,1], R[2,2])
                            pitch = np.arctan2(-R[2,0], sy)
                            yaw = np.arctan2(R[1,0], R[0,0])
                        else:
                            roll = np.arctan2(-R[1,2], R[1,1])
                            pitch = np.arctan2(-R[2,0], sy)
                            yaw = 0
                        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
                    
                    roll_cam, pitch_cam, yaw_cam = rotation_matrix_to_euler(R_cam)
                    
                    # ========== 夹爪坐标系 ==========
                    gripper_T_tag = link6_T_camera @ camera_T_tag
                    x_grip, y_grip, z_grip = gripper_T_tag[:3, 3]
                    R_grip = gripper_T_tag[:3, :3]
                    roll_grip, pitch_grip, yaw_grip = rotation_matrix_to_euler(R_grip)
                    
                    # 发布夹爪坐标系欧拉角（兼容）
                    euler_msg = Vector3()
                    euler_msg.x = roll_grip
                    euler_msg.y = pitch_grip
                    euler_msg.z = yaw_grip
                    node.normal_pub.publish(euler_msg)
                    
                    # ========== 基座坐标系 ==========
                    base_info = None
                    gripper_in_base_rpy = None
                    if node.piper_arm is not None and current_joints is not None:
                        try:
                            # 计算正向运动学
                            base_T_link6 = node.piper_arm.forward_kinematics(current_joints)
                            base_T_tag = base_T_link6 @ link6_T_camera @ camera_T_tag
                            
                            x_base, y_base, z_base = base_T_tag[:3, 3]
                            R_base = base_T_tag[:3, :3]
                            roll_base, pitch_base, yaw_base = rotation_matrix_to_euler(R_base)
                            
                            # 发布基座坐标系欧拉角（推荐使用）
                            euler_base_msg = Vector3()
                            euler_base_msg.x = roll_base
                            euler_base_msg.y = pitch_base
                            euler_base_msg.z = yaw_base
                            node.normal_base_pub.publish(euler_base_msg)
                            
                            base_info = {
                                'position': [x_base, y_base, z_base],
                                'rpy': [roll_base, pitch_base, yaw_base]
                            }
                            
                            # 计算夹爪在基座系下的RPY（直接从FK结果提取）
                            R_gripper_in_base = base_T_link6[:3, :3]
                            roll_grip_base, pitch_grip_base, yaw_grip_base = rotation_matrix_to_euler(R_gripper_in_base)
                            gripper_in_base_rpy = [roll_grip_base, pitch_grip_base, yaw_grip_base]
                        except:
                            pass
                    
                    # 收集位姿信息用于UI显示
                    pose_info = {
                        'tag_id': det.tag_id,
                        'camera': {
                            'position': [x_cam, y_cam, z_cam],
                            'rpy': [roll_cam, pitch_cam, yaw_cam]
                        },
                        'gripper': {
                            'position': [x_grip, y_grip, z_grip],
                            'rpy': [roll_grip, pitch_grip, yaw_grip]
                        },
                        'gripper_in_base_rpy': gripper_in_base_rpy,
                        'base': base_info
                    }
                    apriltag_pose_info.append(pose_info)
                    
                    # 每秒打印一次到终端（减少信息量）
                    if frame_counter % 30 == 0:
                        node.get_logger().info(f"[AprilTag ID:{det.tag_id}] Base XYZ: ({x_base:.3f}, {y_base:.3f}, {z_base:.3f}) m")
            
            # 提醒：如果未检测到AprilTag
            if len(apriltag_detections) == 0 and frame_counter % 90 == 0:  # 每3秒提醒一次
                node.get_logger().warn("⚠️  未检测到AprilTag，请确保标签在视野内且光照充足")
            
            # 可视化（包含AprilTag位姿信息）
            display_img = visualize_detections(color_data, all_detections, selected_button_index, apriltag_pose_info)
            cv2.imshow('detection', display_img)
            
            # 帧计数器增加
            frame_counter += 1
            
            # 键盘处理
            wait_time = max(1, int(1000 / UI_REFRESH_RATE))
            key = cv2.waitKey(wait_time) & 0xFF
            
            if key == 32:  # SPACE
                is_paused = not is_paused
                if is_paused:
                    node.get_logger().info("⏸️  画面已暂停")
                    paused_frame = color_data.copy()
                    paused_detections = list(all_detections)
            elif key == 27:  # ESC
                node.get_logger().info("✗ 已取消选择，解除锁定")
                selected_button_index = -1
                selected_button_locked = False
                selected_box_signature = None
            elif key == 13:  # ENTER
                if selected_button_index >= 0 and selected_button_index < len(all_detections):
                    det = all_detections[selected_button_index]
                    x1, y1, x2, y2, class_name, conf, center_3d = det
                    if center_3d is None:
                        # 简化的3D点提取
                        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                        depth = depth_data[cy, cx] * 0.001  # mm -> m
                        if depth > 0:
                            center_3d = np.array([
                                (cx - depth_intrin.ppx) * depth / depth_intrin.fx,
                                (cy - depth_intrin.ppy) * depth / depth_intrin.fy,
                                depth
                            ])
                    if center_3d is not None:
                        # 发布到ROS2话题（相机坐标系，兼容）
                        node.publish_result(center_3d, class_name)
                        
                        # 转换到基座坐标系
                        base_3d = None
                        if node.piper_arm is not None:
                            base_3d = transform_button_camera_to_base(center_3d, node.piper, node.piper_arm)
                        
                        # 发布基座坐标系位置
                        if base_3d is not None:
                            point_base_msg = PointStamped()
                            point_base_msg.header.stamp = node.get_clock().now().to_msg()
                            point_base_msg.header.frame_id = "arm_base"
                            point_base_msg.point.x = base_3d[0]
                            point_base_msg.point.y = base_3d[1]
                            point_base_msg.point.z = base_3d[2]
                            node.point_base_pub.publish(point_base_msg)
                        
                        # 打印详细信息
                        node.get_logger().info(f"\n{'='*70}")
                        node.get_logger().info(f"✓ 已确认并发布按钮:")
                        node.get_logger().info(f"  类型: {class_name}")
                        node.get_logger().info(f"  [相机坐标系] XYZ: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f}) m")
                        
                        if base_3d is not None:
                            node.get_logger().info(f"  [基座坐标系] XYZ: ({base_3d[0]:.3f}, {base_3d[1]:.3f}, {base_3d[2]:.3f}) m")
                        else:
                            node.get_logger().info(f"  [基座坐标系] 转换失败")
                        
                        node.get_logger().info(f"{'='*70}")
                        
                        selected_button_index = -1
                        selected_button_locked = False
                        selected_box_signature = None
            
            elif key == ord('q'):
                break
            
            # ROS2 spin
            rclpy.spin_once(node, timeout_sec=0.001)
    
    except KeyboardInterrupt:
        print("\n用户中断，正在清理资源...")
    except Exception as e:
        print(f"\n程序异常退出: {e}")
    finally:
        print("正在关闭相机...")
        try:
            pipeline.stop()
        except:
            pass
        
        print("正在关闭窗口...")
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)  # 强制刷新窗口事件
        except:
            pass
        
        print("正在清理ROS2节点...")
        try:
            node.destroy_node()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("✓ 清理完成，程序退出")


if __name__ == '__main__':
    main()
