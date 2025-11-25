#!/usr/bin/env python3
"""
交互式按钮检测器 - ROS2 版本（订阅话题）
订阅 RealSense ROS2 节点的图像话题
使用 yolo_button.pt 模型
支持用户点击选择要操作的按钮

性能优化：
- 降低YOLO检测频率（DETECTION_SKIP_FRAMES）
- 滑动窗口平滑检测框（CACHE_SIZE）
- 减少重复渲染
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import numpy as np
import message_filters
import math

# 导入坐标转换所需的模块
from piper_sdk import C_PiperInterface_V2
from piper_arm import PiperArm
from utils.utils_math import quaternion_to_rotation_matrix

PI = math.pi

# ========================================
# 性能调优参数
# ========================================
DETECTION_SKIP_FRAMES = 1  # 每2帧检测1次（平衡流畅度和性能）
CACHE_SIZE = 1  # 不平滑，直接使用当前帧结果
YOLO_CONF_THRESHOLD = 0.5  # 置信度阈值
KEYBOARD_TIMER_HZ = 30  # 键盘响应速度

# ========================================
# 全局变量
# ========================================
# 检测结果存储
all_detections = []  # [(x1, y1, x2, y2, class_name, conf, center_3d), ...]
selected_button_index = -1  # 用户选中的按钮索引
confirmed = False
selected_button_locked = False
selected_box_signature = None  # 记录当前选中的检测框特征（用于重新匹配）

# 检测结果稳定性控制
detection_cache = []  # 缓存最近N帧的检测结果
frame_counter = 0  # 帧计数器

# 鼠标位置
mouse_x, mouse_y = 0, 0

# 当前帧数据
current_depth_data = None
current_color_data = None
current_camera_info = None
current_annotated_img = None  # 缓存标注图像，避免重复生成

# 交互控制
is_paused = False  # 是否暂停画面（方便选择）
paused_frame = None  # 暂停时的图像
paused_detections = []  # 暂停时的检测结果

# FPS统计
import time
last_fps_time = time.time()
fps_counter = 0
current_fps = 0.0


# ========================================
# 鼠标回调函数
# ========================================
def mouse_callback(event, x, y, flags, param):
    """处理鼠标事件，允许用户点击选择按钮"""
    global selected_button_index, mouse_x, mouse_y, all_detections
    global current_depth_data, current_color_data, current_camera_info
    global selected_button_locked
    global is_paused, paused_detections
    global selected_box_signature
    
    mouse_x, mouse_y = x, y
    
    if event == cv2.EVENT_LBUTTONDOWN:  # 左键点击
        # 修正坐标：显示图像顶部有60像素的提示栏
        TIP_BAR_HEIGHT = 60
        corrected_y = y - TIP_BAR_HEIGHT
        
        print(f"\n[鼠标点击] 原始: ({x}, {y}), 修正: ({x}, {corrected_y})")
        
        # 如果点击在提示栏上，忽略
        if corrected_y < 0:
            return
        
        # 检查点击位置是否在某个检测框内
        # 如果暂停，使用暂停时的检测结果
        detections_to_check = paused_detections if is_paused else all_detections
        
        found = False
        for idx, det in enumerate(detections_to_check):
            x1, y1, x2, y2, class_name, conf, center_3d = det
            
            if x1 <= x <= x2 and y1 <= corrected_y <= y2:
                print(f" → ✓ 匹配按钮 #{idx}")
                found = True
                selected_button_index = idx
                selected_button_locked = True
                
                # 立即计算3D位置
                current_det = det
                if center_3d is None and current_depth_data is not None:
                    center_3d, stats = extract_roi_cloud(
                        current_depth_data, 
                        current_color_data, 
                        [x1, y1, x2, y2], 
                        current_camera_info,
                        verbose=False,  # 点击时不打印详细信息，避免卡顿
                        show_depth_map=False  # 可设为True查看深度热力图
                    )
                    
                    # 更新检测结果
                    if is_paused:
                        paused_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                        # 同步更新全局结果（如果对应的话）
                        if idx < len(all_detections):
                            all_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                            current_det = all_detections[idx]
                        else:
                            current_det = paused_detections[idx]
                    else:
                        all_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                        current_det = all_detections[idx]

                # 记录当前选中框的签名，便于后续保持高亮
                remember_selected_detection(current_det)
                
                print(f"\n{'='*60}")
                print(f"✓ 已选择按钮 #{idx}")
                print(f"  类型: {class_name}")
                print(f"  置信度: {conf:.2f}")
                if center_3d is not None:
                    print(f"  相机坐标: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f}) m")
                    # 尝试转换到基座坐标系
                    if param is not None and hasattr(param, 'piper') and param.piper is not None:
                        base_3d = transform_camera_to_base(center_3d, param.piper, param.piper_arm)
                        if base_3d is not None:
                            print(f"  基座坐标: ({base_3d[0]:.3f}, {base_3d[1]:.3f}, {base_3d[2]:.3f}) m")
                print(f"{'='*60}")
                break
        
        if not found:
            print(" → ✗ 未点击到按钮")


# ========================================
# YOLO 检测（带平滑）
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
    
    # 按从左到右、从上到下排序（固定编号）
    target_boxes.sort(key=lambda box: (box[1] // 100, box[0]))  # 先按y坐标分组，再按x排序
    
    return target_boxes


def smooth_detections(new_detections):
    """
    平滑检测结果，减少抖动
    使用滑动窗口平均检测框位置
    """
    global detection_cache
    
    # 添加到缓存
    detection_cache.append(new_detections)
    if len(detection_cache) > CACHE_SIZE:
        detection_cache.pop(0)
    
    # 如果缓存不足，直接返回
    if len(detection_cache) < 2:
        return new_detections
    
    # 对每个检测框，在缓存中找到相似的框并平均
    smoothed = []
    for det in new_detections:
        x1, y1, x2, y2, class_name, conf = det
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        
        # 在缓存中查找相似位置的同类按钮
        similar_boxes = []
        for cached_detections in detection_cache:
            for cached_det in cached_detections:
                cx2, cy2 = (cached_det[0] + cached_det[2]) / 2, (cached_det[1] + cached_det[3]) / 2
                distance = np.sqrt((cx - cx2)**2 + (cy - cy2)**2)
                
                # 如果中心距离<50像素 且 类型相同，认为是同一个按钮
                if distance < 50 and cached_det[4] == class_name:
                    similar_boxes.append(cached_det)
        
        # 平均坐标
        if len(similar_boxes) > 0:
            avg_x1 = int(np.mean([b[0] for b in similar_boxes]))
            avg_y1 = int(np.mean([b[1] for b in similar_boxes]))
            avg_x2 = int(np.mean([b[2] for b in similar_boxes]))
            avg_y2 = int(np.mean([b[3] for b in similar_boxes]))
            avg_conf = np.mean([b[5] for b in similar_boxes])
            smoothed.append((avg_x1, avg_y1, avg_x2, avg_y2, class_name, avg_conf))
        else:
            smoothed.append(det)
    
    return smoothed


def _make_signature_from_detection(det):
    """根据检测框生成唯一签名（类别 + 中心点）"""
    x1, y1, x2, y2, class_name, *_ = det
    center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    return {
        "class": class_name,
        "center": center,
        "bbox": (x1, y1, x2, y2),
    }


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

    # 如果找到足够接近的框，则更新索引；否则清空选择
    if best_idx != -1 and best_dist < 80:  # 80像素阈值，可根据需要调整
        selected_button_index = best_idx
    else:
        selected_button_index = -1
        selected_box_signature = None


# ========================================
# 可视化
# ========================================
def visualize_detections(color_img, detections, selected_idx):
    """可视化检测结果"""
    annotated = color_img.copy()
    
    for idx, det in enumerate(detections):
        x1, y1, x2, y2, class_name, conf, center_3d = det
        
        # 选中的按钮用绿色，其他用蓝色
        if idx == selected_idx:
            color = (0, 255, 0)  # 绿色
            thickness = 3
        else:
            color = (255, 0, 0)  # 蓝色
            thickness = 2
        
        # 绘制边界框
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
        
        # 显示标签和置信度
        label = f"#{idx} {class_name} {conf:.2f}"
        cv2.putText(
            annotated, label, (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )
        
        # 显示3D坐标
        if center_3d is not None:
            coord_text = f"({center_3d[0]:.2f}, {center_3d[1]:.2f}, {center_3d[2]:.2f})"
            cv2.putText(
                annotated, coord_text, (x1, y2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
            )
    
    # 顶部提示（包含FPS）
    tip_bg = np.zeros((60, annotated.shape[1], 3), dtype=np.uint8)
    tip_bg[:] = (50, 50, 50)
    
    cv2.putText(
        tip_bg, "Step 1: Click on a button to select", (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
    )
    cv2.putText(
        tip_bg, "Step 2: Press ENTER to confirm | ESC to cancel", (10, 45),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
    )
    
    # 提示空格键暂停
    cv2.putText(
        tip_bg, "[SPACE] Pause/Resume", (annotated.shape[1] - 350, 35),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1
    )
    
    # 显示FPS
    global current_fps
    fps_text = f"FPS: {current_fps:.1f}"
    cv2.putText(
        tip_bg, fps_text, (annotated.shape[1] - 120, 35),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
    )
    
    annotated = np.vstack([tip_bg, annotated])
    
    # 不显示鼠标位置（避免卡顿）
    
    return annotated


# ========================================
# ROI 点云提取
# ========================================
def extract_roi_cloud(depth_data, color_data, bbox, camera_info, verbose=False, show_depth_map=False):
    """从深度图中提取 ROI 区域的 3D 中心点，并返回完整点云用于debug"""
    x1, y1, x2, y2 = bbox
    
    # 计算2D边界框的中心点
    center_u = int((x1 + x2) / 2)
    center_v = int((y1 + y2) / 2)
    
    # 获取相机内参
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]
    
    # ========== 1. 快速统计ROI深度信息（不生成完整点云） ==========
    roi_depth = depth_data[y1:y2, x1:x2]
    valid_roi_depths = roi_depth[roi_depth > 0]
    
    # 可选：显示深度图热力图（debug用）
    if show_depth_map and roi_depth.size > 0:
        # 归一化深度图用于可视化
        depth_normalized = roi_depth.copy().astype(float)
        depth_normalized[depth_normalized == 0] = np.nan  # 无效值设为nan
        if np.nanmax(depth_normalized) > 0:
            depth_normalized = (depth_normalized - np.nanmin(depth_normalized)) / (np.nanmax(depth_normalized) - np.nanmin(depth_normalized)) * 255
            depth_normalized = np.nan_to_num(depth_normalized, nan=0).astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            # 放大显示
            depth_colormap_large = cv2.resize(depth_colormap, (200, 200), interpolation=cv2.INTER_NEAREST)
            cv2.imshow('ROI Depth Map', depth_colormap_large)

    
    # 点云统计信息（轻量级）
    point_cloud_stats = {
        'bbox_size': (x2-x1, y2-y1),
        'total_pixels': roi_depth.size,
        'valid_count': len(valid_roi_depths),
        'coverage': 100 * len(valid_roi_depths) / roi_depth.size if roi_depth.size > 0 else 0,
        'depth_range': (valid_roi_depths.min(), valid_roi_depths.max()) if len(valid_roi_depths) > 0 else (0, 0),
        'depth_mean': valid_roi_depths.mean() if len(valid_roi_depths) > 0 else 0,
        'depth_std': valid_roi_depths.std() if len(valid_roi_depths) > 0 else 0,
    }
    
    # 只在verbose模式下打印详细信息
    if verbose:
        print(f"\n{'='*60}")
        print(f"[ROI] 尺寸:{point_cloud_stats['bbox_size'][0]}x{point_cloud_stats['bbox_size'][1]} "
              f"覆盖:{point_cloud_stats['coverage']:.1f}% "
              f"深度:{point_cloud_stats['depth_mean']:.0f}±{point_cloud_stats['depth_std']:.0f}mm")
    
    # ========== 2. 智能采样策略：使用中位数+离群点过滤 ==========
    sample_size = 5  # 扩大采样窗口到5x5（更多样本）
    u_min = max(0, center_u - sample_size)
    u_max = min(depth_data.shape[1], center_u + sample_size)
    v_min = max(0, center_v - sample_size)
    v_max = min(depth_data.shape[0], center_v + sample_size)
    
    depth_window = depth_data[v_min:v_max, u_min:u_max]
    valid_depths = depth_window[depth_window > 0]
    
    if len(valid_depths) == 0:
        if verbose:
            print(f"[中心点] ⚠️ 无有效深度")
            print(f"{'='*60}\n")
        return None, point_cloud_stats
    
    # 🔥 关键优化：使用中位数而不是平均值，更抗离群点
    depth_median = np.median(valid_depths)
    depth_mean = np.mean(valid_depths)
    depth_std = np.std(valid_depths)
    
    # 如果标准差太大（>200mm），说明有离群点，过滤掉
    if depth_std > 200:
        # 使用 median ± 1.5*std 过滤离群点
        lower_bound = depth_median - 1.5 * depth_std
        upper_bound = depth_median + 1.5 * depth_std
        filtered_depths = valid_depths[(valid_depths >= lower_bound) & (valid_depths <= upper_bound)]
        
        if len(filtered_depths) > 0:
            if verbose:
                print(f"[中心点] ⚠️ 检测到离群点，过滤前:{len(valid_depths)}个 → 过滤后:{len(filtered_depths)}个")
            valid_depths = filtered_depths
            depth_median = np.median(valid_depths)
    
    # 优先使用中位数（更鲁棒）
    depth_value = depth_median
    depth_m = depth_value / 1000.0
    
    # 扩大合理范围：0.15m ~ 1.5m（按钮操作的实际距离）
    if depth_m < 0.15 or depth_m > 1.5:
        if verbose:
            print(f"[中心点] ⚠️ 深度异常: median={depth_median:.0f}mm ({depth_m:.3f}m), mean={depth_mean:.0f}mm, std={depth_std:.0f}mm")
            print(f"         可能原因: 材质反光/边缘效应/深度孔洞")
            print(f"{'='*60}\n")
        return None, point_cloud_stats
    
    # 使用针孔相机模型计算3D坐标
    x = (center_u - cx) * depth_m / fx
    y = (center_v - cy) * depth_m / fy
    z = depth_m
    
    if verbose:
        print(f"[中心点] 深度:{depth_value:.0f}mm → 3D:({x:.3f}, {y:.3f}, {z:.3f})m")
        print(f"{'='*60}\n")
    
    return [x, y, z], point_cloud_stats


# ========================================
# 坐标转换：相机 → 基座
# ========================================
def transform_camera_to_base(button_camera, piper, piper_arm):
    """
    将相机坐标系的按钮位置转换到基座坐标系
    
    Args:
        button_camera: [x, y, z] 相机坐标系
        piper: Piper SDK实例
        piper_arm: PiperArm实例
    
    Returns:
        button_base: [x, y, z] 基座坐标系
    """
    try:
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
        
        # 基座到link6的变换
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
        print(f"  ⚠️  坐标转换失败: {e}")
        return None


# ========================================
# ROS2 节点
# ========================================
class ButtonDetectorNode(Node):
    """按钮检测节点 - 订阅 ROS2 话题版本"""
    
    def __init__(self):
        super().__init__('button_detector_ros2')
        
        self.get_logger().info("="*70)
        self.get_logger().info("交互式按钮检测器 - ROS2 版本（订阅话题）")
        self.get_logger().info("="*70)
        self.get_logger().info(f"性能配置:")
        self.get_logger().info(f"  - YOLO检测频率: 每{DETECTION_SKIP_FRAMES+1}帧1次")
        self.get_logger().info(f"  - 平滑窗口大小: {CACHE_SIZE}帧")
        self.get_logger().info(f"  - 置信度阈值: {YOLO_CONF_THRESHOLD}")
        self.get_logger().info("="*70)
        
        # 初始化 Piper SDK（用于坐标转换）
        try:
            self.piper = C_PiperInterface_V2("can0")
            self.piper.ConnectPort()
            self.piper_arm = PiperArm()
            self.get_logger().info("✓ Piper SDK 初始化成功（用于坐标转换）")
        except Exception as e:
            self.get_logger().warn(f"⚠️  Piper SDK 初始化失败: {e}")
            self.get_logger().warn("   将只显示相机坐标系坐标")
            self.piper = None
            self.piper_arm = None
        
        # 初始化 CV Bridge
        self.bridge = CvBridge()
        
        # 加载 YOLO 模型
        self.model = YOLO('yolo_button.pt')
        self.get_logger().info("✓ YOLO 模型加载成功: yolo_button.pt")
        
        # 创建发布器
        self.point_pub = self.create_publisher(PointStamped, '/object_point', 10)
        self.type_pub = self.create_publisher(String, '/button_type', 10)
        self.marker_pub = self.create_publisher(Marker, '/object_center_marker', 10)
        
        self.get_logger().info("✓ 发布器已创建")
        self.get_logger().info("  - /object_point")
        self.get_logger().info("  - /button_type")
        self.get_logger().info("  - /object_center_marker")
        
        # 订阅相机话题（使用时间同步）
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw'
        )
        # 🔥 关键修改：订阅对齐到彩色图的深度图，而不是原始深度图
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info'
        )
        
        # 时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.image_callback)
        
        self.get_logger().info("✓ 已订阅相机话题（使用对齐深度图）")
        self.get_logger().info("  - /camera/camera/color/image_raw")
        self.get_logger().info("  - /camera/camera/aligned_depth_to_color/image_raw  🔥 对齐版本")
        self.get_logger().info("  - /camera/camera/aligned_depth_to_color/camera_info")
        
        # 创建 OpenCV 窗口
        cv2.namedWindow('detection', cv2.WINDOW_AUTOSIZE)
        # 将self作为param传递给鼠标回调，以便访问piper实例
        cv2.setMouseCallback('detection', mouse_callback, self)
        
        # 创建定时器处理键盘输入（降低频率，减少CPU占用）
        self.timer = self.create_timer(1.0 / KEYBOARD_TIMER_HZ, self.keyboard_callback)
        
        self.get_logger().info("="*70)
        self.get_logger().info("✓ 初始化完成，等待图像...")
        self.get_logger().info("="*70)
    
    def image_callback(self, color_msg, depth_msg, camera_info_msg):
        """图像话题回调 - 优化版：降低检测频率但保持显示流畅"""
        global all_detections, selected_button_index, selected_button_locked
        global current_depth_data, current_color_data, current_camera_info
        global frame_counter, current_annotated_img
        global last_fps_time, fps_counter, current_fps
        global is_paused, paused_frame, paused_detections
        
        try:
            # FPS统计
            fps_counter += 1
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                current_fps = fps_counter / (current_time - last_fps_time)
                fps_counter = 0
                last_fps_time = current_time
            
            # 如果处于暂停状态，只显示暂停帧，不更新数据
            if is_paused:
                if paused_frame is not None:
                    # 在暂停帧上绘制当前的选中状态
                    display_img = visualize_detections(
                        paused_frame, paused_detections, selected_button_index
                    )
                    # 添加暂停提示
                    cv2.putText(display_img, "PAUSED - Click to Select", (10, 100), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                    cv2.imshow('detection', display_img)
                return

            # 转换 ROS 图像消息为 OpenCV 格式
            color_data_full = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_data = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            # 保存原始数据用于深度采样
            current_depth_data = depth_data
            current_color_data = color_data_full
            current_camera_info = camera_info_msg
            
            # 隔帧检测（降低CPU占用）
            frame_counter += 1
            should_detect = (frame_counter % (DETECTION_SKIP_FRAMES + 1) == 0)
            
            if should_detect:
                # 降低分辨率用于YOLO检测（提速3-4倍）
                scale_factor = 0.5  # 缩小到原来的一半
                color_data_small = cv2.resize(color_data_full, None, fx=scale_factor, fy=scale_factor, 
                                              interpolation=cv2.INTER_LINEAR)
                
                # 使用缩小的图像进行YOLO检测
                target_boxes_small = YOLODetection(self.model, color_data_small, conf_threshold=YOLO_CONF_THRESHOLD)
                
                # 将检测框坐标缩放回原始尺寸
                target_boxes = []
                for x1, y1, x2, y2, class_name, conf in target_boxes_small:
                    target_boxes.append((
                        int(x1 / scale_factor),
                        int(y1 / scale_factor),
                        int(x2 / scale_factor),
                        int(y2 / scale_factor),
                        class_name,
                        conf
                    ))
                
                # 直接更新全局检测结果（不平滑，避免延迟）
                all_detections = []
                for box in target_boxes:
                    x1, y1, x2, y2, class_name, conf = box
                    all_detections.append((x1, y1, x2, y2, class_name, conf, None))

                # 重新关联选中状态
                sync_selection_with_detections()
            
            # 每帧都重新绘制（保持选中框实时更新）
            display_img = visualize_detections(
                color_data_full, all_detections, selected_button_index
            )
            
            # 显示图像
            cv2.imshow('detection', display_img)
            
        except Exception as e:
            self.get_logger().error(f"处理图像错误: {e}")
    
    def keyboard_callback(self):
        """处理键盘输入"""
        global all_detections, selected_button_index, selected_button_locked
        global current_depth_data, current_color_data, current_camera_info
        global is_paused, paused_frame, paused_detections
        global selected_box_signature
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == 32:  # SPACE - 暂停/继续
            is_paused = not is_paused
            if is_paused:
                self.get_logger().info("⏸️  画面已暂停 - 请点击选择按钮")
                # 保存当前状态
                if current_color_data is not None:
                    paused_frame = current_color_data.copy()
                    paused_detections = list(all_detections)
            else:
                self.get_logger().info("▶️  画面继续")
                paused_frame = None
                paused_detections = []
        
        elif key == 27:  # ESC - 取消选择
            self.get_logger().info("✗ 已取消选择")
            selected_button_index = -1
            selected_button_locked = False
            selected_box_signature = None
        
        elif key == 13:  # ENTER - 确认选择
            # 确定要使用的检测列表
            detections_to_use = paused_detections if is_paused else all_detections
            
            if selected_button_index >= 0 and selected_button_index < len(detections_to_use):
                det = detections_to_use[selected_button_index]
                x1, y1, x2, y2, class_name, conf, center_3d = det
                
                # 计算3D位置
                if center_3d is None:
                    center_3d, stats = extract_roi_cloud(
                        current_depth_data, current_color_data, 
                        [x1, y1, x2, y2], 
                        current_camera_info,
                        verbose=True,  # 确认时打印详细信息用于debug
                        show_depth_map=True  # 显示深度热力图帮助诊断
                    )
                    
                    # 显示简洁的统计信息
                    if stats:
                        self.get_logger().info(
                            f"[ROI] 尺寸:{stats['bbox_size'][0]}x{stats['bbox_size'][1]} "
                            f"覆盖率:{stats['coverage']:.1f}% "
                            f"深度:{stats['depth_mean']:.0f}±{stats['depth_std']:.0f}mm"
                        )
                    
                    # 更新列表
                    if is_paused:
                        paused_detections[selected_button_index] = (x1, y1, x2, y2, class_name, conf, center_3d)
                    else:
                        all_detections[selected_button_index] = (x1, y1, x2, y2, class_name, conf, center_3d)
                
                if center_3d is not None:
                    self.get_logger().info(f"\n{'='*70}")
                    self.get_logger().info(f"✓ 确认选择并发布:")
                    self.get_logger().info(f"  按钮类型: {class_name}")
                    self.get_logger().info(f"  置信度: {conf:.2f}")
                    self.get_logger().info(f"  相机坐标: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f}) m")
                    
                    # 显示基座坐标系坐标
                    if self.piper is not None:
                        base_3d = transform_camera_to_base(center_3d, self.piper, self.piper_arm)
                        if base_3d is not None:
                            self.get_logger().info(f"  基座坐标: ({base_3d[0]:.3f}, {base_3d[1]:.3f}, {base_3d[2]:.3f}) m")
                    
                    self.get_logger().info(f"{'='*70}")
                    
                    # 发布到ROS话题
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = "camera"  # ✓ 修改为与TF发布器一致
                    point_msg.point.x = center_3d[0]
                    point_msg.point.y = center_3d[1]
                    point_msg.point.z = center_3d[2]
                    self.point_pub.publish(point_msg)
                    
                    type_msg = String()
                    type_msg.data = class_name
                    self.type_pub.publish(type_msg)
                    
                    # 发布可视化 Marker
                    marker = Marker()
                    marker.header.frame_id = "camera"  # ✓ 修改为与TF发布器一致
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
                    
                    self.get_logger().info("✓ 已发布到 ROS2 话题")
                    
                    # 重置选择
                    selected_button_index = -1
                    selected_button_locked = False
                    selected_box_signature = None
        
        elif key == ord('q'):  # 退出
            self.get_logger().info("退出程序...")
            raise KeyboardInterrupt
    
    def destroy_node(self):
        """清理资源"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ButtonDetectorNode()
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


if __name__ == '__main__':
    main()
