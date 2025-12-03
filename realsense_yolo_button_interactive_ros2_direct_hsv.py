#!/usr/bin/env python3
"""
交互式按钮检测器 - ROS2 混合版本
使用 pyrealsense2 直接读取相机（高性能）+ ROS2 发布结果
解决订阅话题导致的卡顿和幻影问题
修复：ROS2 spin线程独立运行，避免阻塞
新增：面板法向量计算（支持倾斜面板）
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
import threading
from queue import Queue

# 导入坐标转换所需的模块
from piper_sdk import C_PiperInterface_V2
from piper_arm import PiperArm
from utils.utils_math import quaternion_to_rotation_matrix
import math

# 导入面板法向量计算工具
from utils.utils_plane import (
    compute_robust_panel_normal, 
    compute_panel_normal_from_blue_region, 
    visualize_panel_normal,
    BLUE_HSV_LOWER as UTILS_HSV_LOWER,  # 导入 utils_plane.py 中的参数
    BLUE_HSV_UPPER as UTILS_HSV_UPPER
)

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

# 🔧 异步检测相关
detection_lock = threading.Lock()  # 保护检测结果的锁
detection_queue = Queue(maxsize=5)  # 🔧 待检测帧队列（缓存5帧，检测线程自动取最新）
detection_running = True  # 检测线程运行标志
detection_frozen = False  # 🔧 检测冻结标志（选中后停止检测更新）

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

# 面板法向量缓存
global_panel_normal = None
global_panel_normal_base = None  # 🔧 实时存储基座系法向量
global_panel_info = None
panel_last_update = 0.0
PANEL_CACHE_TIME = 0.0  # � 设为0表示每次检测都更新法向量，实现实时效果

# 🎨 HSV颜色过滤参数（蓝色面板）
# 使用 tune_blue_hsv.py 调试得到的最佳参数
# 可通过环境变量覆盖：export BLUE_HSV_LOWER="92,108,43" BLUE_HSV_UPPER="111,179,244"
import os
def _parse_hsv_env(var_name, default):
    env_val = os.environ.get(var_name)
    if env_val:
        try:
            return np.array([int(x) for x in env_val.split(',')])
        except:
            print(f"⚠️  环境变量 {var_name} 格式错误，使用默认值")
    return default

# 🔧 使用 utils_plane.py 中精心调整过的参数作为默认值
BLUE_HSV_LOWER = _parse_hsv_env('BLUE_HSV_LOWER', UTILS_HSV_LOWER)
BLUE_HSV_UPPER = _parse_hsv_env('BLUE_HSV_UPPER', UTILS_HSV_UPPER)

print(f"🎨 HSV颜色过滤参数 (来自 utils_plane.py):")
print(f"  下限: {BLUE_HSV_LOWER}")
print(f"  上限: {BLUE_HSV_UPPER}")
print(f"  注意: 这些参数与 utils_plane.py 同步，已精心调整")


# ========================================
# 鼠标回调函数
# ========================================
def mouse_callback(event, x, y, flags, param):
    """处理鼠标事件，允许用户点击选择按钮"""
    global selected_button_index, mouse_x, mouse_y, all_detections
    global current_depth_data, current_color_data, current_depth_intrin
    global selected_button_locked, is_paused, paused_detections
    global selected_box_signature, global_panel_info
    
    mouse_x, mouse_y = x, y
    
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"\n{'='*70}")
        print(f"[鼠标点击] 位置: ({x}, {y})")
        print(f"[检测状态] 当前检测到 {len(all_detections)} 个按钮")
        
        # 🔒 线程安全地读取检测结果
        with detection_lock:
            detections_to_check = paused_detections if is_paused else list(all_detections)
            panel_info_snapshot = global_panel_info
        
        found = False
        for idx, det in enumerate(detections_to_check):
            x1, y1, x2, y2, class_name, conf, center_3d = det
            
            print(f"  检查按钮 #{idx}: 类型={class_name}, 框=[{x1},{y1},{x2},{y2}]", end="")
            
            if x1 <= x <= x2 and y1 <= y <= y2:
                print(" → ✓ 匹配!")
                found = True
                selected_button_index = idx
                selected_button_locked = True
                
                # 🔧 不再冻结检测，保持检测框实时跟随
                global detection_frozen
                detection_frozen = False  # 改为False，保持检测更新
                
                # 🔧 立即强制刷新显示（不等待下一帧）
                if current_color_data is not None:
                    instant_display = visualize_detections(
                        current_color_data,
                        detections_to_check,
                        selected_button_index,
                        panel_info=panel_info_snapshot
                    )
                    cv2.imshow('detection', instant_display)
                    cv2.waitKey(1)  # 立即刷新
                    
                # 🔊 可选：终端响铃（提供音频反馈）
                print('\a', end='', flush=True)  # ASCII Bell
                
                if center_3d is None and current_depth_data is not None:
                    center_3d, stats = extract_roi_cloud(
                        current_depth_data, 
                        current_color_data, 
                        [x1, y1, x2, y2], 
                        current_depth_intrin,
                        verbose=False
                    )
                    
                    if is_paused:
                        paused_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                    
                    # 🔒 线程安全地更新all_detections
                    with detection_lock:
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
                    print(f"  3D位置: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f})")
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
# 🔧 异步检测线程
# ========================================
def async_detection_worker(model, node):
    """
    后台检测线程：从队列获取帧，执行YOLO检测，更新全局结果
    这样主线程可以保持30fps实时显示画面
    """
    global all_detections, detection_running, last_detect_time_ms
    global global_panel_normal, global_panel_normal_base
    global global_panel_info, panel_last_update
    
    node.get_logger().info("🚀 异步检测线程已启动")
    
    while detection_running and rclpy.ok():
        try:
            # 🔧 极限优化：清空队列，只处理最新帧（避免任何滞后）
            frame_data = None
            while not detection_queue.empty():
                try:
                    frame_data = detection_queue.get_nowait()
                except:
                    break
            
            # 如果队列为空，短暂休眠后继续（非阻塞）
            if frame_data is None:
                time.sleep(0.01)  # 10ms轮询间隔
                continue
            
            # 检查停止信号（None表示退出）
            if not isinstance(frame_data, tuple):
                break
            
            color_data, depth_data, depth_intrin = frame_data
            
            # 🔧 YOLO检测（缩小图像加速）
            detect_start = time.time()
            
            # 使用最快的插值方法
            color_data_small = cv2.resize(color_data, None, 
                                         fx=YOLO_SCALE_FACTOR, fy=YOLO_SCALE_FACTOR, 
                                         interpolation=cv2.INTER_NEAREST)  # 最快插值
            
            target_boxes_small = YOLODetection(model, color_data_small, conf_threshold=YOLO_CONF_THRESHOLD)
            detect_time_ms = (time.time() - detect_start) * 1000
            
            # 缩放回原始尺寸
            target_boxes = []
            for x1, y1, x2, y2, class_name, conf in target_boxes_small:
                target_boxes.append((
                    int(x1 / YOLO_SCALE_FACTOR), int(y1 / YOLO_SCALE_FACTOR),
                    int(x2 / YOLO_SCALE_FACTOR), int(y2 / YOLO_SCALE_FACTOR),
                    class_name, conf
                ))
            
            # 🔒 线程安全地更新检测结果
            with detection_lock:
                all_detections = []
                for box in target_boxes:
                    x1, y1, x2, y2, class_name, conf = box
                    all_detections.append((x1, y1, x2, y2, class_name, conf, None))
                
                last_detect_time_ms = detect_time_ms
                sync_selection_with_detections()
            
            # 🔧 计算面板法向量（带缓存，低频更新）
            # ✅ 新方案：直接从蓝色区域计算，不依赖按钮数量
            current_time = time.time()
            should_update_normal = (
                PANEL_CACHE_TIME <= 0.0 or
                (current_time - panel_last_update > PANEL_CACHE_TIME)
            )
            
            if should_update_normal:
                try:
                    panel_info = compute_panel_normal_from_blue_region(
                        depth_data=depth_data,
                        color_image=color_data,
                        depth_intrin=depth_intrin,
                        hsv_lower=BLUE_HSV_LOWER,
                        hsv_upper=BLUE_HSV_UPPER,
                        verbose=False
                    )
                    
                    if panel_info is not None:
                        normal_camera = panel_info['normal']
                        normal_base = None

                        if node.piper is not None and node.piper_arm is not None:
                            normal_base = transform_normal_camera_to_base(
                                normal_camera, node.piper, node.piper_arm
                            )
                        panel_info['normal_base'] = normal_base
                        panel_info['timestamp'] = current_time

                        with detection_lock:
                            global_panel_normal = normal_camera
                            global_panel_normal_base = normal_base
                            global_panel_info = panel_info
                            panel_last_update = current_time
                        
                        # 🔧 发布相机系法向量（兼容旧订阅者）
                        normal_msg = Vector3()
                        normal_msg.x = float(normal_camera[0])
                        normal_msg.y = float(normal_camera[1])
                        normal_msg.z = float(normal_camera[2])
                        node.normal_pub.publish(normal_msg)

                        # 🔧 额外发布基座系法向量（若可转换）
                        if normal_base is not None and node.normal_base_pub is not None:
                            normal_base_msg = Vector3()
                            normal_base_msg.x = float(normal_base[0])
                            normal_base_msg.y = float(normal_base[1])
                            normal_base_msg.z = float(normal_base[2])
                            node.normal_base_pub.publish(normal_base_msg)
                        
                        node.get_logger().info(
                            f"✓ 法向量(相机系): ({normal_camera[0]:+.3f}, {normal_camera[1]:+.3f}, {normal_camera[2]:+.3f})"
                            + (
                                f" → (基座系): ({normal_base[0]:+.3f}, {normal_base[1]:+.3f}, {normal_base[2]:+.3f})"
                                if normal_base is not None else
                                "（⚠️ 手眼不可用，暂未转换）"
                            )
                        )
                        
                except Exception as e:
                    node.get_logger().warn(f"法向量计算失败: {e}")
        
        except Exception as e:
            if detection_running:  # 忽略正常退出时的异常
                node.get_logger().error(f"❌ 检测线程异常: {e}")
    
    node.get_logger().info("🛑 异步检测线程已停止")


# ========================================
# 可视化
# ========================================
def visualize_detections(color_img, detections, selected_idx, panel_info=None):
    """可视化检测结果"""
    annotated = color_img.copy()
    
    # 🔧 检查鼠标悬停（预览效果）
    hover_idx = -1
    for idx, det in enumerate(detections):
        x1, y1, x2, y2, _, _, _ = det
        if x1 <= mouse_x <= x2 and y1 <= mouse_y <= y2:
            hover_idx = idx
            break
    
    for idx, det in enumerate(detections):
        x1, y1, x2, y2, class_name, conf, center_3d = det
        
        if idx == selected_idx:
            # 🔧 选中效果增强：绿色粗边框 + 半透明填充
            color = (0, 255, 0)
            thickness = 4  # 更粗
            
            # 添加半透明绿色填充
            overlay = annotated.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
            cv2.addWeighted(overlay, 0.2, annotated, 0.8, 0, annotated)
        elif idx == hover_idx:
            # 🔧 鼠标悬停效果：黄色边框
            color = (0, 255, 255)
            thickness = 3
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
    
    global current_fps, last_detect_time_ms, detection_frozen
    
    # ========================================
    # 顶部信息栏（深色半透明背景）
    # ========================================
    overlay = annotated.copy()
    cv2.rectangle(overlay, (0, 0), (annotated.shape[1], 140), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, annotated, 0.5, 0, annotated)
    
    # 第一行：操作提示
    cv2.putText(annotated, "Click:Select | ESC:Cancel | ENTER:Confirm", 
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # 第二行：法向量-相机系（动态，蓝色）
    if panel_info is not None:
        normal_cam = panel_info.get('normal') if isinstance(panel_info, dict) else None
        if normal_cam is not None:
            cam_text = f"Normal_Cam: ({normal_cam[0]:+.3f}, {normal_cam[1]:+.3f}, {normal_cam[2]:+.3f})"
            cv2.putText(annotated, cam_text, (10, 55), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)
        
        # 第三行：法向量-基座系（固定，绿色）
        normal_base = panel_info.get('normal_base') if isinstance(panel_info, dict) else None
        if normal_base is not None:
            base_text = f"Normal_Base: ({normal_base[0]:+.3f}, {normal_base[1]:+.3f}, {normal_base[2]:+.3f})"
            cv2.putText(annotated, base_text, (10, 85), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 100), 2)
        else:
            cv2.putText(annotated, "Normal_Base: Computing...", (10, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 100), 2)
        
        # 第四行：面板深度 + 更新延迟
        panel_depth = panel_info.get('median_depth') if isinstance(panel_info, dict) else None
        timestamp = panel_info.get('timestamp') if isinstance(panel_info, dict) else None
        
        info_parts = []
        if panel_depth is not None:
            info_parts.append(f"Depth:{panel_depth*100:.1f}cm")
        if timestamp is not None:
            age_ms = max(0.0, (time.time() - timestamp) * 1000.0)
            info_parts.append(f"Age:{age_ms:.0f}ms")
        
        if info_parts:
            info_text = " | ".join(info_parts)
            cv2.putText(annotated, info_text, (10, 115),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    else:
        cv2.putText(annotated, "Normal: Waiting for detection...", (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 100), 2)
    
    # ========================================
    # 右上角性能信息（紧凑布局）
    # ========================================
    right_x = annotated.shape[1] - 150
    cv2.putText(annotated, f"FPS:{current_fps:.0f}", (right_x, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    if last_detect_time_ms > 0:
        cv2.putText(annotated, f"Det:{last_detect_time_ms:.0f}ms", (right_x, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)
    
    cv2.putText(annotated, "LIVE", (right_x, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # 🔧 鼠标位置指示（底部状态栏）
    if 0 <= mouse_x < annotated.shape[1] and 0 <= mouse_y < annotated.shape[0]:
        # 底部状态栏
        cv2.rectangle(annotated, (0, annotated.shape[0] - 25), 
                     (annotated.shape[1], annotated.shape[0]), (0, 0, 0), -1)
        cv2.putText(annotated, f"Mouse: ({mouse_x}, {mouse_y})", 
                   (10, annotated.shape[0] - 8),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    return annotated


# ========================================
# ROI 点云提取（优化版）
# ========================================
def extract_roi_cloud(depth_data, color_data, bbox, depth_intrin, verbose=False):
    """从深度图中提取 ROI 区域的 3D 中心点"""
    x1, y1, x2, y2 = bbox
    
    center_u = int((x1 + x2) / 2)
    center_v = int((y1 + y2) / 2)
    
    fx, fy = depth_intrin.fx, depth_intrin.fy
    cx, cy = depth_intrin.ppx, depth_intrin.ppy
    
    # ROI统计
    roi_depth = depth_data[y1:y2, x1:x2]
    valid_roi_depths = roi_depth[roi_depth > 0]
    
    point_cloud_stats = {
        'bbox_size': (x2-x1, y2-y1),
        'valid_count': len(valid_roi_depths),
        'coverage': 100 * len(valid_roi_depths) / roi_depth.size if roi_depth.size > 0 else 0,
        'depth_mean': valid_roi_depths.mean() if len(valid_roi_depths) > 0 else 0,
        'depth_std': valid_roi_depths.std() if len(valid_roi_depths) > 0 else 0,
    }
    
    # 智能采样：5x5窗口 + 中位数过滤
    sample_size = 5
    u_min = max(0, center_u - sample_size)
    u_max = min(depth_data.shape[1], center_u + sample_size)
    v_min = max(0, center_v - sample_size)
    v_max = min(depth_data.shape[0], center_v + sample_size)
    
    depth_window = depth_data[v_min:v_max, u_min:u_max]
    valid_depths = depth_window[depth_window > 0]
    
    if len(valid_depths) == 0:
        return None, point_cloud_stats
    
    # 使用中位数 + 离群点过滤
    depth_median = np.median(valid_depths)
    depth_std = np.std(valid_depths)
    
    if depth_std > 200:
        lower_bound = depth_median - 1.5 * depth_std
        upper_bound = depth_median + 1.5 * depth_std
        filtered_depths = valid_depths[(valid_depths >= lower_bound) & (valid_depths <= upper_bound)]
        if len(filtered_depths) > 0:
            valid_depths = filtered_depths
            depth_median = np.median(valid_depths)
    
    depth_value = depth_median
    depth_m = depth_value / 1000.0
    
    if depth_m < 0.15 or depth_m > 1.5:
        return None, point_cloud_stats
    
    x = (center_u - cx) * depth_m / fx
    y = (center_v - cy) * depth_m / fy
    z = depth_m
    
    return [x, y, z], point_cloud_stats


# ========================================
# 坐标转换：相机 → 基座
# ========================================
def transform_normal_camera_to_base(normal_camera, piper, piper_arm):
    """
    将相机坐标系的法向量转换到基座坐标系
    
    注意：法向量是方向向量，只需要旋转变换（不需要平移）
    
    参数:
        normal_camera: 相机系法向量 [nx, ny, nz]
        piper: 机械臂接口
        piper_arm: 机械臂运动学对象
    
    返回:
        基座系法向量 [nx', ny', nz'] 或 None
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
        
        # 正运动学：获取 link6→base 的变换矩阵（4x4齐次变换）
        base_T_link6 = piper_arm.forward_kinematics(current_joints)
        
        # 提取旋转矩阵（link6→base）
        # 注意：base_T_link6 表示将 link6系的点/向量 转换到 base系
        R_link6_to_base = base_T_link6[:3, :3]
        
        # 手眼标定：获取 camera→link6 的固定变换
        link6_T_camera = np.eye(4)
        link6_T_camera[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
        link6_T_camera[:3, 3] = piper_arm.link6_t_camera
        
        # 提取旋转矩阵（camera→link6）
        R_camera_to_link6 = link6_T_camera[:3, :3]
        
        # 组合旋转矩阵：camera → link6 → base
        # 转换链：相机系 → link6系 → 基座系
        R_camera_to_base = R_link6_to_base @ R_camera_to_link6
        
        # 旋转法向量（法向量只需要旋转，不需要平移）
        normal_base = R_camera_to_base @ np.array(normal_camera)
        
        # 归一化
        normal_base = normal_base / np.linalg.norm(normal_base)
        
        return normal_base.tolist()
    
    except Exception as e:
        print(f"❌ 法向量坐标转换失败: {e}")
        import traceback
        traceback.print_exc()
        return None


def transform_camera_to_base(button_camera, piper, piper_arm):
    """将相机坐标系的按钮位置转换到基座坐标系"""
    try:
        msg = piper.GetArmJointMsgs()
        current_joints = [
            msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
            msg.joint_state.joint_6 * 1e-3 * PI / 180.0,
        ]
        
        base_T_link6 = piper_arm.forward_kinematics(current_joints)
        
        link6_T_cam = np.eye(4)
        link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
        link6_T_cam[:3, 3] = piper_arm.link6_t_camera
        
        button_cam_h = np.array([button_camera[0], button_camera[1], button_camera[2], 1.0])
        button_base = base_T_link6 @ link6_T_cam @ button_cam_h
        
        return button_base[:3]
    except Exception as e:
        print(f"  ⚠️  坐标转换失败: {e}")
        return None


# ========================================
# ROS2 节点（仅用于发布结果）
# ========================================
class ButtonDetectorNode(Node):
    """按钮检测节点 - 直接读取相机版本"""
    
    def __init__(self):
        super().__init__('button_detector_ros2_direct')
        
        self.get_logger().info("="*70)
        self.get_logger().info("交互式按钮检测器 - ROS2 直接读取版本（高性能）")
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
        self.point_pub = self.create_publisher(PointStamped, '/object_point', 10)
        self.type_pub = self.create_publisher(String, '/button_type', 10)
        self.marker_pub = self.create_publisher(Marker, '/object_center_marker', 10)
        self.normal_pub = self.create_publisher(Vector3, '/button_normal', 10)  # 相机系法向量
        self.normal_base_pub = self.create_publisher(Vector3, '/button_normal_base', 10)  # 基座系法向量
        
        self.get_logger().info("✓ ROS2发布器已创建（包括法向量话题）")
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
    
    def publish_result_with_normal(self, center_3d, class_name, normal_vector, in_camera_frame=True):
        """
        发布检测结果到ROS2话题（包含法向量）
        
        参数:
            center_3d: 按钮中心3D坐标
            class_name: 按钮类型
            normal_vector: 面板法向量
            in_camera_frame: 法向量是否在相机坐标系（True）还是基座坐标系（False）
        """
        # 发布基本信息
        self.publish_result(center_3d, class_name)
        
        # 发布法向量（注意：使用Stamped消息可以标记坐标系）
        # 但Vector3没有header，所以通过日志说明
        normal_msg = Vector3()
        normal_msg.x = float(normal_vector[0])
        normal_msg.y = float(normal_vector[1])
        normal_msg.z = float(normal_vector[2])
        self.normal_pub.publish(normal_msg)
        
        if not in_camera_frame and self.normal_base_pub is not None:
            normal_base_msg = Vector3()
            normal_base_msg.x = normal_msg.x
            normal_base_msg.y = normal_msg.y
            normal_base_msg.z = normal_msg.z
            self.normal_base_pub.publish(normal_base_msg)
        
        frame_info = "相机坐标系" if in_camera_frame else "基座坐标系"
        self.get_logger().info(f"  ✓ 法向量已发布 ({frame_info}): ({normal_vector[0]:.4f}, {normal_vector[1]:.4f}, {normal_vector[2]:.4f})")
        
        # 发布法向量可视化（箭头）
        arrow_marker = Marker()
        # 根据法向量坐标系设置frame_id
        arrow_marker.header.frame_id = "camera" if in_camera_frame else "base_link"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = "panel_normal"
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        # 箭头起点：按钮中心
        arrow_marker.points = []
        from geometry_msgs.msg import Point
        start_point = Point()
        start_point.x = center_3d[0]
        start_point.y = center_3d[1]
        start_point.z = center_3d[2]
        arrow_marker.points.append(start_point)
        
        # 箭头终点：沿法向量延伸10cm
        # 如果在相机系，法向量指向相机（减去）
        # 如果在基座系，法向量指向外侧（加上）
        direction = -1.0 if in_camera_frame else 1.0
        end_point = Point()
        end_point.x = center_3d[0] + direction * normal_vector[0] * 0.1
        end_point.y = center_3d[1] + direction * normal_vector[1] * 0.1
        end_point.z = center_3d[2] + direction * normal_vector[2] * 0.1
        arrow_marker.points.append(end_point)
        
        # 箭头样式
        arrow_marker.scale.x = 0.005  # 箭头轴直径
        arrow_marker.scale.y = 0.01   # 箭头头部直径
        arrow_marker.scale.z = 0.01   # 箭头头部长度
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 1.0
        arrow_marker.color.a = 1.0
        self.marker_pub.publish(arrow_marker)


def main(args=None):
    rclpy.init(args=args)
    
    node = ButtonDetectorNode()
    
    # ========================================
    # ✅ 关键修复：启动ROS2独立spin线程
    # ========================================
    ros_spin_running = threading.Event()
    ros_spin_running.set()
    
    def ros_spin_thread():
        """ROS2事件循环独立线程，避免被OpenCV阻塞"""
        try:
            node.get_logger().info("🔄 ROS2 spin线程已启动（独立运行）")
            while ros_spin_running.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
        except Exception as e:
            node.get_logger().error(f"❌ ROS2 spin线程异常: {e}")
    
    spin_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    spin_thread.start()
    node.get_logger().info("✅ ROS2消息处理已独立运行，不会被相机/OpenCV阻塞")
    # ========================================
    
    # 配置 RealSense（直接读取）
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    
    node.get_logger().info("✓ RealSense相机启动成功（直接读取模式）")
    
    # 创建 OpenCV 窗口
    cv2.namedWindow('detection', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('detection', mouse_callback, node)
    
    global all_detections, selected_button_index, selected_button_locked
    global current_depth_data, current_color_data, current_depth_intrin
    global frame_counter, last_fps_time, fps_counter, current_fps
    global is_paused, paused_frame, paused_detections
    global selected_box_signature
    global global_panel_normal, global_panel_normal_base
    global global_panel_info, panel_last_update
    global detection_running, detection_frozen
    
    # 🚀 启动异步检测线程
    detection_thread = threading.Thread(
        target=async_detection_worker, 
        args=(node.model, node), 
        daemon=True
    )
    detection_thread.start()
    node.get_logger().info("✅ 异步检测线程已启动，画面将保持实时刷新")
    
    try:
        node.get_logger().info("✓ 开始检测...")
        
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
                    with detection_lock:
                        paused_panel_info = global_panel_info
                    display_img = visualize_detections(
                        paused_frame,
                        paused_detections,
                        selected_button_index,
                        panel_info=paused_panel_info
                    )
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
            
            # 🔧 异步检测：将帧发送到检测队列（非阻塞）
            frame_counter += 1
            should_detect = (frame_counter % (DETECTION_SKIP_FRAMES + 1) == 0)
            
            if should_detect:
                # 尝试将帧放入队列（不阻塞，如果队列满了就跳过）
                try:
                    detection_queue.put_nowait((color_data.copy(), depth_data.copy(), depth_intrin))
                except:
                    pass  # 队列满了，跳过本帧检测
            
            # 🔧 显示（高优先级，每帧都刷新）- 使用锁保护读取
            with detection_lock:
                current_detections = list(all_detections)
                current_panel_info = global_panel_info
            
            display_img = visualize_detections(
                color_data,
                current_detections,
                selected_button_index,
                panel_info=current_panel_info
            )
            
            # 🔧 叠加面板法向量可视化
            if current_panel_info is not None and len(current_detections) >= 2:
                display_img = visualize_panel_normal(
                    display_img, 
                    current_detections, 
                    current_panel_info,
                    show_rings=False  # 不显示环形区域，避免画面混乱
                )
            
            cv2.imshow('detection', display_img)
            
            # 键盘处理（使用更短的等待时间提高响应速度）
            wait_time = max(1, int(1000 / UI_REFRESH_RATE))  # 30Hz = 33ms
            key = cv2.waitKey(wait_time) & 0xFF
            
            if key == 32:  # SPACE
                is_paused = not is_paused
                if is_paused:
                    node.get_logger().info("⏸️  画面已暂停")
                    paused_frame = color_data.copy()
                    paused_detections = list(all_detections)
            
            elif key == 27:  # ESC
                node.get_logger().info("✗ 已取消选择，解除锁定，恢复检测")
                selected_button_index = -1
                selected_button_locked = False
                selected_box_signature = None
                detection_frozen = False  # 🔧 恢复检测
            
            if key == 13:  # ENTER
                # 🔒 线程安全地读取检测结果
                with detection_lock:
                    detections_to_use = paused_detections if is_paused else list(all_detections)
                    current_global_normal = global_panel_normal
                    current_global_normal_base = global_panel_normal_base
                
                if selected_button_index >= 0 and selected_button_index < len(detections_to_use):
                    det = detections_to_use[selected_button_index]
                    x1, y1, x2, y2, class_name, conf, center_3d = det
                    
                    if center_3d is None:
                        center_3d, stats = extract_roi_cloud(
                            current_depth_data, current_color_data, 
                            [x1, y1, x2, y2], 
                            current_depth_intrin,
                            verbose=True
                        )
                    
                    if center_3d is not None:
                        node.get_logger().info(f"\n{'='*70}")
                        node.get_logger().info(f"✓ 确认选择:")
                        node.get_logger().info(f"  按钮类型: {class_name}")
                        node.get_logger().info(f"  相机坐标: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f}) m")
                        
                        # 🔧 使用全局法向量（如果可用）
                        use_normal = None
                        use_normal_base = None
                        if current_global_normal is not None:
                            use_normal = current_global_normal
                            use_normal_base = current_global_normal_base
                            node.get_logger().info(f"  ✓ 使用缓存的面板法向量")
                        else:
                            # 回退：尝试局部计算（仅当前按钮）
                            node.get_logger().info(f"  ⚠️  无全局法向量，尝试局部计算...")
                            try:
                                local_panel_info = compute_robust_panel_normal(
                                    [det],  # 只用当前按钮
                                    current_depth_data,
                                    current_depth_intrin,
                                    expand_ratio=0.3,  # 局部计算时扩展稍大
                                    min_buttons=1,
                                    verbose=True,
                                    color_image=current_color_data,  # 🔧 传入彩色图像
                                    use_color_filter=True,  # 🔧 启用蓝色面板过滤
                                    hsv_lower=BLUE_HSV_LOWER,  # 🎨 HSV下限
                                    hsv_upper=BLUE_HSV_UPPER   # 🎨 HSV上限
                                )
                                if local_panel_info is not None:
                                    use_normal = local_panel_info['normal']
                                    node.get_logger().info(f"  ✓ 局部法向量计算成功")
                            except Exception as e:
                                node.get_logger().warn(f"  ⚠️  局部法向量计算失败: {e}")
                        
                        if use_normal is not None:
                            node.get_logger().info(f"  面板法向量(相机系): ({use_normal[0]:.4f}, {use_normal[1]:.4f}, {use_normal[2]:.4f})")
                        else:
                            node.get_logger().warn(f"  ⚠️  无法计算法向量，将使用默认垂直方向")
                            use_normal = np.array([0.0, 0.0, -1.0])  # 默认：垂直于相机
                        
                        # 🔧 转换法向量到基座坐标系（默认启用）
                        normal_in_camera_frame = True  # 初始：相机系
                        if use_normal_base is not None:
                            use_normal = use_normal_base
                            normal_in_camera_frame = False
                            node.get_logger().info(
                                f"  面板法向量(基座系-缓存): ({use_normal_base[0]:.4f}, {use_normal_base[1]:.4f}, {use_normal_base[2]:.4f})"
                            )
                        if node.piper is not None:
                            base_3d = transform_camera_to_base(center_3d, node.piper, node.piper_arm)
                            if base_3d is not None:
                                node.get_logger().info(f"  按钮位置(基座系): ({base_3d[0]:.3f}, {base_3d[1]:.3f}, {base_3d[2]:.3f}) m")
                            
                            # 转换法向量到基座系
                            if use_normal_base is None:
                                converted_normal = transform_normal_camera_to_base(use_normal, node.piper, node.piper_arm)
                                if converted_normal is not None:
                                    use_normal = converted_normal
                                    use_normal_base = converted_normal
                                    node.get_logger().info(f"  面板法向量(基座系): ({converted_normal[0]:.4f}, {converted_normal[1]:.4f}, {converted_normal[2]:.4f})")
                                    normal_in_camera_frame = False
                                else:
                                    node.get_logger().warn(f"  ⚠️  法向量坐标转换失败，将使用相机系法向量")
                        
                        node.get_logger().info(f"{'='*70}")
                        
                        # 发布到ROS2（包含法向量）
                        node.publish_result_with_normal(center_3d, class_name, use_normal, in_camera_frame=normal_in_camera_frame)
                        node.get_logger().info("✓ 已发布到 ROS2 话题（包含法向量）")
                        
                        selected_button_index = -1
                        selected_button_locked = False
                        selected_box_signature = None
                        detection_frozen = False  # 🔧 恢复检测
            
            elif key == ord('q'):
                break
            
            # ========================================
            # ❌ 删除：不再需要spin_once（独立线程已处理）
            # rclpy.spin_once(node, timeout_sec=0)
            # ========================================
    
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 🔧 停止异步检测线程
        detection_running = False
        detection_queue.put(None)  # 发送停止信号
        detection_thread.join(timeout=2.0)
        node.get_logger().info("✓ 异步检测线程已停止")
        
        # 停止ROS2 spin线程
        ros_spin_running.clear()
        spin_thread.join(timeout=2.0)
        
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
