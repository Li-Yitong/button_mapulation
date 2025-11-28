#!/usr/bin/env python
"""
交互式按钮检测器 - 使用 yolo_button.pt 模型
支持用户点击选择要操作的按钮
"""
import rospy
import pyrealsense2 as rs
import cv2
from ultralytics import YOLO
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from utils.utils_ros import publish_target_point, publish_sphere_marker
import numpy as np

# ========================================
# 全局变量
# ========================================
# 加载按钮检测模型
model = YOLO("yolo_button.pt")
class_names = model.names

# 检测结果存储
all_detections = []  # [(x1, y1, x2, y2, class_name, conf, center_3d), ...]
selected_button_index = -1  # 用户选中的按钮索引 (-1 表示未选择)
confirmed = False  # 用户是否按下 ENTER 确认
selected_button_locked = False  # 选择锁定标志（防止跳动）
detection_frozen = False  # 检测冻结标志（选中后停止更新检测）

# 鼠标位置
mouse_x, mouse_y = 0, 0

# 检测频率控制
frame_count = 0
detection_interval = 5  # 每5帧检测一次（降低检测频率）

# 当前帧数据（用于鼠标回调访问）
current_depth_data = None
current_color_data = None
current_depth_intrin = None


# ========================================
# 鼠标回调函数
# ========================================
def mouse_callback(event, x, y, flags, param):
    """
    处理鼠标事件，允许用户点击选择按钮
    """
    global selected_button_index, mouse_x, mouse_y, all_detections
    global current_depth_data, current_color_data, current_depth_intrin
    global selected_button_locked, detection_frozen
    
    mouse_x, mouse_y = x, y
    
    if event == cv2.EVENT_LBUTTONDOWN:  # 左键点击
        # 修正坐标：显示图像顶部有80像素的提示栏，需要减去这个偏移
        TIP_BAR_HEIGHT = 80
        corrected_y = y - TIP_BAR_HEIGHT
        
        print(f"\n[鼠标点击] 原始坐标: ({x}, {y}), 修正后: ({x}, {corrected_y})")
        print(f"[检测状态] 当前检测到 {len(all_detections)} 个按钮")
        
        # 如果点击在提示栏上，忽略
        if corrected_y < 0:
            print("  → 点击在提示栏上，忽略")
            return
        
        # 检查点击位置是否在某个检测框内
        found = False
        for idx, det in enumerate(all_detections):
            x1, y1, x2, y2, class_name, conf, center_3d = det
            
            print(f"  检查按钮 #{idx}: 类型={class_name}, 框=[{x1},{y1},{x2},{y2}]", end="")
            
            # 使用修正后的y坐标
            if x1 <= x <= x2 and y1 <= corrected_y <= y2:
                print(" → ✓ 匹配!")
                print(" → ✓ 匹配!")
                found = True
                selected_button_index = idx
                selected_button_locked = True  # 锁定选择，防止检测框抖动导致选择变化
                detection_frozen = True  # 冻结检测，停止更新检测结果
                
                # 立即计算3D位置（只计算选中的按钮）
                if center_3d is None and current_depth_data is not None:
                    print("  → 计算3D位置...")
                    center_3d = extract_roi_cloud(
                        current_depth_data, 
                        current_color_data, 
                        [x1, y1, x2, y2], 
                        current_depth_intrin
                    )
                    # 更新检测结果
                    all_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                
                print(f"\n{'='*60}")
                print(f"✓✓✓ 已选择按钮 #{idx} 【已锁定】✓✓✓")
                print(f"  类型: {class_name}")
                print(f"  置信度: {conf:.2f}")
                print(f"  检测框: [{x1}, {y1}, {x2}, {y2}]")
                print(f"  2D中心: ({int((x1+x2)/2)}, {int((y1+y2)/2)})")
                if center_3d is not None:
                    print(f"  3D位置: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f})")
                print(f"  提示: 按 ENTER 确认 | 按 ESC 取消选择")
                print(f"{'='*60}")
                break  # 找到后立即返回
            else:
                print(" → ✗ 不匹配")
        
        # 如果没有找到匹配的按钮
        if not found:
            print(f"\n  ✗✗✗ 点击位置 ({x}, {corrected_y}) 不在任何按钮内")
            if not selected_button_locked:
                selected_button_index = -1  # 只有在未锁定时才取消选择


# ========================================
# YOLO 检测函数
# ========================================
def YOLODetection(image, conf_threshold=0.5):
    """
    使用 YOLO 检测按钮
    
    Args:
        image: BGR 图像
        conf_threshold: 置信度阈值
    
    Returns:
        target_boxes: [(x1, y1, x2, y2, class_name, confidence), ...]
    """
    # 转换为 RGB 格式
    img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # 执行检测
    results = model.predict(img_rgb, verbose=False)
    
    target_boxes = []
    for result in results:
        boxes = result.boxes
        for box in boxes:
            class_id = int(box.cls)
            confidence = float(box.conf)
            class_name = class_names[class_id]
            
            # 过滤低置信度检测
            if confidence > conf_threshold:
                xyxy = box.xyxy[0].tolist()
                target_boxes.append([
                    int(xyxy[0]), int(xyxy[1]), 
                    int(xyxy[2]), int(xyxy[3]),
                    class_name, confidence
                ])
                print(f"  检测到: {class_name} (置信度: {confidence:.2f}), 位置: {xyxy}")
    
    return target_boxes


# ========================================
# 可视化函数
# ========================================
def visualize_detections(image, detections, selected_idx):
    """
    可视化所有检测结果，高亮显示选中的按钮
    
    Args:
        image: 原始图像
        detections: [(x1, y1, x2, y2, class_name, conf, center_3d), ...]
        selected_idx: 选中的按钮索引 (-1 表示未选择)
    
    Returns:
        annotated: 标注后的图像
    """
    annotated = image.copy()
    
    for idx, det in enumerate(detections):
        x1, y1, x2, y2, class_name, conf, center_3d = det
        
        # 颜色和线宽
        if idx == selected_idx:
            color = (0, 255, 0)  # 绿色 - 选中
            thickness = 4
        else:
            color = (255, 128, 0)  # 橙色 - 未选中
            thickness = 2
        
        # 绘制矩形框
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
        
        # 标签背景
        label = f"#{idx}: {class_name} ({conf:.2f})"
        (label_w, label_h), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
        )
        cv2.rectangle(
            annotated, 
            (x1, y1 - label_h - 10), 
            (x1 + label_w, y1), 
            color, -1
        )
        
        # 标签文字
        cv2.putText(
            annotated, label, (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )
        
        # 显示3D坐标（如果有）
        if center_3d is not None:
            coord_text = f"({center_3d[0]:.2f}, {center_3d[1]:.2f}, {center_3d[2]:.2f})"
            cv2.putText(
                annotated, coord_text, (x1, y2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
            )
    
    # 顶部提示信息
    tip_bg = np.zeros((80, annotated.shape[1], 3), dtype=np.uint8)
    tip_bg[:] = (50, 50, 50)  # 深灰色背景
    
    cv2.putText(
        tip_bg, "Step 1: Click on a button to select", (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
    )
    cv2.putText(
        tip_bg, "Step 2: Press ENTER to confirm | ESC to cancel", (10, 45),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
    )
    
    # 显示检测状态
    global detection_frozen
    status_text = "LOCKED" if detection_frozen else "DETECTING"
    status_color = (0, 255, 0) if detection_frozen else (0, 255, 255)
    cv2.putText(
        tip_bg, f"Status: {status_text}", (10, 70),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2
    )
    
    # 拼接提示和图像
    annotated = np.vstack([tip_bg, annotated])
    
    # 在图像上显示当前鼠标位置（帮助调试）
    if mouse_y > 80:  # 只在图像区域显示
        corrected_mouse_y = mouse_y - 80
        cv2.circle(annotated, (mouse_x, mouse_y), 5, (0, 255, 255), -1)  # 黄色圆点
        cv2.putText(
            annotated, f"Mouse: ({mouse_x}, {corrected_mouse_y})", 
            (mouse_x + 10, mouse_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
        )
    
    return annotated


# ========================================
# ROI 点云提取（简化版：使用2D中心点）
# ========================================
def extract_roi_cloud(depth_data, color_data, bbox, depth_intrin):
    """
    从深度图中提取 ROI 区域的 3D 中心点
    使用2D边界框对角线中点的深度值
    
    Args:
        depth_data: 深度图
        color_data: 彩色图
        bbox: [x1, y1, x2, y2]
        depth_intrin: 相机内参
    
    Returns:
        center: 3D 中心坐标 [x, y, z]
    """
    x1, y1, x2, y2 = bbox
    
    # 计算2D边界框的中心点
    center_u = int((x1 + x2) / 2)
    center_v = int((y1 + y2) / 2)
    
    # 获取中心点周围小区域的深度值（3x3采样求平均，更鲁棒）
    sample_size = 3
    u_start = max(0, center_u - sample_size)
    u_end = min(depth_data.shape[1], center_u + sample_size)
    v_start = max(0, center_v - sample_size)
    v_end = min(depth_data.shape[0], center_v + sample_size)
    
    depth_region = depth_data[v_start:v_end, u_start:u_end]
    valid_depths = depth_region[depth_region > 0]  # 过滤无效深度
    
    if len(valid_depths) == 0:
        return None
    
    # 取平均深度并转换为米
    depth_value = np.mean(valid_depths) / 1000.0
    
    # 检查深度范围
    if depth_value < 0.1 or depth_value > 2.0:
        return None
    
    # 使用相机内参将2D像素+深度投影到3D空间
    fx, fy = depth_intrin.fx, depth_intrin.fy
    cx, cy = depth_intrin.ppx, depth_intrin.ppy
    
    x = (center_u - cx) * depth_value / fx
    y = (center_v - cy) * depth_value / fy
    z = depth_value
    
    return np.array([x, y, z])


# ========================================
# 主函数
# ========================================
def main():
    global all_detections, selected_button_index, confirmed
    global frame_count, detection_frozen
    
    # 初始化 ROS 节点
    rospy.init_node('realsense_yolo_button_interactive')
    
    # 创建发布器
    pub_object_point = rospy.Publisher('/object_point', PointStamped, queue_size=10)
    pub_button_type = rospy.Publisher('/button_type', String, queue_size=10)
    pub_object_marker = rospy.Publisher('/object_center_marker', Marker, queue_size=10)
    
    print("\n" + "="*70)
    print("交互式按钮检测器")
    print("="*70)
    print("模型: yolo_button.pt")
    print("支持类型: toggle, plugin, push, knob")
    print("="*70 + "\n")
    
    # 配置 RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # 启动管道
    profile = pipeline.start(config)
    
    # 对齐到彩色相机坐标系（深度对齐到彩色）
    align = rs.align(rs.stream.color)
    
    # 获取彩色相机内参（用于2D→3D投影）
    color_stream = profile.get_stream(rs.stream.color)
    color_intrin = color_stream.as_video_stream_profile().intrinsics
    
    print(f"✓ 彩色相机内参: fx={color_intrin.fx:.2f}, fy={color_intrin.fy:.2f}, "
          f"cx={color_intrin.ppx:.2f}, cy={color_intrin.ppy:.2f}")
    
    # 创建窗口并设置鼠标回调
    cv2.namedWindow('detection', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('detection', mouse_callback)
    
    print("✓ 相机已启动")
    print("✓ 等待检测按钮...\n")
    
    try:
        while not rospy.is_shutdown():
            # 获取帧
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            aligned_depth = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not (aligned_depth and color_frame):
                continue
            
            # 转换为 numpy 数组
            depth_data = np.asanyarray(aligned_depth.get_data())
            color_data = np.asanyarray(color_frame.get_data())
            
            # ⚠️ 关键修复：使用彩色相机内参，因为YOLO检测在彩色图上进行
            # aligned_depth 已经对齐到彩色相机坐标系，所以深度值对应彩色图像素
            # 因此投影时应该使用彩色相机内参
            
            # 保存当前帧数据供鼠标回调使用（用于即时计算3D位置）
            global current_depth_data, current_color_data, current_depth_intrin
            current_depth_data = depth_data
            current_color_data = color_data
            current_depth_intrin = color_intrin  # 使用彩色相机内参
            
            # 检测频率控制：只在特定帧执行检测，且未冻结时
            frame_count += 1
            if not detection_frozen and (frame_count % detection_interval == 0):
                # 执行 YOLO 检测
                target_boxes = YOLODetection(color_data, conf_threshold=0.5)
                
                # 更新全局检测结果（暂不计算3D，等点击时再计算）
                all_detections = []
                for box in target_boxes:
                    x1, y1, x2, y2, class_name, conf = box
                    # 先不计算3D位置，设为None，只在选中时才计算
                    all_detections.append((x1, y1, x2, y2, class_name, conf, None))
            
            # 可视化
            annotated_img = visualize_detections(
                color_data, all_detections, selected_button_index
            )
            cv2.imshow('detection', annotated_img)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC - 取消选择
                print("\n✗ 已取消选择，解除锁定，恢复检测")
                selected_button_index = -1
                selected_button_locked = False  # 解除锁定
                detection_frozen = False  # 恢复检测
                confirmed = False
            
            elif key == 13:  # ENTER - 确认选择
                if selected_button_index >= 0 and selected_button_index < len(all_detections):
                    det = all_detections[selected_button_index]
                    x1, y1, x2, y2, class_name, conf, center_3d = det
                    
                    # 如果还没计算3D位置，现在计算
                    if center_3d is None:
                        print("  计算3D位置...")
                        center_3d = extract_roi_cloud(
                            depth_data, color_data, 
                            [x1, y1, x2, y2], 
                            color_intrin  # 使用彩色相机内参
                        )
                        all_detections[selected_button_index] = (x1, y1, x2, y2, class_name, conf, center_3d)
                    
                    if center_3d is not None:
                        print("\n" + "="*70)
                        print("✓✓✓ 确认选择！✓✓✓")
                        print(f"  按钮类型: {class_name}")
                        print(f"  3D位置: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f})")
                        print("="*70 + "\n")
                        
                        # 发布按钮位置
                        publish_target_point(pub_object_point, center_3d)
                        
                        # 发布按钮类型
                        button_type_msg = String()
                        button_type_msg.data = class_name
                        pub_button_type.publish(button_type_msg)
                        
                        # 发布可视化标记
                        publish_sphere_marker(pub_object_marker, center_3d)
                        
                        print("✓ 已发布到话题:")
                        print(f"  - /object_point: {center_3d}")
                        print(f"  - /button_type: {class_name}\n")
                        
                        # 重置选择并解除锁定
                        selected_button_index = -1
                        selected_button_locked = False  # 解除锁定
                        detection_frozen = False  # 恢复检测
                        confirmed = True
                    else:
                        print("\n✗ 该按钮没有有效的3D坐标，请重新选择\n")
                else:
                    print("\n✗ 请先点击选择一个按钮\n")
            
            elif key == ord('q'):  # 退出
                print("\n退出程序...")
                break
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("✓ 相机已关闭")


if __name__ == '__main__':
    main()
