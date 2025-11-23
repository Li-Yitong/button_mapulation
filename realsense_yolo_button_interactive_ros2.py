#!/usr/bin/env python3
"""
交互式按钮检测器 - ROS2 版本
使用 yolo_button.pt 模型
支持用户点击选择要操作的按钮
"""
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import numpy as np

# ========================================
# 全局变量
# ========================================
# 检测结果存储
all_detections = []  # [(x1, y1, x2, y2, class_name, conf, center_3d), ...]
selected_button_index = -1  # 用户选中的按钮索引
confirmed = False
selected_button_locked = False

# 鼠标位置
mouse_x, mouse_y = 0, 0

# 当前帧数据
current_depth_data = None
current_color_data = None
current_depth_intrin = None


# ========================================
# 鼠标回调函数
# ========================================
def mouse_callback(event, x, y, flags, param):
    """处理鼠标事件，允许用户点击选择按钮"""
    global selected_button_index, mouse_x, mouse_y, all_detections
    global current_depth_data, current_color_data, current_depth_intrin
    global selected_button_locked
    
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
        found = False
        for idx, det in enumerate(all_detections):
            x1, y1, x2, y2, class_name, conf, center_3d = det
            
            if x1 <= x <= x2 and y1 <= corrected_y <= y2:
                print(f" → ✓ 匹配按钮 #{idx}")
                found = True
                selected_button_index = idx
                selected_button_locked = True
                
                # 立即计算3D位置
                if center_3d is None and current_depth_data is not None:
                    center_3d = extract_roi_cloud(
                        current_depth_data, 
                        current_color_data, 
                        [x1, y1, x2, y2], 
                        current_depth_intrin
                    )
                    all_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                
                print(f"\n{'='*60}")
                print(f"✓ 已选择按钮 #{idx}")
                print(f"  类型: {class_name}")
                print(f"  置信度: {conf:.2f}")
                if center_3d is not None:
                    print(f"  3D位置: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f})")
                print(f"  提示: 按 ENTER 确认 | 按 ESC 取消")
                print(f"{'='*60}")
                break
        
        if not found and not selected_button_locked:
            selected_button_index = -1


# ========================================
# YOLO 检测
# ========================================
def YOLODetection(model, image, conf_threshold=0.5):
    """使用 YOLO 检测按钮"""
    img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = model.predict(img_rgb, verbose=False)
    
    target_boxes = []
    for result in results:
        boxes = result.boxes
        for box in boxes:
            class_id = int(box.cls)
            confidence = float(box.conf)
            class_name = model.names[class_id]
            
            if confidence > conf_threshold:
                xyxy = box.xyxy[0].tolist()
                target_boxes.append([
                    int(xyxy[0]), int(xyxy[1]), 
                    int(xyxy[2]), int(xyxy[3]),
                    class_name, confidence
                ])
    
    return target_boxes


# ========================================
# 可视化
# ========================================
def visualize_detections(image, detections, selected_idx):
    """可视化所有检测结果"""
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
        
        # 标签
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
    
    # 顶部提示
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
    
    annotated = np.vstack([tip_bg, annotated])
    
    # 显示鼠标位置
    if mouse_y > 60:
        corrected_mouse_y = mouse_y - 60
        cv2.circle(annotated, (mouse_x, mouse_y), 5, (0, 255, 255), -1)
        cv2.putText(
            annotated, f"Mouse: ({mouse_x}, {corrected_mouse_y})", 
            (mouse_x + 10, mouse_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
        )
    
    return annotated


# ========================================
# ROI 点云提取
# ========================================
def extract_roi_cloud(depth_data, color_data, bbox, depth_intrin):
    """从深度图中提取 ROI 区域的 3D 中心点"""
    x1, y1, x2, y2 = bbox
    
    # 计算2D边界框的中心点
    center_u = int((x1 + x2) / 2)
    center_v = int((y1 + y2) / 2)
    
    # 3x3采样求平均
    sample_size = 3
    u_start = max(0, center_u - sample_size)
    u_end = min(depth_data.shape[1], center_u + sample_size)
    v_start = max(0, center_v - sample_size)
    v_end = min(depth_data.shape[0], center_v + sample_size)
    
    depth_region = depth_data[v_start:v_end, u_start:u_end]
    valid_depths = depth_region[depth_region > 0]
    
    if len(valid_depths) == 0:
        return None
    
    # 取平均深度并转换为米
    depth_value = np.mean(valid_depths) / 1000.0
    
    # 检查深度范围
    if depth_value < 0.1 or depth_value > 2.0:
        return None
    
    # 投影到3D
    fx, fy = depth_intrin.fx, depth_intrin.fy
    cx, cy = depth_intrin.ppx, depth_intrin.ppy
    
    x = (center_u - cx) * depth_value / fx
    y = (center_v - cy) * depth_value / fy
    z = depth_value
    
    return np.array([x, y, z])


# ========================================
# ROS2 节点
# ========================================
class ButtonDetectorNode(Node):
    """交互式按钮检测器节点"""
    
    def __init__(self):
        super().__init__('button_detector_ros2')
        
        self.get_logger().info("="*70)
        self.get_logger().info("交互式按钮检测器 - ROS2 版本")
        self.get_logger().info("="*70)
        
        # 加载 YOLO 模型
        try:
            self.model = YOLO("yolo_button.pt")
            self.get_logger().info("✓ YOLO 模型加载成功: yolo_button.pt")
        except Exception as e:
            self.get_logger().warn(f"加载 yolo_button.pt 失败: {e}")
            self.get_logger().info("尝试使用 yolo11n.pt...")
            self.model = YOLO("yolo11n.pt")
        
        # 创建发布器
        self.pub_object_point = self.create_publisher(PointStamped, '/object_point', 10)
        self.pub_button_type = self.create_publisher(String, '/button_type', 10)
        self.pub_object_marker = self.create_publisher(Marker, '/object_center_marker', 10)
        
        self.get_logger().info("✓ 发布器已创建")
        self.get_logger().info("  - /object_point")
        self.get_logger().info("  - /button_type")
        self.get_logger().info("  - /object_center_marker")
        
        # 配置 RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 启动管道
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        
        self.get_logger().info("✓ RealSense 相机已启动")
        self.get_logger().info("="*70)
        
        # 创建窗口
        cv2.namedWindow('detection', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('detection', mouse_callback)
        
        # 创建定时器（30Hz）
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
    
    def timer_callback(self):
        """定时器回调：处理相机帧和检测"""
        global all_detections, selected_button_index, selected_button_locked
        global current_depth_data, current_color_data, current_depth_intrin
        
        try:
            # 获取帧
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            aligned_frames = self.align.process(frames)
            
            aligned_depth = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not (aligned_depth and color_frame):
                return
            
            # 转换为 numpy 数组
            depth_data = np.asanyarray(aligned_depth.get_data())
            color_data = np.asanyarray(color_frame.get_data())
            depth_intrin = aligned_depth.profile.as_video_stream_profile().intrinsics
            
            # 保存当前帧数据
            current_depth_data = depth_data
            current_color_data = color_data
            current_depth_intrin = depth_intrin
            
            # 执行 YOLO 检测
            target_boxes = YOLODetection(self.model, color_data, conf_threshold=0.5)
            
            # 更新全局检测结果
            all_detections = []
            for box in target_boxes:
                x1, y1, x2, y2, class_name, conf = box
                all_detections.append((x1, y1, x2, y2, class_name, conf, None))
            
            # 可视化
            annotated_img = visualize_detections(
                color_data, all_detections, selected_button_index
            )
            cv2.imshow('detection', annotated_img)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC - 取消选择
                self.get_logger().info("✗ 已取消选择")
                selected_button_index = -1
                selected_button_locked = False
            
            elif key == 13:  # ENTER - 确认选择
                if selected_button_index >= 0 and selected_button_index < len(all_detections):
                    det = all_detections[selected_button_index]
                    x1, y1, x2, y2, class_name, conf, center_3d = det
                    
                    # 计算3D位置
                    if center_3d is None:
                        center_3d = extract_roi_cloud(
                            depth_data, color_data, 
                            [x1, y1, x2, y2], 
                            depth_intrin
                        )
                        all_detections[selected_button_index] = (x1, y1, x2, y2, class_name, conf, center_3d)
                    
                    if center_3d is not None:
                        self.get_logger().info(f"\n{'='*70}")
                        self.get_logger().info(f"✓ 确认选择！")
                        self.get_logger().info(f"  按钮类型: {class_name}")
                        self.get_logger().info(f"  3D位置: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f})")
                        self.get_logger().info(f"{'='*70}")
                        
                        # 发布按钮位置
                        point_msg = PointStamped()
                        point_msg.header.stamp = self.get_clock().now().to_msg()
                        point_msg.header.frame_id = "camera"
                        point_msg.point.x = float(center_3d[0])
                        point_msg.point.y = float(center_3d[1])
                        point_msg.point.z = float(center_3d[2])
                        self.pub_object_point.publish(point_msg)
                        
                        # 发布按钮类型
                        type_msg = String()
                        type_msg.data = class_name
                        self.pub_button_type.publish(type_msg)
                        
                        # 发布可视化标记
                        marker = Marker()
                        marker.header.frame_id = "camera"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = "button_markers"
                        marker.id = 0
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = float(center_3d[0])
                        marker.pose.position.y = float(center_3d[1])
                        marker.pose.position.z = float(center_3d[2])
                        marker.pose.orientation.w = 1.0
                        marker.scale.x = 0.04
                        marker.scale.y = 0.04
                        marker.scale.z = 0.04
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0
                        self.pub_object_marker.publish(marker)
                        
                        self.get_logger().info("✓ 已发布到话题")
                        
                        # 重置选择
                        selected_button_index = -1
                        selected_button_locked = False
                    else:
                        self.get_logger().warn("✗ 该按钮没有有效的3D坐标")
                else:
                    self.get_logger().warn("✗ 请先点击选择一个按钮")
            
            elif key == ord('q'):  # 退出
                self.get_logger().info("退出程序...")
                raise KeyboardInterrupt
        
        except Exception as e:
            if "timeout" not in str(e).lower():
                self.get_logger().error(f"处理帧错误: {e}")
    
    def destroy_node(self):
        """清理资源"""
        self.pipeline.stop()
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
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
