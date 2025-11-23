#!/usr/bin/env python3
"""
交互式按钮检测器 - ROS2 版本（订阅话题）
订阅 RealSense ROS2 节点的图像话题
使用 yolo_button.pt 模型
支持用户点击选择要操作的按钮
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
current_camera_info = None


# ========================================
# 鼠标回调函数
# ========================================
def mouse_callback(event, x, y, flags, param):
    """处理鼠标事件，允许用户点击选择按钮"""
    global selected_button_index, mouse_x, mouse_y, all_detections
    global current_depth_data, current_color_data, current_camera_info
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
                        current_camera_info
                    )
                    all_detections[idx] = (x1, y1, x2, y2, class_name, conf, center_3d)
                
                print(f"\n{'='*60}")
                print(f"✓ 已选择按钮 #{idx}")
                print(f"  类型: {class_name}")
                print(f"  置信度: {conf:.2f}")
                if center_3d is not None:
                    print(f"  3D坐标: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f})")
                print(f"{'='*60}")
                break
        
        if not found:
            print(" → ✗ 未点击到按钮")


# ========================================
# YOLO 检测
# ========================================
def YOLODetection(model, color_img, conf_threshold=0.5):
    """YOLO 检测"""
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
    
    return target_boxes


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
def extract_roi_cloud(depth_data, color_data, bbox, camera_info):
    """从深度图中提取 ROI 区域的 3D 中心点"""
    x1, y1, x2, y2 = bbox
    
    # 计算2D边界框的中心点
    center_u = int((x1 + x2) / 2)
    center_v = int((y1 + y2) / 2)
    
    # 获取相机内参
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]
    
    # 在中心点周围取一个小区域的深度值（避免噪声）
    WINDOW_SIZE = 5
    u_min = max(0, center_u - WINDOW_SIZE)
    u_max = min(depth_data.shape[1], center_u + WINDOW_SIZE)
    v_min = max(0, center_v - WINDOW_SIZE)
    v_max = min(depth_data.shape[0], center_v + WINDOW_SIZE)
    
    depth_window = depth_data[v_min:v_max, u_min:u_max]
    valid_depths = depth_window[depth_window > 0]
    
    if len(valid_depths) == 0:
        print(f"  ✗ 警告: 区域 ({center_u}, {center_v}) 无有效深度值")
        return None
    
    # 使用中位数深度
    depth_value = np.median(valid_depths)
    depth_m = depth_value / 1000.0  # 转换为米
    
    # 使用针孔相机模型计算3D坐标
    x = (center_u - cx) * depth_m / fx
    y = (center_v - cy) * depth_m / fy
    z = depth_m
    
    print(f"  → 3D坐标: ({x:.3f}, {y:.3f}, {z:.3f}) m")
    
    return [x, y, z]


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
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/depth/image_rect_raw'
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/depth/camera_info'
        )
        
        # 时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.image_callback)
        
        self.get_logger().info("✓ 已订阅相机话题")
        self.get_logger().info("  - /camera/camera/color/image_raw")
        self.get_logger().info("  - /camera/camera/depth/image_rect_raw")
        self.get_logger().info("  - /camera/camera/depth/camera_info")
        
        # 创建 OpenCV 窗口
        cv2.namedWindow('detection', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('detection', mouse_callback)
        
        # 创建定时器处理键盘输入
        self.timer = self.create_timer(0.01, self.keyboard_callback)
        
        self.get_logger().info("="*70)
        self.get_logger().info("✓ 初始化完成，等待图像...")
        self.get_logger().info("="*70)
    
    def image_callback(self, color_msg, depth_msg, camera_info_msg):
        """图像话题回调"""
        global all_detections, selected_button_index, selected_button_locked
        global current_depth_data, current_color_data, current_camera_info
        
        try:
            # 转换 ROS 图像消息为 OpenCV 格式
            color_data = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_data = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            # 保存当前帧数据
            current_depth_data = depth_data
            current_color_data = color_data
            current_camera_info = camera_info_msg
            
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
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"处理图像错误: {e}")
    
    def keyboard_callback(self):
        """处理键盘输入"""
        global all_detections, selected_button_index, selected_button_locked
        global current_depth_data, current_color_data, current_camera_info
        
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
                        current_depth_data, current_color_data, 
                        [x1, y1, x2, y2], 
                        current_camera_info
                    )
                    all_detections[selected_button_index] = (x1, y1, x2, y2, class_name, conf, center_3d)
                
                if center_3d is not None:
                    self.get_logger().info(f"\n{'='*70}")
                    self.get_logger().info(f"✓ 确认选择:")
                    self.get_logger().info(f"  按钮类型: {class_name}")
                    self.get_logger().info(f"  置信度: {conf:.2f}")
                    self.get_logger().info(f"  3D位置: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f}) m")
                    self.get_logger().info(f"{'='*70}")
                    
                    # 发布到ROS话题
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = "camera_color_optical_frame"
                    point_msg.point.x = center_3d[0]
                    point_msg.point.y = center_3d[1]
                    point_msg.point.z = center_3d[2]
                    self.point_pub.publish(point_msg)
                    
                    type_msg = String()
                    type_msg.data = class_name
                    self.type_pub.publish(type_msg)
                    
                    # 发布可视化 Marker
                    marker = Marker()
                    marker.header.frame_id = "camera_color_optical_frame"
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
