#!/usr/bin/env python3
"""
检查深度图与彩色图是否对齐
可视化深度图叠加到彩色图上，检查边缘是否吻合
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import sys

class AlignmentChecker(Node):
    def __init__(self):
        super().__init__('alignment_checker')
        
        print("="*70)
        print("深度图对齐检查工具")
        print("="*70)
        
        self.bridge = CvBridge()
        
        # 检查可用话题
        print("\n[1/2] 扫描可用话题...")
        available_topics = dict(self.get_topic_names_and_types())
        
        color_topics = [t for t in available_topics.keys() if 'color/image_raw' in t]
        depth_topics = [t for t in available_topics.keys() if 'depth' in t and 'image' in t]
        
        if not color_topics:
            print("  ❌ 未找到彩色图话题！")
            sys.exit(1)
        if not depth_topics:
            print("  ❌ 未找到深度图话题！")
            sys.exit(1)
        
        print(f"  ✅ 彩色图话题:")
        for t in color_topics:
            print(f"     - {t}")
        
        print(f"  ✅ 深度图话题:")
        for t in depth_topics:
            print(f"     - {t}")
        
        # 选择话题
        self.color_topic = color_topics[0]
        
        # 优先选择对齐的深度图
        self.aligned_depth_topic = None
        self.raw_depth_topic = None
        
        for t in depth_topics:
            if 'aligned_depth_to_color' in t and 'image_raw' in t:
                self.aligned_depth_topic = t
            elif 'depth/image_rect_raw' in t or 'depth/image_raw' in t:
                self.raw_depth_topic = t
        
        print(f"\n[2/2] 订阅话题...")
        print(f"  彩色图: {self.color_topic}")
        
        # 创建订阅器
        self.color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        
        # 对比两种深度图
        if self.aligned_depth_topic:
            print(f"  对齐深度: {self.aligned_depth_topic}")
            self.aligned_sub = message_filters.Subscriber(self, Image, self.aligned_depth_topic)
            self.ts_aligned = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.aligned_sub], queue_size=10, slop=0.1
            )
            self.ts_aligned.registerCallback(self.aligned_callback)
        
        if self.raw_depth_topic:
            print(f"  原始深度: {self.raw_depth_topic}")
            self.raw_sub = message_filters.Subscriber(self, Image, self.raw_depth_topic)
            self.ts_raw = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.raw_sub], queue_size=10, slop=0.1
            )
            self.ts_raw.registerCallback(self.raw_callback)
        
        print("\n" + "="*70)
        print("按键说明:")
        print("  [1] 切换显示模式：叠加/并排/深度热力图")
        print("  [2] 切换深度图：对齐/原始")
        print("  [SPACE] 暂停/继续")
        print("  [Q] 退出")
        print("="*70)
        
        # 状态
        self.display_mode = 0  # 0=叠加, 1=并排, 2=热力图
        self.show_aligned = True
        self.paused = False
        
        self.aligned_color = None
        self.aligned_depth = None
        self.raw_color = None
        self.raw_depth = None
        
        cv2.namedWindow('Alignment Check', cv2.WINDOW_NORMAL)
        
    def aligned_callback(self, color_msg, depth_msg):
        """对齐深度图回调"""
        if not self.paused:
            self.aligned_color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.aligned_depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
    
    def raw_callback(self, color_msg, depth_msg):
        """原始深度图回调"""
        if not self.paused:
            self.raw_color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
    
    def visualize_overlay(self, color, depth):
        """叠加模式：深度边缘叠加到彩色图上"""
        # 深度图归一化
        depth_normalized = depth.copy().astype(float)
        depth_normalized[depth_normalized == 0] = np.nan
        
        # 边缘检测
        depth_8u = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        edges = cv2.Canny(depth_8u, 50, 150)
        
        # 叠加到彩色图
        overlay = color.copy()
        overlay[edges > 0] = [0, 255, 0]  # 绿色边缘
        
        # 在中心画十字线和采样点
        h, w = color.shape[:2]
        cx, cy = w // 2, h // 2
        
        # 十字线
        cv2.line(overlay, (cx - 50, cy), (cx + 50, cy), (0, 255, 255), 1)
        cv2.line(overlay, (cx, cy - 50), (cx, cy + 50), (0, 255, 255), 1)
        
        # 采样点深度值
        sample_points = [
            (cx, cy, "中心"),
            (cx - 100, cy, "左"),
            (cx + 100, cy, "右"),
            (cx, cy - 100, "上"),
            (cx, cy + 100, "下"),
        ]
        
        y_offset = 30
        for px, py, label in sample_points:
            if 0 <= px < w and 0 <= py < h:
                d = depth[py, px]
                if d > 0:
                    cv2.circle(overlay, (px, py), 3, (255, 0, 255), -1)
                    cv2.putText(overlay, f"{label}:{d}mm", (10, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    y_offset += 20
        
        return overlay
    
    def visualize_side_by_side(self, color, depth):
        """并排模式：彩色图 | 深度热力图"""
        # 深度热力图
        depth_normalized = depth.copy().astype(float)
        depth_normalized[depth_normalized == 0] = np.nan
        
        if not np.all(np.isnan(depth_normalized)):
            depth_min = np.nanmin(depth_normalized)
            depth_max = np.nanmax(depth_normalized)
            
            depth_normalized = (depth_normalized - depth_min) / (depth_max - depth_min) * 255
            depth_normalized = np.nan_to_num(depth_normalized, nan=0).astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # 添加范围信息
            cv2.putText(depth_colormap, f"Min: {depth_min:.0f}mm", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(depth_colormap, f"Max: {depth_max:.0f}mm", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            depth_colormap = np.zeros_like(color)
        
        # 并排显示
        combined = np.hstack([color, depth_colormap])
        return combined
    
    def visualize_heatmap(self, color, depth):
        """热力图模式：深度热力图半透明叠加"""
        depth_normalized = depth.copy().astype(float)
        depth_normalized[depth_normalized == 0] = np.nan
        
        if not np.all(np.isnan(depth_normalized)):
            depth_min = np.nanmin(depth_normalized)
            depth_max = np.nanmax(depth_normalized)
            
            depth_normalized = (depth_normalized - depth_min) / (depth_max - depth_min) * 255
            depth_normalized = np.nan_to_num(depth_normalized, nan=0).astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # 半透明叠加
            alpha = 0.5
            overlay = cv2.addWeighted(color, 1 - alpha, depth_colormap, alpha, 0)
            
            # 添加颜色条
            h, w = overlay.shape[:2]
            colorbar = np.zeros((h, 50, 3), dtype=np.uint8)
            for i in range(h):
                val = int((1 - i / h) * 255)
                colorbar[i, :] = cv2.applyColorMap(np.array([[val]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
            
            # 添加刻度
            for i in range(0, h, h // 5):
                depth_val = depth_max - (depth_max - depth_min) * i / h
                cv2.putText(colorbar, f"{depth_val:.0f}", (5, i + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            overlay = np.hstack([overlay, colorbar])
        else:
            overlay = color.copy()
        
        return overlay
    
    def check_alignment_quality(self, color, depth):
        """检查对齐质量"""
        h, w = color.shape[:2]
        
        # 采样多个点
        test_points = [
            (w // 2, h // 2),  # 中心
            (w // 4, h // 4),  # 左上
            (3 * w // 4, h // 4),  # 右上
            (w // 4, 3 * h // 4),  # 左下
            (3 * w // 4, 3 * h // 4),  # 右下
        ]
        
        valid_count = 0
        for px, py in test_points:
            # 在彩色图上检测是否有物体（梯度变化）
            roi = color[max(0, py - 5):min(h, py + 5), max(0, px - 5):min(w, px + 5)]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gradient = np.std(gray)
            
            # 深度值
            d = depth[py, px]
            
            # 如果彩色图有物体（梯度>10）且深度有效，认为对齐
            if gradient > 10 and d > 0:
                valid_count += 1
        
        quality = valid_count / len(test_points) * 100
        return quality
    
    def run(self):
        """主循环"""
        print("\n✅ 开始检测...\n")
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # 选择要显示的数据
                if self.show_aligned and self.aligned_color is not None and self.aligned_depth is not None:
                    color = self.aligned_color
                    depth = self.aligned_depth
                    title = "对齐深度图 (Aligned)"
                elif not self.show_aligned and self.raw_color is not None and self.raw_depth is not None:
                    color = self.raw_color
                    depth = self.raw_depth
                    title = "原始深度图 (Raw)"
                else:
                    cv2.waitKey(1)
                    continue
                
                # 根据模式渲染
                if self.display_mode == 0:
                    display = self.visualize_overlay(color, depth)
                    mode_text = "叠加模式 (Overlay)"
                elif self.display_mode == 1:
                    display = self.visualize_side_by_side(color, depth)
                    mode_text = "并排模式 (Side-by-Side)"
                else:
                    display = self.visualize_heatmap(color, depth)
                    mode_text = "热力图模式 (Heatmap)"
                
                # 检查对齐质量
                quality = self.check_alignment_quality(color, depth)
                
                # 添加状态栏
                status_bar = np.zeros((80, display.shape[1], 3), dtype=np.uint8)
                status_bar[:] = (40, 40, 40)
                
                cv2.putText(status_bar, f"{title} | {mode_text}", (10, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(status_bar, f"对齐质量: {quality:.0f}%", (10, 55),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                           (0, 255, 0) if quality > 80 else (0, 165, 255), 2)
                
                # 判断
                if quality > 80:
                    align_status = "✅ 对齐良好"
                elif quality > 50:
                    align_status = "⚠️ 对齐一般"
                else:
                    align_status = "❌ 对齐不佳"
                
                cv2.putText(status_bar, align_status, (display.shape[1] - 200, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                if self.paused:
                    cv2.putText(status_bar, "PAUSED", (display.shape[1] // 2 - 50, 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                final_display = np.vstack([status_bar, display])
                cv2.imshow('Alignment Check', final_display)
                
                # 键盘
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('1'):
                    self.display_mode = (self.display_mode + 1) % 3
                    print(f"切换显示模式: {['叠加', '并排', '热力图'][self.display_mode]}")
                elif key == ord('2'):
                    self.show_aligned = not self.show_aligned
                    print(f"切换深度图: {'对齐' if self.show_aligned else '原始'}")
                elif key == ord(' '):
                    self.paused = not self.paused
                    print(f"{'暂停' if self.paused else '继续'}")
        
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            cv2.destroyAllWindows()
            self.destroy_node()


def main():
    rclpy.init()
    
    try:
        checker = AlignmentChecker()
        checker.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
