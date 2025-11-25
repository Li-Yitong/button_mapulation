#!/usr/bin/env python3
"""
诊断深度值单位问题
对比 pyrealsense2 直接读取 vs ROS2 订阅的深度值
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import time
import sys

class DepthDiagnostic:
    def __init__(self):
        print("="*70)
        print("深度值单位诊断工具")
        print("="*70)
        
        # 初始化ROS2
        rclpy.init()
        self.node = Node('depth_diagnostic')
        self.bridge = CvBridge()
        
        # 先检查ROS2话题
        print("\n[1/3] 检查ROS2话题...")
        available_topics = self.node.get_topic_names_and_types()
        depth_topics = [t for t, types in available_topics if 'depth' in t.lower()]
        
        if not depth_topics:
            print("  ❌ 未找到任何深度话题！")
            print("  💡 请先启动相机：")
            print("     ros2 launch realsense2_camera rs_launch.py align_depth:=true")
            sys.exit(1)
        
        print(f"  ✅ 找到 {len(depth_topics)} 个深度话题:")
        for topic in depth_topics:
            print(f"     - {topic}")
        
        # 选择话题（优先选对齐的）
        aligned_topic = None
        rect_topic = None
        for topic in depth_topics:
            if 'aligned_depth_to_color' in topic and 'image_raw' in topic:
                aligned_topic = topic
            elif 'depth/image_rect_raw' in topic:
                rect_topic = topic
        
        if aligned_topic:
            self.depth_topic = aligned_topic
            print(f"\n  🎯 使用对齐深度图: {self.depth_topic}")
        elif rect_topic:
            self.depth_topic = rect_topic
            print(f"\n  ⚠️  使用未对齐深度图: {self.depth_topic}")
            print(f"     （这可能导致坐标不匹配！）")
        else:
            self.depth_topic = depth_topics[0]
            print(f"\n  ⚠️  使用第一个话题: {self.depth_topic}")
        
        # 订阅
        self.ros_depth = None
        self.ros_depth_count = 0
        self.sub = self.node.create_subscription(
            Image, self.depth_topic,
            self.depth_callback, 10
        )
        
        # 初始化相机
        print("\n[2/3] 初始化pyrealsense2直接读取...")
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)
            print("  ✅ 相机初始化成功")
        except Exception as e:
            print(f"  ❌ 相机初始化失败: {e}")
            print(f"  💡 可能相机已被ROS2占用，这是正常的")
            print(f"     将只对比ROS2不同话题的数据")
            self.pipeline = None
    
    def depth_callback(self, msg):
        self.ros_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        self.ros_depth_count += 1
    
    def run(self):
        print("\n[3/3] 采集深度数据...")
        print("  等待ROS2数据...", end='', flush=True)
        
        # 等待ROS2数据（最多5秒）
        start_time = time.time()
        while self.ros_depth is None and time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.ros_depth_count > 0:
                print(f" ✅ 已收到 {self.ros_depth_count} 帧")
                break
            print(".", end='', flush=True)
        
        if self.ros_depth is None:
            print("\n  ❌ 5秒内未收到ROS2深度图！")
            print(f"  💡 检查话题: ros2 topic hz {self.depth_topic}")
            self.cleanup()
            sys.exit(1)
        
        print("\n  ✅ ROS2深度图接收正常\n")
        print("="*70)
        print("开始采样分析（10个样本）")
        print("="*70)
        
        samples_collected = 0
        for i in range(30):  # 尝试30次，收集10个有效样本
            # ROS2订阅
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.ros_depth is None:
                continue
            
            # 采样多个区域（避免单点误差）
            test_points = [
                (320, 240, "中心"),
                (160, 120, "左上"),
                (480, 360, "右下"),
                (320, 120, "上中"),
                (320, 360, "下中"),
            ]
            
            print(f"\n[样本 {samples_collected + 1}]")
            
            # 直接读取（如果可用）
            direct_depth = None
            if self.pipeline is not None:
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=100)
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        direct_depth = np.asanyarray(depth_frame.get_data())
                except:
                    pass
            
            # 分析每个测试点
            for cx, cy, label in test_points[:3]:  # 只分析3个点，避免输出过多
                ros_sample = self.ros_depth[cy-2:cy+3, cx-2:cx+3]
                ros_valid = ros_sample[ros_sample > 0]
                
                if len(ros_valid) == 0:
                    continue
                
                ros_mean = np.mean(ros_valid)
                ros_min = np.min(ros_valid)
                ros_max = np.max(ros_valid)
                
                print(f"  {label} ({cx}, {cy}): ROS2={ros_mean:.1f} (范围:{ros_min:.0f}~{ros_max:.0f})")
                
                # 判断单位
                if ros_mean < 10:
                    print(f"    ⚠️  疑似单位错误：{ros_mean:.3f} 太小（应该是mm，显示为m？）")
                elif ros_mean > 10000:
                    print(f"    ⚠️  疑似异常：{ros_mean:.0f} 太大")
                else:
                    print(f"    ✅ 单位正常：{ros_mean:.0f}mm = {ros_mean/1000:.3f}m")
                
                # 如果有直接读取数据，对比
                if direct_depth is not None:
                    direct_sample = direct_depth[cy-2:cy+3, cx-2:cx+3]
                    direct_valid = direct_sample[direct_sample > 0]
                    if len(direct_valid) > 0:
                        direct_mean = np.mean(direct_valid)
                        diff = abs(direct_mean - ros_mean)
                        if diff < 10:
                            print(f"    ✅ 与直接读取一致 (diff={diff:.1f}mm)")
                        else:
                            print(f"    ⚠️  与直接读取不一致: direct={direct_mean:.1f}, diff={diff:.1f}")
            
            samples_collected += 1
            if samples_collected >= 10:
                break
            
            time.sleep(0.3)
        
        print("\n" + "="*70)
        print("诊断结论")
        print("="*70)
        
        # 最终分析
        if self.ros_depth is not None:
            center_sample = self.ros_depth[238:243, 318:323]
            valid = center_sample[center_sample > 0]
            if len(valid) > 0:
                mean_val = np.mean(valid)
                print(f"\n中心区域平均深度: {mean_val:.1f}")
                
                if 200 < mean_val < 2000:
                    print(f"✅ 深度值正常范围 ({mean_val:.0f}mm = {mean_val/1000:.3f}m)")
                    print(f"   单位：毫米 (mm)")
                    print(f"   转换：depth_m = depth_value / 1000.0")
                elif mean_val < 10:
                    print(f"⚠️  深度值异常偏小 ({mean_val:.3f})")
                    print(f"   可能已经是米单位，不要再除1000")
                    print(f"   转换：depth_m = depth_value")
                else:
                    print(f"⚠️  深度值异常: {mean_val:.1f}")
        
        print("\n" + "="*70)
        self.cleanup()
    
    def cleanup(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except:
                pass
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        diag = DepthDiagnostic()
        diag.run()
    except KeyboardInterrupt:
        print("\n\n用户中断")
        diag.cleanup()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
