#!/usr/bin/env python3
"""
测试深度提取方法
验证 ROI 点云方法的效果
"""
import numpy as np
import pyrealsense2 as rs
import cv2

def test_depth_extraction():
    """测试深度提取"""
    # 初始化相机
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    
    print("✓ 相机已启动，按 'q' 退出")
    print("="*70)
    
    try:
        for i in range(100):
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            aligned_depth = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not (aligned_depth and color_frame):
                continue
            
            depth_data = np.asanyarray(aligned_depth.get_data())
            color_data = np.asanyarray(color_frame.get_data())
            
            # 统计深度图
            valid_pixels = np.count_nonzero(depth_data > 0)
            total_pixels = depth_data.size
            valid_ratio = valid_pixels / total_pixels * 100
            
            if valid_pixels > 0:
                depth_min = np.min(depth_data[depth_data > 0])
                depth_max = np.max(depth_data)
                depth_mean = np.mean(depth_data[depth_data > 0])
                
                print(f"帧 {i:3d}: 有效像素 {valid_pixels:6d}/{total_pixels} ({valid_ratio:5.1f}%), "
                      f"深度范围 {depth_min:4d}-{depth_max:4d}mm, 平均 {depth_mean:6.0f}mm")
            else:
                print(f"帧 {i:3d}: ❌ 深度图完全无效！")
            
            # 显示
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_data, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            cv2.imshow('Depth', depth_colormap)
            cv2.imshow('Color', color_data)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\n✓ 测试完成")

if __name__ == '__main__':
    test_depth_extraction()
