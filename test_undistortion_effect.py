#!/usr/bin/env python3
"""
测试相机畸变校正效果
对比原始图像和去畸变图像的差异
"""
import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    print("="*70)
    print("RealSense 畸变校正效果测试")
    print("="*70)
    
    # 配置相机
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    print("启动相机...")
    profile = pipeline.start(config)
    
    # 获取内参
    color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    
    # 提取畸变系数
    distortion_coeffs = np.array(color_intrin.coeffs, dtype=np.float64)
    camera_matrix = np.array([
        [color_intrin.fx, 0, color_intrin.ppx],
        [0, color_intrin.fy, color_intrin.ppy],
        [0, 0, 1]
    ], dtype=np.float64)
    
    print(f"\n相机内参:")
    print(f"  fx={color_intrin.fx:.2f}, fy={color_intrin.fy:.2f}")
    print(f"  ppx={color_intrin.ppx:.2f}, ppy={color_intrin.ppy:.2f}")
    print(f"\n畸变系数 (k1, k2, p1, p2, k3):")
    print(f"  {distortion_coeffs}")
    
    # 判断是否有明显畸变
    max_distortion = np.max(np.abs(distortion_coeffs))
    print(f"\n最大畸变系数: {max_distortion:.6f}")
    
    if max_distortion < 0.001:
        print("⚠️  畸变系数接近0，可能已经过工厂校准或使用鱼眼模型")
    else:
        print(f"✓ 检测到明显畸变（系数>{max_distortion:.3f}），去畸变将改善精度")
    
    print("\n" + "="*70)
    print("按 'q' 退出 | 按 's' 保存对比图")
    print("="*70)
    
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            # 原始图像
            original = np.asanyarray(color_frame.get_data())
            
            # 去畸变图像
            undistorted = cv2.undistort(original, camera_matrix, distortion_coeffs)
            
            # 计算差异图
            diff = cv2.absdiff(original, undistorted)
            diff_enhanced = cv2.convertScaleAbs(diff, alpha=5.0)  # 放大5倍便于观察
            
            # 在图像上绘制网格（便于观察畸变）
            h, w = original.shape[:2]
            grid_color = (0, 255, 0)
            for i in range(0, w, 50):
                cv2.line(original, (i, 0), (i, h), grid_color, 1)
                cv2.line(undistorted, (i, 0), (i, h), grid_color, 1)
            for i in range(0, h, 50):
                cv2.line(original, (0, i), (w, i), grid_color, 1)
                cv2.line(undistorted, (0, i), (w, i), grid_color, 1)
            
            # 拼接显示
            top_row = np.hstack([original, undistorted])
            bottom_row = np.hstack([diff_enhanced, np.zeros_like(diff_enhanced)])
            comparison = np.vstack([top_row, bottom_row])
            
            # 添加文字标注
            cv2.putText(comparison, "Original (with grid)", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(comparison, "Undistorted", (w + 10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(comparison, "Difference (x5)", (10, h + 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Undistortion Comparison', comparison)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"undistortion_comparison_{int(cv2.getTickCount())}.png"
                cv2.imwrite(filename, comparison)
                print(f"✓ 已保存对比图: {filename}")
    
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("测试结束")

if __name__ == '__main__':
    main()
