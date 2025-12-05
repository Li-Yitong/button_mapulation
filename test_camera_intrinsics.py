#!/usr/bin/env python3
"""
测试脚本：验证相机内参获取
"""

import pyrealsense2 as rs
import numpy as np

def test_camera_intrinsics():
    """测试并显示相机内参"""
    print("=" * 70)
    print("📷 RealSense 相机内参测试")
    print("=" * 70)
    
    # 1. 初始化
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # 2. 启动
    profile = pipeline.start(config)
    
    # 3. 对齐
    align = rs.align(rs.stream.color)
    
    # 4. 获取一帧
    print("\n⏳ 正在获取相机帧...")
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not depth_frame or not color_frame:
        print("❌ 无法获取相机帧！")
        return
    
    # 5. 获取内参
    depth_intrin_raw = depth_frame.profile.as_video_stream_profile().intrinsics
    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    
    print("\n" + "=" * 70)
    print("📊 原始深度相机内参（未对齐）")
    print("=" * 70)
    print(f"  分辨率: {depth_intrin_raw.width} × {depth_intrin_raw.height}")
    print(f"  焦距: fx = {depth_intrin_raw.fx:.2f}, fy = {depth_intrin_raw.fy:.2f}")
    print(f"  主点: ppx = {depth_intrin_raw.ppx:.2f}, ppy = {depth_intrin_raw.ppy:.2f}")
    print(f"  畸变模型: {depth_intrin_raw.model}")
    print(f"  畸变系数: {depth_intrin_raw.coeffs}")
    
    print("\n" + "=" * 70)
    print("📊 彩色相机内参")
    print("=" * 70)
    print(f"  分辨率: {color_intrin.width} × {color_intrin.height}")
    print(f"  焦距: fx = {color_intrin.fx:.2f}, fy = {color_intrin.fy:.2f}")
    print(f"  主点: ppx = {color_intrin.ppx:.2f}, ppy = {color_intrin.ppy:.2f}")
    print(f"  畸变模型: {color_intrin.model}")
    print(f"  畸变系数: {color_intrin.coeffs}")
    
    print("\n" + "=" * 70)
    print("✅ 对齐后应该使用的内参")
    print("=" * 70)
    print("  ➡️  使用 彩色相机内参 (color_intrin)")
    print("  ➡️  因为深度图已经对齐到彩色相机视角")
    
    print("\n" + "=" * 70)
    print("🔍 测试坐标投影")
    print("=" * 70)
    
    # 测试中心点投影
    cx, cy = 320, 240  # 图像中心
    depth_value = depth_frame.get_distance(cx, cy)  # 单位：米
    
    if depth_value > 0:
        print(f"  图像中心点 ({cx}, {cy}) 的深度: {depth_value:.3f}m")
        
        # 使用彩色相机内参计算3D坐标
        x_3d = (cx - color_intrin.ppx) * depth_value / color_intrin.fx
        y_3d = (cy - color_intrin.ppy) * depth_value / color_intrin.fy
        z_3d = depth_value
        
        print(f"  ✅ 使用彩色相机内参计算的3D坐标:")
        print(f"     X = {x_3d:.4f}m")
        print(f"     Y = {y_3d:.4f}m")
        print(f"     Z = {z_3d:.4f}m")
        
        # 验证：使用 rs2_deproject_pixel_to_point
        point_rs = rs.rs2_deproject_pixel_to_point(color_intrin, [cx, cy], depth_value)
        print(f"  🔍 RealSense SDK 内置函数验证:")
        print(f"     X = {point_rs[0]:.4f}m")
        print(f"     Y = {point_rs[1]:.4f}m")
        print(f"     Z = {point_rs[2]:.4f}m")
        
        # 检查差异
        diff = np.abs(np.array([x_3d, y_3d, z_3d]) - np.array(point_rs))
        print(f"  📊 差异: ΔX={diff[0]*1000:.2f}mm, ΔY={diff[1]*1000:.2f}mm, ΔZ={diff[2]*1000:.2f}mm")
    else:
        print("  ⚠️  中心点无深度值")
    
    print("\n" + "=" * 70)
    print("💡 总结")
    print("=" * 70)
    print("  1. 对齐后的深度图 → 使用彩色相机内参")
    print("  2. 坐标公式: X = (u - ppx) * Z / fx")
    print("  3.           Y = (v - ppy) * Z / fy")
    print("  4. 验证方法: 与 rs2_deproject_pixel_to_point 结果对比")
    print("=" * 70)
    
    # 关闭
    pipeline.stop()
    print("\n✅ 测试完成！")

if __name__ == "__main__":
    test_camera_intrinsics()
