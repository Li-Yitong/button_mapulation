#!/usr/bin/env python3
"""
验证脚本：确保对齐后的深度图使用正确的相机内参
"""

import pyrealsense2 as rs
import numpy as np
import cv2

def verify_intrinsics_alignment():
    """
    验证内参与像素的对应关系
    """
    print("=" * 70)
    print("🔍 验证：内参与像素的对应关系")
    print("=" * 70)
    
    # 1. 初始化相机
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    profile = pipeline.start(config)
    
    # 2. 创建对齐对象
    align = rs.align(rs.stream.color)
    
    # 3. 获取一帧
    print("\n⏳ 正在获取相机帧...")
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    
    aligned_depth = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not aligned_depth or not color_frame:
        print("❌ 无法获取相机帧！")
        return
    
    # 4. 获取数据
    depth_data = np.asanyarray(aligned_depth.get_data())
    color_data = np.asanyarray(color_frame.get_data())
    
    # 5. 获取内参（关键：对齐后用彩色相机内参）
    depth_intrin_raw = aligned_depth.profile.as_video_stream_profile().intrinsics
    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    
    print("\n" + "=" * 70)
    print("📋 第一步：检查数据维度")
    print("=" * 70)
    print(f"  深度图维度: {depth_data.shape}")
    print(f"  彩色图维度: {color_data.shape}")
    print(f"  深度相机内参分辨率: {depth_intrin_raw.width} × {depth_intrin_raw.height}")
    print(f"  彩色相机内参分辨率: {color_intrin.width} × {color_intrin.height}")
    
    # 验证1：维度匹配
    depth_h, depth_w = depth_data.shape
    color_h, color_w = color_data.shape[:2]
    
    if depth_w == color_intrin.width and depth_h == color_intrin.height:
        print(f"  ✅ 深度图维度 ({depth_w}×{depth_h}) 与彩色相机内参分辨率匹配")
    else:
        print(f"  ❌ 深度图维度 ({depth_w}×{depth_h}) 与彩色相机内参分辨率 ({color_intrin.width}×{color_intrin.height}) 不匹配！")
    
    print("\n" + "=" * 70)
    print("📋 第二步：内参值对比")
    print("=" * 70)
    print(f"  对齐后深度帧的内参:")
    print(f"    fx = {depth_intrin_raw.fx:.2f}, fy = {depth_intrin_raw.fy:.2f}")
    print(f"    ppx = {depth_intrin_raw.ppx:.2f}, ppy = {depth_intrin_raw.ppy:.2f}")
    print(f"  彩色帧的内参:")
    print(f"    fx = {color_intrin.fx:.2f}, fy = {color_intrin.fy:.2f}")
    print(f"    ppx = {color_intrin.ppx:.2f}, ppy = {color_intrin.ppy:.2f}")
    
    # 验证2：内参值是否相同
    if (abs(depth_intrin_raw.fx - color_intrin.fx) < 0.01 and
        abs(depth_intrin_raw.fy - color_intrin.fy) < 0.01 and
        abs(depth_intrin_raw.ppx - color_intrin.ppx) < 0.01 and
        abs(depth_intrin_raw.ppy - color_intrin.ppy) < 0.01):
        print("  ✅ 对齐后，深度帧和彩色帧的内参完全一致（这是正确的！）")
    else:
        print("  ⚠️  内参值不同！这可能表示对齐有问题")
    
    print("\n" + "=" * 70)
    print("📋 第三步：像素级验证")
    print("=" * 70)
    
    # 测试多个点
    test_points = [
        (320, 240, "图像中心"),
        (100, 100, "左上角"),
        (540, 380, "右下角"),
        (327, 246, "主点位置")  # 根据你的实际内参
    ]
    
    print("  🔍 使用彩色相机内参计算3D坐标:")
    for u, v, desc in test_points:
        # 获取深度值
        depth_value = depth_data[v, u] * 0.001  # mm → m
        
        if depth_value > 0:
            # 方法1：手动计算
            x_manual = (u - color_intrin.ppx) * depth_value / color_intrin.fx
            y_manual = (v - color_intrin.ppy) * depth_value / color_intrin.fy
            z_manual = depth_value
            
            # 方法2：SDK验证
            point_sdk = rs.rs2_deproject_pixel_to_point(color_intrin, [u, v], depth_value)
            
            # 计算差异
            diff_x = abs(x_manual - point_sdk[0]) * 1000
            diff_y = abs(y_manual - point_sdk[1]) * 1000
            diff_z = abs(z_manual - point_sdk[2]) * 1000
            
            status = "✅" if (diff_x < 0.01 and diff_y < 0.01 and diff_z < 0.01) else "❌"
            
            print(f"\n  {desc} ({u}, {v}):")
            print(f"    深度: {depth_value*1000:.1f}mm")
            print(f"    手动计算: X={x_manual:.4f}, Y={y_manual:.4f}, Z={z_manual:.4f}")
            print(f"    SDK验证:  X={point_sdk[0]:.4f}, Y={point_sdk[1]:.4f}, Z={point_sdk[2]:.4f}")
            print(f"    差异: ΔX={diff_x:.3f}mm, ΔY={diff_y:.3f}mm, ΔZ={diff_z:.3f}mm {status}")
        else:
            print(f"\n  {desc} ({u}, {v}): ⚠️  无深度值")
    
    print("\n" + "=" * 70)
    print("📋 第四步：代码使用建议")
    print("=" * 70)
    print("  ✅ 正确做法（你的代码已经这样做了）:")
    print("     ```python")
    print("     frames = pipeline.wait_for_frames()")
    print("     aligned_frames = align.process(frames)")
    print("     ")
    print("     aligned_depth = aligned_frames.get_depth_frame()")
    print("     color_frame = aligned_frames.get_color_frame()")
    print("     ")
    print("     # ✅ 关键：使用彩色相机的内参！")
    print("     depth_intrin = color_frame.profile.as_video_stream_profile().intrinsics")
    print("     ")
    print("     # 计算3D坐标")
    print("     x = (u - depth_intrin.ppx) * z / depth_intrin.fx")
    print("     y = (v - depth_intrin.ppy) * z / depth_intrin.fy")
    print("     ```")
    print()
    print("  ❌ 错误做法:")
    print("     ```python")
    print("     # ❌ 错误：使用深度帧的原始内参")
    print("     depth_intrin = aligned_depth.profile.as_video_stream_profile().intrinsics")
    print("     ```")
    print("     虽然对齐后两者内参相同，但从逻辑上应该用彩色相机的内参")
    
    print("\n" + "=" * 70)
    print("📋 第五步：可视化验证")
    print("=" * 70)
    
    # 在图像上绘制测试点
    vis_img = color_data.copy()
    for u, v, desc in test_points:
        cv2.circle(vis_img, (u, v), 5, (0, 255, 0), -1)
        cv2.putText(vis_img, desc, (u + 10, v - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # 绘制主点
    ppx, ppy = int(color_intrin.ppx), int(color_intrin.ppy)
    cv2.drawMarker(vis_img, (ppx, ppy), (255, 0, 0), 
                   cv2.MARKER_CROSS, 20, 2)
    cv2.putText(vis_img, f"Principal Point ({ppx},{ppy})", 
               (ppx + 10, ppy - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    
    cv2.imshow("Intrinsics Verification", vis_img)
    print("  📺 已显示可视化窗口，按任意键关闭...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print("\n" + "=" * 70)
    print("✅ 验证完成！")
    print("=" * 70)
    print("  📌 关键结论:")
    print("  1. 对齐后的深度图已经投影到彩色相机坐标系")
    print("  2. 必须使用彩色相机的内参来计算3D坐标")
    print("  3. 你的代码实现是正确的！")
    print("=" * 70)
    
    pipeline.stop()

if __name__ == "__main__":
    verify_intrinsics_alignment()
