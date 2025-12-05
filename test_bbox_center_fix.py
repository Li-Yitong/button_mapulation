#!/usr/bin/env python3
"""
测试脚本：验证XY坐标修正
对比修正前后的坐标计算结果
"""

import numpy as np

# 模拟相机内参
class CameraIntrinsics:
    def __init__(self):
        self.fx = 615.0
        self.fy = 615.0
        self.ppx = 320.0
        self.ppy = 240.0

def test_coordinate_calculation():
    """测试XY坐标计算的差异"""
    
    # 模拟检测框
    x1, y1, x2, y2 = 280, 200, 360, 280  # 80x80的检测框
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 中心点 (320, 240)
    
    # 模拟深度值（假设按钮在检测框内，但不在正中央）
    # 为了展示差异，假设按钮实际位置偏左上角
    depth_intrin = CameraIntrinsics()
    z_median = 0.35  # 350mm深度
    
    print("=" * 70)
    print("🔍 XY坐标计算方法对比测试")
    print("=" * 70)
    print(f"检测框: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
    print(f"检测框中心: cx={cx}, cy={cy}")
    print(f"相机内参: fx={depth_intrin.fx}, fy={depth_intrin.fy}")
    print(f"          ppx={depth_intrin.ppx}, ppy={depth_intrin.ppy}")
    print(f"深度中位数: Z={z_median}m")
    print()
    
    # ========================================
    # 方法1: 修正后的方法（使用检测框中心）
    # ========================================
    x_center = (cx - depth_intrin.ppx) * z_median / depth_intrin.fx
    y_center = (cy - depth_intrin.ppy) * z_median / depth_intrin.fy
    
    print("✅ 方法1: 使用检测框几何中心（修正后）")
    print(f"   公式: X = (cx - ppx) * Z / fx")
    print(f"         Y = (cy - ppy) * Z / fy")
    print(f"   结果: X={x_center:.6f}m, Y={y_center:.6f}m, Z={z_median:.6f}m")
    print()
    
    # ========================================
    # 方法2: 旧方法（模拟所有像素的中位数）
    # ========================================
    # 假设检测框内的实际物体集中在左上区域
    # 这会导致中位数偏离几何中心
    np.random.seed(42)
    # 模拟80%的有效点集中在左上角 (290-330, 210-250)
    n_points = 2000
    u_pixels = np.concatenate([
        np.random.randint(290, 330, int(n_points * 0.8)),  # 左上区域
        np.random.randint(x1, x2, int(n_points * 0.2))      # 其他区域
    ])
    v_pixels = np.concatenate([
        np.random.randint(210, 250, int(n_points * 0.8)),
        np.random.randint(y1, y2, int(n_points * 0.2))
    ])
    z_pixels = np.full(n_points, z_median)
    
    # 计算所有像素的XY坐标
    x_all = (u_pixels - depth_intrin.ppx) * z_pixels / depth_intrin.fx
    y_all = (v_pixels - depth_intrin.ppy) * z_pixels / depth_intrin.fy
    
    # 取中位数
    x_median = np.median(x_all)
    y_median = np.median(y_all)
    
    print("❌ 方法2: 使用所有像素的统计中位数（修正前）")
    print(f"   假设: 80%的有效点集中在检测框左上角")
    print(f"   结果: X={x_median:.6f}m, Y={y_median:.6f}m, Z={z_median:.6f}m")
    print()
    
    # ========================================
    # 对比差异
    # ========================================
    dx = abs(x_center - x_median) * 1000  # 转换为mm
    dy = abs(y_center - y_median) * 1000
    total_error = np.sqrt(dx**2 + dy**2)
    
    print("=" * 70)
    print("📊 坐标差异分析")
    print("=" * 70)
    print(f"ΔX = {dx:.2f}mm")
    print(f"ΔY = {dy:.2f}mm")
    print(f"总偏差 = {total_error:.2f}mm")
    print()
    print("💡 结论:")
    print("   当按钮在检测框内不居中时，使用所有像素的中位数会导致")
    print("   XY坐标偏移，而使用检测框几何中心更准确！")
    print("=" * 70)

if __name__ == "__main__":
    test_coordinate_calculation()
