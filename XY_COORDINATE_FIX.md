# XY坐标计算修正说明

## 📋 问题描述

在 `realsense_yolo_button_interactive_ros2_direct_april.py` 的 `get_bbox_depth_cloud()` 函数中，原实现使用**所有像素的统计中位数**来计算XYZ坐标：

```python
# ❌ 修正前（不准确）
pc = np.column_stack((x_valid, y_valid, z_valid))
center_3d = np.median(pc, axis=0)  # XYZ都使用中位数
```

### ⚠️ 问题分析

当按钮在检测框内**不居中**时，这种方法会导致：
- **XY坐标偏移**：如果有效深度点集中在检测框的某一侧，中位数会偏向那一侧
- **不符合标准CV实践**：YOLO检测框的中心应该代表物体的几何中心
- **测试验证**：偏移可达 **6.85mm**（在350mm距离处）

---

## ✅ 修正方案

采用**标准计算机视觉方法**：

```python
# ✅ 修正后（准确）
cx = (x1 + x2) // 2  # 检测框几何中心X
cy = (y1 + y2) // 2  # 检测框几何中心Y
z_center = np.median(z_valid)  # 深度使用中位数（鲁棒抗噪）

# 根据针孔相机模型计算真实3D坐标
x_center = (cx - depth_intrin.ppx) * z_center / depth_intrin.fx
y_center = (cy - depth_intrin.ppy) * z_center / depth_intrin.fy

center_3d = np.array([x_center, y_center, z_center])
```

### 🎯 核心原理

| 坐标 | 计算方式 | 理由 |
|------|----------|------|
| **X, Y** | 使用检测框**几何中心** | 代表物体在图像平面的位置 |
| **Z** | 使用深度**中位数** | 鲁棒抗噪，过滤离群点 |

### 📐 针孔相机模型

$$
\begin{aligned}
X &= \frac{(c_x - p_{px}) \cdot Z}{f_x} \\
Y &= \frac{(c_y - p_{py}) \cdot Z}{f_y} \\
Z &= \text{median}(\text{depth}[\text{bbox内有效点}])
\end{aligned}
$$

其中：
- $(c_x, c_y)$ = 检测框几何中心
- $(p_{px}, p_{py})$ = 相机主点
- $(f_x, f_y)$ = 焦距

---

## 📊 修正效果对比

### 测试场景
- **检测框**: 80×80像素，中心在 (320, 240)
- **深度**: 350mm
- **模拟情况**: 80%有效点集中在检测框左上角（模拟按钮不居中）

### 结果对比

| 方法 | X坐标 | Y坐标 | Z坐标 | 偏差 |
|------|-------|-------|-------|------|
| **修正后** | 0.000m | 0.000m | 0.350m | ✅ 基准 |
| **修正前** | -0.00455m | -0.00512m | 0.350m | ❌ 6.85mm |

### 💡 结论
- **ΔX**: 4.55mm
- **ΔY**: 5.12mm
- **总偏差**: **6.85mm** ← 这在按钮操作中是**不可接受**的误差！

---

## 🔧 修改文件

### 主文件
`realsense_yolo_button_interactive_ros2_direct_april.py`

### 修改函数
`get_bbox_depth_cloud()` (约第280-350行)

### 修改内容
1. **计算方式改变**:
   - XY：检测框几何中心 → 针孔模型投影
   - Z：深度中位数（不变）

2. **函数文档更新**:
   - 说明新的坐标计算方式
   - 标注"标准CV方法"

---

## ✅ 验证方法

运行测试脚本：
```bash
python3 test_bbox_center_fix.py
```

预期输出：
- 展示两种方法的坐标差异
- 证明修正的必要性

---

## 📌 备注

### 备用方法不受影响
`get_robust_depth()` 函数**已经是正确实现**：
```python
# ✅ 这个函数从一开始就是对的
center_3d = np.array([
    (cx - depth_intrin.ppx) * depth / depth_intrin.fx,  # 使用中心点cx
    (cy - depth_intrin.ppy) * depth / depth_intrin.fy,  # 使用中心点cy
    depth
])
```

### 为什么Z仍然用中位数？
- **抗噪声**：单个像素深度可能不准确
- **鲁棒性**：中位数过滤离群点
- **合理性**：深度是垂直于图像平面的，使用区域内的统计值更可靠

---

## 🎓 参考资料

- [针孔相机模型](https://en.wikipedia.org/wiki/Pinhole_camera_model)
- [Intel RealSense SDK - Projection](https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0)
- 标准CV实践：YOLO检测框中心 = 物体几何中心

---

**修正日期**: 2024年12月4日  
**修正者**: GitHub Copilot  
**验证状态**: ✅ 已通过测试
