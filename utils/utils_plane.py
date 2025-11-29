"""
面板法向量计算工具
用于处理倾斜面板的按钮操作
Author: GitHub Copilot
Date: 2025-11-29
"""
import numpy as np
import cv2
from typing import Tuple, Optional, List, Dict


def fit_plane_ransac(points_3d: np.ndarray, 
                     max_iterations: int = 1000, 
                     distance_threshold: float = 0.01) -> Tuple[Optional[np.ndarray], Optional[float], Optional[np.ndarray]]:
    """
    使用RANSAC算法拟合3D点云的平面
    
    参数:
        points_3d: Nx3数组，点云坐标
        max_iterations: RANSAC最大迭代次数
        distance_threshold: 内点距离阈值（米）
    
    返回:
        (normal_vector, d, inliers): 法向量、平面偏移、内点索引
        平面方程: normal · [x, y, z] + d = 0
    """
    if len(points_3d) < 3:
        return None, None, None
    
    best_inliers = []
    best_plane = None
    
    for _ in range(max_iterations):
        # 随机选择3个点
        sample_indices = np.random.choice(len(points_3d), 3, replace=False)
        p1, p2, p3 = points_3d[sample_indices]
        
        # 计算平面法向量: (p2-p1) × (p3-p1)
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        
        # 归一化
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        
        # 计算d: ax + by + cz + d = 0
        d = -np.dot(normal, p1)
        
        # 计算所有点到平面的距离
        distances = np.abs(np.dot(points_3d, normal) + d)
        
        # 统计内点
        inliers = np.where(distances < distance_threshold)[0]
        
        # 更新最佳模型
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane = (normal, d)
    
    if best_plane is None:
        return None, None, None
    
    # 使用所有内点重新拟合平面（最小二乘）
    inlier_points = points_3d[best_inliers]
    centroid = np.mean(inlier_points, axis=0)
    centered = inlier_points - centroid
    
    # SVD分解，最小奇异值对应的向量即为法向量
    _, _, vh = np.linalg.svd(centered)
    normal = vh[2, :]
    d = -np.dot(normal, centroid)
    
    return normal, d, best_inliers


def compute_robust_panel_normal(all_detections: List, 
                                depth_data: np.ndarray, 
                                depth_intrin,
                                expand_ratio: float = 0.2,
                                min_buttons: int = 2,
                                verbose: bool = True) -> Optional[Dict]:
    """
    鲁棒的面板法向量计算
    
    策略：
    1. 使用按钮周围的环形区域（避开按钮本身）
    2. 计算深度中位数，过滤离群点
    3. RANSAC拟合平面
    
    参数:
        all_detections: 所有检测到的按钮列表 [(x1, y1, x2, y2, cls, conf, center_3d), ...]
        depth_data: 深度图（单位：毫米）
        depth_intrin: RealSense内参
        expand_ratio: 环形区域扩展比例（默认20%）
        min_buttons: 最少需要的按钮数量
        verbose: 是否打印调试信息
    
    返回:
        字典包含: {
            'normal': 法向量 (3,),
            'd': 平面偏移,
            'inliers': 内点索引,
            'inlier_ratio': 内点比例,
            'total_points': 总点数,
            'median_depth': 中位数深度
        }
        如果失败返回None
    """
    if len(all_detections) < min_buttons:
        if verbose:
            print(f"  ⚠️  检测到的按钮过少: {len(all_detections)} < {min_buttons}")
        return None
    
    # Step 1: 计算所有按钮的深度中位数和分布
    all_button_depths = []
    for det in all_detections:
        if len(det) >= 7:
            x1, y1, x2, y2, _, _, center_3d = det[:7]
        else:
            x1, y1, x2, y2, _, _ = det[:6]
            center_3d = None
        
        if center_3d is not None and len(center_3d) >= 3:
            all_button_depths.append(center_3d[2])
        else:
            # 如果没有预计算的3D中心，从深度图读取
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            depth_mm = depth_data[cy, cx]
            if depth_mm > 0:
                all_button_depths.append(depth_mm / 1000.0)
    
    if len(all_button_depths) < min_buttons:
        if verbose:
            print(f"  ⚠️  有效深度的按钮过少: {len(all_button_depths)}")
        return None
    
    median_depth = np.median(all_button_depths)
    std_depth = np.std(all_button_depths)
    
    # 自适应深度范围：±2σ或至少±5cm（适应高低按钮）
    depth_min = median_depth - max(0.05, 2 * std_depth)
    depth_max = median_depth + max(0.05, 2 * std_depth)
    
    if verbose:
        print(f"\n=== 面板法向量计算 ===")
        print(f"  检测到按钮: {len(all_detections)}个")
        print(f"  深度中位数: {median_depth*100:.1f}cm")
        print(f"  深度标准差: {std_depth*100:.1f}cm")
        print(f"  深度过滤范围: {depth_min*100:.1f}cm ~ {depth_max*100:.1f}cm")
    
    # Step 2: 🔧 新方案：计算全局边界框，提取互补区域的点云
    # 统计所有按钮的全局边界
    all_x1, all_y1, all_x2, all_y2 = [], [], [], []
    button_masks = []  # 存储每个按钮的掩码区域
    
    for det in all_detections:
        if len(det) >= 7:
            x1, y1, x2, y2, _, _, _ = det[:7]
        else:
            x1, y1, x2, y2, _, _ = det[:6]
        
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        all_x1.append(x1)
        all_y1.append(y1)
        all_x2.append(x2)
        all_y2.append(y2)
        button_masks.append((x1, y1, x2, y2))
    
    # 全局边界框（包含所有按钮的最小矩形）
    global_x1 = max(0, min(all_x1))
    global_y1 = max(0, min(all_y1))
    global_x2 = min(depth_data.shape[1], max(all_x2))
    global_y2 = min(depth_data.shape[0], max(all_y2))
    
    bbox_width = global_x2 - global_x1
    bbox_height = global_y2 - global_y1
    
    if verbose:
        print(f"  全局边界框: ({global_x1}, {global_y1}) → ({global_x2}, {global_y2})")
        print(f"  边界框尺寸: {bbox_width} x {bbox_height} pixels")
    
    # 提取面板区域点云（边界框内 - 按钮区域 = 互补区域）
    background_points = []
    fx, fy = depth_intrin.fx, depth_intrin.fy
    ppx, ppy = depth_intrin.ppx, depth_intrin.ppy
    
    # 遍历全局边界框内的所有像素（隔行采样提速）
    for v in range(global_y1, global_y2, 2):
        for u in range(global_x1, global_x2, 2):
            # 检查是否在任何按钮区域内
            is_inside_button = False
            for (bx1, by1, bx2, by2) in button_masks:
                if bx1 <= u <= bx2 and by1 <= v <= by2:
                    is_inside_button = True
                    break
            
            # 跳过按钮区域，只保留面板区域
            if is_inside_button:
                continue
            
            depth_mm = depth_data[v, u]
            if depth_mm <= 0:
                continue
            
            depth_m = depth_mm / 1000.0
            
            # 深度过滤：只保留接近中位数的点
            if depth_m < depth_min or depth_m > depth_max:
                continue
            
            # 转换为3D点（相机坐标系）
            x = (u - ppx) * depth_m / fx
            y = (v - ppy) * depth_m / fy
            z = depth_m
            
            background_points.append([x, y, z])
    
    if len(background_points) < 100:
        if verbose:
            print(f"  ⚠️  背景点过少: {len(background_points)}")
        return None
    
    background_points = np.array(background_points)
    if verbose:
        print(f"  ✓ 提取背景点: {len(background_points)}个")
    
    # Step 3: RANSAC拟合平面
    normal, d, inliers = fit_plane_ransac(
        background_points, 
        max_iterations=1000,
        distance_threshold=0.008  # 8mm容差（适应微小起伏）
    )
    
    if normal is None:
        if verbose:
            print("  ❌ 平面拟合失败")
        return None
    
    # Step 4: 验证法向量方向（应该指向相机，即Z分量<0）
    if normal[2] > 0:
        normal = -normal
        d = -d
    
    inlier_ratio = len(inliers) / len(background_points)
    
    if verbose:
        print(f"  ✓ 平面拟合成功:")
        print(f"    内点数: {len(inliers)}/{len(background_points)} ({inlier_ratio*100:.1f}%)")
        print(f"    法向量: ({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})")
        print(f"    平面方程: {normal[0]:.4f}x + {normal[1]:.4f}y + {normal[2]:.4f}z + {d:.4f} = 0")
    
    # Step 5: 验证拟合质量
    if inlier_ratio < 0.6:
        if verbose:
            print(f"  ⚠️  内点率过低 ({inlier_ratio*100:.1f}%)，平面拟合质量可能不佳")
    
    return {
        'normal': normal,
        'd': d,
        'inliers': inliers,
        'inlier_ratio': inlier_ratio,
        'total_points': len(background_points),
        'median_depth': median_depth
    }


def compute_approach_pose(button_center: np.ndarray, 
                         normal_vector: np.ndarray, 
                         approach_distance: float = 0.30) -> np.ndarray:
    """
    计算接近位姿（按钮上方距离approach_distance处）
    
    参数:
        button_center: 按钮中心3D坐标 (相机系) [x, y, z]
        normal_vector: 面板法向量 (相机系) [nx, ny, nz]
        approach_distance: 接近距离（米，默认30cm）
    
    返回:
        4x4齐次变换矩阵（相机系下的接近位姿）
    """
    # 接近点 = 按钮中心 + 法向量 * 距离
    # 注意：法向量指向相机（Z<0），所以实际上是减去距离
    approach_point = button_center - normal_vector * approach_distance
    
    # 构造旋转矩阵：Gripper的Z轴对准法向量
    z_axis = normal_vector / np.linalg.norm(normal_vector)
    
    # 选择一个世界"上"方向来构造X轴
    # 相机坐标系：X右 Y下 Z前
    # 如果法向量接近竖直（与Y轴平行），使用X轴作为参考
    world_up = np.array([0, -1, 0])  # 相机Y轴向下
    if abs(np.dot(z_axis, world_up)) > 0.9:  # 接近竖直
        world_up = np.array([1, 0, 0])  # 使用X轴
    
    # X轴 = 世界上方向 × Z轴
    x_axis = np.cross(world_up, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    
    # Y轴 = Z轴 × X轴
    y_axis = np.cross(z_axis, x_axis)
    
    # 构造齐次变换矩阵
    T_approach = np.eye(4)
    T_approach[:3, 0] = x_axis
    T_approach[:3, 1] = y_axis
    T_approach[:3, 2] = z_axis
    T_approach[:3, 3] = approach_point
    
    return T_approach


def visualize_panel_normal(color_img: np.ndarray, 
                          all_detections: List,
                          panel_info: Optional[Dict],
                          show_rings: bool = True) -> np.ndarray:
    """
    在图像上可视化面板法向量计算过程
    
    参数:
        color_img: 原始彩色图像
        all_detections: 所有检测到的按钮列表
        panel_info: compute_robust_panel_normal的返回结果
        show_rings: 是否显示环形采样区域
    
    返回:
        可视化后的图像
    """
    vis = color_img.copy()
    
    if panel_info is None:
        # 显示失败信息
        cv2.putText(vis, "Panel Normal: FAILED", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return vis
    
    # 1. 绘制环形采样区域（半透明绿色）
    if show_rings:
        overlay = vis.copy()
        for det in all_detections:
            if len(det) >= 7:
                x1, y1, x2, y2, _, _, _ = det[:7]
            else:
                x1, y1, x2, y2, _, _ = det[:6]
            
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            w, h = x2 - x1, y2 - y1
            
            # 外圈（扩展20%）
            expand_ratio = 0.2
            outer_rect = [
                int(cx - w * (0.5 + expand_ratio)), 
                int(cy - h * (0.5 + expand_ratio)),
                int(cx + w * (0.5 + expand_ratio)), 
                int(cy + h * (0.5 + expand_ratio))
            ]
            cv2.rectangle(overlay, 
                         (outer_rect[0], outer_rect[1]), 
                         (outer_rect[2], outer_rect[3]),
                         (0, 255, 0), 2)
        
        vis = cv2.addWeighted(vis, 0.7, overlay, 0.3, 0)
    
    # 2. 显示统计信息
    normal = panel_info['normal']
    info_lines = [
        f"Panel Normal Detection",
        f"Points: {panel_info['total_points']}",
        f"Inliers: {panel_info['inlier_ratio']*100:.1f}%",
        f"Normal: ({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})",
        f"Depth: {panel_info['median_depth']*100:.1f}cm"
    ]
    
    y_offset = 30
    for i, text in enumerate(info_lines):
        color = (0, 255, 255) if i == 0 else (255, 255, 0)
        thickness = 2 if i == 0 else 1
        cv2.putText(vis, text, (10, y_offset + i*25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness)
    
    # 3. 质量指示器
    quality_color = (0, 255, 0) if panel_info['inlier_ratio'] > 0.7 else \
                   (0, 165, 255) if panel_info['inlier_ratio'] > 0.5 else (0, 0, 255)
    cv2.circle(vis, (color_img.shape[1] - 30, 30), 15, quality_color, -1)
    
    return vis
