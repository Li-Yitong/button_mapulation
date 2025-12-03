"""
面板法向量计算工具
用于处理倾斜面板的按钮操作
Author: GitHub Copilot
Date: 2025-11-29
"""
import numpy as np
import cv2
from typing import Tuple, Optional, List, Dict
import os

# ========================================
# 🎨 HSV颜色过滤参数（蓝色面板）
# ========================================
# 通过 tune_blue_hsv.py 调试得到的最佳参数
# 可通过环境变量覆盖，便于现场调试

def _parse_hsv_env(var_name: str, default_list: list) -> np.ndarray:
    """从环境变量解析HSV参数"""
    env_val = os.environ.get(var_name)
    if env_val:
        try:
            return np.array([int(x) for x in env_val.split(',')])
        except:
            print(f"⚠️  环境变量 {var_name} 格式错误，使用默认值: {default_list}")
    return np.array(default_list)

# 默认HSV范围（优化后的参数 - 精心调整过的）
# DEFAULT_HSV_LOWER = [21, 96, 57]   # H: 蓝色色相, S: 中高饱和度, V: 中等亮度
# DEFAULT_HSV_UPPER = [103, 151, 89] # H: 蓝色范围, S: 高饱和, V: 较高亮度
# lower_blue = np.array([90, 124, 46])
# upper_blue = np.array([120, 255, 81])
DEFAULT_HSV_LOWER = [90, 124, 46]   # H: 蓝色色相, S: 中高饱和度, V: 中等亮度
DEFAULT_HSV_UPPER = [120, 255, 81] # H: 蓝色范围, S: 高饱和, V: 较高亮度
# 从环境变量读取（如果设置）
# 使用方法: export BLUE_HSV_LOWER="92,108,43" BLUE_HSV_UPPER="111,179,244"
BLUE_HSV_LOWER = _parse_hsv_env('BLUE_HSV_LOWER', DEFAULT_HSV_LOWER)
BLUE_HSV_UPPER = _parse_hsv_env('BLUE_HSV_UPPER', DEFAULT_HSV_UPPER)

# RANSAC参数
RANSAC_MAX_ITERATIONS = 1000      # RANSAC最大迭代次数
RANSAC_DISTANCE_THRESHOLD = 0.008 # 内点距离阈值（8mm）

# 深度过滤参数
DEPTH_MIN = 0.15  # 最小深度（米）
DEPTH_MAX = 1.5   # 最大深度（米）

# 采样参数
SAMPLING_STRIDE = 4  # 采样步长（每N个像素采样1个）

# 统计过滤参数
IQR_MULTIPLIER = 1.5  # IQR方法的倍数（Tukey's fence）

print(f"🎨 面板法向量计算 - HSV参数:")
print(f"  HSV下限: H={BLUE_HSV_LOWER[0]}, S={BLUE_HSV_LOWER[1]}, V={BLUE_HSV_LOWER[2]}")
print(f"  HSV上限: H={BLUE_HSV_UPPER[0]}, S={BLUE_HSV_UPPER[1]}, V={BLUE_HSV_UPPER[2]}")
print(f"  RANSAC: 迭代={RANSAC_MAX_ITERATIONS}, 阈值={RANSAC_DISTANCE_THRESHOLD*1000:.1f}mm")
print(f"  深度范围: {DEPTH_MIN*100:.0f}cm ~ {DEPTH_MAX*100:.0f}cm")
print(f"  采样步长: 1/{SAMPLING_STRIDE}")
print()


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


def compute_panel_normal_from_blue_region(depth_data: np.ndarray,
                                          color_image: np.ndarray,
                                          depth_intrin,
                                          hsv_lower: Optional[np.ndarray] = None,
                                          hsv_upper: Optional[np.ndarray] = None,
                                          verbose: bool = True) -> Optional[Dict]:
    """
    ✅ 新方案：直接从蓝色面板区域计算法向量（不依赖按钮数量）
    
    关键改进：
    - 法向量只与蓝色面板有关，与按钮数量无关
    - 直接对整个蓝色区域进行 RANSAC 拟合
    - 避免了按钮数量 < 3 时的拟合失败问题
    
    参数:
        depth_data: 深度图（单位：毫米）
        color_image: 彩色图像（BGR格式）
        depth_intrin: RealSense内参
        hsv_lower: HSV下限 (默认 [92, 108, 43])
        hsv_upper: HSV上限 (默认 [111, 179, 244])
        verbose: 是否打印调试信息
    
    返回:
        {
            'normal': 法向量 (3,),
            'd': 平面偏移,
            'inliers': 内点索引,
            'inlier_ratio': 内点比例,
            'total_points': 总点数,
            'median_depth': 中位数深度
        }
    """
    # 1. 提取蓝色区域
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    
    # 使用全局常量或传入参数
    if hsv_lower is None:
        lower_blue = BLUE_HSV_LOWER
    else:
        lower_blue = hsv_lower
    
    if hsv_upper is None:
        upper_blue = BLUE_HSV_UPPER
    else:
        upper_blue = hsv_upper
    
    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    
    # 形态学操作：去除噪点
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    
    blue_pixel_count = np.sum(blue_mask > 0)
    
    if verbose:
        print(f"\n=== 面板法向量计算（纯蓝色区域法） ===")
        print(f"  🎨 HSV范围: H=[{lower_blue[0]},{upper_blue[0]}], S=[{lower_blue[1]},{upper_blue[1]}], V=[{lower_blue[2]},{upper_blue[2]}]")
        print(f"  蓝色像素数: {blue_pixel_count} ({blue_pixel_count/blue_mask.size*100:.1f}%)")
    
    if blue_pixel_count < 500:
        if verbose:
            print(f"  ❌ 蓝色区域过小，无法拟合")
        return None
    
    # 2. 从蓝色区域提取3D点云
    fx, fy = depth_intrin.fx, depth_intrin.fy
    ppx, ppy = depth_intrin.ppx, depth_intrin.ppy
    
    blue_points = []
    blue_coords = np.argwhere(blue_mask > 0)  # 返回 (y, x) 坐标
    
    # 隔行采样加速
    blue_coords = blue_coords[::SAMPLING_STRIDE]
    
    for v, u in blue_coords:
        depth_mm = depth_data[v, u]
        if depth_mm <= 0:
            continue
        
        depth_m = depth_mm / 1000.0
        
        # 基本深度过滤
        if depth_m < DEPTH_MIN or depth_m > DEPTH_MAX:
            continue
        
        x = (u - ppx) * depth_m / fx
        y = (v - ppy) * depth_m / fy
        z = depth_m
        
        blue_points.append([x, y, z])
    
    if len(blue_points) < 100:
        if verbose:
            print(f"  ❌ 有效3D点过少: {len(blue_points)}")
        return None
    
    blue_points = np.array(blue_points)
    
    if verbose:
        print(f"  ✓ 提取3D点: {len(blue_points)}个")
    
    # 3. IQR统计学过滤（去除离群点）
    z_coords = blue_points[:, 2]
    q1 = np.percentile(z_coords, 25)
    q3 = np.percentile(z_coords, 75)
    iqr = q3 - q1
    
    z_min = q1 - IQR_MULTIPLIER * iqr
    z_max = q3 + IQR_MULTIPLIER * iqr
    
    valid_mask = (z_coords >= z_min) & (z_coords <= z_max)
    blue_points = blue_points[valid_mask]
    
    median_depth = np.median(z_coords)
    
    if verbose:
        print(f"  📊 深度统计: 中位数={median_depth*100:.1f}cm, IQR=[{q1*100:.1f}, {q3*100:.1f}]cm")
        print(f"  过滤后点数: {len(blue_points)}个")
    
    if len(blue_points) < 100:
        if verbose:
            print(f"  ❌ 过滤后点数不足")
        return None
    
    # 4. RANSAC拟合平面
    normal, d, inliers = fit_plane_ransac(
        blue_points,
        max_iterations=RANSAC_MAX_ITERATIONS,
        distance_threshold=RANSAC_DISTANCE_THRESHOLD
    )
    
    if normal is None:
        if verbose:
            print(f"  ❌ RANSAC拟合失败")
        return None
    
    # 5. 确保法向量指向相机（Z < 0）
    if normal[2] > 0:
        normal = -normal
        d = -d
    
    inlier_ratio = len(inliers) / len(blue_points)
    
    if verbose:
        print(f"  ✓ RANSAC成功: 内点率={inlier_ratio*100:.1f}%")
        print(f"    法向量: ({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})")
    
    return {
        'normal': normal,
        'd': d,
        'inliers': inliers,
        'inlier_ratio': inlier_ratio,
        'total_points': len(blue_points),
        'median_depth': median_depth
    }


def compute_robust_panel_normal(all_detections: List, 
                                depth_data: np.ndarray, 
                                depth_intrin,
                                expand_ratio: float = 0.2,
                                min_buttons: int = 2,
                                verbose: bool = True,
                                color_image: Optional[np.ndarray] = None,
                                use_color_filter: bool = True,
                                hsv_lower: Optional[np.ndarray] = None,
                                hsv_upper: Optional[np.ndarray] = None) -> Optional[Dict]:
    """
    鲁棒的面板法向量计算（保留旧接口，优先使用新方法）
    
    ⚠️ 已弃用：此函数依赖按钮数量，当按钮 < 3 时可能失败
    ✅ 推荐使用：compute_panel_normal_from_blue_region（纯蓝色区域法）
    
    策略：
    1. 如果启用颜色过滤且有彩色图像，优先使用纯蓝色区域法
    2. 否则使用旧方法（按钮周围环形区域）
    
    参数:
        all_detections: 所有检测到的按钮列表 [(x1, y1, x2, y2, cls, conf, center_3d), ...]
        depth_data: 深度图（单位：毫米）
        depth_intrin: RealSense内参
        expand_ratio: 环形区域扩展比例（默认20%）
        min_buttons: 最少需要的按钮数量
        verbose: 是否打印调试信息
        color_image: 彩色图像（BGR格式，用于颜色过滤）
        use_color_filter: 是否启用蓝色面板颜色过滤（默认True）
        hsv_lower: HSV下限 (可选，默认 [92, 108, 43])
        hsv_upper: HSV上限 (可选，默认 [111, 179, 244])
    
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
    # ✅ 优先使用纯蓝色区域法（不依赖按钮数量）
    if use_color_filter and color_image is not None:
        result = compute_panel_normal_from_blue_region(
            depth_data=depth_data,
            color_image=color_image,
            depth_intrin=depth_intrin,
            hsv_lower=hsv_lower,
            hsv_upper=hsv_upper,
            verbose=verbose
        )
        if result is not None:
            return result
        
        if verbose:
            print("  ⚠️  纯蓝色区域法失败，尝试旧方法（按钮周围区域）")
    
    # ⚠️ 降级到旧方法（需要按钮数量）
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
    
    # 🔧 新增：准备颜色过滤器（蓝色面板）
    color_filter_enabled = use_color_filter and color_image is not None
    if color_filter_enabled:
        # 转换到HSV空间进行颜色过滤
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # 🎨 HSV参数：优先使用传入参数，否则使用全局常量
        if hsv_lower is None:
            lower_blue = BLUE_HSV_LOWER
        else:
            lower_blue = hsv_lower
        
        if hsv_upper is None:
            upper_blue = BLUE_HSV_UPPER
        else:
            upper_blue = hsv_upper
        
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        
        if verbose:
            blue_pixel_count = np.sum(blue_mask > 0)
            total_pixels = (global_x2 - global_x1) * (global_y2 - global_y1)
            print(f"  🎨 颜色过滤 (HSV范围):")
            print(f"    下限: H={lower_blue[0]}, S={lower_blue[1]}, V={lower_blue[2]}")
            print(f"    上限: H={upper_blue[0]}, S={upper_blue[1]}, V={upper_blue[2]}")
            print(f"    蓝色像素占比: {blue_pixel_count/total_pixels*100:.1f}%")
    
    # 提取面板区域点云（边界框内 - 按钮区域 = 互补区域）
    background_points = []
    fx, fy = depth_intrin.fx, depth_intrin.fy
    ppx, ppy = depth_intrin.ppx, depth_intrin.ppy
    
    color_filtered_count = 0  # 统计被颜色过滤掉的点
    
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
            
            # 🔧 新增：颜色过滤（只保留蓝色像素）
            if color_filter_enabled:
                if blue_mask[v, u] == 0:  # 不是蓝色，跳过
                    color_filtered_count += 1
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
    
    if verbose and color_filter_enabled:
        print(f"  🎨 颜色过滤移除: {color_filtered_count} 个非蓝色点")
    
    if len(background_points) < 100:
        if verbose:
            print(f"  ⚠️  背景点过少: {len(background_points)}")
        return None
    
    background_points = np.array(background_points)
    original_count = len(background_points)
    
    if verbose:
        print(f"  ✓ 提取背景点: {original_count}个")
    
    # Step 3: 🔧 新增：统计学离群点过滤
    # 方法选择：IQR（四分位距）更鲁棒，不受极端值影响；3σ适用于正态分布
    # 这里使用IQR方法（更适合有噪声的深度数据）
    z_coords = background_points[:, 2]
    
    # IQR方法（Inter-Quartile Range）
    q1 = np.percentile(z_coords, 25)  # 第一四分位数（25%）
    q3 = np.percentile(z_coords, 75)  # 第三四分位数（75%）
    iqr = q3 - q1  # 四分位距
    
    # Tukey's fence: Q1 - k×IQR ~ Q3 + k×IQR
    # k=1.5时覆盖约99.3%的正常数据（假设正态分布）
    z_min = q1 - IQR_MULTIPLIER * iqr
    z_max = q3 + IQR_MULTIPLIER * iqr
    
    # 过滤离群点
    valid_mask = (z_coords >= z_min) & (z_coords <= z_max)
    background_points = background_points[valid_mask]
    outliers_removed = original_count - len(background_points)
    
    if verbose:
        z_median = np.median(z_coords)
        print(f"  📊 统计学过滤 (IQR方法):")
        print(f"    深度中位数: {z_median*100:.1f}cm")
        print(f"    四分位距: Q1={q1*100:.1f}cm, Q3={q3*100:.1f}cm, IQR={iqr*100:.1f}cm")
        print(f"    有效区间: [{z_min*100:.1f}cm, {z_max*100:.1f}cm] (Tukey's fence)")
        print(f"    剔除离群点: {outliers_removed}个 ({outliers_removed/original_count*100:.1f}%)")
        print(f"    保留有效点: {len(background_points)}个")
    
    if len(background_points) < 100:
        if verbose:
            print(f"  ⚠️  过滤后点数不足: {len(background_points)}")
        return None
    
    # Step 4: RANSAC拟合平面（处理剩余的小范围离群点）
    normal, d, inliers = fit_plane_ransac(
        background_points, 
        max_iterations=RANSAC_MAX_ITERATIONS,
        distance_threshold=RANSAC_DISTANCE_THRESHOLD
    )
    
    if normal is None:
        if verbose:
            print("  ❌ 平面拟合失败")
        return None
    
    # Step 5: 验证法向量方向（应该指向相机，即Z分量<0）
    if normal[2] > 0:
        normal = -normal
        d = -d
    
    inlier_ratio = len(inliers) / len(background_points)
    
    if verbose:
        print(f"  ✓ RANSAC平面拟合成功:")
        print(f"    内点数: {len(inliers)}/{len(background_points)} ({inlier_ratio*100:.1f}%)")
        print(f"    法向量: ({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})")
        print(f"    平面方程: {normal[0]:.4f}x + {normal[1]:.4f}y + {normal[2]:.4f}z + {d:.4f} = 0")
    
    # Step 6: 验证拟合质量
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
                          show_rings: bool = True,
                          show_color_mask: bool = False) -> np.ndarray:
    """
    在图像上可视化面板法向量计算过程
    
    参数:
        color_img: 原始彩色图像
        all_detections: 所有检测到的按钮列表
        panel_info: compute_robust_panel_normal的返回结果
        show_rings: 是否显示环形采样区域
        show_color_mask: 是否显示蓝色掩码叠加（调试用）
    
    返回:
        可视化后的图像
    """
    vis = color_img.copy()
    
    if panel_info is None:
        # 显示失败信息
        cv2.putText(vis, "Panel Normal: FAILED", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return vis
    
    # 0. 🔧 新增：显示蓝色掩码（调试模式）
    if show_color_mask:
        hsv_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 30, 30])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
        
        # 叠加蓝色半透明层
        blue_overlay = vis.copy()
        blue_overlay[blue_mask > 0] = [255, 0, 0]  # 蓝色区域显示为蓝色
        vis = cv2.addWeighted(vis, 0.7, blue_overlay, 0.3, 0)
    
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
        f"Normal(Camera): ({normal[0]:+.3f}, {normal[1]:+.3f}, {normal[2]:+.3f})",
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
