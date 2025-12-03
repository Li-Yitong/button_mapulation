#!/usr/bin/env python3
"""
HSV颜色范围调试工具
用于调整蓝色面板的HSV阈值
"""
import cv2
import numpy as np
import pyrealsense2 as rs

def nothing(x):
    pass

# 初始化RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 创建窗口和滑动条
cv2.namedWindow('HSV Tuner')
cv2.createTrackbar('H_min', 'HSV Tuner', 90, 180, nothing)
cv2.createTrackbar('H_max', 'HSV Tuner', 130, 180, nothing)
cv2.createTrackbar('S_min', 'HSV Tuner', 30, 255, nothing)
cv2.createTrackbar('S_max', 'HSV Tuner', 255, 255, nothing)
cv2.createTrackbar('V_min', 'HSV Tuner', 30, 255, nothing)
cv2.createTrackbar('V_max', 'HSV Tuner', 255, 255, nothing)

print("HSV颜色调试工具")
print("- 调整滑动条找到最佳蓝色范围")
print("- 按 's' 保存当前参数")
print("- 按 'q' 退出")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        
        color_image = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # 获取滑动条值
        h_min = cv2.getTrackbarPos('H_min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H_max', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S_min', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S_max', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V_min', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V_max', 'HSV Tuner')
        
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        
        # 生成掩码
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(color_image, color_image, mask=mask)
        
        # 统计
        blue_pixels = np.sum(mask > 0)
        total_pixels = mask.shape[0] * mask.shape[1]
        blue_ratio = blue_pixels / total_pixels * 100
        
        # 显示参数
        cv2.putText(color_image, f"Blue: {blue_ratio:.1f}%", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(color_image, f"HSV: [{h_min},{s_min},{v_min}]-[{h_max},{s_max},{v_max}]", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # 显示
        cv2.imshow('Original', color_image)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print(f"\n保存的HSV范围:")
            print(f"lower_blue = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"upper_blue = np.array([{h_max}, {s_max}, {v_max}])")
            print(f"蓝色占比: {blue_ratio:.1f}%\n")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
