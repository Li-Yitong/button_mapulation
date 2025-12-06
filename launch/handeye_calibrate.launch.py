#!/usr/bin/env python3
"""
手眼标定启动文件 - eye-in-hand模式
相机安装在机械臂末端，AprilTag固定在桌面/基座附近
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """生成eye-in-hand标定启动描述"""
    
    # Easy_handeye2 标定启动文件
    easy_handeye_calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'),
                'launch',
                'calibrate.launch.py'
            ])
        ]),
        launch_arguments={
            # ========================================
            # 标定名称（多次标定时修改此值）
            # ========================================
            'name': 'piper_realsense_handeye',
            
            # ========================================
            # 标定类型：eye_in_hand（相机安装在机械臂末端）
            # ========================================
            'calibration_type': 'eye_in_hand',  # eye_in_hand 或 eye_on_base
            
            # ========================================
            # 坐标系名称（必须与TF树一致）
            # ========================================
            'robot_base_frame': 'base_link',              # 机器人基座坐标系
            'robot_effector_frame': 'link6',              # 机器人末端坐标系（相机安装位置）
            'tracking_base_frame': 'camera_color_optical_frame',   # 相机光学坐标系（随末端移动）
            'tracking_marker_frame': 'checkerboard',      # 棋盘格坐标系（固定在桌面/被检测）
            
        }.items()
    )
    
    return LaunchDescription([
        easy_handeye_calibrate
    ])
