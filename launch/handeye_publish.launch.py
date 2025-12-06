#!/usr/bin/env python3
"""
手眼标定结果发布启动文件
启动后会持续发布 link6 → camera 的TF变换
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """生成标定结果发布启动描述"""
    
    # Easy_handeye2 发布启动文件
    easy_handeye_publish = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'),
                'launch',
                'publish.launch.py'
            ])
        ]),
        launch_arguments={
            # ========================================
            # 必须与标定时使用的名称一致
            # ========================================
            'name': 'piper_realsense_handeye',
            
            # ========================================
            # 标定类型（必须与标定时一致）
            # ========================================
            'calibration_type': 'eye_in_hand',  # eye_in_hand 或 eye_on_base
            
        }.items()
    )
    
    return LaunchDescription([
        easy_handeye_publish
    ])
