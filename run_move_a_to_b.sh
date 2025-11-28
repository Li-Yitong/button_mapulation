#!/bin/bash
# 运行 move_a_to_b.py 的包装脚本
# 确保正确加载 ROS 环境

cd /home/robot/button/V4.0/project2

# Source piper_ros 工作空间环境
if [ -f "piper_ros/devel/setup.bash" ]; then
    source piper_ros/devel/setup.bash
    echo "✓ 已加载 piper_ros 环境"
else
    echo "⚠️  警告: piper_ros/devel/setup.bash 不存在"
fi

# 运行 Python 程序
python3 move_a_to_b.py
