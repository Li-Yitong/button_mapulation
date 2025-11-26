#!/bin/bash
# 手动启动 MoveIt2 服务

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "======================================================================"
echo "启动 MoveIt2 服务（Piper With Gripper）"
echo "======================================================================"

source /opt/ros/foxy/setup.bash
source ./piper_ros/install/setup.bash

echo "✓ 环境已配置"
echo "启动 MoveIt2..."

LC_NUMERIC=en_US.UTF-8 ros2 launch piper_with_gripper_moveit piper_moveit.launch.py
