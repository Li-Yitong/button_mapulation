#!/bin/bash
# 检查ROS2 topics状态

echo "========================================"
echo "检查ROS2 Topics和TF状态"
echo "========================================"
echo ""

# 清理环境
for var in $(env | grep -E '^(ROS_|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH|PKG_CONFIG_PATH|AMENT_|COLCON_)' | cut -d'=' -f1); do
    unset $var
done

# 加载ROS2
source /opt/ros/foxy/setup.bash
source /home/robot/button/V4.0/project2/piper_ros/install/setup.bash

echo "1. 检查joint_states topic:"
ros2 topic info /joint_states
echo ""
ros2 topic echo /joint_states --once
echo ""

echo "2. 检查TF:"
ros2 run tf2_ros tf2_echo base_link link6 2>&1 | head -20
echo ""

echo "3. 检查move_group相关topics:"
ros2 topic list | grep move_group
echo ""

echo "4. 检查规划场景:"
ros2 topic echo /monitored_planning_scene --once 2>&1 | head -30

echo ""
echo "========================================"
