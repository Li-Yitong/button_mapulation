#!/bin/bash
# 清理环境，只使用ROS2 Foxy

# 清除所有ROS环境变量
unset ROS_DISTRO
unset ROS_VERSION
unset ROS_PYTHON_VERSION
unset ROS_PACKAGE_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset CMAKE_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH

# 只source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Source piper_ros
source /home/unitree/manipulation/button/vba/piper_ros/install/setup.bash

echo "✓ 环境已清理，只加载ROS2 Foxy"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_VERSION=$ROS_VERSION"

# 启动MoveIt2
ros2 launch piper_with_gripper_moveit demo_foxy.launch.py
