#!/bin/bash
# 清理环境运行button_actions.py

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
source /home/robot/button/V4.0/project2/piper_ros/install/setup.bash

cd /home/robot/button/V4.0/project2

echo "✓ 环境已清理，只加载ROS2 Foxy"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_VERSION=$ROS_VERSION"
echo ""

# 运行button_actions.py
python3 button_actions.py
