#!/bin/bash
# 清理所有 ROS1 环境变量，只保留 ROS2 Foxy

# 清除所有 ROS 相关环境变量
unset ROS_ROOT
unset ROS_MASTER_URI
unset ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES
unset ROS_ETC_DIR

# 清理 PATH，移除 noetic 和 piper_ws
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "noetic\|piper_ws/devel" | tr '\n' ':' | sed 's/:$//')

# 清理 PYTHONPATH，移除 noetic 和 piper_ws
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "noetic\|piper_ws/devel" | tr '\n' ':' | sed 's/:$//')

# 清理 LD_LIBRARY_PATH，移除 noetic 和 piper_ws
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "noetic\|piper_ws/devel" | tr '\n' ':' | sed 's/:$//')

# 清理 CMAKE_PREFIX_PATH，移除 noetic 和 piper_ws
export CMAKE_PREFIX_PATH=$(echo "$CMAKE_PREFIX_PATH" | tr ':' '\n' | grep -v "noetic\|piper_ws/devel" | tr '\n' ':' | sed 's/:$//')

# 清理 PKG_CONFIG_PATH，移除 noetic 和 piper_ws
export PKG_CONFIG_PATH=$(echo "$PKG_CONFIG_PATH" | tr ':' '\n' | grep -v "noetic\|piper_ws/devel" | tr '\n' ':' | sed 's/:$//')

# 现在加载纯净的 ROS2 Foxy 环境
source /opt/ros/foxy/setup.bash

# RealSense ROS2 工作空间
if [ -d ~/ros2_foxy_ws/install/setup.bash ]; then
    source ~/ros2_foxy_ws/install/setup.bash
fi

echo "✓ ROS2 Foxy 环境已清理并加载"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"
