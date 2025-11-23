#!/bin/bash

# ========================================
# Piper ROS2 编译脚本
# 清除 conda 环境，使用纯净的 ROS2 环境
# ========================================

echo "========================================"
echo "Piper ROS2 编译脚本"
echo "========================================"

# 清除 conda 环境变量
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset CONDA_PROMPT_MODIFIER
unset CONDA_SHLVL
unset CONDA_PYTHON_EXE
unset CONDA_EXE

# 清理 PATH，移除 conda 路径
export PATH=$(echo $PATH | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')

# Source ROS2 环境
source /opt/ros/foxy/setup.bash

# 确认使用系统 Python
echo "使用的 Python: $(which python3)"
python3 --version

cd /home/robot/button/V4.0/project2/piper_ros

# 清理之前的编译
echo ""
echo "清理之前的编译..."
rm -rf build/ install/ log/

# 开始编译
echo ""
echo "开始编译 piper_ros..."
colcon build --symlink-install

# 检查编译结果
if [ $? -eq 0 ]; then
    echo ""
    echo "========================================"
    echo "✓ 编译成功！"
    echo "========================================"
    echo ""
    echo "下一步："
    echo "source install/setup.bash"
else
    echo ""
    echo "========================================"
    echo "✗ 编译失败"
    echo "========================================"
    exit 1
fi
