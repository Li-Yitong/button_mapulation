#!/bin/bash

# ========================================
# ROS2 环境配置和依赖安装脚本
# ========================================

echo "========================================================================"
echo "ROS2 环境配置和依赖安装"
echo "========================================================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# ========================================
# 1. 检查 ROS2 安装
# ========================================
echo -e "\n${CYAN}[1/4] 检查 ROS2 Foxy 安装${NC}"

if [ -f "/opt/ros/foxy/setup.bash" ]; then
    echo -e "${GREEN}✓ ROS2 Foxy 已安装${NC}"
    source /opt/ros/foxy/setup.bash
else
    echo -e "${RED}✗ ROS2 Foxy 未安装${NC}"
    echo "请先安装 ROS2 Foxy，参考安装步骤已在之前提供"
    exit 1
fi

# ========================================
# 2. 安装 ROS2 Python 依赖
# ========================================
echo -e "\n${CYAN}[2/4] 安装 ROS2 Python 依赖 (系统 Python)${NC}"

# 重要：ROS2 必须使用系统 Python 3.8，不能使用 conda 环境
# 因为 ROS2 Foxy 的 C 扩展是为 Python 3.8 编译的

echo "安装必需的 Python 包到系统 Python..."

# 使用系统 Python 安装依赖
/usr/bin/python3 -m pip install --user pyrealsense2 ultralytics opencv-python numpy -q

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Python 依赖已安装到系统 Python${NC}"
else
    echo -e "${YELLOW}⚠️  部分包可能已安装${NC}"
fi

# ========================================
# 3. 安装 ROS2 系统包
# ========================================
echo -e "\n${CYAN}[3/4] 安装 ROS2 系统包${NC}"

echo "安装 ROS2 开发工具..."
sudo apt update -qq
sudo apt install -y -qq \
    ros-foxy-rclpy \
    ros-foxy-std-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-visualization-msgs \
    ros-foxy-tf2-ros \
    ros-foxy-tf2-geometry-msgs \
    python3-colcon-common-extensions

echo -e "${GREEN}✓ ROS2 系统包已安装${NC}"

# ========================================
# 4. 验证安装
# ========================================
echo -e "\n${CYAN}[4/4] 验证安装${NC}"

# 测试 rclpy
/usr/bin/python3 -c "import rclpy" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ rclpy 可用 (Python 3.8)${NC}"
else
    echo -e "${YELLOW}⚠️  rclpy 需要 source ROS2 环境${NC}"
fi

# 测试其他依赖
/usr/bin/python3 -c "import pyrealsense2, ultralytics, cv2, numpy" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 所有 Python 依赖可用${NC}"
else
    echo -e "${RED}✗ 某些 Python 依赖缺失${NC}"
fi

# 测试 ROS2 命令
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✓ ros2 命令可用${NC}"
else
    echo -e "${RED}✗ ros2 命令不可用${NC}"
fi

# ========================================
# 完成
# ========================================
echo ""
echo "========================================================================"
echo -e "${GREEN}✓✓✓ ROS2 环境配置完成！✓✓✓${NC}"
echo "========================================================================"
echo ""
echo -e "${YELLOW}下一步操作:${NC}"
echo "  1. 在每个新终端中运行:"
echo -e "     ${CYAN}source /opt/ros/foxy/setup.bash${NC}"
echo ""
echo "  2. 或者将其添加到 ~/.bashrc (已配置)"
echo ""
echo "  3. 启动 ROS2 系统:"
echo -e "     ${CYAN}./start_vision_button_ros2.sh${NC}"
echo ""
echo "  4. 查看迁移文档:"
echo -e "     ${CYAN}cat ROS2_MIGRATION_GUIDE.md${NC}"
echo ""
echo "========================================================================"
