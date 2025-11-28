#!/bin/bash

# ========================================
# 视觉按钮操作系统 - 功能测试脚本
# ========================================

echo "========================================================================"
echo "视觉按钮操作系统 - 功能测试"
echo "========================================================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

# ========================================
# 测试函数
# ========================================
test_item() {
    local name=$1
    local command=$2
    
    echo -n "测试: $name ... "
    
    if eval "$command" &>/dev/null; then
        echo -e "${GREEN}✓ PASS${NC}"
        ((PASS++))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC}"
        ((FAIL++))
        return 1
    fi
}

# ========================================
# 1. 环境检查
# ========================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "阶段 1/5: 环境检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

test_item "ROS 环境" "[ ! -z \"\$ROS_MASTER_URI\" ]"
test_item "Python3" "command -v python3"
test_item "conda" "command -v conda"
test_item "conda 环境 'button'" "conda env list | grep -q '^button '"
test_item "MoveIt 库" "python3 -c 'import moveit_commander'"

echo ""

# ========================================
# 2. 文件完整性检查
# ========================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "阶段 2/5: 文件完整性检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

test_item "realsense_yolo_button_interactive.py" "[ -f realsense_yolo_button_interactive.py ]"
test_item "vision_button_action.py" "[ -f vision_button_action.py ]"
test_item "button_actions.py" "[ -f button_actions.py ]"
test_item "piper_tf_publisher.py" "[ -f piper_tf_publisher.py ]"
test_item "start_vision_button.sh" "[ -f start_vision_button.sh ]"
test_item "yolo_button.pt 或 yolo11n.pt" "[ -f yolo_button.pt ] || [ -f yolo11n.pt ]"

echo ""

# ========================================
# 3. Python 依赖检查
# ========================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "阶段 3/5: Python 依赖检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 激活 conda 环境并测试
eval "$(conda shell.bash hook)"
conda activate button 2>/dev/null

test_item "numpy" "python3 -c 'import numpy'"
test_item "cv2" "python3 -c 'import cv2'"
test_item "pyrealsense2" "python3 -c 'import pyrealsense2'"
test_item "ultralytics" "python3 -c 'from ultralytics import YOLO'"
test_item "rospy" "python3 -c 'import rospy'"

echo ""

# ========================================
# 4. ROS 系统检查
# ========================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "阶段 4/5: ROS 系统检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检查 roscore 是否运行
if pgrep -x "roscore" > /dev/null; then
    echo -e "测试: roscore 运行状态 ... ${GREEN}✓ PASS${NC} (运行中)"
    ((PASS++))
    
    # 如果 roscore 运行，检查 ROS 话题
    sleep 1
    test_item "rostopic 命令" "rostopic list &>/dev/null"
    
else
    echo -e "测试: roscore 运行状态 ... ${YELLOW}⊘ SKIP${NC} (未运行，这是正常的)"
    echo "  提示: 启动系统后会自动启动 roscore"
fi

echo ""

# ========================================
# 5. 相机硬件检查
# ========================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "阶段 5/5: 相机硬件检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if command -v rs-enumerate-devices &>/dev/null; then
    echo -n "测试: RealSense 设备连接 ... "
    
    if rs-enumerate-devices 2>/dev/null | grep -q "Intel RealSense"; then
        echo -e "${GREEN}✓ PASS${NC}"
        ((PASS++))
        
        # 检查设备是否锁定
        echo -n "测试: 相机未被锁定 ... "
        if rs-enumerate-devices 2>/dev/null | grep -q "Camera Locked.*YES"; then
            echo -e "${RED}✗ FAIL${NC}"
            echo "  提示: 运行 './reset_camera.sh' 重置相机"
            ((FAIL++))
        else
            echo -e "${GREEN}✓ PASS${NC}"
            ((PASS++))
        fi
    else
        echo -e "${RED}✗ FAIL${NC}"
        echo "  提示: 请检查 RealSense 相机连接"
        ((FAIL++))
    fi
else
    echo -e "测试: rs-enumerate-devices 工具 ... ${YELLOW}⊘ SKIP${NC}"
    echo "  提示: 安装 librealsense2-utils: sudo apt install librealsense2-utils"
fi

echo ""

# ========================================
# 测试总结
# ========================================
echo "========================================================================"
echo "测试总结"
echo "========================================================================"
echo -e "通过: ${GREEN}$PASS${NC}"
echo -e "失败: ${RED}$FAIL${NC}"
echo ""

TOTAL=$((PASS + FAIL))
if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✓✓✓ 所有测试通过！系统已准备就绪。${NC}"
    echo ""
    echo "下一步:"
    echo "  1. 启动系统: ./start_vision_button.sh"
    echo "  2. 在检测器窗口中点击选择按钮"
    echo "  3. 按 ENTER 确认并执行操作"
    exit 0
else
    echo -e "${RED}✗✗✗ 有 $FAIL 项测试失败，请修复后再启动系统。${NC}"
    echo ""
    echo "常见问题解决:"
    echo "  1. ROS 环境: source /opt/ros/noetic/setup.bash"
    echo "  2. conda 环境: conda create -n button python=3.8"
    echo "  3. MoveIt: sudo apt install ros-noetic-moveit"
    echo "  4. 相机驱动: sudo apt install librealsense2-utils"
    exit 1
fi
