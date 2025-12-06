#!/bin/bash
################################################################################
# 手眼标定一键启动脚本 (Eye-in-Hand + 棋盘格)
# Ubuntu 20.04 + ROS2 Foxy
################################################################################

set -e  # 遇到错误立即退出

echo "======================================================================"
echo "  手眼标定系统启动 - Eye-in-Hand + 棋盘格"
echo "======================================================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查工作空间
if [ ! -d "$HOME/handeye_ws" ]; then
    echo -e "${RED}❌ 错误: handeye_ws 工作空间不存在${NC}"
    echo "   请先克隆并编译 easy_handeye2："
    echo "   mkdir -p ~/handeye_ws/src && cd ~/handeye_ws/src"
    echo "   git clone https://github.com/marcoesposito1988/easy_handeye2.git"
    echo "   cd ~/handeye_ws && colcon build"
    exit 1
fi

# Source ROS2 环境
echo -e "${YELLOW}► 设置 ROS2 环境...${NC}"
source /opt/ros/foxy/setup.bash
source "$HOME/handeye_ws/install/setup.bash"
echo -e "${GREEN}✓ ROS2 环境已加载${NC}"
echo ""

# 检查 RealSense 相机是否运行
echo -e "${YELLOW}► 检查 RealSense 相机...${NC}"
if ! ros2 topic list | grep -q "/camera/color/image_raw"; then
    echo -e "${RED}⚠ 相机未运行！${NC}"
    echo "   请在另一个终端运行："
    echo "   ${GREEN}ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true${NC}"
    echo ""
    read -p "相机启动后，按 Enter 继续..."
fi
echo -e "${GREEN}✓ 相机已运行${NC}"
echo ""

# 检查 Piper TF 发布器是否运行
echo -e "${YELLOW}► 检查 Piper TF 发布器...${NC}"
if ! ros2 topic list | grep -q "/tf"; then
    echo -e "${RED}⚠ Piper TF 未运行！${NC}"
    echo "   请在另一个终端运行："
    echo "   ${GREEN}cd /home/unitree/manipulation/button/vba && python3 piper_tf_publisher_ros2.py${NC}"
    echo ""
    read -p "TF 发布器启动后，按 Enter 继续..."
fi
echo -e "${GREEN}✓ Piper TF 已运行${NC}"
echo ""

# 启动棋盘格检测节点（后台）
echo -e "${YELLOW}► 启动棋盘格检测节点...${NC}"
cd /home/unitree/manipulation/button/vba

# 启动棋盘格检测（5x7棋盘格，3cm方格）
python3 chessboard_pose_publisher.py --ros-args \
    -p image_topic:=/camera/camera/color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info \
    -p pattern_rows:=5 \
    -p pattern_cols:=7 \
    -p square_size:=0.03 \
    -p camera_frame:=camera_color_optical_frame \
    -p board_frame:=checkerboard &

CHESSBOARD_PID=$!
echo -e "${GREEN}✓ 棋盘格检测节点已启动 (PID: $CHESSBOARD_PID)${NC}"
sleep 2
echo ""

# 等待棋盘格检测节点就绪
echo -e "${YELLOW}► 等待棋盘格检测...${NC}"
for i in {1..10}; do
    if ros2 topic list | grep -q "/chessboard_pose"; then
        echo -e "${GREEN}✓ 棋盘格检测节点就绪${NC}"
        break
    fi
    if [ $i -eq 10 ]; then
        echo -e "${RED}⚠ 棋盘格检测节点未响应${NC}"
    fi
    sleep 1
done
echo ""

# 启动手眼标定 GUI
echo "======================================================================"
echo -e "${GREEN}  ► 正在启动手眼标定 GUI...${NC}"
echo "======================================================================"
echo ""
echo "📋 标定步骤："
echo "   1. 移动机械臂到不同姿态，确保棋盘格在视野内"
echo "   2. 每个姿态点击 GUI 中的 'Take Sample' 按钮"
echo "   3. 采集至少 12 个样本（推荐 15-20 个）"
echo "   4. 点击 'Compute Calibration'"
echo "   5. 检查重投影误差，满意后点击 'Save Calibration'"
echo ""
echo "💡 注意："
echo "   - 姿态要尽量多样化（不同角度、距离、旋转）"
echo "   - 避免遮挡棋盘格"
echo "   - 重投影误差应小于 3-5 像素"
echo ""

# 启动标定 GUI
ros2 launch /home/unitree/manipulation/button/vba/launch/handeye_calibrate.launch.py

# 清理（当 GUI 关闭时）
echo ""
echo -e "${YELLOW}► 清理进程...${NC}"
kill $CHESSBOARD_PID 2>/dev/null || true
echo -e "${GREEN}✓ 标定完成${NC}"
echo ""
echo "======================================================================"
echo "  下一步："
echo "======================================================================"
echo "1. 读取标定结果并更新代码："
echo "   ${GREEN}cd /home/unitree/manipulation/button/vba${NC}"
echo "   ${GREEN}python3 read_calibration_result.py${NC}"
echo ""
echo "2. 复制输出的代码到 piper_arm.py 中"
echo ""
echo "3. 验证标定结果："
echo "   ${GREEN}ros2 launch /home/unitree/manipulation/button/vba/launch/handeye_publish.launch.py${NC}"
echo "   ${GREEN}python3 verify_handeye_calibration.py${NC}"
echo "======================================================================"
