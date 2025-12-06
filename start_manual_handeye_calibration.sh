#!/bin/bash
################################################################################
# 手眼标定简化版 - 手动采集样本
# 不依赖 easy_handeye2 GUI，直接用终端命令
################################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}► 清理进程...${NC}"
    
    # 强制终止棋盘格检测进程
    if [ ! -z "$CHESSBOARD_PID" ]; then
        kill -9 $CHESSBOARD_PID 2>/dev/null || true
    fi
    
    # 强制终止相机TF发布器
    if [ ! -z "$CAMERA_TF_PID" ]; then
        kill -9 $CAMERA_TF_PID 2>/dev/null || true
    fi
    
    # 清理所有相关进程
    pkill -9 -f "chessboard_pose_publisher" 2>/dev/null || true
    pkill -9 -f "manual_handeye_calibration" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher.*camera_color_optical_frame" 2>/dev/null || true
    
    echo -e "${GREEN}✓ 清理完成${NC}"
    exit 0
}

# 注册清理函数
trap cleanup SIGINT SIGTERM EXIT

echo "======================================================================"
echo "  手眼标定系统 - 手动标定模式"
echo "======================================================================"
echo ""
echo "💡 提示: 按 Ctrl+C 可随时退出并自动清理"
echo ""

# Source ROS2
echo -e "${YELLOW}► 设置 ROS2 环境...${NC}"
source /opt/ros/foxy/setup.bash
echo -e "${GREEN}✓ ROS2 环境已加载${NC}"
echo ""

# 检查相机
echo -e "${YELLOW}► 检查 RealSense 相机...${NC}"
if ! ros2 topic list | grep -q "/camera/camera/color/image_raw"; then
    echo -e "${RED}❌ 相机未运行！${NC}"
    echo "   请在另一个终端运行："
    echo "   ${GREEN}ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 相机已运行${NC}"
echo ""

# 检查 Piper TF
echo -e "${YELLOW}► 检查 Piper TF 发布器...${NC}"
if ! ros2 topic list | grep -q "/tf"; then
    echo -e "${RED}❌ Piper TF 未运行！${NC}"
    echo "   请在另一个终端运行："
    echo "   ${GREEN}cd /home/unitree/manipulation/button/vba && python3 piper_tf_publisher_ros2.py --ros-args -p publish_camera_tf:=false${NC}"
    exit 1
fi

# 检查Piper TF发布器是否禁用了相机TF
if timeout 1 ros2 topic echo /tf 2>/dev/null | grep -q "child_frame_id: camera$"; then
    echo -e "${RED}❌ 警告：Piper TF发布器正在发布旧的相机TF！${NC}"
    echo "   请重启TF发布器并添加参数："
    echo "   ${GREEN}python3 piper_tf_publisher_ros2.py --ros-args -p publish_camera_tf:=false${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Piper TF 已运行（手眼标定模式）${NC}"
echo ""

# 发布初始相机TF（标定前的估计值）
echo -e "${YELLOW}► 发布初始相机TF (link6 → camera_color_optical_frame)...${NC}"
# eye-in-hand: 相机固定在末端link6上
# 初始估计：相机在link6前方约5cm，稍微向下偏移
ros2 run tf2_ros static_transform_publisher 0.05 0 0.03 0 0 0 link6 camera_color_optical_frame &
CAMERA_TF_PID=$!
echo -e "${GREEN}✓ 初始相机TF已发布 (PID: $CAMERA_TF_PID)${NC}"
sleep 1
echo ""

# 启动棋盘格检测（带可视化）
echo -e "${YELLOW}► 启动棋盘格检测节点（带图像显示）...${NC}"
cd /home/unitree/manipulation/button/vba

python3 chessboard_pose_publisher.py --ros-args \
    -p image_topic:=/camera/camera/color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info \
    -p pattern_rows:=8 \
    -p pattern_cols:=11 \
    -p square_size:=0.015 \
    -p camera_frame:=camera_color_optical_frame \
    -p board_frame:=checkerboard \
    -p show_image:=true &

CHESSBOARD_PID=$!
echo -e "${GREEN}✓ 棋盘格检测节点已启动 (PID: $CHESSBOARD_PID)${NC}"
echo ""

# 等待TF树完全建立
echo -e "${YELLOW}► 等待TF树稳定...${NC}"
sleep 3

# 验证完整TF链
echo -e "${YELLOW}► 验证TF链: base_link → link6 → camera_color_optical_frame${NC}"
if ! timeout 3 ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame 2>&1 | grep -q "Translation"; then
    echo -e "${RED}❌ TF链验证失败！${NC}"
    exit 1
fi
echo -e "${GREEN}✓ TF链验证成功${NC}"
echo ""

# 启动手动标定工具
echo "======================================================================"
echo -e "${GREEN}  ► 正在启动手动标定工具...${NC}"
echo "======================================================================"
echo ""
echo "📋 标定流程："
echo "   1. 在弹出的图像窗口中查看棋盘格检测情况"
echo "   2. 手动移动机械臂到不同姿态（确保棋盘格被检测到）"
echo "   3. 每个姿态在终端输入 's' 并按 Enter 采集样本"
echo "   4. 采集 12-20 个样本后，输入 'c' 并按 Enter 计算标定"
echo "   5. 输入 'q' 并按 Enter 退出"
echo ""
echo "💡 提示："
echo "   - 姿态要多样化（旋转机械臂末端，而不是平移）"
echo "   - 确保每个姿态都能看到完整的棋盘格"
echo "   - 图像窗口显示 'Chessboard Detected!' 表示检测成功"
echo ""
echo "======================================================================"
echo ""

# 等待棋盘格检测稳定
sleep 2

# 启动手动标定工具
python3 manual_handeye_calibration.py --ros-args \
    -p robot_base_frame:=base_link \
    -p robot_effector_frame:=link6 \
    -p tracking_base_frame:=camera_color_optical_frame \
    -p tracking_marker_frame:=checkerboard \
    -p calibration_name:=piper_realsense_handeye

# 正常退出时的清理（cleanup函数会自动调用）
echo ""
echo "======================================================================"
echo "  下一步："
echo "======================================================================"
echo "1. 读取标定结果并更新代码："
echo "   ${GREEN}python3 read_calibration_result.py${NC}"
echo ""
echo "2. 复制输出的代码到 piper_arm.py 中"
echo ""
echo "3. 验证标定结果："
echo "   ${GREEN}ros2 launch /home/unitree/manipulation/button/vba/launch/handeye_publish.launch.py${NC}"
echo "   ${GREEN}python3 verify_handeye_calibration.py${NC}"
echo "======================================================================"
