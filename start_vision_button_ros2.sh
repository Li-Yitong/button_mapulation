#!/bin/bash

# ========================================
# 视觉按钮操作系统启动脚本 - ROS2 版本
# 整合视觉检测 + 交互式选择 + 自动动作执行
# ========================================

# 获取脚本所在目录（项目根目录）
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================================================"
echo "视觉按钮操作系统启动脚本 - ROS2 Foxy 版本"
echo "========================================================================"
echo "项目路径: $PROJECT_ROOT"
echo "========================================================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 默认参数
USE_MOVEIT=true     # 使用 MoveIt2 进行路径规划（推荐）
USE_SDK_ONLY=false  # 是否仅使用 SDK 直接控制（不使用 MoveIt2）
USE_RVIZ=false

# ========================================
# 参数解析
# ========================================
echo -e "${CYAN}[参数配置]${NC}"

while [[ $# -gt 0 ]]; do
    case $1 in
        --sdk-only)
            USE_SDK_ONLY=true
            USE_MOVEIT=false
            shift
            ;;
        --moveit)
            USE_MOVEIT=true
            USE_SDK_ONLY=false
            shift
            ;;
        --rviz)
            USE_RVIZ=true
            shift
            ;;
        --help|-h)
            echo "使用方法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --moveit         使用 MoveIt2 进行路径规划（默认）"
            echo "  --sdk-only       仅使用 SDK 直接控制（不使用 MoveIt2）"
            echo "  --rviz           启动 RViz2 可视化"
            echo "  --help, -h       显示帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                # 使用 MoveIt2（默认）"
            echo "  $0 --sdk-only     # 仅使用 SDK 控制"
            echo "  $0 --moveit --rviz  # MoveIt2 + RViz2"
            echo ""
            echo "说明:"
            echo "  - ROS2 Foxy 支持 MoveIt2（推荐使用）"
            echo "  - MoveIt2 提供更平滑的路径规划和碰撞检测"
            echo "  - SDK 模式直接控制，速度更快但无碰撞检测"
            exit 0
            ;;
        *)
            echo -e "${RED}未知参数: $1${NC}"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 显示配置
if [ "$USE_MOVEIT" = true ]; then
    echo "  控制模式: MoveIt2 路径规划"
else
    echo "  控制模式: SDK 直接控制"
fi
echo "  ROS2 版本: Foxy"
echo "  RViz2 可视化: $USE_RVIZ"
echo ""

# ========================================
# 环境检查
# ========================================
echo -e "${CYAN}[环境检查]${NC}"

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ 错误: ROS2 环境未配置${NC}"
    echo "  请运行: source /opt/ros/foxy/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "foxy" ]; then
    echo -e "${YELLOW}⚠️  警告: ROS 版本为 $ROS_DISTRO，推荐使用 foxy${NC}"
fi
echo -e "${GREEN}✓ ROS2 环境已配置 ($ROS_DISTRO)${NC}"

# 检查系统 Python
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
if [ "$PYTHON_VERSION" != "3.8" ]; then
    echo -e "${YELLOW}⚠️  Python 版本为 $PYTHON_VERSION，推荐 3.8${NC}"
else
    echo -e "${GREEN}✓ Python 3.8${NC}"
fi

# 检查 piper_sdk
if ! python3 -c "from piper_sdk import C_PiperInterface_V2" 2>/dev/null; then
    echo -e "${RED}✗ 错误: piper_sdk 未安装${NC}"
    exit 1
fi
echo -e "${GREEN}✓ piper_sdk 已安装${NC}"

# 检查 rclpy
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo -e "${RED}✗ 错误: rclpy 未安装${NC}"
    exit 1
fi
echo -e "${GREEN}✓ rclpy 已安装${NC}"

# 检查 RealSense SDK
if ! python3 -c "import pyrealsense2" 2>/dev/null; then
    echo -e "${YELLOW}⚠️  警告: pyrealsense2 未安装${NC}"
    echo "  将使用 ROS2 RealSense 驱动"
fi

# 检查 RealSense ROS2 工作空间
if [ -d "$HOME/ros2_foxy_ws/install" ]; then
    echo -e "${GREEN}✓ RealSense ROS2 工作空间存在${NC}"
else
    echo -e "${RED}✗ 错误: RealSense ROS2 工作空间不存在${NC}"
    echo "  预期路径: ~/ros2_foxy_ws/"
    exit 1
fi

# 检查 YOLO 模型
if [ ! -f "$PROJECT_ROOT/yolo_button.pt" ]; then
    echo -e "${YELLOW}⚠️  警告: yolo_button.pt 模型文件不存在${NC}"
    echo "  将尝试使用 yolo11n.pt 作为替代"
fi

echo ""

# ========================================
# 清理残留进程
# ========================================
echo -e "${CYAN}[清理残留进程]${NC}"

# 关闭已有的 ROS2 节点
echo "  清理 ROS2 Python 节点..."
pkill -f "piper_tf_publisher_ros2.py" 2>/dev/null
pkill -f "realsense_yolo_button_interactive_ros2" 2>/dev/null
pkill -f "vision_button_action_ros2.py" 2>/dev/null

# 关闭 RealSense 相机节点
echo "  清理 RealSense 节点..."
pkill -f "realsense2_camera_node" 2>/dev/null
pkill -f "rs_launch.py" 2>/dev/null

# 清理可能的其他相关进程
echo "  清理其他相关进程..."
pkill -f "button_detector" 2>/dev/null
pkill -f "vision_button_action" 2>/dev/null

# 等待进程完全退出
sleep 2

# 验证清理结果
REMAINING=$(pgrep -f "piper_tf_publisher_ros2\|realsense_yolo_button\|vision_button_action_ros2\|realsense2_camera_node" | wc -l)
if [ $REMAINING -gt 0 ]; then
    echo -e "${YELLOW}  ⚠️  仍有 $REMAINING 个相关进程在运行，强制终止...${NC}"
    pkill -9 -f "piper_tf_publisher_ros2" 2>/dev/null
    pkill -9 -f "realsense_yolo_button" 2>/dev/null
    pkill -9 -f "vision_button_action_ros2" 2>/dev/null
    pkill -9 -f "realsense2_camera_node" 2>/dev/null
    sleep 1
fi

echo -e "${GREEN}✓ ROS2 节点已清理${NC}"

echo ""

# ========================================
# 启动服务
# ========================================

# Source 清理后的 ROS2 环境
source $PROJECT_ROOT/setup_ros2_clean.sh
source $HOME/ros2_foxy_ws/install/setup.bash
source $PROJECT_ROOT/piper_ros/install/setup.bash

# ========================================
# [0/5] 启动 MoveIt2 (如果启用)
# ========================================
if [ "$USE_MOVEIT" = true ]; then
    echo -e "${CYAN}[0/5] 启动 MoveIt2${NC}"
    gnome-terminal --tab --title="moveit2" -- bash -c \
        "source $PROJECT_ROOT/setup_ros2_clean.sh && \
        source $PROJECT_ROOT/piper_ros/install/setup.bash && \
        ros2 launch piper_with_gripper_moveit demo.launch.py; exec bash"
    sleep 3
    echo -e "${GREEN}✓ MoveIt2 已启动${NC}"
    echo ""
fi

# ========================================
# [1/5] 启动 RealSense 相机节点
# ========================================
echo -e "${CYAN}[1/5] 启动 RealSense 相机节点${NC}"
gnome-terminal --tab --title="realsense_camera" -- bash -c \
    "source $PROJECT_ROOT/setup_ros2_clean.sh && \
    source $HOME/ros2_foxy_ws/install/setup.bash && \
    ros2 launch realsense2_camera rs_launch.py \
        depth_module.depth_profile:=640x480x30 \
        rgb_camera.color_profile:=640x480x30 \
        align_depth.enable:=true; exec bash"
sleep 5
echo -e "${GREEN}✓ RealSense 相机已启动${NC}"

echo ""

# ========================================
# [2/5] 启动 Piper TF 发布器
# ========================================
echo -e "${CYAN}[2/5] 启动 Piper TF 发布器${NC}"
gnome-terminal --tab --title="piper_tf_publisher" -- bash -c \
    "source $PROJECT_ROOT/setup_ros2_clean.sh && \
    cd $PROJECT_ROOT && \
    python3 piper_tf_publisher_ros2.py; exec bash"
sleep 2
echo -e "${GREEN}✓ TF 发布器已启动${NC}"

echo ""

# ========================================
# [3/5] 启动交互式按钮检测器
# ========================================
echo -e "${CYAN}[3/5] 启动交互式按钮检测器${NC}"

# 使用 ROS2 话题订阅版本（推荐，避免相机硬件冲突）
echo "  使用 ROS2 话题订阅版本"
gnome-terminal --tab --title="button_detector" -- bash -c \
    "source $PROJECT_ROOT/setup_ros2_clean.sh && \
    source $HOME/ros2_foxy_ws/install/setup.bash && \
    cd $PROJECT_ROOT && \
    python3 realsense_yolo_button_interactive_ros2_sub.py; exec bash"

sleep 3
echo -e "${GREEN}✓ 按钮检测器已启动${NC}"

echo ""

# ========================================
# [4/5] 启动视觉按钮操作执行器
# ========================================
echo -e "${CYAN}[4/5] 启动视觉按钮操作执行器${NC}"

# 根据 USE_MOVEIT 决定是否传递 --moveit 参数
if [ "$USE_MOVEIT" = true ]; then
    echo "  使用 MoveIt2 模式"
    gnome-terminal --tab --title="vision_button_action" -- bash -c \
        "source $PROJECT_ROOT/setup_ros2_clean.sh && \
        source $HOME/ros2_foxy_ws/install/setup.bash && \
        cd $PROJECT_ROOT && \
        python3 vision_button_action_ros2.py --moveit; exec bash"
else
    echo "  使用 SDK 直接控制模式"
    gnome-terminal --tab --title="vision_button_action" -- bash -c \
        "source $PROJECT_ROOT/setup_ros2_clean.sh && \
        source $HOME/ros2_foxy_ws/install/setup.bash && \
        cd $PROJECT_ROOT && \
        python3 vision_button_action_ros2.py; exec bash"
fi

sleep 2
echo -e "${GREEN}✓ 视觉按钮操作执行器已启动${NC}"

echo ""

# ========================================
# [可选] 启动 RViz2
# ========================================
if [ "$USE_RVIZ" = true ]; then
    echo -e "${CYAN}[5/5] 启动 RViz2${NC}"
    gnome-terminal --tab --title="rviz2" -- bash -c \
        "source $PROJECT_ROOT/setup_ros2_clean.sh && \
        source $PROJECT_ROOT/piper_ros/install/setup.bash && \
        rviz2; exec bash"
    sleep 2
    echo -e "${GREEN}✓ RViz2 已启动${NC}"
    echo ""
fi

# ========================================
# 启动完成
# ========================================
echo "========================================================================"
echo -e "${GREEN}✓✓✓ ROS2 节点已启动完成！✓✓✓${NC}"
echo "========================================================================"
echo ""
echo -e "${YELLOW}系统配置:${NC}"
echo "  - ROS2 版本: Foxy"
if [ "$USE_MOVEIT" = true ]; then
    echo "  - 控制模式: MoveIt2 路径规划"
else
    echo "  - 控制模式: SDK 直接控制"
fi
echo "  - RViz2 可视化: $USE_RVIZ"
echo ""
echo -e "${YELLOW}终端窗口说明:${NC}"
if [ "$USE_MOVEIT" = true ]; then
echo "  0. moveit2            - MoveIt2 运动规划"
fi
echo "  1. realsense_camera   - RealSense 相机驱动 (ROS2)"
echo "  2. piper_tf_publisher - 机械臂 TF 发布"
echo "  3. button_detector    - 交互式按钮检测 (相机+YOLO)"
echo "  4. vision_button_action - 视觉按钮操作执行器"
if [ "$USE_RVIZ" = true ]; then
echo "  5. rviz2              - 可视化工具"
fi
echo ""
echo -e "${YELLOW}使用说明:${NC}"
echo "  1. 在 'button_detector' 窗口中:"
echo "     - 系统会自动检测按钮并显示蓝色边框"
echo "     - ${CYAN}用鼠标点击${NC}你想要操作的按钮"
echo "     - 选中的按钮会变为${GREEN}绿色边框${NC}"
echo "     - 按 ${CYAN}ENTER${NC} 确认选择"
echo "     - 按 ${CYAN}ESC${NC} 取消选择"
echo "     - 按 ${CYAN}Q${NC} 退出程序"
echo ""
echo "  2. 确认后:"
echo "     - 按钮信息会发布到 ROS2 话题:"
echo "       - /object_point (按钮3D位置)"
echo "       - /button_type (按钮类型)"
echo ""
echo -e "${YELLOW}验证命令:${NC}"
echo "  # 查看话题列表"
echo "  ros2 topic list"
echo ""
echo "  # 查看按钮位置"
echo "  ros2 topic echo /object_point"
echo ""
echo "  # 查看按钮类型"
echo "  ros2 topic echo /button_type"
echo ""
echo "  # 查看 TF 树"
echo "  ros2 run tf2_tools view_frames"
echo ""
echo -e "${CYAN}提示: 按 Ctrl+C 可在各个终端中停止对应节点${NC}"
echo "========================================================================"
