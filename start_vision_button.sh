#!/bin/bash

# ========================================
# 视觉按钮操作系统启动脚本
# 整合视觉检测 + 交互式选择 + 自动动作执行
# ========================================

echo "========================================================================"
echo "视觉按钮操作系统启动脚本"
echo "========================================================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 默认参数
USE_MOVEIT=true
STRATEGY="cartesian"  # 使用笛卡尔路径规划（最丝滑）
USE_RVIZ=false

# ========================================
# 参数解析
# ========================================
echo -e "${CYAN}[参数配置]${NC}"

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-moveit)
            USE_MOVEIT=false
            shift
            ;;
        --strategy)
            STRATEGY="$2"
            shift 2
            ;;
        --rviz)
            USE_RVIZ=true
            shift
            ;;
        --help|-h)
            echo "使用方法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --no-moveit      不使用 MoveIt (使用 SDK 直接控制)"
            echo "  --strategy TYPE  运动策略: incremental/cartesian/hybrid (默认: cartesian)"
            echo "  --rviz           启动 RViz 可视化"
            echo "  --help, -h       显示帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                          # 使用默认配置（MoveIt + cartesian）"
            echo "  $0 --strategy incremental   # 使用增量运动策略"
            echo "  $0 --rviz                   # 启动 RViz"
            echo "  $0 --no-moveit              # 仅使用 SDK 控制"
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
echo "  MoveIt 模式: $USE_MOVEIT"
echo "  运动策略: $STRATEGY"
echo "  RViz 可视化: $USE_RVIZ"
echo ""

# ========================================
# 环境检查
# ========================================
echo -e "${CYAN}[环境检查]${NC}"

# 检查 ROS 环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo -e "${RED}✗ 错误: ROS 环境未配置${NC}"
    echo "  请运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi
echo -e "${GREEN}✓ ROS 环境已配置${NC}"

# 检查 MoveIt（如果需要）
if [ "$USE_MOVEIT" = true ]; then
    if ! python3 -c "import moveit_commander" 2>/dev/null; then
        echo -e "${RED}✗ 错误: MoveIt 未安装${NC}"
        echo "  请运行: sudo apt install ros-noetic-moveit"
        exit 1
    fi
    echo -e "${GREEN}✓ MoveIt 已安装${NC}"
fi

# 检查 conda 环境
if ! command -v conda &> /dev/null; then
    echo -e "${RED}✗ 错误: conda 未安装${NC}"
    exit 1
fi
echo -e "${GREEN}✓ conda 已安装${NC}"

# 检查 button conda 环境
if ! conda env list | grep -q "^button "; then
    echo -e "${RED}✗ 错误: conda 环境 'button' 不存在${NC}"
    exit 1
fi
echo -e "${GREEN}✓ conda 环境 'button' 存在${NC}"

# 检查 yolo_button.pt 模型
if [ ! -f "yolo_button.pt" ]; then
    echo -e "${YELLOW}⚠️  警告: yolo_button.pt 模型文件不存在${NC}"
    echo "  将尝试使用 yolo11n.pt 作为替代"
fi

echo ""

# ========================================
# 更新 button_actions.py 的配置
# ========================================
echo -e "${CYAN}[配置 button_actions.py]${NC}"

# 更新运动策略
if grep -q "^MOTION_STRATEGY = " button_actions.py 2>/dev/null; then
    sed -i "s/^MOTION_STRATEGY = .*/MOTION_STRATEGY = \"$STRATEGY\"  # 启动脚本自动设置/" button_actions.py
    echo -e "${GREEN}✓ 已设置运动策略: $STRATEGY${NC}"
else
    echo -e "${YELLOW}⚠️  未找到 MOTION_STRATEGY 配置${NC}"
fi

# 更新 MoveIt 模式
if grep -q "^USE_MOVEIT = " button_actions.py 2>/dev/null; then
    if [ "$USE_MOVEIT" = true ]; then
        sed -i "s/^USE_MOVEIT = .*/USE_MOVEIT = True  # 启动脚本自动设置/" button_actions.py
    else
        sed -i "s/^USE_MOVEIT = .*/USE_MOVEIT = False  # 启动脚本自动设置/" button_actions.py
    fi
    echo -e "${GREEN}✓ 已设置 MoveIt 模式: $USE_MOVEIT${NC}"
fi

echo ""

# ========================================
# 清理残留进程
# ========================================
echo -e "${CYAN}[清理残留进程]${NC}"

# 清理 ROS 节点
if command -v rosnode &>/dev/null; then
    echo "y" | rosnode cleanup &>/dev/null
    sleep 1
    echo -e "${GREEN}✓ ROS 节点已清理${NC}"
fi

# 关闭已有的 realsense 进程
REALSENSE_PIDS=$(ps aux | grep -E "realsense.*\.py" | grep -v grep | awk '{print $2}')
if [ ! -z "$REALSENSE_PIDS" ]; then
    echo "  关闭 realsense 进程: $REALSENSE_PIDS"
    echo "$REALSENSE_PIDS" | xargs kill -9 2>/dev/null
    sleep 1
fi
echo -e "${GREEN}✓ 相机进程已清理${NC}"

# 设置 matplotlib 配置目录
export MPLCONFIGDIR=/tmp/matplotlib-cache
mkdir -p $MPLCONFIGDIR

echo ""

# ========================================
# 启动 ROS 核心服务
# ========================================
echo -e "${CYAN}[1/6] 启动 roscore${NC}"
if ! pgrep -x "roscore" > /dev/null; then
    gnome-terminal --tab --title="roscore" -- bash -c "roscore; exec bash"
    sleep 3
    echo -e "${GREEN}✓ roscore 已启动${NC}"
else
    echo -e "${GREEN}✓ roscore 已在运行${NC}"
fi

echo ""

# ========================================
# 启动机械臂控制
# ========================================
echo -e "${CYAN}[2/6] 启动 piper_control.launch${NC}"
gnome-terminal --tab --title="piper_control" -- bash -c \
    "cd /home/robot/button/V4.0/project2 && \
    roslaunch launch/piper_control.launch; exec bash"
sleep 3
echo -e "${GREEN}✓ piper_control 已启动${NC}"

echo ""

# ========================================
# 启动 MoveIt (可选)
# ========================================
if [ "$USE_MOVEIT" = true ]; then
    echo -e "${CYAN}[3/6] 启动 MoveIt move_group${NC}"
    
    if [ -d "/home/robot/button/V4.0/project2/piper_ros/devel" ]; then
        RVIZ_ARG="false"
        if [ "$USE_RVIZ" = true ]; then
            RVIZ_ARG="true"
        fi
        
        gnome-terminal --tab --title="moveit_group" -- bash -c \
            "source /home/robot/button/V4.0/project2/piper_ros/devel/setup.bash && \
            roslaunch piper_with_gripper_moveit demo.launch use_rviz:=$RVIZ_ARG; exec bash"
        
        echo -e "${YELLOW}  等待 MoveIt 完全启动（10秒）...${NC}"
        sleep 10
        echo -e "${GREEN}✓ MoveIt move_group 已启动${NC}"
    else
        echo -e "${RED}✗ 错误: MoveIt 工作空间不存在${NC}"
        exit 1
    fi
else
    echo -e "${CYAN}[3/6] 跳过 MoveIt (SDK 模式)${NC}"
fi

echo ""

# ========================================
# 启动交互式按钮检测器
# ========================================
echo -e "${CYAN}[4/6] 启动交互式按钮检测器${NC}"
gnome-terminal --tab --title="button_detector" -- bash -c \
    "export MPLCONFIGDIR=/tmp/matplotlib-cache && \
    eval \"\$(conda shell.bash hook)\" && \
    conda activate button && \
    cd /home/robot/button/V4.0/project2 && \
    python3 realsense_yolo_button_interactive.py; exec bash"
sleep 3
echo -e "${GREEN}✓ 按钮检测器已启动${NC}"

echo ""

# ========================================
# 启动 TF 发布器
# ========================================
echo -e "${CYAN}[5/6] 启动 piper_tf_publisher${NC}"
gnome-terminal --tab --title="tf_publisher" -- bash -c \
    "export MPLCONFIGDIR=/tmp/matplotlib-cache && \
    eval \"\$(conda shell.bash hook)\" && \
    conda activate button && \
    cd /home/robot/button/V4.0/project2 && \
    python3 piper_tf_publisher.py; exec bash"
sleep 2
echo -e "${GREEN}✓ TF 发布器已启动${NC}"

echo ""

# ========================================
# 启动视觉按钮操作执行器
# ========================================
echo -e "${CYAN}[6/6] 启动视觉按钮操作执行器${NC}"

if [ "$USE_MOVEIT" = true ]; then
    # MoveIt 模式（使用系统 Python）
    gnome-terminal --tab --title="vision_button_action" -- bash -c \
        "source /home/robot/button/V4.0/project2/piper_ros/devel/setup.bash && \
        cd /home/robot/button/V4.0/project2 && \
        python3 vision_button_action.py; exec bash"
else
    # SDK 模式（使用 conda）
    gnome-terminal --tab --title="vision_button_action" -- bash -c \
        "export MPLCONFIGDIR=/tmp/matplotlib-cache && \
        eval \"\$(conda shell.bash hook)\" && \
        conda activate button && \
        cd /home/robot/button/V4.0/project2 && \
        python3 vision_button_action.py; exec bash"
fi

sleep 2
echo -e "${GREEN}✓ 视觉按钮操作执行器已启动${NC}"

echo ""

# ========================================
# 启动完成
# ========================================
echo "========================================================================"
echo -e "${GREEN}✓✓✓ 所有节点已启动完成！✓✓✓${NC}"
echo "========================================================================"
echo ""
echo -e "${YELLOW}系统配置:${NC}"
echo "  - MoveIt 模式: $USE_MOVEIT"
echo "  - 运动策略: $STRATEGY"
echo "  - RViz 可视化: $USE_RVIZ"
echo ""
echo -e "${YELLOW}终端窗口说明:${NC}"
echo "  1. roscore            - ROS 主节点"
echo "  2. piper_control      - 机械臂控制"
if [ "$USE_MOVEIT" = true ]; then
echo "  3. moveit_group       - MoveIt 轨迹规划"
else
echo "  3. (跳过 MoveIt)"
fi
echo "  4. button_detector    - 交互式按钮检测 (相机+YOLO)"
echo "  5. tf_publisher       - TF 坐标变换"
echo "  6. vision_button_action - 视觉按钮操作执行器"
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
echo "     - 系统会自动识别按钮类型 (toggle/plugin/push/knob)"
echo "     - 机械臂会自动执行对应的操作"
echo "     - 操作完成后可以继续选择下一个按钮"
echo ""
echo -e "${YELLOW}支持的按钮类型:${NC}"
echo "  - toggle: 拨动开关"
echo "  - plugin: 插拔连接器"
echo "  - push:   按压按钮"
echo "  - knob:   旋转旋钮"
echo ""
echo -e "${CYAN}提示: 按 Ctrl+C 可在各个终端中停止对应节点${NC}"
echo "========================================================================"
