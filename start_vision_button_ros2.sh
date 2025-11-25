#!/bin/bash
set -euo pipefail

# ================================================================
# ROS2 视觉按钮操作系统一键启动脚本
# 组合 RealSense + YOLO 检测 + TF 发布 + button_actions 执行
# 依赖: ROS2 Foxy, realsense2_camera, conda(button), ultralytics, Piper SDK
# ================================================================

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

USE_RVIZ=false
START_CAMERA=true
START_TF=true
USE_MOVEIT=true
MOTION_STRATEGY="cartesian"
YOLO_ENV="button"
REALSENSE_LAUNCH="ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    pointcloud.enable:=true \
    enable_sync:=true \
    spatial_filter.enable:=true \
    temporal_filter.enable:=true \
    hole_filling_filter.enable:=true \
    decimation_filter.enable:=false"

usage() {
    cat <<'EOF'
用法: ./start_vision_button_ros2.sh [选项]
  --strategy <name>    运动策略 (incremental/cartesian/hybrid)，默认 cartesian
  --skip-camera        不自动启动 RealSense 相机节点
  --skip-tf            不启动 TF 发布器 (自备外部 TF)
  --rviz               打开 RViz (随 MoveIt2 场景或 RViz2)
  --moveit             启用 MoveIt2 粗定位 (需要先启动 MoveIt2 服务)
  --yolo-env <env>     切换运行 YOLO 的 conda 环境名称 (默认 button)
  --help               查看说明
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --strategy)
            MOTION_STRATEGY="$2"; shift 2 ;;
        --skip-camera)
            START_CAMERA=false; shift ;;
        --skip-tf)
            START_TF=false; shift ;;
        --rviz)
            USE_RVIZ=true; shift ;;
        --moveit)
            USE_MOVEIT=true; shift ;;
        --yolo-env)
            YOLO_ENV="$2"; shift 2 ;;
        --help|-h)
            usage; exit 0 ;;
        *)
            echo -e "${RED}未知参数: $1${NC}" >&2
            usage; exit 1 ;;
    esac
done

header() {
    echo -e "${CYAN}====================================================================${NC}"
    echo -e "${CYAN}[ROS2] 视觉按钮操作系统启动${NC}"
    echo -e "项目根目录: ${PROJECT_ROOT}"
    echo -e "运动策略: ${MOTION_STRATEGY}"
    echo -e "相机节点: ${START_CAMERA}"
    echo -e "TF 发布器: ${START_TF}"
    echo -e "使用 MoveIt2: ${USE_MOVEIT}"
    echo -e "YOLO 环境: ${YOLO_ENV}"
    echo -e "RViz: ${USE_RVIZ}"
    echo -e "${CYAN}====================================================================${NC}"
}

header

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo -e "${RED}缺少命令: $1${NC}" >&2
        exit 1
    fi
}

# --- 基础环境检查 ---
require_cmd python3
require_cmd ros2
require_cmd gnome-terminal
require_cmd sed

if [[ -z "${ROS_DISTRO:-}" || "${ROS_DISTRO}" != "foxy" ]]; then
    if [[ -f /opt/ros/foxy/setup.bash ]]; then
        # shellcheck source=/dev/null
        source /opt/ros/foxy/setup.bash
        export ROS_DISTRO=foxy
        echo -e "${GREEN}✓ 已 source /opt/ros/foxy/setup.bash${NC}"
    else
        echo -e "${RED}✗ 未找到 ROS2 Foxy，请先安装 /opt/ros/foxy${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ ROS2 环境: ${ROS_DISTRO}${NC}"
fi

ROS2_WORKSPACE_SETUP="${HOME}/ros2_foxy_ws/install/setup.bash"
if [[ -f "${ROS2_WORKSPACE_SETUP}" ]]; then
    # shellcheck source=/dev/null
    # 临时关闭 set -u 以允许 setup.bash 引用未初始化变量
    set +u
    source "${ROS2_WORKSPACE_SETUP}" 2>/dev/null || true
    set -u
    echo -e "${GREEN}✓ 已 source ~/ros2_foxy_ws/install/setup.bash${NC}"
else
    echo -e "${YELLOW}⚠️  未找到 ~/ros2_foxy_ws/install/setup.bash, 将跳过本地 workspace${NC}"
fi

if ! python3 -c "import rclpy" >/dev/null 2>&1; then
    echo -e "${RED}✗ Python rclpy 不可用，请检查 ROS2 Python 依赖${NC}"
    exit 1
fi

echo -e "${GREEN}✓ rclpy 已就绪${NC}"

if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
    if ! conda env list | grep -q "^${YOLO_ENV} "; then
        echo -e "${RED}✗ 未找到 conda 环境 '${YOLO_ENV}'${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ conda 环境可用: ${YOLO_ENV}${NC}"
else
    echo -e "${YELLOW}⚠️  未检测到 conda，将直接使用系统 Python 执行 YOLO${NC}"
fi

# --- 检查 YOLO 模型 ---
if [[ ! -f "$PROJECT_ROOT/yolo_button.pt" ]]; then
    echo -e "${YELLOW}⚠️  未发现 yolo_button.pt，Ultralytics 将 fallback 默认模型${NC}"
fi

# --- 自动写入 button_actions.py 配置 ---
update_config() {
    local file="$PROJECT_ROOT/button_actions.py"
    if [[ -f "$file" ]]; then
        if grep -q '^MOTION_STRATEGY =' "$file"; then
            sed -i "s/^MOTION_STRATEGY = .*/MOTION_STRATEGY = \"${MOTION_STRATEGY}\"  # ROS2启动脚本自动设置/" "$file"
            echo -e "${GREEN}✓ 已刷新 MOTION_STRATEGY=${MOTION_STRATEGY}${NC}"
        fi
        if grep -q '^USE_MOVEIT =' "$file"; then
            if $USE_MOVEIT; then
                sed -i "s/^USE_MOVEIT = .*/USE_MOVEIT = True  # ROS2启动脚本: 启用MoveIt2粗定位/" "$file"
                echo -e "${GREEN}✓ 已启用 USE_MOVEIT=True (需要MoveIt2服务运行)${NC}"
            else
                sed -i "s/^USE_MOVEIT = .*/USE_MOVEIT = False  # ROS2启动脚本: 纯SDK模式/" "$file"
                echo -e "${GREEN}✓ 已设置 USE_MOVEIT=False (纯SDK模式)${NC}"
            fi
        fi
    else
        echo -e "${YELLOW}⚠️  未找到 button_actions.py，跳过配置${NC}"
    fi
}

# --- MoveIt2 环境检查 ---
check_moveit2() {
    if $USE_MOVEIT; then
        echo -e "${CYAN}检查 MoveIt2 环境...${NC}"
        if ! ros2 pkg list | grep -q moveit_ros_planning; then
            echo -e "${RED}✗ 未找到 MoveIt2，请先安装:${NC}"
            echo -e "  sudo apt install ros-foxy-moveit"
            exit 1
        fi
        echo -e "${GREEN}✓ MoveIt2 已安装${NC}"
        
        # 检查 MoveIt2 服务是否运行
        if ! ros2 node list 2>/dev/null | grep -q move_group; then
            echo -e "${YELLOW}⚠️  MoveIt2 服务未运行，将自动启动...${NC}"
            gnome_launch "moveit2" "cd $PROJECT_ROOT && source /opt/ros/foxy/setup.bash && ros2 launch piper_moveit_config demo.launch.py"
            sleep 5
            echo -e "${GREEN}✓ MoveIt2 服务已启动${NC}"
        else
            echo -e "${GREEN}✓ MoveIt2 服务已运行${NC}"
        fi
    fi
}

update_config

# --- MoveIt2 检查 ---
check_moveit2

# --- 清理历史节点 ---
echo -e "${CYAN}清理历史进程...${NC}"
if pgrep -f vision_button_action_ros2.py >/dev/null; then
    pkill -f vision_button_action_ros2.py || true
fi
if pgrep -f realsense_yolo_button_interactive_ros2_sub.py >/dev/null; then
    pkill -f realsense_yolo_button_interactive_ros2_sub.py || true
fi
if pgrep -f piper_tf_publisher_ros2.py >/dev/null; then
    pkill -f piper_tf_publisher_ros2.py || true
fi

echo -e "${GREEN}✓ 旧节点已清除${NC}"

export MPLCONFIGDIR=/tmp/mpl-cache-ros2
mkdir -p "$MPLCONFIGDIR"

gnome_launch() {
    local title="$1"
    shift
    local cmd="$*"
    gnome-terminal --tab --title="$title" -- bash -c "$cmd; exec bash"
}

STEP=1

if $START_CAMERA; then
    echo -e "${CYAN}[${STEP}/5] 启动 RealSense 相机${NC}"
    gnome_launch "realsense" "cd $PROJECT_ROOT && source /opt/ros/foxy/setup.bash && ${REALSENSE_LAUNCH}"
    sleep 3
    STEP=$((STEP+1))
else
    echo -e "${YELLOW}跳过 RealSense 启动，假定外部已提供 /camera/** 话题${NC}"
fi

if $START_TF; then
    echo -e "${CYAN}[${STEP}/5] 启动 TF 发布器${NC}"
    gnome_launch "piper_tf" "cd $PROJECT_ROOT && source /opt/ros/foxy/setup.bash && python3 piper_tf_publisher_ros2.py"
    sleep 2
    STEP=$((STEP+1))
fi

echo -e "${CYAN}[${STEP}/5] 启动交互式按钮检测器${NC}"
# 注意：必须使用系统 Python 3.8 以匹配 ROS2 Foxy 的 rclpy，不能激活 conda 3.9 环境
gnome_launch "button_detector" "cd $PROJECT_ROOT && source /opt/ros/foxy/setup.bash && source ~/ros2_foxy_ws/install/setup.bash 2>/dev/null && export MPLCONFIGDIR=${MPLCONFIGDIR} && /usr/bin/python3 realsense_yolo_button_interactive_ros2_sub.py"
sleep 2
STEP=$((STEP+1))

if $USE_RVIZ; then
    echo -e "${CYAN}[${STEP}/5] 启动 RViz2${NC}"
    gnome_launch "rviz2" "cd $PROJECT_ROOT && source /opt/ros/foxy/setup.bash && rviz2"
    sleep 2
fi

echo -e "${CYAN}[${STEP}/5] 启动视觉按钮执行节点${NC}"
gnome_launch "vision_button_action" "cd $PROJECT_ROOT && source /opt/ros/foxy/setup.bash && python3 vision_button_action_ros2.py"
sleep 2

cat <<'NOTE'
====================================================================
ROS2 视觉按钮工作流已启动：
  1. RealSense 相机 (可选)
  2. Piper TF 发布器 (可选)
  3. 交互式按钮检测 YOLO
  4. (可选) RViz2 可视化
  5. Vision Button Action 执行节点
按顺序在各终端中观察日志。若需停止，请在对应 tab 内 Ctrl+C，或执行 pkill -f <脚本名>。
====================================================================
NOTE

