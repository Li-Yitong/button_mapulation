#!/bin/bash
# MoveIt2 快速启动脚本

cd /home/robot/button/V4.0/project2

echo "========================================="
echo "  MoveIt2 for Piper Arm - Quick Start"
echo "========================================="
echo ""

# 加载环境
echo "1. 加载 ROS2 环境..."
source ./setup_ros2_clean.sh
source piper_ros/install/setup.bash

echo ""
echo "2. 启动 MoveIt2..."
echo ""
echo "选项:"
echo "  --no-rviz    : 不启动 RViz2（节省资源）"
echo "  --background : 后台运行"
echo ""

# 解析参数
NO_RVIZ=false
BACKGROUND=false

for arg in "$@"; do
    case $arg in
        --no-rviz)
            NO_RVIZ=true
            ;;
        --background)
            BACKGROUND=true
            ;;
    esac
done

# 构建启动命令
LAUNCH_CMD="ros2 launch piper_with_gripper_moveit demo_foxy.launch.py"

# 启动
if [ "$BACKGROUND" = true ]; then
    echo "在后台启动 MoveIt2..."
    $LAUNCH_CMD > /tmp/moveit2.log 2>&1 &
    MOVEIT_PID=$!
    echo "MoveIt2 PID: $MOVEIT_PID"
    echo "日志文件: /tmp/moveit2.log"
    echo ""
    echo "查看日志: tail -f /tmp/moveit2.log"
    echo "停止: kill $MOVEIT_PID"
else
    echo "启动 MoveIt2（前台运行）..."
    echo "按 Ctrl+C 停止"
    echo ""
    $LAUNCH_CMD
fi
