#!/bin/bash
# 完整的视觉按钮系统启动脚本（带 MoveIt2）

cd /home/robot/button/V4.0/project2

echo "========================================="
echo "  Vision Button System with MoveIt2"
echo "========================================="
echo ""

# 加载环境
echo "[1/5] 加载 ROS2 环境..."
source ./setup_ros2_clean.sh
source piper_ros/install/setup.bash

# 解析参数
USE_MOVEIT=false
USE_RVIZ=false

for arg in "$@"; do
    case $arg in
        --moveit)
            USE_MOVEIT=true
            ;;
        --rviz)
            USE_RVIZ=true
            ;;
        --help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --moveit    启用 MoveIt2 规划（推荐）"
            echo "  --rviz      启动 RViz2 可视化"
            echo "  --help      显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                    # SDK 直接控制模式"
            echo "  $0 --moveit           # 使用 MoveIt2（无可视化）"
            echo "  $0 --moveit --rviz    # 使用 MoveIt2 + RViz2"
            exit 0
            ;;
    esac
done

# 启动 MoveIt2（如果需要）
if [ "$USE_MOVEIT" = true ]; then
    echo ""
    echo "[2/5] 启动 MoveIt2..."
    if [ "$USE_RVIZ" = true ]; then
        ros2 launch piper_with_gripper_moveit demo_foxy.launch.py > /tmp/moveit2_vision.log 2>&1 &
    else
        # TODO: 创建无 RViz 的 launch 文件
        ros2 launch piper_with_gripper_moveit demo_foxy.launch.py > /tmp/moveit2_vision.log 2>&1 &
    fi
    MOVEIT_PID=$!
    echo "  MoveIt2 PID: $MOVEIT_PID"
    echo "  日志: tail -f /tmp/moveit2_vision.log"
    
    # 等待 MoveIt2 启动
    echo "  等待 MoveIt2 启动（15秒）..."
    sleep 15
    
    # 验证 move_group 是否运行
    if ros2 node list 2>/dev/null | grep -q move_group; then
        echo "  ✓ move_group 节点已启动"
    else
        echo "  ⚠️  警告: move_group 节点未检测到"
    fi
else
    echo ""
    echo "[2/5] 跳过 MoveIt2 启动（SDK 模式）"
fi

# 启动 TF 发布器
echo ""
echo "[3/5] 启动 TF 发布器..."
python3 piper_tf_publisher_ros2.py > /tmp/tf_publisher.log 2>&1 &
TF_PID=$!
echo "  TF Publisher PID: $TF_PID"
sleep 2

# 启动相机和 YOLO 检测
echo ""
echo "[4/5] 启动视觉检测系统..."
python3 realsense_yolo_button_interactive_ros2.py > /tmp/vision_detector.log 2>&1 &
VISION_PID=$!
echo "  Vision Detector PID: $VISION_PID"
echo "  ⚠️  请在弹出的窗口中:"
echo "     1. 点击选择按钮"
echo "     2. 按 ENTER 键确认"
sleep 3

# 启动按钮操作执行器
echo ""
echo "[5/5] 启动按钮操作执行器..."
if [ "$USE_MOVEIT" = true ]; then
    echo "  模式: MoveIt2 规划"
    python3 vision_button_action_ros2.py --moveit
else
    echo "  模式: SDK 直接控制"
    python3 vision_button_action_ros2.py
fi

# 清理函数
cleanup() {
    echo ""
    echo "========================================="
    echo "  正在停止所有进程..."
    echo "========================================="
    
    # 停止按钮操作节点（前台进程会自动停止）
    
    # 停止视觉检测
    if [ ! -z "$VISION_PID" ]; then
        echo "停止视觉检测 (PID: $VISION_PID)..."
        kill $VISION_PID 2>/dev/null
    fi
    
    # 停止 TF 发布器
    if [ ! -z "$TF_PID" ]; then
        echo "停止 TF 发布器 (PID: $TF_PID)..."
        kill $TF_PID 2>/dev/null
    fi
    
    # 停止 MoveIt2
    if [ "$USE_MOVEIT" = true ] && [ ! -z "$MOVEIT_PID" ]; then
        echo "停止 MoveIt2 (PID: $MOVEIT_PID)..."
        kill $MOVEIT_PID 2>/dev/null
        sleep 2
        # 强制停止相关进程
        pkill -f "ros2 launch piper_with_gripper_moveit" 2>/dev/null
        pkill -f move_group 2>/dev/null
        pkill -f rviz2 2>/dev/null
    fi
    
    echo "✓ 所有进程已停止"
    exit 0
}

# 注册清理函数
trap cleanup SIGINT SIGTERM

# 等待前台进程结束
wait
