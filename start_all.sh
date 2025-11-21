#!/bin/bash

# 一键启动所有必要的ROS节点和程序

# 获取脚本所在目录（项目根目录）
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "======================================================"
echo "启动 Piper 机械臂视觉抓取系统"
echo "======================================================"
echo "项目路径: $PROJECT_ROOT"
echo "======================================================"

# 初始化 conda
eval "$(conda shell.bash hook)"

# 检查 ROS 环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "错误: ROS 环境未配置，请先运行 source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查 roscore 是否已经运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "[1/5] 启动 roscore..."
    gnome-terminal --tab --title="roscore" -- bash -c "roscore; exec bash"
    sleep 3
else
    echo "[1/5] roscore 已经在运行"
fi

# 启动 piper_control.launch
echo "[2/5] 启动 piper_control.launch..."
gnome-terminal --tab --title="piper_control" -- bash -c "eval \"\$(conda shell.bash hook)\" && conda activate button && cd $PROJECT_ROOT && roslaunch launch/piper_control.launch; exec bash"
sleep 3

# 启动 realsense_yolo_pc_roi.py
echo "[3/5] 启动 realsense_yolo_pc_roi.py..."
gnome-terminal --tab --title="realsense_yolo" -- bash -c "eval \"\$(conda shell.bash hook)\" && conda activate button && cd $PROJECT_ROOT && python3 realsense_yolo_pc_roi.py; exec bash"
sleep 2

# 启动 piper_tf_publisher.py
echo "[4/5] 启动 piper_tf_publisher.py..."
gnome-terminal --tab --title="piper_tf_publisher" -- bash -c "eval \"\$(conda shell.bash hook)\" && conda activate button && cd $PROJECT_ROOT && python3 piper_tf_publisher.py; exec bash"
sleep 2

# 启动 grasp_action.py
echo "[5/5] 启动 grasp_action.py..."
gnome-terminal --tab --title="grasp_action" -- bash -c "eval \"\$(conda shell.bash hook)\" && conda activate button && cd $PROJECT_ROOT && python3 grasp_action.py; exec bash"

echo ""
echo "======================================================"
echo "所有节点已启动完成！"
echo "======================================================"
echo ""
echo "终端窗口说明:"
echo "  - 窗口1: roscore (ROS主节点)"
echo "  - 窗口2: piper_control.launch (机械臂控制)"
echo "  - 窗口3: realsense_yolo_pc_roi.py (相机+YOLO视觉检测)"
echo "  - 窗口4: piper_tf_publisher.py (TF坐标变换发布)"
echo "  - 窗口5: grasp_action.py (抓取动作控制)"
echo ""
echo "按 Ctrl+C 可以在各个终端窗口中停止对应的节点"
