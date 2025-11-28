#!/bin/bash

# 使用 MoveIt 轨迹规划的启动脚本
# 注意：不激活 conda 环境，使用系统 Python 以避免 libffi 版本冲突

echo "======================================================"
echo "启动 Piper 机械臂视觉抓取系统 (MoveIt 模式)"
echo "======================================================"

# 清理残留的 ROS 节点
echo "[0/6] 清理残留的 ROS 节点..."
if command -v rosnode &>/dev/null && [ ! -z "$ROS_MASTER_URI" ]; then
    echo "y" | rosnode cleanup &>/dev/null
    sleep 1
    echo "  ✓ 节点清理完成"
fi

# 检查 ROS 环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "错误: ROS 环境未配置，请先运行 source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查 MoveIt 是否安装
if ! python3 -c "import moveit_commander" 2>/dev/null; then
    echo "⚠️  警告: MoveIt 未安装或无法导入"
    echo "  请运行: sudo apt install ros-noetic-moveit"
    exit 1
fi

# 检查 roscore 是否已经运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "[1/6] 启动 roscore..."
    gnome-terminal --tab --title="roscore" -- bash -c "roscore; exec bash"
    sleep 3
else
    echo "[1/6] roscore 已经在运行"
fi

# 启动 piper_control.launch
echo "[2/6] 启动 piper_control.launch..."
gnome-terminal --tab --title="piper_control" -- bash -c "cd /home/robot/button/V4.0/project2 && roslaunch launch/piper_control.launch; exec bash"
sleep 3

# 启动 MoveIt move_group 节点
echo "[3/6] 启动 MoveIt move_group..."
if [ -d "/home/robot/button/V4.0/project2/piper_ros/devel" ]; then
    echo "  启动 MoveIt 规划服务..."
    gnome-terminal --tab --title="moveit_group" -- bash -c "source /home/robot/button/V4.0/project2/piper_ros/devel/setup.bash && roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false; exec bash"
else
    echo "  ⚠️  未找到 MoveIt 配置，跳过 move_group 启动"
    echo "  MoveIt 功能可能受限，但仍可使用基本SDK控制"
fi
sleep 5

# 启动 realsense_yolo_pc_roi.py (使用 conda 环境)
echo "[4/6] 启动 realsense_yolo_pc_roi.py..."
echo "  检查相机状态..."

# 检查并关闭已有的 realsense 进程
REALSENSE_PID=$(ps aux | grep "realsense_yolo_pc_roi.py" | grep -v grep | awk '{print $2}')
if [ ! -z "$REALSENSE_PID" ]; then
    echo "  ⚠️  检测到已有 realsense 进程 (PID: $REALSENSE_PID)，正在关闭..."
    kill -9 $REALSENSE_PID 2>/dev/null
    sleep 1
    echo "  ✓ 旧进程已关闭"
fi

# 检查相机是否被锁定
if command -v rs-enumerate-devices &>/dev/null; then
    if rs-enumerate-devices 2>/dev/null | grep -q "Camera Locked.*YES"; then
        echo "  ⚠️  检测到相机被锁定，尝试重置..."
        echo "  如果下面提示需要密码，请输入密码来重置相机驱动"
        sudo rmmod uvcvideo 2>/dev/null
        sleep 1
        sudo modprobe uvcvideo 2>/dev/null
        sleep 2
        echo "  ✓ 相机驱动已重置"
    fi
fi

# 设置 matplotlib 配置目录避免权限问题
export MPLCONFIGDIR=/tmp/matplotlib-cache

gnome-terminal --tab --title="realsense_yolo" -- bash -c "export MPLCONFIGDIR=/tmp/matplotlib-cache && eval \"\$(conda shell.bash hook)\" && conda activate button && cd /home/robot/button/V4.0/project2 && python3 realsense_yolo_pc_roi.py; exec bash"
sleep 2

# 启动 piper_tf_publisher.py (使用 conda 环境)
echo "[5/6] 启动 piper_tf_publisher.py..."
gnome-terminal --tab --title="piper_tf_publisher" -- bash -c "export MPLCONFIGDIR=/tmp/matplotlib-cache && eval \"\$(conda shell.bash hook)\" && conda activate button && cd /home/robot/button/V4.0/project2 && python3 piper_tf_publisher.py; exec bash"
sleep 2

# 启动 grasp_action.py (使用系统 Python，启用 MoveIt)
echo "[6/6] 启动 grasp_action.py (MoveIt 模式)..."
gnome-terminal --tab --title="grasp_action_moveit" -- bash -c "source /home/robot/button/V4.0/project2/piper_ros/devel/setup.bash && cd /home/robot/button/V4.0/project2 && python3 grasp_action_backup.py; exec bash"

echo ""
echo "======================================================"
echo "所有节点已启动完成！(MoveIt 模式)"
echo "======================================================"
echo ""
echo "终端窗口说明:"
echo "  - 窗口1: roscore (ROS主节点)"
echo "  - 窗口2: piper_control.launch (机械臂控制)"
echo "  - 窗口3: MoveIt move_group (轨迹规划服务)"
echo "  - 窗口4: realsense_yolo_pc_roi.py (相机+YOLO视觉检测)"
echo "  - 窗口5: piper_tf_publisher.py (TF坐标变换发布)"
echo "  - 窗口6: grasp_action.py (抓取动作控制 - MoveIt轨迹规划)"
echo ""
echo "✓ MoveIt 轨迹规划已启用"
echo "按 Ctrl+C 可以在各个终端窗口中停止对应的节点"
