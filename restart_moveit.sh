#!/bin/bash
# 清理ROS环境并重新启动MoveIt

echo "======================================"
echo "清理ROS环境并重启MoveIt"
echo "======================================"

# 步骤1: 停止所有ROS节点
echo ""
echo "步骤1: 停止所有ROS节点..."
pkill -9 -f roscore
pkill -9 -f roslaunch
pkill -9 -f rosmaster
pkill -9 -f move_group
pkill -9 -f robot_state_publisher
pkill -9 -f joint_state_publisher
pkill -9 -f "piper_press_moveit"
pkill -9 -f "move_a_to_b.py"
sleep 3
echo "✓ 已清理所有ROS进程"

# 步骤2: 清理ROS日志 (可选)
echo ""
echo "步骤2: 清理ROS日志..."
rm -rf ~/.ros/log/*
echo "✓ 日志已清理"

# 步骤3: 启动roscore
echo ""
echo "步骤3: 启动roscore..."
roscore &
ROSCORE_PID=$!
sleep 3
echo "✓ roscore已启动 (PID: $ROSCORE_PID)"

# 步骤4: 启动MoveIt
echo ""
echo "步骤4: 启动MoveIt..."
cd /home/robot/button/V4.0/project2/piper_ros
source devel/setup.bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false load_robot_description:=true &
MOVEIT_PID=$!
sleep 8
echo "✓ MoveIt已启动 (PID: $MOVEIT_PID)"

# 步骤5: 验证节点
echo ""
echo "步骤5: 验证节点状态..."
echo ""
echo "当前运行的ROS节点:"
rosnode list
echo ""

if rosnode list | grep -q "move_group"; then
    echo "✓ move_group节点运行正常"
else
    echo "❌ move_group节点未运行"
fi

if rosparam get /robot_description > /dev/null 2>&1; then
    echo "✓ robot_description已加载"
else
    echo "❌ robot_description未加载"
fi

echo ""
echo "======================================"
echo "✓ MoveIt环境准备完成！"
echo "======================================"
echo ""
echo "现在可以运行: python3 move_a_to_b.py"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo ""

# 保持脚本运行
wait
