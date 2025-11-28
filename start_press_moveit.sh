#!/bin/bash
# filepath: /home/robot/button/V4.0/project2/start_press_moveit.sh
# 启动前向按压程序 (使用MoveIt轨迹规划)
# 3步启动流程: roscore → MoveIt(含robot_description) → 按压程序

set -e  # 遇到错误立即退出

PROJECT_DIR="/home/robot/button/V4.0/project2"
PIPER_ROS_WS="${PROJECT_DIR}/piper_ros"

echo "======================================"
echo "Piper按压程序 - MoveIt模式启动脚本"
echo "======================================"

# 步骤1: 启动roscore (如果未运行)
echo ""
echo "步骤1: 检查roscore..."
if ! pgrep -x "roscore" > /dev/null; then
    echo "启动roscore..."
    roscore &
    sleep 3
    echo "✓ roscore已启动"
else
    echo "✓ roscore已在运行"
fi

# 步骤2: 启动MoveIt (包含robot_description和move_group)
echo ""
echo "步骤2: 启动MoveIt (含robot_description和move_group节点)..."
cd ${PIPER_ROS_WS}
source devel/setup.bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false load_robot_description:=true &
sleep 6
echo "✓ MoveIt已启动"

# 验证robot_description参数是否加载
echo ""
echo "验证robot_description..."
if rosparam get /robot_description > /dev/null 2>&1; then
    echo "✓ robot_description已正确加载"
else
    echo "⚠️  警告: robot_description未找到"
fi

# 步骤3: 启动按压程序 (系统Python + workspace环境)
echo ""
echo "步骤3: 启动按压程序 (MoveIt模式)..."
echo "======================================"
echo "注意事项:"
echo "1. 确保机械臂在安全位置"
echo "2. 检查目标位置参数是否正确"
echo "3. 程序将使用MoveIt进行轨迹规划"
echo "4. 如果MoveIt失败，会自动回退到SDK模式"
echo "======================================"
echo ""

cd ${PROJECT_DIR}
source ${PIPER_ROS_WS}/devel/setup.bash
python3 move_a_to_b.py

echo ""
echo "======================================"
echo "按压程序已结束"
echo "======================================"