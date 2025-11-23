#!/bin/bash
# 详细诊断joint_states问题

echo "========================================"
echo "详细诊断joint_states"
echo "========================================"
echo ""

source /home/robot/button/V4.0/project2/setup_ros2_clean.sh

echo "1. 检查joint_state_publisher节点是否存在:"
ros2 node list | grep joint_state
echo ""

echo "2. 检查joint_states topic信息:"
ros2 topic info /joint_states
echo ""

echo "3. 尝试获取joint_states数据 (超时2秒):"
timeout 2 ros2 topic echo /joint_states 2>&1 || echo "❌ 没有数据"
echo ""

echo "4. 检查robot_description topic:"
ros2 topic list | grep robot_description
echo ""

echo "5. 检查robot_description是否有数据:"
timeout 1 ros2 topic echo /robot_description --once 2>&1 | head -10 || echo "❌ 没有robot_description数据"
echo ""

echo "6. 查看joint_state_publisher的参数:"
ros2 param list /joint_state_publisher 2>&1
echo ""

echo "7. 检查joint_state_publisher是否有错误日志:"
ros2 node info /joint_state_publisher 2>&1
echo ""

echo "========================================"
