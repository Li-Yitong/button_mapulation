#!/bin/bash

# ========================================
# Piper ROS2 快速测试脚本
# 测试 Piper ROS2 节点是否正常工作
# ========================================

echo "========================================"
echo "Piper ROS2 快速测试"
echo "========================================"
echo ""

# Source 环境
source /opt/ros/foxy/setup.bash
source /home/robot/button/V4.0/project2/piper_ros/install/setup.bash

echo "[1/4] 检查 ROS2 包..."
echo ""
echo "已安装的 Piper 包:"
ros2 pkg list | grep piper
echo ""

echo "[2/4] 检查 Launch 文件..."
echo ""
ros2 launch piper --show-args start_single_piper.launch.py
echo ""

echo "[3/4] 检查消息类型..."
echo ""
echo "PiperStatusMsg:"
ros2 interface show piper_msgs/msg/PiperStatusMsg | head -10
echo ""

echo "[4/4] 检查服务类型..."
echo ""
echo "Enable 服务:"
ros2 interface show piper_msgs/srv/Enable
echo ""

echo "========================================"
echo "✓ 测试完成！"
echo "========================================"
echo ""
echo "下一步："
echo "  1. 连接机械臂并激活 CAN 接口："
echo "     bash can_activate.sh can0 1000000"
echo ""
echo "  2. 启动控制节点："
echo "     ros2 launch piper start_single_piper_rviz.launch.py"
echo ""
echo "  3. 查看完整文档："
echo "     cat PIPER_ROS2_SETUP.md"
echo ""
