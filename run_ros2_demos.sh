#!/bin/bash

# ROS2 Demo 测试脚本
# 用于测试 Piper 机械臂的 ROS2 控制功能

echo ""
echo "================================================================"
echo "  Piper 机械臂 ROS2 Demo 测试"
echo "================================================================"
echo ""

# 进入项目目录
cd /home/robot/button/V4.0/project2

# 设置 ROS2 环境
source /opt/ros/foxy/setup.bash

# 检查 CAN 接口
echo "检查 CAN 接口状态..."
if ifconfig | grep -q "can0.*UP"; then
    echo "✓ CAN0 已启动"
else
    echo "✗ CAN0 未启动，请先启动 CAN 接口"
    exit 1
fi

echo ""
echo "可用的测试 Demo:"
echo ""
echo "  1. Demo 1 - 读取机械臂状态"
echo "     功能: 实时显示关节角度和末端执行器位置"
echo ""
echo "  2. Demo 2 - 使能/失能机械臂"
echo "     功能: 测试机械臂的使能和失能控制"
echo ""
echo "  3. Demo 3 - 归零功能"
echo "     功能: 将所有关节移动到 0 度位置"
echo ""
echo "  4. Demo 4 - 夹爪控制"
echo "     功能: 测试夹爪的开合和位置控制"
echo ""
echo "  0. 退出"
echo ""

read -p "请选择要运行的 Demo (0-4): " choice

case $choice in
    1)
        echo ""
        echo "运行 Demo 1: 读取机械臂状态"
        echo "按 Ctrl+C 停止..."
        /usr/bin/python3 demo_01_read_status_ros2.py
        ;;
    2)
        echo ""
        echo "运行 Demo 2: 使能/失能机械臂"
        /usr/bin/python3 demo_02_enable_arm_ros2.py
        ;;
    3)
        echo ""
        echo "运行 Demo 3: 归零功能"
        /usr/bin/python3 demo_03_go_zero_ros2.py
        ;;
    4)
        echo ""
        echo "运行 Demo 4: 夹爪控制"
        /usr/bin/python3 demo_04_gripper_control_ros2.py
        ;;
    0)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效的选择"
        exit 1
        ;;
esac

echo ""
echo "Demo 运行完成！"
