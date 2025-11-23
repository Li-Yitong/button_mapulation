#!/bin/bash
# 测试MoveIt2规划功能的完整流程
# 
# 使用方法:
# 1. 终端1: ./start_moveit2_clean.sh
# 2. 终端2: ./test_moveit2_full.sh

echo "=========================================="
echo "MoveIt2完整功能测试"
echo "=========================================="
echo ""

# 清理环境
echo "正在清理ROS环境变量..."
for var in $(env | grep -E '^(ROS_|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH|PKG_CONFIG_PATH|AMENT_|COLCON_)' | cut -d'=' -f1); do
    unset $var
done
echo "✓ 环境已清理"
echo ""

# 加载ROS2 Foxy
echo "正在加载ROS2 Foxy..."
source /opt/ros/foxy/setup.bash
source /home/robot/button/V4.0/project2/piper_ros/install/setup.bash
echo "✓ ROS2 Foxy已加载"
echo "  ROS_DISTRO=$ROS_DISTRO"
echo "  ROS_VERSION=$ROS_VERSION"
echo ""

# 等待MoveIt2启动
echo "等待MoveIt2 action server启动..."
timeout 15 bash -c '
while ! ros2 action list 2>/dev/null | grep -q "/move_action"; do
    sleep 0.5
done
' && echo "✓ MoveIt2 action server已就绪" || echo "⚠️  超时，但继续测试"
echo ""

# 运行button_actions测试
echo "=========================================="
echo "运行button_actions.py测试..."
echo "=========================================="
cd /home/robot/button/V4.0/project2
python3 button_actions.py

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
