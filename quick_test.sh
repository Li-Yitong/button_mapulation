#!/bin/bash
# 快速测试脚本 - 用于调试MoveIt2规划问题

echo "============================================"
echo "快速测试MoveIt2规划功能"
echo "============================================"
echo ""
echo "⚠️  使用前确保终端1已经运行: ./start_moveit2_clean.sh"
echo ""
read -p "按回车继续..." dummy

# 清理环境
for var in $(env | grep -E '^(ROS_|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH|PKG_CONFIG_PATH|AMENT_|COLCON_)' | cut -d'=' -f1); do
    unset $var
done

# 加载ROS2
source /opt/ros/foxy/setup.bash
source /home/robot/button/V4.0/project2/piper_ros/install/setup.bash

cd /home/robot/button/V4.0/project2

echo ""
echo "正在运行button_actions.py..."
echo "============================================"
python3 button_actions.py 2>&1 | tee /tmp/button_actions_test.log

echo ""
echo "============================================"
echo "测试完成！日志已保存到 /tmp/button_actions_test.log"
echo ""
echo "如果仍然超时，请："
echo "1. 复制终端1（MoveIt2服务器）的输出"
echo "2. 特别是包含 [move_group-3] 的所有行"
echo "3. 发送给我分析"
echo "============================================"
