#!/bin/bash
# 完整启动脚本：MoveIt2服务器 + button_actions.py测试
# 此脚本会同时启动MoveIt2和button_actions，并自动发布joint_states

echo "========================================="
echo "完整MoveIt2测试 - button_actions.py"
echo "========================================="
echo ""

cd /home/robot/button/V4.0/project2

# 检查MoveIt2是否已经运行
if ros2 node list 2>/dev/null | grep -q move_group; then
    echo "✓ MoveIt2服务器已在运行"
else
    echo "❌ MoveIt2服务器未运行"
    echo "   请在另一个终端运行: ./start_moveit2_clean.sh"
    echo ""
    read -p "按Enter键继续（如果MoveIt2已启动）或Ctrl+C取消..."
fi

echo ""
echo "========================================="
echo "启动button_actions.py"
echo "========================================="
echo ""

# 清理环境并运行
source setup_ros2_clean.sh
python3 button_actions.py

echo ""
echo "========================================="
echo "测试完成"
echo "========================================="
