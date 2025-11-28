#!/bin/bash
# 快速测试预设零位配置

echo "=========================================="
echo "测试预设零位配置"
echo "=========================================="
echo ""

echo "1. 测试回零脚本..."
echo "   运行: python3 test_go_zero.py"
echo ""
read -p "是否测试回零？(y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    python3 test_go_zero.py
fi

echo ""
echo "2. 测试按钮动作（Push模式）..."
echo "   这将测试完整的动作流程，包括："
echo "   - 从预设零位开始"
echo "   - 执行按压动作"
echo "   - 回到预设零位"
echo ""
read -p "是否测试按钮动作？(y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "启动ROS节点和MoveIt..."
    echo "请确保已经source ROS环境："
    echo "  source /opt/ros/noetic/setup.bash"
    echo ""
    read -p "按任意键继续..." -n 1 -r
    echo
    ./start_press_moveit.sh
fi

echo ""
echo "=========================================="
echo "测试完成！"
echo "=========================================="
