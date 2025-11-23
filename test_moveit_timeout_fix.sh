#!/bin/bash
# 快速测试修复后的button_actions.py

echo "========================================="
echo "  MoveIt2规划超时问题修复验证"
echo "========================================="
echo ""
echo "修复内容:"
echo "  1. 规划组名称: 'piper_arm' → 'arm' ✅"
echo "  2. SDK等待到达: 添加estimated_time等待 ✅"
echo "  3. 段错误修复: 改进ROS2清理逻辑 ✅"
echo ""
echo "测试步骤:"
echo "  1. 确保MoveIt2正在运行"
echo "  2. 运行 python3 button_actions.py"
echo "  3. 观察是否仍然超时"
echo ""
echo "========================================="
echo ""

# 检查MoveIt2状态
if pgrep -f "move_group" > /dev/null; then
    echo "✓ MoveIt2 move_group 正在运行"
else
    echo "❌ MoveIt2 move_group 未运行！"
    echo "   请先运行: cd /home/robot/button/V4.0/project2 && ./start_moveit2_clean.sh"
    exit 1
fi

echo ""
echo "✓ 准备就绪，可以运行测试"
echo ""
echo "预期结果:"
echo "  - MoveIt2规划请求应该被接受（不再超时）"
echo "  - 规划成功并执行轨迹"
echo "  - 程序结束时不再段错误"
echo ""
