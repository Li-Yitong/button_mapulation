#!/bin/bash
# 测试多次运行button_actions.py（验证节点名称冲突修复）

echo "========================================="
echo "  多次运行测试 - 验证ROS2节点冲突修复"
echo "========================================="
echo ""
echo "修复内容:"
echo "  1. 节点名称添加时间戳（避免冲突） ✅"
echo "  2. action server等待时间延长（15秒） ✅"
echo "  3. 异常处理改进（清理资源） ✅"
echo "  4. ROS2上下文自动清理 ✅"
echo ""
echo "测试计划:"
echo "  运行 2 次 button_actions.py (Ctrl+C 快速退出)"
echo "  观察第二次是否成功初始化"
echo ""
echo "========================================="
echo ""

# 检查MoveIt2状态
if ! pgrep -f "move_group" > /dev/null; then
    echo "❌ MoveIt2 move_group 未运行！"
    echo "   请先运行: cd /home/robot/button/V4.0/project2 && ./start_moveit2_clean.sh"
    exit 1
fi

echo "✓ MoveIt2 move_group 正在运行"
echo ""
echo "开始测试..."
echo ""

# 第一次运行
echo "========================================="
echo "  第1次运行"
echo "========================================="
timeout 5 python3 /home/robot/button/V4.0/project2/button_actions.py 2>&1 | head -30
echo ""
echo "等待2秒..."
sleep 2
echo ""

# 第二次运行
echo "========================================="
echo "  第2次运行（关键测试）"
echo "========================================="
timeout 5 python3 /home/robot/button/V4.0/project2/button_actions.py 2>&1 | head -30
echo ""

echo "========================================="
echo "测试完成"
echo "========================================="
echo ""
echo "预期结果:"
echo "  ✓ 第1次：正常初始化"
echo "  ✓ 第2次：也能正常初始化（不再段错误）"
echo "  ✓ Action server检测成功（不再超时）"
echo ""
