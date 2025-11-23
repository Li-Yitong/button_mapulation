#!/bin/bash
# 测试ROS2 MoveIt2丝滑执行修复效果

echo "=================================================="
echo "ROS2 MoveIt2 丝滑执行测试"
echo "=================================================="
echo ""
echo "此测试会执行一次Push动作，观察机械臂运动是否流畅"
echo ""
echo "期望结果:"
echo "  ✅ 机械臂连续流畅运动"
echo "  ✅ 无抖动、无重复往返"
echo "  ✅ 控制台显示: 'MoveIt2规划XX点 → SDK插值执行YY点' (YY >> XX)"
echo ""
echo "如果出现问题:"
echo "  ❌ 一抖一抖的运动 → 插值逻辑未生效"
echo "  ❌ 重复往返 → 可能是时间同步问题"
echo ""
read -p "按Enter键开始测试..."

# 设置环境
cd /home/robot/button/V4.0/project2
source setup_ros2_clean.sh

# 启动MoveIt2（后台）
echo ""
echo "1. 启动MoveIt2..."
bash start_moveit2_clean.sh > /tmp/moveit2.log 2>&1 &
MOVEIT_PID=$!
sleep 8

# 检查MoveIt2是否启动
if ! ros2 action list 2>/dev/null | grep -q '/move_action'; then
    echo "❌ MoveIt2未成功启动，检查日志: /tmp/moveit2.log"
    kill $MOVEIT_PID 2>/dev/null
    exit 1
fi
echo "✓ MoveIt2已启动"

# 运行button_actions.py（只执行一次Push）
echo ""
echo "2. 执行Push动作（观察机械臂是否流畅）..."
echo ""
python3 button_actions.py 2>&1 | tee /tmp/button_actions_test.log

# 检查日志关键信息
echo ""
echo "=================================================="
echo "执行结果分析"
echo "=================================================="

if grep -q "SDK插值执行" /tmp/button_actions_test.log; then
    PLANNED=$(grep "SDK插值执行" /tmp/button_actions_test.log | head -1 | sed -n 's/.*规划\([0-9]*\)点.*/\1/p')
    EXECUTED=$(grep "SDK插值执行" /tmp/button_actions_test.log | head -1 | sed -n 's/.*执行\([0-9]*\)点.*/\1/p')
    
    echo "✅ 插值执行已生效:"
    echo "   - MoveIt2规划: ${PLANNED}点"
    echo "   - SDK插值执行: ${EXECUTED}点"
    echo "   - 插值倍数: $((EXECUTED / PLANNED))x"
    echo ""
    
    if [ $EXECUTED -gt $((PLANNED * 5)) ]; then
        echo "✅ 插值密度足够（>5倍），运动应该流畅"
    else
        echo "⚠️  插值密度偏低（<5倍），可能不够流畅"
        echo "   建议: 增加 COMMAND_SEND_RATE 参数"
    fi
else
    echo "❌ 未检测到插值执行，可能使用了旧版本代码"
    echo "   检查: button_actions.py是否包含插值逻辑"
fi

echo ""
if grep -q "✓✓✓ Push 操作完成！✓✓✓" /tmp/button_actions_test.log; then
    echo "✅ Push操作成功完成"
else
    echo "❌ Push操作未完成，检查日志: /tmp/button_actions_test.log"
fi

# 清理
echo ""
echo "=================================================="
read -p "测试完成，按Enter键关闭MoveIt2..."
kill $MOVEIT_PID 2>/dev/null
echo "✓ 已清理进程"
echo ""
echo "完整日志:"
echo "  - MoveIt2: /tmp/moveit2.log"
echo "  - button_actions: /tmp/button_actions_test.log"
