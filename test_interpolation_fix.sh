#!/bin/bash
# 快速测试插值执行修复效果

echo "========================================"
echo "测试ROS2 MoveIt2插值执行修复"
echo "========================================"
echo ""
echo "修复内容:"
echo "1. ✅ 基于轨迹总时长而非点索引的插值循环"
echo "2. ✅ 每个周期都记录执行数据（用于精确PVAT图表）"
echo "3. ✅ 修复ROS2时间戳API (rospy.Time.now() → moveit_node.get_clock().now().to_msg())"
echo "4. ✅ 生成PVAT (Position-Velocity-Acceleration-Time) 分析图表"
echo ""
echo "预期结果:"
echo "  - MoveIt2规划22点 → SDK插值执行~160点 (而非35点)"
echo "  - 插值倍数: ~7-8x (而非1.6x)"
echo "  - 生成完整的PVAT对比图表"
echo ""
read -p "按Enter键开始测试..."

cd /home/robot/button/V4.0/project2
source setup_ros2_clean.sh

# 启动MoveIt2（后台）
echo ""
echo "1. 启动MoveIt2..."
bash start_moveit2_clean.sh > /tmp/moveit2_test.log 2>&1 &
MOVEIT_PID=$!
sleep 8

# 检查MoveIt2
if ! ros2 action list 2>/dev/null | grep -q '/move_action'; then
    echo "❌ MoveIt2未启动，检查日志: /tmp/moveit2_test.log"
    kill $MOVEIT_PID 2>/dev/null
    exit 1
fi
echo "✓ MoveIt2已启动"

# 运行button_actions.py
echo ""
echo "2. 执行Push动作（观察插值效果）..."
echo ""
python3 button_actions.py 2>&1 | tee /tmp/button_test.log

# 分析结果
echo ""
echo "========================================"
echo "执行结果分析"
echo "========================================"

# 检查插值执行数
if grep -q "SDK插值执行" /tmp/button_test.log; then
    echo "✓ 找到插值执行日志"
    grep "SDK插值执行" /tmp/button_test.log | head -4
    
    # 提取第一次执行的插值数据
    FIRST_LINE=$(grep "SDK插值执行" /tmp/button_test.log | head -1)
    PLANNED=$(echo "$FIRST_LINE" | grep -oP '规划\K[0-9]+')
    EXECUTED=$(echo "$FIRST_LINE" | grep -oP '执行\K[0-9]+')
    
    if [ -n "$PLANNED" ] && [ -n "$EXECUTED" ]; then
        RATIO=$((EXECUTED * 100 / PLANNED))
        echo ""
        echo "📊 插值统计 (第1次规划):"
        echo "  规划点数: ${PLANNED}"
        echo "  执行命令数: ${EXECUTED}"
        echo "  插值倍数: $((RATIO / 100)).$((RATIO % 100))x"
        echo ""
        
        if [ $RATIO -ge 700 ]; then
            echo "✅ 插值倍数正常 (≥7x)，修复成功！"
        elif [ $RATIO -ge 500 ]; then
            echo "⚠️  插值倍数偏低 (5-7x)，可能需要调整COMMAND_SEND_RATE"
        else
            echo "❌ 插值倍数过低 (<5x)，插值逻辑可能有问题"
        fi
    fi
else
    echo "❌ 未找到插值执行日志"
fi

# 检查PVAT图表
echo ""
if [ -f trajectory/pvat_data.pkl ]; then
    echo "✓ PVAT数据已保存: trajectory/pvat_data.pkl"
    if ls trajectory/pvat_analysis_*.png 1> /dev/null 2>&1; then
        LATEST_PNG=$(ls -t trajectory/pvat_analysis_*.png | head -1)
        echo "✓ PVAT图表已生成: $LATEST_PNG"
    else
        echo "⚠️  PVAT图表未生成"
    fi
else
    echo "⚠️  PVAT数据未保存"
fi

# 检查完成状态
echo ""
if grep -q "✓✓✓ Push 操作完成！✓✓✓" /tmp/button_test.log; then
    echo "✅ Push操作成功完成"
else
    echo "❌ Push操作未完成"
fi

# 清理
echo ""
echo "========================================"
read -p "测试完成，按Enter键关闭MoveIt2..."
kill $MOVEIT_PID 2>/dev/null
echo "✓ 已清理进程"
echo ""
echo "日志文件:"
echo "  - MoveIt2: /tmp/moveit2_test.log"
echo "  - button_actions: /tmp/button_test.log"
echo "  - PVAT数据: trajectory/pvat_data.pkl"
echo "  - PVAT图表: trajectory/pvat_analysis_*.png"
