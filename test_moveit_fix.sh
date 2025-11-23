#!/bin/bash
# 测试MoveIt2配置修复

echo "========================================="
echo "测试MoveIt2规划组名称修复"
echo "========================================="

echo ""
echo "1. 检查SRDF中的规划组名称..."
if grep -q 'group name="arm"' /home/robot/piper_ros2/src/piper_description/config/piper.srdf 2>/dev/null; then
    echo "   ✓ SRDF中定义了规划组: arm"
else
    echo "   ⚠️  未找到SRDF文件或规划组定义"
fi

echo ""
echo "2. 检查代码中的规划组名称..."
if grep -q 'group_name = "arm"' /home/robot/button/V4.0/project2/button_action_ros2.py; then
    echo "   ✓ button_action_ros2.py 使用规划组: arm"
else
    echo "   ❌ button_action_ros2.py 规划组名称不匹配！"
fi

echo ""
echo "3. 检查SDK失能保护..."
if grep -q 'piper.EnableArm(7).*防止.*失能' /home/robot/button/V4.0/project2/button_action_ros2.py; then
    echo "   ✓ button_action_ros2.py 已添加失能保护"
else
    echo "   ⚠️  未检测到失能保护代码"
fi

echo ""
echo "4. 检查MoveIt2服务状态..."
if pgrep -f "move_group" > /dev/null; then
    echo "   ✓ MoveIt2 move_group 正在运行"
    echo ""
    echo "   检查可用的planning groups:"
    timeout 2 ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{request:{group_name: 'arm'}}" --feedback 2>&1 | grep -i "group\|error" | head -5
else
    echo "   ❌ MoveIt2 move_group 未运行！"
    echo "   请先启动: ./start_moveit2_clean.sh"
fi

echo ""
echo "========================================="
echo "修复总结:"
echo "========================================="
echo "✓ 规划组名称: 'piper_arm' → 'arm'"
echo "✓ 添加失能保护: control_arm_sdk() 中增加 EnableArm(7)"
echo ""
echo "下一步:"
echo "1. 确保MoveIt2正在运行: ./start_moveit2_clean.sh"
echo "2. 运行测试: python3 button_action_ros2.py"
echo "========================================="
