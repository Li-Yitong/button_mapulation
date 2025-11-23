#!/bin/bash
# 完整的端到端测试流程

echo "========================================="
echo "完整端到端测试"
echo "========================================="
echo ""

cd /home/robot/button/V4.0/project2

# Step 1: 重启MoveIt2（清除任何阻塞状态）
echo "Step 1: 重启MoveIt2服务器..."
echo "  请在终端1按Ctrl+C停止旧的MoveIt2"
echo "  然后运行: ./start_moveit2_clean.sh"
echo ""
read -p "MoveIt2重启完成后按Enter继续..."

# Step 2: 验证MoveIt2正在运行
echo ""
echo "Step 2: 验证MoveIt2..."
source setup_ros2_clean.sh
if ros2 node list | grep -q move_group; then
    echo "  ✓ MoveIt2正在运行"
else
    echo "  ❌ MoveIt2未运行！"
    exit 1
fi

# Step 3: 运行集成测试
echo ""
echo "Step 3: 运行集成测试..."
echo "========================================="
python3 test_button_integration.py
RESULT=$?

echo ""
echo "========================================="
if [ $RESULT -eq 0 ]; then
    echo "✅ 测试成功！"
    echo ""
    echo "现在可以运行完整的button_actions.py:"
    echo "  cd ~/button/V4.0/project2"
    echo "  source setup_ros2_clean.sh"
    echo "  python3 button_actions.py"
else
    echo "❌ 测试失败"
    echo "请检查终端1的MoveIt2输出"
fi
echo "========================================="

exit $RESULT
