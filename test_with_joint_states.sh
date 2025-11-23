#!/bin/bash
# 测试MoveIt2与手动发布的joint_states

echo "========================================"
echo "测试MoveIt2规划 (手动发布joint_states)"
echo "========================================"
echo ""

# 清理环境
source /home/robot/button/V4.0/project2/setup_ros2_clean.sh

echo "1️⃣ 启动手动joint_states发布器 (后台运行)..."
python3 /home/robot/button/V4.0/project2/publish_joint_states.py &
JS_PID=$!
echo "   PID: $JS_PID"
sleep 2

echo ""
echo "2️⃣ 检查joint_states是否在发布..."
timeout 1 ros2 topic hz /joint_states 2>&1 | head -5
echo ""

echo "3️⃣ 运行规划测试..."
python3 /home/robot/button/V4.0/project2/test_frame_fix.py
TEST_RESULT=$?

echo ""
echo "4️⃣ 清理..."
kill $JS_PID 2>/dev/null
wait $JS_PID 2>/dev/null

echo ""
echo "========================================"
if [ $TEST_RESULT -eq 0 ]; then
    echo "✅ 测试成功！"
else
    echo "❌ 测试失败 (退出码: $TEST_RESULT)"
fi
echo "========================================"
