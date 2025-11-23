#!/bin/bash
# 使用手动joint_states测试MoveIt2

echo "========================================="
echo "测试方案：手动发布joint_states"
echo "========================================="
echo ""

cd /home/robot/button/V4.0/project2
source setup_ros2_clean.sh

echo "1️⃣ 启动joint_states发布器 (后台)..."
python3 publish_joint_states.py &
JS_PID=$!
echo "   进程ID: $JS_PID"
sleep 3

echo ""
echo "2️⃣ 验证joint_states..."
timeout 1 ros2 topic hz /joint_states 2>&1 | head -3

echo ""
echo "3️⃣ 测试MoveIt2规划..."
python3 test_frame_fix.py
RESULT=$?

echo ""
echo "4️⃣ 清理..."
kill $JS_PID 2>/dev/null
wait $JS_PID 2>/dev/null

echo ""
echo "========================================="
if [ $RESULT -eq 0 ]; then
    echo "✅ 测试完成"
else
    echo "❌ 测试失败"
fi
echo "========================================="

exit $RESULT
