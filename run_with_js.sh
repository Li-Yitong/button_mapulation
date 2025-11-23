#!/bin/bash
# 快速测试：手动joint_states + button_actions

cd /home/robot/button/V4.0/project2
source setup_ros2_clean.sh

echo "启动joint_states发布器（后台）..."
python3 publish_joint_states.py &
JS_PID=$!
sleep 2

echo "验证joint_states..."
timeout 1 ros2 topic hz /joint_states 2>&1 | head -3

echo ""
echo "运行button_actions.py..."
python3 button_actions.py

# 清理
kill $JS_PID 2>/dev/null
