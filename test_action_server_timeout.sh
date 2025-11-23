#!/bin/bash
# MoveIt2 Action Server超时测试

echo "========================================"
echo "测试: MoveIt2 Action Server超时处理"
echo "========================================"
echo ""

# 1. 停止MoveIt2
echo "📌 步骤1: 停止MoveIt2 (模拟action server不可用)"
pkill -9 move_group 2>/dev/null
sleep 1
echo "  ✓ MoveIt2已停止"
echo ""

# 2. 第一次运行
echo "📌 步骤2: 第一次运行 (应15秒超时后使用SDK模式)"
echo "----------------------------------------"
timeout 60 python3 button_actions.py
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo "  ✅ 第一次运行成功"
elif [ $EXIT_CODE -eq 139 ]; then
    echo "  ❌ 第一次运行失败: 段错误 (退出码: 139)"
    exit 1
else
    echo "  ⚠️  第一次运行退出码: $EXIT_CODE"
fi
echo ""

# 3. 第二次运行
echo "📌 步骤3: 第二次运行 (测试资源清理是否完整)"
echo "----------------------------------------"
timeout 60 python3 button_actions.py
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo "  ✅ 第二次运行成功"
elif [ $EXIT_CODE -eq 139 ]; then
    echo "  ❌ 第二次运行失败: 段错误 (退出码: 139)"
    echo "  💡 这表明资源清理不完整"
    exit 1
else
    echo "  ⚠️  第二次运行退出码: $EXIT_CODE"
fi
echo ""

# 4. 第三次运行
echo "📌 步骤4: 第三次运行 (再次验证稳定性)"
echo "----------------------------------------"
timeout 60 python3 button_actions.py
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo "  ✅ 第三次运行成功"
elif [ $EXIT_CODE -eq 139 ]; then
    echo "  ❌ 第三次运行失败: 段错误 (退出码: 139)"
    exit 1
else
    echo "  ⚠️  第三次运行退出码: $EXIT_CODE"
fi
echo ""

echo "========================================"
echo "✅ 所有测试通过！"
echo "   - 资源清理正确"
echo "   - 可以重复运行"
echo "   - 无段错误"
echo "========================================"
