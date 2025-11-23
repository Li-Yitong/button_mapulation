#!/bin/bash
# 快速测试语法修复

echo "========================================="
echo "  语法错误修复验证"
echo "========================================="
echo ""

# 检查语法
echo "检查 Python 语法..."
if python3 -m py_compile button_actions.py 2>&1; then
    echo "✅ 语法检查通过！"
else
    echo "❌ 语法错误仍然存在"
    exit 1
fi

echo ""
echo "修复内容:"
echo "  ✅ global MOVEIT_AVAILABLE 移到函数开始"
echo "  ✅ 删除 except 块中的重复声明"
echo ""
echo "现在可以运行程序了:"
echo "  python3 button_actions.py"
echo ""
