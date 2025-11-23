#!/bin/bash
# MoveIt2 测试快速指南

echo "=========================================="
echo "  MoveIt2 测试指南"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}测试分为两个阶段:${NC}"
echo ""
echo "阶段 1: 静态测试（无需机器人运动）"
echo "  - 检查代码结构"
echo "  - 验证导入"
echo "  - 检查 MoveIt2 环境"
echo ""
echo "阶段 2: 动态测试（需要机器人运动）"
echo "  - 测试 SDK 控制"
echo "  - 测试 MoveIt2 规划"
echo "  - 性能对比"
echo ""

# 阶段 1: 静态测试
echo -e "${GREEN}=== 阶段 1: 静态测试 ===${NC}"
echo ""

echo "[1/5] 测试 button_actions 导入..."
python3 -c "import button_actions; print('  ✓ 导入成功')" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "  ${GREEN}✓ button_actions 导入成功${NC}"
else
    echo -e "  ${RED}❌ button_actions 导入失败${NC}"
fi

echo ""
echo "[2/5] 测试 MoveIt2 模块..."
python3 -c "
import button_actions
if button_actions.MOVEIT_AVAILABLE:
    print('  ✓ MoveIt2 模块可用')
else:
    print('  ⚠️  MoveIt2 模块不可用')
" 2>/dev/null

echo ""
echo "[3/5] 检查函数结构..."
python3 test_moveit2_function.py 2>/dev/null | grep -E "(✓|❌|通过)"

echo ""
echo "[4/5] 检查 ROS2 环境..."
source /opt/ros/foxy/setup.bash 2>/dev/null
ros2 node list > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo -e "  ${GREEN}✓ ROS2 环境正常${NC}"
    NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
    echo "  运行中的节点数: $NODE_COUNT"
else
    echo -e "  ${RED}❌ ROS2 环境异常${NC}"
fi

echo ""
echo "[5/5] 检查 MoveIt2 服务..."
ACTION_LIST=$(ros2 action list 2>/dev/null)
if echo "$ACTION_LIST" | grep -q "move_action"; then
    echo -e "  ${GREEN}✓ MoveIt2 action server 运行中${NC}"
    MOVEIT_RUNNING=1
else
    echo -e "  ${YELLOW}⚠️  MoveIt2 action server 未运行${NC}"
    echo "  启动命令: ./start_moveit2.sh --background"
    MOVEIT_RUNNING=0
fi

echo ""
echo -e "${GREEN}=== 阶段 1 完成 ===${NC}"
echo ""

# 询问是否继续
echo -e "${YELLOW}阶段 2 需要机器人连接和运动${NC}"
echo ""
echo "准备工作:"
echo "  1. 确保机器人已通电"
echo "  2. 确保机器人已使能"
echo "  3. 确保周围环境安全"

if [ $MOVEIT_RUNNING -eq 0 ]; then
    echo ""
    echo -e "${YELLOW}建议: 先启动 MoveIt2 以测试完整功能${NC}"
    echo "  ./start_moveit2.sh --background"
fi

echo ""
read -p "是否继续阶段 2 测试? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "测试结束"
    exit 0
fi

# 阶段 2: 动态测试
echo ""
echo -e "${GREEN}=== 阶段 2: 动态测试 ===${NC}"
echo ""

echo "启动实时测试..."
python3 test_moveit2_realtime.py

echo ""
echo -e "${GREEN}=== 测试完成 ===${NC}"
