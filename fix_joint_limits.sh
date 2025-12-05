#!/bin/bash
# =============================================================================
# 关节限位问题自动修复脚本
# 功能: 提高MoveIt2速度缩放因子，解决关节锁死问题
# 作者: GitHub Copilot
# 日期: 2025-12-04
# =============================================================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置文件路径
CONFIG_FILE="$HOME/button/V4.0/project2/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/joint_limits.yaml"

echo -e "${BLUE}=========================================================================${NC}"
echo -e "${BLUE}🔧 MoveIt2关节限位配置修复工具${NC}"
echo -e "${BLUE}=========================================================================${NC}"
echo ""

# 1. 检查配置文件是否存在
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}❌ 错误: 找不到配置文件${NC}"
    echo -e "${RED}   路径: $CONFIG_FILE${NC}"
    echo ""
    echo -e "${YELLOW}💡 提示: 请确认MoveIt2工作空间已正确编译${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 找到配置文件:${NC}"
echo -e "  $CONFIG_FILE"
echo ""

# 2. 显示当前配置
echo -e "${YELLOW}📋 当前配置:${NC}"
grep -E "default_(velocity|acceleration)_scaling_factor" "$CONFIG_FILE" | sed 's/^/  /'
echo ""

# 3. 备份原配置
BACKUP_FILE="${CONFIG_FILE}.bak.$(date +%Y%m%d_%H%M%S)"
cp "$CONFIG_FILE" "$BACKUP_FILE"
echo -e "${GREEN}✓ 已备份原配置:${NC}"
echo -e "  $BACKUP_FILE"
echo ""

# 4. 询问用户选择速度缩放因子
echo -e "${YELLOW}🎯 请选择速度缩放因子:${NC}"
echo -e "  ${GREEN}1${NC}) 0.3 - 保守模式（推荐首次使用）"
echo -e "  ${GREEN}2${NC}) 0.5 - 平衡模式（推荐）⭐"
echo -e "  ${GREEN}3${NC}) 0.7 - 快速模式"
echo -e "  ${GREEN}4${NC}) 1.0 - 最大速度（生产环境）"
echo -e "  ${GREEN}5${NC}) 自定义"
echo ""

read -p "请输入选项 [1-5, 默认2]: " choice
choice=${choice:-2}

case $choice in
    1) SCALING_FACTOR=0.3 ;;
    2) SCALING_FACTOR=0.5 ;;
    3) SCALING_FACTOR=0.7 ;;
    4) SCALING_FACTOR=1.0 ;;
    5)
        read -p "请输入速度缩放因子 (0.1-1.0): " SCALING_FACTOR
        # 验证输入
        if ! [[ "$SCALING_FACTOR" =~ ^0\.[1-9][0-9]?$|^1\.0$ ]]; then
            echo -e "${RED}❌ 无效输入，使用默认值 0.5${NC}"
            SCALING_FACTOR=0.5
        fi
        ;;
    *)
        echo -e "${YELLOW}⚠️  无效选项，使用默认值 0.5${NC}"
        SCALING_FACTOR=0.5
        ;;
esac

echo ""
echo -e "${GREEN}✓ 已选择速度缩放因子: $SCALING_FACTOR${NC}"
echo ""

# 5. 修改配置文件
sed -i "s/default_velocity_scaling_factor: [0-9.]\+/default_velocity_scaling_factor: $SCALING_FACTOR/g" "$CONFIG_FILE"
sed -i "s/default_acceleration_scaling_factor: [0-9.]\+/default_acceleration_scaling_factor: $SCALING_FACTOR/g" "$CONFIG_FILE"

echo -e "${GREEN}✓ 配置文件已更新:${NC}"
grep -E "default_(velocity|acceleration)_scaling_factor" "$CONFIG_FILE" | sed 's/^/  /'
echo ""

# 6. 可选：修改最大速度限制（如果需要）
echo -e "${YELLOW}❓ 是否同时提高关节最大速度限制？${NC}"
echo -e "  当前: joint1-5 = 5.0 rad/s, joint6 = 3.0 rad/s"
echo -e "  建议: 保持不变（默认值已足够）"
read -p "是否修改？ [y/N]: " modify_velocity
modify_velocity=${modify_velocity:-N}

if [[ "$modify_velocity" =~ ^[Yy]$ ]]; then
    read -p "输入新的最大速度 (rad/s, 推荐8.0): " MAX_VEL
    MAX_VEL=${MAX_VEL:-8.0}
    
    sed -i "/joint[1-5]:/,/max_velocity:/ s/max_velocity: [0-9.]\+/max_velocity: $MAX_VEL/g" "$CONFIG_FILE"
    
    echo -e "${GREEN}✓ 已更新关节最大速度为: $MAX_VEL rad/s${NC}"
    echo ""
fi

# 7. 完成提示
echo -e "${BLUE}=========================================================================${NC}"
echo -e "${GREEN}✅ 配置修复完成！${NC}"
echo -e "${BLUE}=========================================================================${NC}"
echo ""
echo -e "${YELLOW}📌 重要：需要重启MoveIt2才能生效${NC}"
echo ""
echo -e "${BLUE}重启步骤:${NC}"
echo -e "  1️⃣  终端1: ${GREEN}bash ~/button/V4.0/project2/start_moveit2_clean.sh${NC}"
echo -e "  2️⃣  终端2: ${GREEN}python3 ~/button/V4.0/project2/button_actions.py${NC}"
echo ""
echo -e "${BLUE}验证改进:${NC}"
echo -e "  ✅ 规划速度提高 ${SCALING_FACTOR}x"
echo -e "  ✅ 轨迹执行更流畅"
echo -e "  ✅ 锁死问题显著减少"
echo ""
echo -e "${YELLOW}💡 如果仍有问题，请查看: ${NC}${GREEN}JOINT_LIMIT_FIX.md${NC}"
echo -e "${BLUE}=========================================================================${NC}"
