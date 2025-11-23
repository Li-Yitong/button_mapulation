#!/bin/bash

# ========================================
# 从 Humble 分支移植 MoveIt2 配置到 Foxy
# ========================================

set -e

echo "========================================================================"
echo "从 Humble 分支移植 MoveIt2 配置到 Foxy"
echo "========================================================================"

PIPER_ROS_DIR="/home/robot/button/V4.0/project2/piper_ros"
TEMP_DIR="/tmp/piper_humble_moveit"

cd "$PIPER_ROS_DIR"

# 1. 创建临时目录
echo "[1/6] 创建临时目录..."
rm -rf "$TEMP_DIR"
mkdir -p "$TEMP_DIR"

# 2. 从 humble 分支提取 MoveIt 配置
echo "[2/6] 从 humble 分支提取 MoveIt 配置..."
git archive origin/humble src/piper_moveit | tar -x -C "$TEMP_DIR"

# 3. 复制到当前分支
echo "[3/6] 复制 MoveIt 配置到当前 foxy 分支..."
if [ -d "$PIPER_ROS_DIR/src/piper_moveit" ]; then
    echo "  ⚠️  警告: src/piper_moveit 已存在，将备份..."
    mv "$PIPER_ROS_DIR/src/piper_moveit" "$PIPER_ROS_DIR/src/piper_moveit.backup.$(date +%Y%m%d_%H%M%S)"
fi

cp -r "$TEMP_DIR/src/piper_moveit" "$PIPER_ROS_DIR/src/"

# 4. 检查 Foxy 兼容性
echo "[4/6] 检查 Foxy 兼容性..."

# 检查关键配置文件
MOVEIT_PKG="$PIPER_ROS_DIR/src/piper_moveit/piper_with_gripper_moveit"
if [ ! -f "$MOVEIT_PKG/config/piper.srdf" ]; then
    echo "  ✗ 错误: 缺少 SRDF 文件"
    exit 1
fi

if [ ! -f "$MOVEIT_PKG/config/kinematics.yaml" ]; then
    echo "  ✗ 错误: 缺少运动学配置文件"
    exit 1
fi

echo "  ✓ 关键配置文件完整"

# 5. 编译 MoveIt 配置包
echo "[5/6] 编译 MoveIt 配置包..."
cd "$PIPER_ROS_DIR"

# 清理并重新编译
source /opt/ros/foxy/setup.bash
colcon build --packages-select piper_with_gripper_moveit piper_no_gripper_moveit --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo "  ✓ 编译成功"
else
    echo "  ✗ 编译失败"
    echo "  提示: 可能需要手动调整 package.xml 或 CMakeLists.txt"
    exit 1
fi

# 6. 清理临时文件
echo "[6/6] 清理临时文件..."
rm -rf "$TEMP_DIR"

echo ""
echo "========================================================================"
echo "✓✓✓ MoveIt2 配置移植完成！✓✓✓"
echo "========================================================================"
echo ""
echo "移植的包:"
echo "  - piper_with_gripper_moveit    (带夹爪版本)"
echo "  - piper_no_gripper_moveit      (不带夹爪版本)"
echo ""
echo "下一步:"
echo "  1. source $PIPER_ROS_DIR/install/setup.bash"
echo "  2. ros2 launch piper_with_gripper_moveit demo.launch.py"
echo ""
echo "注意:"
echo "  - Humble 配置可能需要微调才能在 Foxy 上完美运行"
echo "  - 如果遇到错误，请检查日志并手动修改配置文件"
echo "========================================================================"
