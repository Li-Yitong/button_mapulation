#!/bin/bash

# ========================================
# 按钮操作执行器启动脚本 (支持三种运动策略)
# ========================================

echo "========================================================================"
echo "按钮操作执行器启动脚本 (支持三种运动策略)"
echo "========================================================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
STRATEGY="incremental"
ACTION="knob"
USE_RVIZ=false

# 解析命令行参数
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -s, --strategy STRATEGY   运动策略 (incremental/cartesian/hybrid)"
    echo "  -a, --action ACTION       动作类型 (toggle/plugin/push/knob)"
    echo "  -r, --rviz               启用RViz可视化"
    echo "  -h, --help               显示帮助信息"
    echo ""
    echo "运动策略说明:"
    echo "  incremental: 增量式SDK控制 - 小步长，关节角度监控"
    echo "  cartesian:   MoveIt笛卡尔路径规划 - 保持姿态恒定"
    echo "  hybrid:      混合模式 - 根据距离自动选择策略"
    echo ""
    echo "示例:"
    echo "  $0 --strategy incremental --action plugin"
    echo "  $0 -s cartesian -a toggle --rviz"
    echo "  $0 -r  # 启用RViz"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--strategy)
            STRATEGY="$2"
            shift 2
            ;;
        -a|--action)
            ACTION="$2"
            shift 2
            ;;
        -r|--rviz)
            USE_RVIZ=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}未知参数: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 验证参数
valid_strategies=("incremental" "cartesian" "hybrid")
valid_actions=("toggle" "plugin" "push" "knob")

if [[ ! " ${valid_strategies[@]} " =~ " ${STRATEGY} " ]]; then
    echo -e "${RED}❌ 无效的运动策略: $STRATEGY${NC}"
    echo -e "${YELLOW}支持的策略: ${valid_strategies[*]}${NC}"
    exit 1
fi

if [[ ! " ${valid_actions[@]} " =~ " ${ACTION} " ]]; then
    echo -e "${RED}❌ 无效的动作类型: $ACTION${NC}"
    echo -e "${YELLOW}支持的动作: ${valid_actions[*]}${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 运动策略: $STRATEGY${NC}"
echo -e "${GREEN}✓ 动作类型: $ACTION${NC}"
echo -e "${GREEN}✓ RViz可视化: $([ "$USE_RVIZ" = "true" ] && echo "启用" || echo "禁用")${NC}"

# 工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${BLUE}工作目录: $SCRIPT_DIR${NC}"

# ========================================
# 步骤1: 检查 ROS 环境
# ========================================
echo ""
echo -e "${YELLOW}步骤1: 检查 ROS 环境...${NC}"

if ! command -v roscore &> /dev/null; then
    echo -e "${RED}❌ ROS 未安装或未配置${NC}"
    exit 1
fi

# 检查 roscore 是否运行
if ! pgrep -x roscore > /dev/null; then
    echo -e "${YELLOW}⚠️  roscore 未运行，正在启动...${NC}"
    roscore &
    ROSCORE_PID=$!
    sleep 3
    echo -e "${GREEN}✓ roscore 已启动 (PID: $ROSCORE_PID)${NC}"
else
    echo -e "${GREEN}✓ roscore 已运行${NC}"
fi

# ========================================
# 步骤2: 设置 ROS 环境变量
# ========================================
echo ""
echo -e "${YELLOW}步骤2: 设置 ROS 环境变量...${NC}"

# 设置 ROS_PACKAGE_PATH
PIPER_ROS_PATH="$SCRIPT_DIR/piper_ros"
if [ -d "$PIPER_ROS_PATH/src" ]; then
    export ROS_PACKAGE_PATH="$PIPER_ROS_PATH/src:$ROS_PACKAGE_PATH"
    echo -e "${GREEN}✓ ROS_PACKAGE_PATH 已设置${NC}"
    echo -e "  $PIPER_ROS_PATH/src"
else
    echo -e "${YELLOW}⚠️  piper_ros/src 目录不存在，跳过设置${NC}"
fi

# ========================================
# 步骤3: 启动 MoveIt (可选)
# ========================================
echo ""
echo -e "${YELLOW}步骤3: 检查 MoveIt 配置...${NC}"

# 读取 button_actions.py 中的 USE_MOVEIT 配置
USE_MOVEIT=$(grep "^USE_MOVEIT = " button_actions.py | sed 's/USE_MOVEIT = //' | sed 's/ *#.*//')

if [ "$USE_MOVEIT" = "True" ]; then
    echo -e "${BLUE}📊 USE_MOVEIT = True，准备启动 MoveIt...${NC}"
    
    # 检查 MoveIt 是否已运行
    if pgrep -f "piper_with_gripper_moveit" > /dev/null; then
        echo -e "${GREEN}✓ MoveIt 已运行${NC}"
    else
        # 根据参数决定是否启用RViz
        if [ "$USE_RVIZ" = "true" ]; then
            echo -e "${YELLOW}正在启动 MoveIt (含 RViz)...${NC}"
            RVIZ_FLAG="true"
        else
            echo -e "${YELLOW}正在启动 MoveIt (无 RViz)...${NC}"
            RVIZ_FLAG="false"
        fi
        
        # 启动 MoveIt
        cd "$PIPER_ROS_PATH"
        source devel/setup.bash 2>/dev/null
        roslaunch piper_with_gripper_moveit demo.launch use_rviz:=$RVIZ_FLAG > /tmp/moveit_launch.log 2>&1 &
        MOVEIT_PID=$!
        
        # 等待 MoveIt 启动
        echo -e "${YELLOW}等待 MoveIt 初始化...${NC}"
        if [ "$USE_RVIZ" = "true" ]; then
            sleep 15  # RViz需要更长启动时间
        else
            sleep 8
        fi
        
        if pgrep -f "piper_with_gripper_moveit" > /dev/null; then
            echo -e "${GREEN}✓ MoveIt 已启动 (PID: $MOVEIT_PID)${NC}"
            if [ "$USE_RVIZ" = "true" ]; then
                echo -e "${GREEN}✓ RViz 已启动，可视化窗口应该已打开${NC}"
                echo ""
                echo -e "${YELLOW}=== RViz 可视化说明 ===${NC}"
                echo -e "${BLUE}RViz 应该已显示：${NC}"
                echo -e "  ${GREEN}✓${NC} Piper 机械臂 3D 模型（RobotModel）"
                echo -e "  ${GREEN}✓${NC} 坐标系（TF）"
                echo -e "  ${GREEN}✓${NC} MoveIt 规划轨迹（MotionPlanning）"
                echo ""
                echo -e "${BLUE}手动添加末端轨迹可视化：${NC}"
                echo -e "${GREEN}1. Marker${NC} - 点击 'Add' → By topic → /end_effector_trail"
                echo -e "${GREEN}2. Path${NC} - 点击 'Add' → By topic → /end_effector_path"
                echo -e "${YELLOW}========================${NC}"
                echo ""
            fi
        else
            echo -e "${RED}❌ MoveIt 启动失败，查看日志: /tmp/moveit_launch.log${NC}"
            echo -e "${YELLOW}将继续使用 SDK 模式运行${NC}"
        fi
        
        cd "$SCRIPT_DIR"
    fi
else
    echo -e "${BLUE}📊 USE_MOVEIT = False，使用 SDK 模式${NC}"
    
    # 即使不使用 MoveIt，也可以启动 RViz 显示机器人
    if [ "$USE_RVIZ" = "true" ]; then
        echo ""
        echo -e "${YELLOW}启动 RViz 可视化...${NC}"
        
        if ! pgrep -f "rviz" > /dev/null; then
            # 使用配置文件启动 RViz
            if [ -f "$SCRIPT_DIR/config/ee_trajectory.rviz" ]; then
                rviz -d "$SCRIPT_DIR/config/ee_trajectory.rviz" > /tmp/rviz.log 2>&1 &
            else
                rviz > /tmp/rviz.log 2>&1 &
            fi
            RVIZ_PID=$!
            sleep 3
            
            if pgrep -f "rviz" > /dev/null; then
                echo -e "${GREEN}✓ RViz 已启动 (PID: $RVIZ_PID)${NC}"
            else
                echo -e "${RED}❌ RViz 启动失败${NC}"
            fi
        else
            echo -e "${GREEN}✓ RViz 已运行${NC}"
        fi
    fi
fi

# ========================================
# 步骤4: 启动 robot_state_publisher
# ========================================
echo ""
echo -e "${YELLOW}步骤4: 启动 robot_state_publisher...${NC}"

if pgrep -f "robot_state_publisher" > /dev/null; then
    echo -e "${GREEN}✓ robot_state_publisher 已运行${NC}"
else
    echo -e "${YELLOW}正在启动 robot_state_publisher...${NC}"
    rosrun robot_state_publisher robot_state_publisher \
        _robot_description:=/robot_description \
        > /tmp/robot_state_publisher.log 2>&1 &
    RSP_PID=$!
    sleep 2
    
    if pgrep -f "robot_state_publisher" > /dev/null; then
        echo -e "${GREEN}✓ robot_state_publisher 已启动 (PID: $RSP_PID)${NC}"
    else
        echo -e "${YELLOW}⚠️  robot_state_publisher 启动失败（可能不影响运行）${NC}"
    fi
fi

# ========================================
# 步骤5: 显示当前配置
# ========================================
echo ""
echo "========================================================================"
echo -e "${BLUE}当前配置:${NC}"
echo "========================================================================"

# 读取配置参数
TARGET_X=$(grep "^TARGET_X = " button_actions.py | sed 's/TARGET_X = //' | sed 's/ *#.*//')
TARGET_Y=$(grep "^TARGET_Y = " button_actions.py | sed 's/TARGET_Y = //' | sed 's/ *#.*//')
TARGET_Z=$(grep "^TARGET_Z = " button_actions.py | sed 's/TARGET_Z = //' | sed 's/ *#.*//')
ACTION_TYPE=$(grep "^ACTION_TYPE = " button_actions.py | sed 's/ACTION_TYPE = //' | sed "s/'//g" | sed 's/ *#.*//')

echo -e "${GREEN}📍 目标位置:${NC} ($TARGET_X, $TARGET_Y, $TARGET_Z)"
echo -e "${GREEN}🎯 动作类型:${NC} $ACTION_TYPE"
echo -e "${GREEN}🔧 控制模式:${NC} $([ "$USE_MOVEIT" = "True" ] && echo "MoveIt" || echo "SDK")"

# 显示动作特定参数
case "$ACTION_TYPE" in
    toggle)
        DIRECTION=$(grep "^TOGGLE_DIRECTION = " button_actions.py | sed 's/TOGGLE_DIRECTION = //' | sed "s/'//g" | sed 's/ *#.*//')
        DISTANCE=$(grep "^TOGGLE_PUSH_DISTANCE = " button_actions.py | sed 's/TOGGLE_PUSH_DISTANCE = //' | sed 's/ *#.*//')
        echo -e "${BLUE}拨动开关配置:${NC}"
        echo -e "  方向: $DIRECTION"
        echo -e "  行程: $(echo "$DISTANCE * 100" | bc)cm"
        ;;
    plugin)
        PLUGIN_ACTION=$(grep "^PLUGIN_ACTION = " button_actions.py | sed 's/PLUGIN_ACTION = //' | sed "s/'//g" | sed 's/ *#.*//')
        DEPTH=$(grep "^PLUGIN_INSERT_DEPTH = " button_actions.py | sed 's/PLUGIN_INSERT_DEPTH = //' | sed 's/ *#.*//')
        echo -e "${BLUE}插拔连接器配置:${NC}"
        echo -e "  动作: $PLUGIN_ACTION"
        echo -e "  深度: $(echo "$DEPTH * 100" | bc)cm"
        ;;
    push)
        DEPTH=$(grep "^PUSH_PRESS_DEPTH = " button_actions.py | sed 's/PUSH_PRESS_DEPTH = //' | sed 's/ *#.*//')
        HOLD=$(grep "^PUSH_HOLD_TIME = " button_actions.py | sed 's/PUSH_HOLD_TIME = //' | sed 's/ *#.*//')
        echo -e "${BLUE}按压按钮配置:${NC}"
        echo -e "  深度: $(echo "$DEPTH * 100" | bc)cm"
        echo -e "  保持: ${HOLD}秒"
        ;;
    knob)
        ANGLE=$(grep "^KNOB_ROTATION_ANGLE = " button_actions.py | sed 's/KNOB_ROTATION_ANGLE = //' | sed 's/ *#.*//')
        DIRECTION=$(grep "^KNOB_ROTATION_DIRECTION = " button_actions.py | sed 's/KNOB_ROTATION_DIRECTION = //' | sed "s/'//g" | sed 's/ *#.*//')
        echo -e "${BLUE}旋转旋钮配置:${NC}"
        echo -e "  角度: ${ANGLE}°"
        echo -e "  方向: $DIRECTION"
        ;;
esac

echo "========================================================================"

# ========================================
# 步骤6: 启动主程序
# ========================================
echo ""
echo -e "${YELLOW}步骤6: 启动按钮操作执行器...${NC}"
echo ""

# 检查 Python 环境
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 未安装${NC}"
    exit 1
fi

# 如果启用了RViz，提醒用户
if [ "$USE_RVIZ" = "true" ]; then
    echo -e "${YELLOW}=== RViz 可视化提示 ===${NC}"
    echo -e "${BLUE}运动轨迹将在 RViz 中实时显示${NC}"
    echo -e "${GREEN}✓${NC} Piper 机械臂模型"
    echo -e "${GREEN}✓${NC} MoveIt 规划轨迹"
    echo -e "${GREEN}✓${NC} 末端执行器轨迹（需手动添加 Marker 和 Path）"
    echo -e "${YELLOW}===================${NC}"
    echo ""
fi

# 启动主程序
echo -e "${GREEN}▶▶▶ 开始执行动作 ◀◀◀${NC}"
echo ""

python3 button_actions.py --strategy "$STRATEGY" --action "$ACTION"

EXIT_CODE=$?

# ========================================
# 结束处理
# ========================================
echo ""
echo "========================================================================"
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✓✓✓ 程序执行完成！✓✓✓${NC}"
else
    echo -e "${RED}❌ 程序执行失败 (退出码: $EXIT_CODE)${NC}"
fi
echo "========================================================================"

# 询问是否清理 ROS 进程
echo ""
read -p "是否关闭 ROS 相关进程? (y/n, 默认n): " CLEANUP
if [ "$CLEANUP" = "y" ] || [ "$CLEANUP" = "Y" ]; then
    echo -e "${YELLOW}正在清理 ROS 进程...${NC}"
    killall -9 roscore rosmaster rosout 2>/dev/null
    pkill -9 -f "piper_with_gripper_moveit" 2>/dev/null
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    pkill -9 -f ros 2>/dev/null
    sleep 1
    echo -e "${GREEN}✓ 已清理所有 ROS 进程${NC}"
else
    echo -e "${BLUE}保持 ROS 进程运行（下次启动更快）${NC}"
fi

echo ""
echo "脚本结束"
