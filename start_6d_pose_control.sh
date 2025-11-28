#!/bin/bash

# ========================================
# 6D位姿控制启动脚本
# ========================================

echo "========================================================================"
echo "6D位姿控制启动脚本 (MoveIt + RRTconnect)"
echo "========================================================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
USE_RVIZ=true

# 解析命令行参数
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -r, --rviz               启用RViz可视化"
    echo "  -h, --help               显示帮助信息"
    echo ""
    echo "功能说明:"
    echo "  - 订阅话题: /target_6d_pose (std_msgs/Float64MultiArray)"
    echo "  - 数据格式: [x, y, z, roll, pitch, yaw]"
    echo "  - 规划算法: RRTconnect"
    echo "  - 可视化: 轨迹+尾迹渐变显示"
    echo ""
    echo "示例:"
    echo "  $0              # 不启用RViz"
    echo "  $0 --rviz       # 启用RViz可视化"
    echo ""
    echo "发布目标位姿:"
    echo "  rostopic pub /target_6d_pose std_msgs/Float64MultiArray \"data: [0.2, 0.0, 0.25, 0.0, 0.0, 0.0]\""
}

while [[ $# -gt 0 ]]; do
    case $1 in
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
    echo -e "${RED}❌ piper_ros/src 目录不存在${NC}"
    exit 1
fi

# ========================================
# 步骤3: 启动 MoveIt
# ========================================
echo ""
echo -e "${YELLOW}步骤3: 启动 MoveIt...${NC}"

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
        fi
    else
        echo -e "${RED}❌ MoveIt 启动失败，查看日志: /tmp/moveit_launch.log${NC}"
        exit 1
    fi
    
    cd "$SCRIPT_DIR"
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
# 步骤4.5: 启动 piper_tf_publisher (用于RViz显示机器人)
# ========================================
if [ "$USE_RVIZ" = "true" ]; then
    echo ""
    echo -e "${YELLOW}步骤4.5: 启动 piper_tf_publisher (RViz机器人显示)...${NC}"
    
    if pgrep -f "piper_tf_publisher.py" > /dev/null; then
        echo -e "${GREEN}✓ piper_tf_publisher 已运行${NC}"
    else
        if [ -f "piper_tf_publisher.py" ]; then
            echo -e "${YELLOW}正在启动 piper_tf_publisher...${NC}"
            python3 piper_tf_publisher.py > /tmp/piper_tf_publisher.log 2>&1 &
            TF_PUB_PID=$!
            sleep 2
            
            if pgrep -f "piper_tf_publisher.py" > /dev/null; then
                echo -e "${GREEN}✓ piper_tf_publisher 已启动 (PID: $TF_PUB_PID)${NC}"
                echo -e "${GREEN}✓ RViz 现在可以显示机器人模型了${NC}"
            else
                echo -e "${YELLOW}⚠️  piper_tf_publisher 启动失败${NC}"
                echo -e "${YELLOW}    RViz 可能无法显示机器人模型${NC}"
                echo -e "${YELLOW}    查看日志: /tmp/piper_tf_publisher.log${NC}"
            fi
        else
            echo -e "${YELLOW}⚠️  未找到 piper_tf_publisher.py${NC}"
            echo -e "${YELLOW}    RViz 可能无法显示机器人模型${NC}"
        fi
    fi
fi

# ========================================
# 步骤5: 等待 robot_description 参数
# ========================================
echo ""
echo -e "${YELLOW}步骤5: 检查 robot_description 参数...${NC}"

MAX_WAIT=10
WAIT_COUNT=0
while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    if rosparam get /robot_description > /dev/null 2>&1; then
        echo -e "${GREEN}✓ robot_description 参数已加载${NC}"
        break
    fi
    echo -e "${YELLOW}等待 robot_description 参数... ($((WAIT_COUNT+1))/$MAX_WAIT)${NC}"
    sleep 1
    WAIT_COUNT=$((WAIT_COUNT+1))
done

if [ $WAIT_COUNT -eq $MAX_WAIT ]; then
    echo -e "${RED}❌ robot_description 参数加载超时${NC}"
    exit 1
fi

# ========================================
# 步骤6: 显示配置信息
# ========================================
echo ""
echo "========================================================================"
echo -e "${BLUE}6D位姿控制配置:${NC}"
echo "========================================================================"
echo -e "${GREEN}📡 订阅话题:${NC} /target_6d_pose"
echo -e "${GREEN}📝 消息格式:${NC} std_msgs/Float64MultiArray"
echo -e "${GREEN}📊 数据格式:${NC} [x, y, z, roll, pitch, yaw]"
echo -e "${GREEN}🔧 规划算法:${NC} RRTconnect"
echo -e "${GREEN}📈 频率控制:${NC}"
echo -e "   - RViz发布: 10 Hz"
echo -e "   - SDK执行: 50 Hz"
echo -e "   - 命令发布: 100 Hz"
echo -e "${GREEN}🎨 可视化:${NC}"
echo -e "   - /move_group/display_planned_path (轨迹)"
echo -e "   - /end_effector_path (路径)"
echo -e "   - /end_effector_trail (尾迹渐变)"
echo "========================================================================"

# ========================================
# 步骤7: 配置 RViz (如果启用)
# ========================================
if [ "$USE_RVIZ" = "true" ]; then
    echo ""
    echo -e "${YELLOW}步骤7: 配置 RViz 显示...${NC}"
    echo -e "${BLUE}请在 RViz 中手动添加以下显示项：${NC}"
    echo -e "  1. Marker (话题: /end_effector_trail) - 显示尾迹"
    echo -e "  2. Path (话题: /end_effector_path) - 显示路径"
    echo -e "  3. DisplayTrajectory 已自动添加"
    echo ""
fi

# ========================================
# 步骤8: 启动 6D 位姿控制节点
# ========================================
echo ""
echo -e "${YELLOW}步骤8: 启动 6D 位姿控制节点...${NC}"
echo ""

# 检查 Python 环境
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 未安装${NC}"
    exit 1
fi

# 检查脚本文件
if [ ! -f "moveit_6d_pose_control.py" ]; then
    echo -e "${RED}❌ moveit_6d_pose_control.py 文件不存在${NC}"
    exit 1
fi

# 启动主程序
echo -e "${GREEN}▶▶▶ 6D位姿控制节点启动中... ◀◀◀${NC}"
echo ""
echo -e "${BLUE}使用示例:${NC}"
echo -e "${YELLOW}# 发布目标位姿 (命令行)${NC}"
echo -e "rostopic pub /target_6d_pose std_msgs/Float64MultiArray \"data: [0.2, 0.0, 0.25, 0.0, 0.0, 0.0]\""
echo ""
echo -e "${YELLOW}# Python代码示例${NC}"
echo -e "from std_msgs.msg import Float64MultiArray"
echo -e "import rospy"
echo -e "rospy.init_node('test_publisher')"
echo -e "pub = rospy.Publisher('/target_6d_pose', Float64MultiArray, queue_size=1)"
echo -e "msg = Float64MultiArray()"
echo -e "msg.data = [0.2, 0.0, 0.25, 0.0, 0.0, 0.0]  # x, y, z, roll, pitch, yaw"
echo -e "rospy.sleep(1)"
echo -e "pub.publish(msg)"
echo ""
echo "========================================================================"
echo ""

python3 moveit_6d_pose_control.py

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
