#!/bin/bash
# URDF 加载诊断脚本

echo "======================================"
echo "Piper URDF 加载状态检查"
echo "======================================"

# 检查 roscore 是否运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ roscore 未运行"
    echo "   请先启动 roscore"
    exit 1
else
    echo "✓ roscore 已运行"
fi

# 检查 robot_description 参数
echo ""
echo "检查 /robot_description 参数..."
if rosparam get /robot_description > /dev/null 2>&1; then
    echo "✓ robot_description 参数已加载"
    
    # 检查内容长度
    URDF_LENGTH=$(rosparam get /robot_description | wc -c)
    echo "  URDF 长度: ${URDF_LENGTH} 字符"
    
    if [ $URDF_LENGTH -lt 100 ]; then
        echo "  ⚠️  URDF 内容过短，可能未正确加载"
    else
        echo "  ✓ URDF 内容长度正常"
    fi
    
    # 检查是否包含关键元素
    if rosparam get /robot_description | grep -q "<robot"; then
        echo "  ✓ 包含 <robot> 标签"
    else
        echo "  ❌ 缺少 <robot> 标签"
    fi
    
    if rosparam get /robot_description | grep -q "base_link"; then
        echo "  ✓ 包含 base_link"
    else
        echo "  ❌ 缺少 base_link"
    fi
    
else
    echo "❌ robot_description 参数未找到"
    echo ""
    echo "可能原因："
    echo "1. 未启动加载 URDF 的 launch 文件"
    echo "2. URDF 文件路径错误"
    echo "3. launch 文件中的 load_robot_description 参数为 false"
fi

# 检查 robot_state_publisher
echo ""
echo "检查 robot_state_publisher 节点..."
if rosnode list | grep -q "robot_state_publisher"; then
    echo "✓ robot_state_publisher 节点已运行"
else
    echo "⚠️  robot_state_publisher 节点未运行"
    echo "   RViz 可能无法正确显示机器人模型"
fi

# 检查 joint_states 话题
echo ""
echo "检查 /joint_states 话题..."
if rostopic list | grep -q "/joint_states"; then
    echo "✓ /joint_states 话题存在"
    
    # 尝试获取一条消息
    if timeout 2 rostopic echo /joint_states -n 1 > /dev/null 2>&1; then
        echo "  ✓ /joint_states 话题有数据发布"
    else
        echo "  ⚠️  /joint_states 话题无数据 (超时)"
    fi
else
    echo "❌ /joint_states 话题不存在"
fi

# 检查 tf
echo ""
echo "检查 TF 树..."
if rosrun tf tf_echo base_link link6 > /dev/null 2>&1; then
    echo "✓ TF 树正常 (base_link → link6 可达)"
else
    echo "⚠️  TF 树可能不完整"
fi

# 检查 MoveIt 相关参数
echo ""
echo "检查 MoveIt 配置..."
if rosparam get /robot_description_semantic > /dev/null 2>&1; then
    echo "✓ robot_description_semantic (SRDF) 已加载"
else
    echo "⚠️  robot_description_semantic 未加载 (MoveIt 未启动?)"
fi

if rosparam get /robot_description_planning > /dev/null 2>&1; then
    echo "✓ robot_description_planning 已加载"
else
    echo "⚠️  robot_description_planning 未加载"
fi

# 检查 move_group 节点
echo ""
echo "检查 move_group 节点..."
if rosnode list | grep -q "move_group"; then
    echo "✓ move_group 节点已运行"
    
    # 检查规划组
    if rosservice list | grep -q "get_planning_scene"; then
        echo "  ✓ MoveIt 服务可用"
    fi
else
    echo "⚠️  move_group 节点未运行 (MoveIt 未启动)"
fi

echo ""
echo "======================================"
echo "诊断完成"
echo "======================================"
echo ""
echo "💡 建议："
echo "1. 如果 robot_description 未加载，检查 launch 文件路径"
echo "2. 如果 RViz 显示异常，确保 robot_state_publisher 运行"
echo "3. 如果 MoveIt 无法使用，检查 move_group 节点"
echo "4. 确保所有路径指向 V3.0 而不是 V0.0"
