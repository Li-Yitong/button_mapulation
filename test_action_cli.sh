#!/bin/bash
# 直接使用ros2命令行测试MoveIt2 action
# 这能帮助我们看到服务器的原始响应

echo "========================================"
echo "使用ros2命令行测试MoveIt2 action"
echo "========================================"
echo ""

# 清理环境
for var in $(env | grep -E '^(ROS_|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH|PKG_CONFIG_PATH|AMENT_|COLCON_)' | cut -d'=' -f1); do
    unset $var
done

# 加载ROS2
source /opt/ros/foxy/setup.bash
source /home/robot/button/V4.0/project2/piper_ros/install/setup.bash

echo "1. 检查action server状态:"
ros2 action list
echo ""

echo "2. 查看action详细信息:"
ros2 action info /move_action
echo ""

echo "3. 尝试发送一个最简单的action goal (只规划，不执行):"
echo "   这会显示服务器的详细响应..."
echo ""

# 使用ros2 action send_goal测试
# 注意：这个命令可能会卡住，说明服务器有问题
timeout 15 ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{
  request: {
    workspace_parameters: {
      header: {frame_id: 'base_link'},
      min_corner: {x: -1.0, y: -1.0, z: -1.0},
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    },
    group_name: 'arm',
    num_planning_attempts: 1,
    allowed_planning_time: 2.0,
    max_velocity_scaling_factor: 0.5,
    max_acceleration_scaling_factor: 0.5
  },
  planning_options: {
    plan_only: true
  }
}" --feedback 2>&1

echo ""
echo "========================================"
echo "测试完成"
echo "========================================"
