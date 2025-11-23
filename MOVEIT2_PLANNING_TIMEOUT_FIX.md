# MoveIt2 规划超时问题 - 解决方案

## 问题诊断

从终端1的MoveIt2服务器日志中发现关键问题:

```
[move_group-3] [INFO] [1763822345.019753879] Received request
[move_group-3] [INFO] [1763822345.019908431] executing..
```

**规划请求已接受并开始执行，但30秒后仍无结果返回**。

根本原因:
```
[move_group-3] [ERROR] Action client not connected: arm_controller/follow_joint_trajectory
[move_group-3] [ERROR] Action client not connected: gripper_controller/follow_joint_trajectory
[move_group-3] [INFO] Returned 0 controllers in list
```

**MoveIt2即使在`plan_only=True`模式下，仍然等待真实的robot controllers连接**，导致规划结果无法返回。

## 解决方案

### 1. 修改MoveIt2启动配置

编辑 `demo_foxy.launch.py`:
```python
trajectory_execution = {
    'moveit_manage_controllers': False,  # 关键修改：不管理controllers
    ...
}
```

**作用**: 告诉MoveIt2不要等待/管理controllers，纯规划模式。

### 2. 明确指定规划参数

在 `button_actions.py` 中添加:
```python
goal.request.pipeline_id = 'ompl'           # 明确规划管道
goal.request.planner_id = PLANNER_ID        # 明确规划器
goal.planning_options.look_around = False   # 不扫描环境
goal.planning_options.replan = False        # 不重新规划
```

**作用**: 避免不必要的检查和延迟，直接进入规划。

### 3. 重新构建MoveIt2包

```bash
cd /home/robot/button/V4.0/project2/piper_ros
source /opt/ros/foxy/setup.bash
colcon build --packages-select piper_with_gripper_moveit
```

## 测试步骤

### 方法1: 使用现有脚本

**终端1 (启动MoveIt2):**
```bash
cd /home/robot/button/V4.0/project2
./start_moveit2_clean.sh
```

等待看到:
```
[move_group-3] You can start planning now!
```

**终端2 (运行测试):**
```bash
cd /home/robot/button/V4.0/project2
./test_moveit2_full.sh
```

### 方法2: 手动测试

**终端1:**
```bash
# 清理环境
for var in $(env | grep -E '^(ROS_|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH)' | cut -d'=' -f1); do unset $var; done

# 启动MoveIt2
source /opt/ros/foxy/setup.bash
source ~/button/V4.0/project2/piper_ros/install/setup.bash
ros2 launch piper_with_gripper_moveit demo_foxy.launch.py
```

**终端2:**
```bash
# 清理环境
for var in $(env | grep -E '^(ROS_|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH)' | cut -d'=' -f1); do unset $var; done

# 运行测试
cd ~/button/V4.0/project2
source /opt/ros/foxy/setup.bash
source piper_ros/install/setup.bash
python3 button_actions.py
```

## 预期结果

### 成功的标志:

**终端1 (MoveIt2服务器):**
```
[move_group-3] [INFO] Received request
[move_group-3] [INFO] executing..
[move_group-3] [INFO] Planning request received for MoveGroup action
[move_group-3] [INFO] Planner found a valid solution
[move_group-3] [INFO] Returning planning result
```

**终端2 (客户端):**
```
[MoveIt2] 发送规划请求...
✓ 规划请求已接受，等待规划结果...
✓ 规划成功！
📊 轨迹点数: XX
[MoveIt2] 使用SDK执行轨迹...
```

### 如果仍然超时:

1. **检查终端1是否有新的错误信息** (特别是[move_group-3]开头的行)
2. **确认controller管理已禁用**:
   ```bash
   grep "moveit_manage_controllers" piper_ros/src/piper_moveit/piper_with_gripper_moveit/launch/demo_foxy.launch.py
   ```
   应该看到: `'moveit_manage_controllers': False`

3. **查看规划器配置**:
   ```bash
   ros2 param get /move_group planning_plugin
   ```
   应该返回: `ompl_interface/OMPLPlanner`

## 技术细节

### 为什么test_moveit.py能"成功"?

因为它**只等待goal acceptance，不等待result**:
```python
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
# future是send_goal_async()的返回值，只等待goal被接受
# 没有调用goal_handle.get_result_async()等待规划完成
```

### button_actions.py的完整流程:

1. `send_goal_async()` → 等待goal被接受 ✓
2. `goal_handle.get_result_async()` → 等待规划结果 ✗ (之前在这里超时)
3. 解析trajectory，使用SDK执行

### moveit_manage_controllers的作用:

- `True`: MoveIt2管理controllers，会等待controllers连接才返回结果
- `False`: MoveIt2仅负责规划，不检查controllers，立即返回轨迹

## 调试命令

```bash
# 检查action server状态
ros2 action list
ros2 action info /move_action

# 查看move_group参数
ros2 param list /move_group
ros2 param get /move_group allow_trajectory_execution
ros2 param get /move_group moveit_manage_controllers

# 查看日志级别
ros2 run rqt_console rqt_console

# 检查topics
ros2 topic list | grep move_group
```

## 文件修改总结

1. **demo_foxy.launch.py** (已修改)
   - `moveit_manage_controllers: False`

2. **button_actions.py** (已修改)
   - 添加 `pipeline_id`, `planner_id`
   - 添加 `look_around=False`, `replan=False`

3. **已重新编译** piper_with_gripper_moveit 包

## 下一步

如果此次修改成功解决超时问题:
- ✅ MoveIt2规划功能正常
- ✅ 可以获取完整轨迹
- ✅ 使用SDK执行轨迹

如果仍有问题，请提供:
1. 终端1的**完整输出** (从启动到超时)
2. 终端2的输出
3. `ros2 param list /move_group` 的结果
