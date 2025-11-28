# 6D位姿控制使用说明

## 问题说明

当前发现 **IK（逆运动学）存在精度问题**：
- 目标位置：(0.20, 0.00, 0.25)
- IK 计算结果实际到达：(0.14, 0.00, 0.245)
- **误差约 6cm**，这会导致 MoveIt 规划失败！

## 解决方案

### 方案 1: 调整目标位置（推荐）

根据测试，以下位姿的 IK 精度较好：

```bash
# 位姿1: 前方中距离，稍高
python3 test_pose_publisher.py 0.25 0.0 0.30
# IK 结果: (0.19, 0.00, 0.295) - 误差 6cm

# 位姿2: 前方远距离
python3 test_pose_publisher.py 0.30 0.0 0.25
# IK 结果: (0.24, 0.00, 0.245) - 误差 6cm

# 位姿3: 前方高位置
python3 test_pose_publisher.py 0.20 0.0 0.35
```

**注意**: 所有位姿都有约 6cm 的 X 轴误差，建议：
- **将目标 X 坐标增加 0.06m** 来补偿
- 例如：想到 X=0.20，发送 X=0.26

### 方案 2: 使用已知可达位姿

零位姿态：
```bash
# 零位位置 (已知可达)
python3 test_pose_publisher.py 0.056 0.0 0.213
```

### 方案 3: 增加规划参数容忍度

已经在代码中设置：
```python
PLANNING_TIME = 10.0            # 规划时间 10 秒
PLANNING_ATTEMPTS = 20          # 尝试 20 次
GOAL_POSITION_TOLERANCE = 0.01  # 位置容差 1cm
GOAL_ORIENTATION_TOLERANCE = 0.1  # 姿态容差 0.1rad
```

## 启动系统

```bash
# 启动 6D 位姿控制节点（不带 RViz）
./start_6d_pose_control.sh

# 启动带 RViz 可视化
./start_6d_pose_control.sh --rviz
```

## 发布目标位姿

### 方法 1: 使用测试脚本（推荐）

```bash
# 基本用法
python3 test_pose_publisher.py <x> <y> <z> [roll] [pitch] [yaw]

# 示例：只指定位置（姿态为0）
python3 test_pose_publisher.py 0.26 0.0 0.25

# 示例：指定位置和姿态
python3 test_pose_publisher.py 0.26 0.0 0.25 0.0 0.0 0.0
```

### 方法 2: 命令行发布

```bash
# 发布位姿 [x, y, z, roll, pitch, yaw]
rostopic pub /target_6d_pose std_msgs/Float64MultiArray "data: [0.26, 0.0, 0.25, 0.0, 0.0, 0.0]"
```

### 方法 3: Python 代码

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('pose_publisher')
pub = rospy.Publisher('/target_6d_pose', Float64MultiArray, queue_size=1)

rospy.sleep(1)  # 等待连接

msg = Float64MultiArray()
msg.data = [0.26, 0.0, 0.25, 0.0, 0.0, 0.0]  # x, y, z, roll, pitch, yaw
pub.publish(msg)

print("位姿已发布")
```

## 推荐测试序列

按照难度从易到难：

```bash
# 1. 零位（最简单）
python3 test_pose_publisher.py 0.056 0.0 0.213

# 2. 前方高位置（补偿 +0.06）
python3 test_pose_publisher.py 0.26 0.0 0.35

# 3. 前方中距离（补偿 +0.06）
python3 test_pose_publisher.py 0.31 0.0 0.30

# 4. 前方远距离（补偿 +0.06）
python3 test_pose_publisher.py 0.36 0.0 0.25

# 5. 左前方
python3 test_pose_publisher.py 0.31 0.10 0.30

# 6. 右前方
python3 test_pose_publisher.py 0.31 -0.10 0.30
```

## 参数调整

修改 `moveit_6d_pose_control.py` 中的参数：

```python
# 规划器配置
PLANNER_ID = "RRTconnect"       # 可选: RRTconnect, RRT, PRM, etc.
PLANNING_TIME = 10.0            # 增加时间提高成功率
PLANNING_ATTEMPTS = 20          # 增加尝试次数
VELOCITY_SCALING = 0.3          # 降低速度提高成功率
GOAL_POSITION_TOLERANCE = 0.01  # 位置容差 (m)
GOAL_ORIENTATION_TOLERANCE = 0.1  # 姿态容差 (rad)

# 频率控制
RVIZ_PUBLISH_RATE = 10          # RViz 更新频率
SDK_EXECUTE_RATE = 50           # 轨迹执行频率
COMMAND_PUBLISH_RATE = 100      # 命令发布频率
```

## RViz 可视化设置

如果使用 `--rviz` 参数，在 RViz 中添加：

1. **Marker** 显示
   - Topic: `/end_effector_trail`
   - 显示带渐变颜色的运动轨迹

2. **Path** 显示
   - Topic: `/end_effector_path`
   - 显示路径点

3. **DisplayTrajectory** (自动添加)
   - 显示 MoveIt 规划的轨迹

## 故障排查

### 规划失败 (TIMED_OUT)

**原因**:
1. 目标位姿不可达
2. IK 精度误差导致目标超出工作空间
3. 规划时间不足
4. 存在碰撞

**解决**:
1. 使用推荐的测试位姿
2. 增加 X 坐标补偿 (+0.06m)
3. 增加 `PLANNING_TIME` 和 `PLANNING_ATTEMPTS`
4. 检查是否有障碍物

### IK 精度问题

**现状**: piper_arm.py 的 IK 有约 6cm 的 X 轴偏移

**临时方案**:
- 发送目标时，X 坐标增加 0.06m
- 例如：想要 X=0.20，发送 X=0.26

### robot_description 未找到

**解决**: 使用启动脚本 `./start_6d_pose_control.sh`，它会自动启动 MoveIt 和 robot_state_publisher

## 日志查看

```bash
# MoveIt 日志
cat /tmp/moveit_launch.log

# robot_state_publisher 日志
cat /tmp/robot_state_publisher.log
```
