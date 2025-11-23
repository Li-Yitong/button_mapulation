# MoveIt2 与视觉按钮系统集成指南

## 概述

成功将 MoveIt2 集成到视觉按钮操作系统中，支持两种控制模式：
1. **SDK 直接控制模式**：直接发送关节命令（原有方式）
2. **MoveIt2 规划模式**：使用路径规划和碰撞检测（新增）

---

## 集成架构

```
┌─────────────────────────────────────────────────────────────┐
│                    视觉按钮系统                              │
├─────────────────────────────────────────────────────────────┤
│  1. realsense_yolo_button_interactive_ros2.py               │
│     └─> 发布: /object_point, /button_type                   │
│                                                              │
│  2. vision_button_action_ros2.py (主控节点)                 │
│     ├─> 订阅: /object_point, /button_type                   │
│     ├─> 初始化: Piper SDK, PiperArm, MoveIt2 (可选)        │
│     └─> 调用: button_actions.py                             │
│                                                              │
│  3. button_actions.py (动作执行)                            │
│     ├─> SDK 模式: 直接关节控制                              │
│     └─> MoveIt2 模式: 路径规划 + 碰撞检测                  │
│                                                              │
│  4. MoveIt2 (可选)                                           │
│     ├─> move_group 节点                                      │
│     ├─> OMPL 规划器                                          │
│     └─> RViz2 可视化 (可选)                                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 修改内容

### 1. `button_actions.py`

**修改点**：
- 更新 MoveIt 导入为 ROS2 兼容版本
- 添加 `moveit_node` 全局变量（ROS2 节点引用）
- 保持 `control_arm_moveit()` 函数（待适配 ROS2 API）

```python
# 旧版 (ROS1)
import moveit_commander

# 新版 (ROS2)
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup as MoveGroupAction
from rclpy.action import ActionClient
```

### 2. `vision_button_action_ros2.py`

**新增功能**：

#### a. 构造函数支持 MoveIt2
```python
def __init__(self, enable_moveit=False):
    # ...
    self.enable_moveit = enable_moveit
```

#### b. MoveIt2 初始化函数
```python
def initialize_moveit(self):
    """初始化 MoveIt2 action client"""
    self._moveit_action_client = ActionClient(
        self,
        MoveGroupAction,
        '/move_action'
    )
    # 等待 move_group server
    # 传递给 button_actions
```

#### c. 命令行参数支持
```python
parser.add_argument('--moveit', action='store_true',
                   help='启用 MoveIt2 规划')
```

### 3. 新启动脚本 `start_vision_button_moveit.sh`

**功能**：
- 自动启动 MoveIt2（后台）
- 启动 TF 发布器
- 启动视觉检测
- 启动按钮操作执行器
- 优雅的进程清理

**用法**：
```bash
./start_vision_button_moveit.sh           # SDK 模式
./start_vision_button_moveit.sh --moveit  # MoveIt2 模式
./start_vision_button_moveit.sh --moveit --rviz  # MoveIt2 + RViz2
```

---

## 使用方法

### 方法 1: SDK 直接控制（原有方式）

```bash
# 终端 1: 启动视觉检测
cd /home/robot/button/V4.0/project2
source ./setup_ros2_clean.sh
python3 realsense_yolo_button_interactive_ros2.py

# 终端 2: 启动按钮操作（SDK 模式）
cd /home/robot/button/V4.0/project2
source ./setup_ros2_clean.sh
python3 vision_button_action_ros2.py
```

### 方法 2: MoveIt2 规划模式（新增）

```bash
# 终端 1: 启动 MoveIt2
cd /home/robot/button/V4.0/project2
./start_moveit2.sh --background

# 终端 2: 启动视觉检测
source ./setup_ros2_clean.sh
python3 realsense_yolo_button_interactive_ros2.py

# 终端 3: 启动按钮操作（MoveIt2 模式）
source ./setup_ros2_clean.sh
python3 vision_button_action_ros2.py --moveit
```

### 方法 3: 一键启动（推荐）

```bash
# SDK 模式
./start_vision_button_moveit.sh

# MoveIt2 模式（无可视化）
./start_vision_button_moveit.sh --moveit

# MoveIt2 模式（带 RViz2）
./start_vision_button_moveit.sh --moveit --rviz
```

---

## 待完成工作

### 1. 适配 `control_arm_moveit()` 函数

**当前状态**：函数存在但使用 ROS1 API

**需要修改**：
- 将 `moveit_commander.MoveGroupCommander` 改为 ROS2 action client
- 轨迹规划：使用 `MoveGroup.action` 的 `send_goal_async()`
- 轨迹执行：通过 action 结果获取执行状态

**参考代码**：
```python
def control_arm_moveit(joints, speed=50, gripper_value=None):
    """MoveIt 规划控制模式 (ROS2)"""
    global moveit_node, move_group  # move_group 是 ActionClient
    
    # 创建规划目标
    goal = MoveGroupAction.Goal()
    goal.request.group_name = 'arm'
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    
    # 设置目标关节角度
    joint_constraints = []
    for i, angle in enumerate(joints):
        constraint = JointConstraint()
        constraint.joint_name = f'joint{i+1}'
        constraint.position = angle
        constraint.tolerance_above = 0.01
        constraint.tolerance_below = 0.01
        constraint.weight = 1.0
        joint_constraints.append(constraint)
    
    goal.request.goal_constraints = [Constraints(joint_constraints=joint_constraints)]
    
    # 发送规划请求
    future = move_group.send_goal_async(goal)
    rclpy.spin_until_future_complete(moveit_node, future)
    
    # 处理结果...
```

### 2. 创建无 RViz 的 MoveIt2 launch 文件

**目标**：节省资源，仅启动 move_group

**方法**：复制 `demo_foxy.launch.py`，注释掉 RViz2 节点

```python
# demo_foxy_no_rviz.launch.py
# 注释掉：
# rviz_node = Node(...)
# return LaunchDescription([..., rviz_node])
```

### 3. 轨迹可视化适配

**当前状态**：使用 ROS1 的 `rospy.Publisher`

**需要修改**：
```python
# 旧版
marker_pub = rospy.Publisher('/trajectory_comparison', Marker, queue_size=10)

# 新版
marker_pub = moveit_node.create_publisher(Marker, '/trajectory_comparison', 10)
```

---

## 测试步骤

### 测试 1: SDK 模式（验证原有功能）

```bash
# 1. 启动系统
./start_vision_button_moveit.sh

# 2. 在视觉窗口中点击按钮

# 3. 观察日志
# 应该看到: "🔧 SDK 直接控制模式"
# 动作应该正常执行
```

### 测试 2: MoveIt2 模式（验证新功能）

```bash
# 1. 启动系统
./start_vision_button_moveit.sh --moveit

# 2. 等待 MoveIt2 启动（约15秒）

# 3. 验证 move_group
ros2 node list | grep move_group

# 4. 在视觉窗口中点击按钮

# 5. 观察日志
# 应该看到: "🤖 MoveIt2 模式已启用"
# 如果 control_arm_moveit() 未完成，会回退到 SDK 模式
```

### 测试 3: MoveIt2 连接测试

```bash
# 1. 启动 MoveIt2
./start_moveit2.sh --background

# 2. 测试连接
python3 test_moveit2_simple.py

# 应该显示: "✅ MoveIt2 运行正常!"
```

---

## 优势对比

### SDK 直接控制模式
✅ **优点**：
- 快速响应
- 实现简单
- 资源占用低

❌ **缺点**：
- 无碰撞检测
- 直线轨迹（可能奇异）
- 无障碍物规避

### MoveIt2 规划模式
✅ **优点**：
- 自动碰撞检测
- 平滑轨迹规划
- 支持障碍物规避
- 关节限制检查
- 可视化支持

❌ **缺点**：
- 规划耗时（~1-3秒）
- 资源占用高
- 实现复杂

---

## 故障排查

### 问题 1: MoveIt2 启动失败

**症状**：
```
⚠️  警告: move_group 节点未检测到
```

**排查**：
```bash
# 检查 MoveIt2 日志
tail -f /tmp/moveit2_vision.log

# 手动启动测试
ros2 launch piper_with_gripper_moveit demo_foxy.launch.py

# 检查节点
ros2 node list
```

### 问题 2: Action client 连接失败

**症状**：
```
✗ move_group action server 未响应
```

**解决**：
```bash
# 1. 确认 move_group 运行
ros2 node info /move_group

# 2. 检查 action server
ros2 action list

# 3. 延长等待时间
# 修改 initialize_moveit() 中的 timeout_sec=10.0
```

### 问题 3: 规划失败回退到 SDK

**症状**：
```
MoveIt规划失败，回退到SDK模式
```

**原因**：`control_arm_moveit()` 函数未完成适配

**临时方案**：使用 SDK 模式（功能正常）

---

## 下一步计划

1. **优先级 1**：完成 `control_arm_moveit()` ROS2 适配
2. **优先级 2**：创建无 RViz 的 launch 文件
3. **优先级 3**：适配轨迹可视化功能
4. **优先级 4**：添加碰撞物体（桌面、按钮面板）
5. **优先级 5**：优化规划参数（速度、精度）

---

## 文件清单

**新增文件**：
- `start_vision_button_moveit.sh` - 集成启动脚本
- `MOVEIT2_INTEGRATION.md` - 本文档

**修改文件**：
- `button_actions.py` - 更新 MoveIt 导入
- `vision_button_action_ros2.py` - 添加 MoveIt2 支持

**相关文件**：
- `start_moveit2.sh` - MoveIt2 独立启动脚本
- `test_moveit2_simple.py` - MoveIt2 连接测试
- `MOVEIT2_SUCCESS_REPORT.md` - MoveIt2 移植报告

---

*生成时间: 2025-11-22*  
*状态: 集成框架完成，待适配 ROS2 API*
