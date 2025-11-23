# MoveIt2 集成快速使用指南

## 🚀 快速开始

### 1. 启动 MoveIt2（如果尚未运行）

```bash
cd /home/robot/button/V4.0/project2

# 后台启动 MoveIt2
./start_moveit2.sh --background
```

### 2. 启动视觉按钮系统（带 MoveIt2）

```bash
# 方法 1: 使用一键启动脚本（推荐）
./start_vision_button_moveit.sh --moveit

# 方法 2: 手动启动
python3 vision_button_action_ros2.py --moveit
```

### 3. 测试 MoveIt2 功能

```bash
# 测试导入和函数结构
python3 test_moveit2_function.py

# 测试完整集成（需要机器人连接）
python3 test_integration_quick.py
```

---

## 📋 使用示例

### Python 代码示例

```python
import button_actions

# 初始化 MoveIt2（在启动脚本中完成）
# button_actions.initialize_moveit(node, move_group_client)

# 示例 1: 移动到特定关节位置
joints = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
success = button_actions.control_arm_moveit(
    joints,
    speed=50,
    gripper_value=500000
)

# 示例 2: 使用统一接口（自动选择 MoveIt2 或 SDK）
success = button_actions.control_arm(
    joints,
    speed=50,
    use_moveit=True,  # 启用 MoveIt2
    gripper_value=500000
)

# 示例 3: 检查 MoveIt2 状态
if button_actions.MOVEIT_AVAILABLE:
    print("✓ MoveIt2 可用")
else:
    print("⚠️  MoveIt2 不可用")
```

---

## 🔧 配置选项

### 命令行参数

```bash
# 启用 MoveIt2 规划
python3 vision_button_action_ros2.py --moveit

# 不使用 MoveIt2（仅 SDK）
python3 vision_button_action_ros2.py
```

### 运行时选择

```python
# 在代码中选择使用 MoveIt2 或 SDK
use_moveit = True   # 使用 MoveIt2
use_moveit = False  # 使用 SDK

success = button_actions.control_arm(
    joints,
    speed=50,
    use_moveit=use_moveit
)
```

---

## ⚙️ 参数说明

### `control_arm_moveit()` 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `joints` | list[float] | 必需 | 目标关节角度（弧度），6个元素 |
| `speed` | int | 50 | 速度百分比 (0-100) |
| `gripper_value` | int/None | None | 夹爪开度（0-1000000） |

### 返回值
- `True` - 执行成功
- `False` - 执行失败（会自动回退到 SDK 模式）

---

## 🎯 功能特性

### ✅ 已实现
- [x] 关节空间规划
- [x] 速度控制
- [x] 夹爪控制
- [x] 自动 SDK 回退
- [x] 错误处理和恢复
- [x] 轨迹提取和记录
- [x] 详细日志输出

### ⏳ 计划中
- [ ] 笛卡尔空间规划（`move_along_end_effector_z()`）
- [ ] 碰撞检测
- [ ] RViz 可视化增强
- [ ] 多目标规划

---

## 🐛 故障排查

### 问题 1: "MoveIt2 未初始化"

**错误信息**:
```
⚠️  MoveIt2 未初始化，回退到 SDK 模式
```

**解决方案**:
```bash
# 1. 检查 MoveIt2 是否运行
ros2 action list | grep move_action

# 2. 重启 MoveIt2
./restart_moveit.sh

# 3. 使用 --moveit 参数启动
python3 vision_button_action_ros2.py --moveit
```

### 问题 2: "规划失败"

**错误信息**:
```
❌ 规划失败 (错误码: X)
```

**常见原因和解决方案**:

| 错误码 | 原因 | 解决方案 |
|--------|------|----------|
| -1 | 规划失败 | 检查目标是否可达 |
| -2 | 超时 | 增加规划时间 |
| -31 | 起始状态冲突 | 检查机器人状态 |

**调试步骤**:
```python
# 1. 检查目标关节角度
joints = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
print("目标关节:", joints)

# 2. 获取当前关节
current = button_actions.get_current_joints()
print("当前关节:", current)

# 3. 尝试使用 SDK 模式
success = button_actions.control_arm_sdk(joints, speed=50)
```

### 问题 3: 执行缓慢

**可能原因**:
- 速度参数设置过低
- 规划时间过长
- 网络延迟

**优化方法**:
```python
# 增加速度
success = button_actions.control_arm_moveit(
    joints,
    speed=80,  # 提高到 80%
)

# 或调整规划参数（需要修改代码）
goal.request.num_planning_attempts = 5  # 减少尝试次数
goal.request.allowed_planning_time = 2.0  # 减少规划时间
```

---

## 📊 性能对比

| 模式 | 响应时间 | CPU 使用 | 平滑度 | 可靠性 |
|------|----------|----------|--------|--------|
| MoveIt2 | 中等 | 低 | 高 | 高 |
| SDK | 快速 | 中等 | 中等 | 中等 |
| MoveIt2+SDK回退 | 自适应 | 中等 | 高 | 最高 |

**推荐配置**:
- **精确任务**: 使用 MoveIt2（规划更优）
- **快速响应**: 使用 SDK（直接控制）
- **生产环境**: 使用 MoveIt2 + SDK 回退（最可靠）

---

## 📝 日志解读

### 正常执行日志

```
[MoveIt2] 规划轨迹...
  📍 起始点 (弧度): [0.0000, -0.5000, 0.5000, 0.0000, 0.5000, 0.0000]
  📍 目标点 (弧度): [0.5000, -0.3000, 0.6000, 0.1000, 0.4000, 0.2000]
[MoveIt2] 发送规划请求...
  ✓ 规划请求已接受，等待规划结果...
  ✓ 规划和执行成功！
  📊 轨迹点数: 42
  ✓ 已提取规划轨迹的末端XYZ (本步骤: 42个点, 累计: 42个点)
  ⏳ 等待机械臂到达目标位置...
  ✓ 机械臂已到达目标位置 (最大误差: 0.00123 rad)
  ✓ MoveIt2 执行完成
```

### 回退到 SDK 的日志

```
[MoveIt2] 规划轨迹...
  ❌ 规划失败 (错误码: -1)，切换到SDK模式
  [SDK] 使用SDK直接控制
  ✓ SDK执行完成
```

---

## 🔗 相关文件

- `button_actions.py` - 核心实现
- `vision_button_action_ros2.py` - ROS2 节点
- `start_vision_button_moveit.sh` - 一键启动脚本
- `test_moveit2_function.py` - 单元测试
- `MOVEIT2_CORE_MIGRATION_COMPLETE.md` - 详细文档

---

## 📞 获取帮助

### 查看日志
```bash
# ROS2 日志
ros2 node list
ros2 topic list
ros2 action list

# 检查 MoveIt2 状态
ros2 action info /move_action
```

### 运行测试
```bash
# 基础测试
python3 test_moveit2_function.py

# 集成测试
python3 test_integration_quick.py
```

### 重启服务
```bash
# 重启 MoveIt2
./restart_moveit.sh

# 完全重启
./start_all_moveit.sh
```

---

## ✅ 检查清单

在使用 MoveIt2 之前，确保：

- [ ] ROS2 环境已 source
- [ ] MoveIt2 已启动（`./start_moveit2.sh --background`）
- [ ] 机器人已连接并使能
- [ ] 关节角度在安全范围内
- [ ] 使用 `--moveit` 参数启动应用

---

**准备好了吗？开始使用 MoveIt2！** 🎉

```bash
./start_vision_button_moveit.sh --moveit
```
