# MoveIt2 测试路线图

## 📋 测试概述

本文档提供 MoveIt2 实现的完整测试流程，分为 3 个级别。

---

## 🎯 测试级别

### 级别 1: 静态测试（无需机器人）✅

**目的**: 验证代码结构和环境配置

**耗时**: 1-2 分钟

**步骤**:
```bash
cd /home/robot/button/V4.0/project2

# 测试 1: 代码结构验证
python3 test_moveit2_function.py

# 预期输出:
# 通过: 4/4
# 🎉 所有测试通过！MoveIt2 函数已成功迁移到 ROS2
```

**检查项目**:
- ✅ button_actions 导入
- ✅ MoveIt2 模块可用
- ✅ 函数签名正确
- ✅ ROS2 API 就位
- ✅ 无 ROS1 代码残留

---

### 级别 2: 连接测试（需要 MoveIt2 运行）⏳

**目的**: 验证 MoveIt2 服务连接

**耗时**: 3-5 分钟

**准备工作**:
```bash
# 1. 启动 MoveIt2（如果尚未运行）
./start_moveit2.sh --background

# 等待 10-15 秒让 MoveIt2 完全启动
# 看到 "You can start planning now!" 表示成功
```

**执行测试**:
```bash
# 测试 2: MoveIt2 连接测试
python3 test_moveit2_connection.py

# 预期输出:
# [6/6] 等待 MoveIt2 Action Server...
#   ✓ MoveIt2 Action Server 已连接！
# 🎉 MoveIt2 连接测试通过！
```

**检查项目**:
- ✅ ROS2 环境正确
- ✅ rclpy 可用
- ✅ moveit_msgs 可用
- ✅ ROS2 节点创建成功
- ✅ Action Client 创建成功
- ✅ MoveIt2 服务器响应

**故障排查**:

如果看到 "❌ MoveIt2 Action Server 未响应":

```bash
# 检查 MoveIt2 进程
ps aux | grep move_group

# 检查 ROS2 节点
source /opt/ros/foxy/setup.bash
ros2 node list

# 检查 action 服务
ros2 action list

# 如果没有 /move_action，重启 MoveIt2
./restart_moveit.sh
```

---

### 级别 3: 运动测试（需要真实机器人）⚠️

**⚠️ 警告**: 此测试会让机器人运动！

**目的**: 验证实际控制功能

**耗时**: 5-10 分钟

**安全准备**:
- ✅ 机器人已通电
- ✅ 机器人已使能
- ✅ 周围环境安全（无人员、无障碍物）
- ✅ 准备好急停
- ✅ 机器人处于安全位置（零位或教学点）

**执行测试**:
```bash
# 测试 3: 完整运动测试
python3 test_moveit2_realtime.py

# 程序会询问确认:
# "继续测试? (y/N):"
# 输入 'y' 继续
```

**测试流程**:

1. **环境检查** (自动)
   - ROS2 环境
   - Python 包
   - MoveIt2 服务
   - 机器人连接

2. **获取当前状态** (自动)
   - 读取关节角度
   - 显示当前位置

3. **SDK 控制测试** (需确认)
   - 小幅运动 (+2.8°)
   - 验证到达精度
   - 返回初始位置

4. **MoveIt2 控制测试** (需确认)
   - 小幅运动 (-2.8°)
   - 验证到达精度
   - 返回初始位置

5. **性能对比** (可选)
   - SDK vs MoveIt2
   - 时间对比
   - 精度对比

**预期输出**:
```
测试总结
======================================================================
  ✓ SDK 控制: 通过
  ✓ MoveIt2 控制: 通过

🎉 所有测试通过！MoveIt2 集成工作正常
```

---

## 🚀 快速测试（推荐）

使用自动化脚本：

```bash
# 一键测试（包含静态和连接测试）
chmod +x test_moveit2.sh
./test_moveit2.sh

# 脚本会:
# 1. 运行静态测试
# 2. 检查环境
# 3. 询问是否进行运动测试
# 4. (可选) 运行完整测试
```

---

## 📊 测试结果解读

### 成功情况

#### 1. 完全成功 🎉
```
✓ SDK 控制: 通过
✓ MoveIt2 控制: 通过
```
**含义**: MoveIt2 集成完全工作
**下一步**: 可以在应用中使用 MoveIt2

#### 2. 部分成功 ✓
```
✓ SDK 控制: 通过
⚠️  MoveIt2 控制: 未测试或失败
```
**含义**: SDK 正常，MoveIt2 需要检查
**可能原因**:
- MoveIt2 未启动
- MoveIt2 未初始化
- 规划失败（目标不可达）

**解决方案**: 查看日志，检查 MoveIt2 状态

### 失败情况

#### 3. SDK 失败 ❌
```
❌ SDK 控制: 失败
```
**含义**: 基础通信问题
**可能原因**:
- 机器人未连接
- 机器人未使能
- 串口问题

**解决方案**: 检查机器人连接

#### 4. MoveIt2 连接失败 ❌
```
❌ MoveIt2 Action Server 未响应
```
**含义**: MoveIt2 服务未运行或无法连接
**解决方案**:
```bash
./start_moveit2.sh --background
# 等待 15 秒
python3 test_moveit2_connection.py
```

---

## 🔧 常见问题

### Q1: "MoveIt2 未初始化"

**现象**:
```
⚠️  MoveIt2 未初始化，回退到 SDK 模式
```

**原因**: `control_arm_moveit()` 调用时 `move_group` 为 None

**解决方案**:

方法 1 - 使用集成脚本（推荐）:
```bash
./start_vision_button_moveit.sh --moveit
```

方法 2 - 手动初始化:
```python
import button_actions
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup

rclpy.init()
node = rclpy.create_node('test_node')
client = ActionClient(node, MoveGroup, '/move_action')

button_actions.initialize_moveit(node, client)
```

### Q2: "规划失败"

**现象**:
```
❌ 规划失败 (错误码: -1)，切换到SDK模式
```

**可能原因**:
1. 目标位置超出关节限制
2. 目标位置导致自碰撞
3. 逆运动学无解

**调试**:
```python
# 检查目标关节角度
import button_actions
joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 零位

# 先用 SDK 测试是否可达
success = button_actions.control_arm_sdk(joints, speed=30)

# 如果 SDK 可达，再用 MoveIt2
success = button_actions.control_arm_moveit(joints, speed=30)
```

### Q3: 运动不流畅

**现象**: 机器人运动抖动或停顿

**可能原因**:
- 速度设置过快
- CPU 负载过高
- 网络延迟

**优化**:
```python
# 降低速度
success = button_actions.control_arm_moveit(
    joints,
    speed=30,  # 从 50 降到 30
)
```

---

## 📝 测试检查清单

运行测试前检查：

**环境准备**:
- [ ] ROS2 环境已 source (`source /opt/ros/foxy/setup.bash`)
- [ ] 当前目录为 `/home/robot/button/V4.0/project2`
- [ ] MoveIt2 已启动（级别 2、3 需要）

**机器人准备**（级别 3 需要）:
- [ ] 机器人已通电
- [ ] 机器人已使能（运行过 `demo_02_enable_arm_ros2.py`）
- [ ] 机器人在安全位置
- [ ] 周围环境安全

**测试文件**:
- [ ] `test_moveit2_function.py` 存在
- [ ] `test_moveit2_connection.py` 存在
- [ ] `test_moveit2_realtime.py` 存在
- [ ] `test_moveit2.sh` 存在且可执行

---

## 🎓 测试最佳实践

### 推荐顺序

1. **首次测试**: 
   ```bash
   # 静态 → 连接 → 手动
   python3 test_moveit2_function.py
   python3 test_moveit2_connection.py
   # 然后手动小幅测试
   ```

2. **日常验证**:
   ```bash
   # 使用自动化脚本
   ./test_moveit2.sh
   ```

3. **问题排查**:
   ```bash
   # 分步骤测试
   python3 test_moveit2_function.py  # 代码
   python3 test_moveit2_connection.py  # 连接
   # 查看详细日志
   ```

### 安全建议

- ✅ 始终从静态测试开始
- ✅ 首次运动测试使用小幅度 (< 5°)
- ✅ 保持急停准备
- ✅ 使用低速度 (speed=20-30)
- ✅ 一次测试一个功能
- ⚠️ 不要在生产环境首次测试

---

## 📞 获取帮助

### 查看日志

```bash
# MoveIt2 日志
ros2 node list
ros2 topic echo /move_action/feedback

# Python 错误
# 程序会自动打印 traceback
```

### 重启服务

```bash
# 重启 MoveIt2
./restart_moveit.sh

# 重启机器人（如需要）
python3 demo_02_enable_arm_ros2.py
```

### 手动验证

```bash
# 验证 action 服务
ros2 action list
ros2 action info /move_action

# 验证节点
ros2 node list
ros2 node info /move_group
```

---

## ✅ 测试完成后

### 如果测试通过 🎉

1. **集成到应用**:
   ```bash
   ./start_vision_button_moveit.sh --moveit
   ```

2. **实际使用**:
   - 在视觉窗口点击按钮
   - 观察机器人使用 MoveIt2 规划
   - 对比 SDK 模式的差异

3. **性能优化**:
   - 调整速度参数
   - 调整规划参数
   - 记录性能数据

### 如果测试失败 ❌

1. **查看错误信息**
2. **参考"常见问题"章节**
3. **运行诊断命令**
4. **查看详细日志**

---

## 📚 相关文档

- `MOVEIT2_CORE_MIGRATION_COMPLETE.md` - 详细技术文档
- `MOVEIT2_QUICK_START.md` - 快速使用指南
- `MOVEIT2_INTEGRATION_COMPLETE.md` - 集成文档

---

**准备好开始测试了吗？** 

从最简单的开始：
```bash
python3 test_moveit2_function.py
```
