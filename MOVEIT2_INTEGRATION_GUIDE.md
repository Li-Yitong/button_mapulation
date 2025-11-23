# MoveIt2 集成测试指南

## ✅ 已完成的工作

1. **成功安装 MoveIt2**：ROS2 Foxy 版本
2. **成功移植配置**：从 Humble 分支移植了 MoveIt2 配置
3. **编译成功**：`piper_with_gripper_moveit` 和 `piper_no_gripper_moveit` 包已编译
4. **更新启动脚本**：添加了 `--moveit` 选项

## 📋 测试步骤

### 测试 1：验证 MoveIt2 包安装

```bash
source /home/robot/button/V4.0/project2/setup_ros2_clean.sh
source /home/robot/button/V4.0/project2/piper_ros/install/setup.bash
ros2 pkg list | grep moveit
```

**预期输出**：应该看到 `piper_with_gripper_moveit` 和 `piper_no_gripper_moveit`

### 测试 2：启动 MoveIt2 Demo（模拟模式）

在新终端中：

```bash
cd /home/robot/button/V4.0/project2
source setup_ros2_clean.sh
source piper_ros/install/setup.bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

这将启动：
- MoveIt2 运动规划服务器
- RViz2 可视化
- 虚拟机械臂（fake hardware）

**验证**：
- RViz2 窗口应该打开
- 可以看到 Piper 机械臂模型
- 可以用 MotionPlanning 插件拖动末端执行器

### 测试 3：完整系统测试（SDK + 视觉）

使用更新后的启动脚本：

```bash
cd /home/robot/button/V4.0/project2
./start_vision_button_ros2.sh --moveit --rviz
```

或者使用 SDK 模式（无 MoveIt2）：

```bash
./start_vision_button_ros2.sh --sdk-only
```

## 🔧 配置文件说明

### 关键文件位置

```
piper_ros/src/piper_moveit/piper_with_gripper_moveit/
├── config/
│   ├── piper.srdf                 # 语义机器人描述（规划组、姿态等）
│   ├── kinematics.yaml            # 运动学插件配置（KDL）
│   ├── joint_limits.yaml          # 关节速度/加速度限制
│   ├── moveit_controllers.yaml    # 控制器配置
│   └── ros2_controllers.yaml      # ROS2 控制器
└── launch/
    ├── demo.launch.py             # Demo 模式（虚拟硬件）
    └── piper_moveit.launch.py     # 真实硬件模式
```

### 规划组配置 (SRDF)

- **arm**: 包含 joint1 到 joint6（6 DOF 机械臂）
- **gripper**: 包含 joint7（夹爪）

### 预定义姿态

- **zero**: 零位（所有关节归零）
- **open**: 夹爪张开（0.035m）
- **close**: 夹爪闭合（0m）

## 🎯 MoveIt2 vs SDK 模式对比

| 特性 | MoveIt2 模式 | SDK 模式 |
|------|-------------|---------|
| 路径规划 | ✅ OMPL 规划器 | ❌ 直接移动 |
| 碰撞检测 | ✅ 自碰撞+环境 | ❌ 无 |
| 路径平滑 | ✅ 轨迹优化 | ❌ 直线插值 |
| 笛卡尔路径 | ✅ 支持 | ⚠️ 需要手动IK |
| 速度 | ⚠️ 较慢（规划） | ✅ 快速 |
| 实时性 | ⚠️ 非实时 | ✅ 实时 |
| 可靠性 | ✅ 高（避障） | ⚠️ 依赖IK精度 |

## 🚀 下一步工作

### 选项 A：继续使用 SDK 模式（推荐短期）

**优点**：
- 已测试通过
- 快速响应
- 简单可靠

**执行**：
```bash
./start_vision_button_ros2.sh --sdk-only
```

### 选项 B：集成 MoveIt2（推荐长期）

**需要做的**：
1. ✅ 验证 MoveIt2 Demo 能正常启动
2. ⏳ 修改 `button_actions.py` 使用 MoveIt2 Python API
3. ⏳ 测试真实硬件集成
4. ⏳ 调优规划参数

**执行**：
```bash
# 先测试 MoveIt2 Demo
ros2 launch piper_with_gripper_moveit demo.launch.py

# 然后运行集成测试
python3 test_moveit2_integration.py
```

## 🐛 故障排除

### 问题 1：MoveIt2 启动失败

**检查**：
```bash
ros2 pkg list | grep moveit
```

**解决**：如果没有看到 moveit 包，重新编译：
```bash
cd /home/robot/button/V4.0/project2/piper_ros
colcon build --packages-select piper_with_gripper_moveit
```

### 问题 2：Humble 配置不兼容 Foxy

**症状**：Launch 文件报错

**解决**：手动修改 launch 文件，将 Humble 特有的 API 改为 Foxy 兼容版本

### 问题 3：运动学求解失败

**症状**：IK solver timeout

**解决**：调整 `kinematics.yaml` 中的超时和搜索分辨率：
```yaml
kinematics_solver_timeout: 0.05  # 增加到 0.05
kinematics_solver_search_resolution: 0.005  # 保持 0.005
```

## 📊 当前状态

- ✅ MoveIt2 包已安装
- ✅ 配置文件已移植
- ✅ 编译成功
- ✅ 启动脚本已更新
- ⏳ 等待测试 Demo
- ⏳ 等待真实硬件集成

## 💬 建议的测试流程

1. **首先**：测试 MoveIt2 Demo（虚拟硬件）
   ```bash
   ros2 launch piper_with_gripper_moveit demo.launch.py
   ```

2. **如果成功**：测试真实硬件（需要修改 launch 文件）

3. **如果真实硬件成功**：集成到 `button_actions.py`

4. **最后**：完整的视觉按钮操作测试

---

**准备好开始测试了吗？建议先运行 MoveIt2 Demo！**
