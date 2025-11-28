# move_a_to_b.py - MoveIt集成说明

## 概述

`move_a_to_b.py` 已升级支持 **MoveIt轨迹规划**，采用与 `grasp_action.py` 相同的混合控制策略：
- **MoveIt** 负责规划无碰撞的平滑轨迹
- **SDK** 负责执行轨迹点控制真实硬件

## 工作模式

### 1. MoveIt模式（推荐）
- 使用MoveIt进行轨迹规划，避免碰撞
- 轨迹更平滑，运动更安全
- 需要ROS环境和move_group节点

### 2. SDK模式（备用）
- 直接使用SDK控制关节
- 不需要ROS环境
- MoveIt不可用或规划失败时自动启用

### 3. 自动降级
- 如果MoveIt导入失败 → 自动使用SDK模式
- 如果MoveIt规划失败 → 自动回退到SDK模式
- 无需手动切换，系统自动选择最佳方案

## 使用方法

### 方式1: 使用MoveIt启动脚本（推荐）

```bash
cd /home/robot/button/V4.0/project2
./start_press_moveit.sh
```

该脚本会自动完成以下步骤：
1. 检查并重置摄像头（如果锁定）
2. 启动roscore
3. 启动Piper硬件接口
4. 启动MoveIt move_group节点
5. 运行按压程序（MoveIt模式）

### 方式2: 手动启动（SDK模式）

```bash
cd /home/robot/button/V4.0/project2
python3 move_a_to_b.py
```

如果没有ROS环境，程序会自动使用SDK模式。

## 配置参数

### 速度宏定义（文件顶部）

```python
# 运动速度宏定义 (1-100, 数值越大越快)
SPEED_ZERO = 100              # 回零位速度
SPEED_TO_START = 100          # 移动到目标前方的速度
SPEED_APPROACH = 100          # 接近物体表面的速度
SPEED_PRESS = 100             # 按压时的速度
SPEED_RETREAT = 100           # 后退的速度
```

### MoveIt配置

```python
# ========== MoveIt配置 ==========
USE_MOVEIT = True             # 是否使用MoveIt（需要MOVEIT_AVAILABLE=True）
VELOCITY_SCALING = 1.0        # MoveIt速度缩放 (0.1-1.0)
ACCELERATION_SCALING = 1.0    # MoveIt加速度缩放 (0.1-1.0)
```

### 按压参数

```python
# 目标位置 (基座坐标系, 单位:米)
TARGET_X = 0.25  # 目标物体x坐标 (表面位置)
TARGET_Y = 0.0   # 目标物体y坐标
TARGET_Z = 0.20  # 目标物体z坐标

# 按压参数（沿夹爪z轴方向）
PRESS_DISTANCE_BEFORE = 0.05  # 目标前方停留距离 (米) 默认5cm
PRESS_DEPTH = 0.02            # 按压深度 (米) 默认2cm
PRESS_DURATION = 2.0          # 按压持续时间 (秒)

# 夹爪状态
GRIPPER_CLOSE_VALUE = 0       # 按压时夹爪闭合 (0-70000)
```

## 技术细节

### 混合控制策略

1. **MoveIt规划阶段**
   - 设置目标关节角度
   - 使用MoveIt计算无碰撞轨迹
   - 提取轨迹点序列

2. **SDK执行阶段**
   - 采样轨迹点（最多20个）
   - 使用SDK逐点控制机械臂
   - 每个点等待0.15秒
   - 最终位置确认（5次重复发送，确保到达）

3. **速度控制**
   - MoveIt: 使用VELOCITY_SCALING参数（0.1-1.0）
   - SDK: 使用SPEED_*宏定义（1-100）
   - 两者配合实现平滑高速运动

### 关键函数

- `control_arm_moveit()`: MoveIt轨迹规划 + SDK执行
- `move_to_position()`: 自动选择MoveIt或SDK模式
- `press_action()`: 完整的按压流程（支持MoveIt）

## 速度调优建议

### 提高速度

```python
# 方法1: 提高MoveIt速度缩放
VELOCITY_SCALING = 1.0        # 已设为最大值

# 方法2: 提高SDK速度
SPEED_TO_START = 100          # 已设为最大值
SPEED_APPROACH = 100
SPEED_PRESS = 100
SPEED_RETREAT = 100
```

### 保证精度

如果速度过快导致精度下降：
```python
# 降低关键步骤的速度
SPEED_APPROACH = 50           # 接近时慢一点
SPEED_PRESS = 30              # 按压时要慢
```

在 `control_arm_moveit()` 中调整等待时间：
```python
time.sleep(0.15)  # 每个轨迹点等待时间（增加可提高精度）
time.sleep(0.3)   # 最终位置确认等待（增加可提高精度）
```

## 故障排除

### 问题1: MoveIt规划失败

**现象**: 程序显示 "❌ MoveIt规划失败"

**原因**: 
- 目标位置超出工作空间
- 目标姿态无法达到
- move_group节点未启动

**解决**:
1. 检查目标位置是否在工作空间内
2. 使用 `start_press_moveit.sh` 确保move_group启动
3. 程序会自动回退到SDK模式

### 问题2: 速度太快精度下降

**现象**: 机械臂未到达目标位置就执行下一步

**解决**:
1. 降低SPEED_*宏定义的值
2. 增加 `control_arm_moveit()` 中的等待时间
3. 增加最终位置确认的重复次数

### 问题3: MoveIt导入失败

**现象**: 启动时显示 "⚠️ MoveIt导入失败"

**原因**: conda环境与ROS不兼容

**解决**: 使用系统Python运行（脚本已自动处理）

## 与grasp_action.py的区别

| 特性 | move_a_to_b.py | grasp_action.py |
|------|----------------|-----------------|
| 功能 | 前向按压 | 视觉抓取 |
| 运动方式 | 沿夹爪z轴直线按压 | 多步骤抓取流程 |
| 需要摄像头 | 否 | 是 |
| MoveIt支持 | ✓ | ✓ |
| 速度配置 | 5个宏定义 | 每步指定 |
| 启动脚本 | start_press_moveit.sh | start_all_moveit.sh |

## 参考资料

- `grasp_action.py`: 相同的MoveIt集成模式
- `MOVEIT_CONDA_GUIDE.md`: MoveIt与conda环境兼容性说明
- `start_press_moveit.sh`: MoveIt模式启动脚本
