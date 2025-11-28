# MoveIt 轨迹规划详解

## 问题回答

### Q1: 起始点为什么是 [0, 0, 0, 0, 0, 0]？

**答案：这是误导性的显示！**

实际情况：
1. **规划起始点** ≠ **当前实际位置**
2. MoveIt 的轨迹规划从**当前机械臂状态**开始
3. 但 `traj_points[0]` 显示的可能是相对时间 `T=0.000s` 的状态
4. 如果机械臂在零位，那确实是 `[0, 0, 0, 0, 0, 0]`

**修复后的显示**：
```
当前实际关节角度: [0.1234, 0.5678, -0.3456, ...]  ← 真实的起始点
起始点位置 (rad): [0.1234, 0.5678, -0.3456, ...]  ← traj_points[0]
终点位置 (rad): [0.9120, 0.2411, -0.5837, ...]  ← traj_points[-1]
```

### Q2: 起始点难道不应该是上一部分的终止点吗？

**答案：完全正确！**

这就是**轨迹连续性**问题：

```
步骤2: 移动到目标位置
  终点关节角度: [j1, j2, j3, j4, j5, j6]
                    ↓
步骤3: 沿末端z轴插入  
  起始关节角度: [j1, j2, j3, j4, j5, j6]  ← 应该相同！
```

**为什么会看到不同？**
- MoveIt 规划时自动从**当前机械臂状态**开始
- 如果步骤2执行完成，机械臂应该在终点
- 步骤3的起始点就是步骤2的终点

**验证方法**：
```python
# 步骤2执行完成后
joints_target = [0.1234, 0.5678, ...]  # 步骤2的终点

# 步骤3开始前
current_joints = get_current_joints()  # 读取当前位置
print(f"步骤2终点: {joints_target}")
print(f"步骤3起点: {current_joints}")
# 应该非常接近（误差<0.01 rad）
```

### Q3: 轨迹规划的 P、V、A、T 是如何生成的？

## MoveIt 轨迹规划原理

### 1. 输入阶段

#### 关节空间规划 (Joint Space Planning)
```python
move_group.set_joint_value_target([j1, j2, j3, j4, j5, j6])
plan = move_group.plan()
```

**输入**：
- 起始关节角度：`q_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- 目标关节角度：`q_goal = [0.2, 0.15, -0.3, 0.0, 0.0, 0.0]`
- 规划器：RRTconnect

#### 笛卡尔空间规划 (Cartesian Planning)
```python
waypoints = [pose1, pose2, pose3, ...]  # 末端位姿序列
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints,
    0.01,      # eef_step: 末端最大步长 1cm
    True       # avoid_collisions
)
```

**输入**：
- 起始位姿：当前末端位姿 `T_start`
- 路径点序列：`[T1, T2, T3, ..., T_goal]`
- 步长：0.01m (1cm)

### 2. 规划阶段

#### RRTconnect 算法流程

```
1. 从起始配置 q_start 构建搜索树
2. 从目标配置 q_goal 构建搜索树
3. 两棵树交替生长，尝试连接
4. 找到连接路径后，进行路径平滑
```

**输出**：离散的关节配置序列
```
q0 = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
q1 = [0.020, 0.015, -0.030, 0.000, 0.000, 0.000]
q2 = [0.040, 0.030, -0.060, 0.000, 0.000, 0.000]
...
qN = [0.200, 0.150, -0.300, 0.000, 0.000, 0.000]
```

#### 笛卡尔规划流程

```
1. 将末端路径点转换为关节空间路径
   T1 --IK--> q1
   T2 --IK--> q2
   T3 --IK--> q3
   ...

2. 检查路径可行性
   - 关节限位
   - 碰撞检测
   - 奇异点检查

3. 计算覆盖率 fraction
   fraction = 成功规划的路径长度 / 总路径长度
```

### 3. 轨迹生成阶段

**关键问题**：如何从离散的关节配置生成平滑的轨迹？

#### 时间参数化 (Time Parameterization)

MoveIt 使用 **时间最优轨迹生成算法** (TOTG - Time-Optimal Trajectory Generation)

```
输入：关节配置序列 [q0, q1, q2, ..., qN]
输出：带时间戳的轨迹点
```

**步骤**：

1. **计算路径长度**
```python
for i in range(N):
    distance[i] = ||q[i+1] - q[i]||  # 关节空间欧氏距离
```

2. **速度限制** (从 MoveIt 配置读取)
```yaml
# joint_limits.yaml
joint1:
  max_velocity: 3.14  # rad/s
  max_acceleration: 5.0  # rad/s²
joint2:
  max_velocity: 3.14
  max_acceleration: 5.0
...
```

3. **时间最优分配**

使用**梯形速度曲线**生成时间戳：

```
速度曲线：
    │     ╱‾‾‾‾‾╲
V   │    ╱       ╲
    │   ╱         ╲
    │  ╱           ╲
    └─┴─────────────┴───> 时间
      加速  匀速  减速
```

**计算公式**：

```python
# 1. 加速阶段
t_accel = v_max / a_max
s_accel = 0.5 * a_max * t_accel^2

# 2. 匀速阶段
s_cruise = total_distance - 2 * s_accel
t_cruise = s_cruise / v_max

# 3. 减速阶段
t_decel = v_max / a_max

# 总时间
t_total = t_accel + t_cruise + t_decel
```

4. **生成轨迹点**

对每个时间点 `t`，计算：

```python
if t < t_accel:
    # 加速阶段
    v(t) = a_max * t
    s(t) = 0.5 * a_max * t^2
    a(t) = a_max
    
elif t < t_accel + t_cruise:
    # 匀速阶段
    v(t) = v_max
    s(t) = s_accel + v_max * (t - t_accel)
    a(t) = 0
    
else:
    # 减速阶段
    t_brake = t - (t_accel + t_cruise)
    v(t) = v_max - a_max * t_brake
    s(t) = s_accel + s_cruise + v_max * t_brake - 0.5 * a_max * t_brake^2
    a(t) = -a_max
```

5. **插值计算关节角度**

根据路径长度 `s(t)` 在关节配置序列中插值：

```python
# 找到当前路径长度对应的区间
for i in range(N):
    if s(t) >= s[i] and s(t) < s[i+1]:
        # 线性插值
        alpha = (s(t) - s[i]) / (s[i+1] - s[i])
        q(t) = q[i] + alpha * (q[i+1] - q[i])
        
        # 速度
        dq(t) = (q[i+1] - q[i]) / (s[i+1] - s[i]) * v(t)
        
        # 加速度
        ddq(t) = (q[i+1] - q[i]) / (s[i+1] - s[i]) * a(t)
```

### 4. 输出格式

最终生成的轨迹点格式：

```python
class JointTrajectoryPoint:
    positions: List[float]        # P: 关节角度 (rad)
    velocities: List[float]       # V: 关节速度 (rad/s)
    accelerations: List[float]    # A: 关节加速度 (rad/s²)
    time_from_start: Duration     # T: 时间戳 (s)
```

**示例输出**：

```
点 #0/15:
  时间戳 T: 0.0000s
  位置 P (rad): [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]
  速度 V (rad/s): [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]  ← 起点速度为0
  加速度 A (rad/s²): [0.5234, 0.3456, -0.2345, 0.0000, -0.1234, 0.0000]  ← 开始加速

点 #5/15:
  时间戳 T: 0.8000s
  位置 P (rad): [0.1000, 0.0750, -0.1500, 0.0000, 0.0000, 0.0000]
  速度 V (rad/s): [0.4123, 0.3456, -0.1234, 0.0000, 0.0000, 0.0000]  ← 加速中
  加速度 A (rad/s²): [0.5234, 0.3456, -0.2345, 0.0000, -0.1234, 0.0000]

点 #10/15:
  时间戳 T: 1.5000s
  位置 P (rad): [0.1500, 0.1125, -0.2250, 0.0000, 0.0000, 0.0000]
  速度 V (rad/s): [0.5000, 0.4000, -0.1500, 0.0000, 0.0000, 0.0000]  ← 匀速
  加速度 A (rad/s²): [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]  ← 加速度为0

点 #15/15:
  时间戳 T: 2.3150s
  位置 P (rad): [0.2000, 0.1500, -0.3000, 0.0000, 0.0000, 0.0000]  ← 到达终点
  速度 V (rad/s): [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]  ← 终点速度为0
  加速度 A (rad/s²): [-0.5234, -0.3456, 0.2345, 0.0000, 0.1234, 0.0000]  ← 减速
```

## 实际执行流程

### 代码中的实现

```python
# 1. MoveIt 规划生成轨迹
traj_points = trajectory.joint_trajectory.points
# traj_points 是一个列表，包含多个 JointTrajectoryPoint

# 2. 采样（减少执行点数）
sample_indices = np.linspace(0, len(traj_points)-1, TRAJECTORY_SAMPLE_POINTS, dtype=int)
sample_points = [traj_points[i] for i in sample_indices]

# 例如：从 156 个点采样 20 个点
# 采样索引: [0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128, 136, 144, 155]

# 3. 按照 SDK_EXECUTE_RATE 频率执行
sdk_rate = rospy.Rate(SDK_EXECUTE_RATE)  # 50Hz

for idx, point in enumerate(sample_points):
    # 转换为整数
    joints_int = [int(point.positions[i] * factor) for i in range(6)]
    
    # 发送给机械臂
    piper.JointCtrl(*joints_int)
    
    # 按频率等待
    sdk_rate.sleep()  # 等待 1/50 = 0.02s
```

### 时间关系

```
MoveIt 规划时间：2.315秒（理论执行时间）
采样点数：20
执行频率：50Hz
实际执行时间：20 / 50 = 0.4秒

为什么实际时间更短？
1. 采样减少了点数（156 → 20）
2. SDK 执行速度参数加速了运动
3. 跳过了一些减速段
```

### 速度和加速度的意义

**速度 V**：
- 表示该点处各关节的运动速度
- 单位：rad/s
- 用于**平滑过渡**和**轨迹跟踪**

**加速度 A**：
- 表示该点处各关节的加速度
- 单位：rad/s²
- 用于**力矩控制**和**动力学仿真**

**注意**：
- 在我们的实现中，**只使用了位置 P**
- 速度和加速度主要用于**可视化和分析**
- SDK 直接控制会忽略速度和加速度信息

## 轨迹连续性保证

### 步骤间的连接

```python
# 步骤2: 移动到目标位置
joints_target = [0.1234, 0.5678, -0.3456, ...]
control_arm(joints_target, ...)
time.sleep(1.0)  # 等待到达

# 步骤3: 沿末端z轴插入
# MoveIt 自动从当前位置开始规划
current_joints = get_current_joints()  # 应该 ≈ joints_target
joints_insert = move_along_end_effector_z(current_joints, distance, speed)
```

### 误差来源

1. **运动控制误差**：机械臂可能没有完全到达目标位置（±1-2mm）
2. **传感器误差**：关节编码器读数误差（±0.01 rad）
3. **时间延迟**：命令发送和执行之间的延迟（~10-50ms）

### 改进建议

```python
# 1. 等待更长时间确保到达
time.sleep(2.0)

# 2. 验证是否到达
current = get_current_joints()
error = np.linalg.norm(np.array(current) - np.array(target))
if error > 0.05:  # 阈值 0.05 rad
    print(f"警告：未到达目标位置，误差: {error:.4f} rad")

# 3. 使用更高的执行频率
SDK_EXECUTE_RATE = 100  # 提高到 100Hz
```

## 总结

1. **起始点**：从**当前机械臂实际位置**开始，不是 `[0,0,0,0,0,0]`
2. **轨迹连续性**：每个步骤的起始点应该是上一步的终点
3. **P、V、A、T 生成**：MoveIt 使用时间最优算法生成平滑轨迹
4. **实际执行**：采样后按固定频率执行，只使用位置信息
5. **调试信息**：现在显示**所有点**的完整信息（点数≤20时）

**现在运行测试，您将看到**：
- ✅ 完整的轨迹点信息（不省略）
- ✅ 每个点的 P、V、A、T 详细数据
- ✅ 速度和加速度的模值
- ✅ 起始点和终点的对比
- ✅ 轨迹统计信息（总时长、平均频率、点间时间）
