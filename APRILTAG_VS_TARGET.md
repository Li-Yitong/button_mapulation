# 📍 位置参数说明 - AprilTag vs Target

## 🔑 关键区别

### 1. AprilTag 位置（仅用于姿态参考）
```python
APRILTAG_BASE_X = 0.413  # AprilTag标签的X坐标
APRILTAG_BASE_Y = 0.042  # AprilTag标签的Y坐标
APRILTAG_BASE_Z = 0.024  # AprilTag标签的Z坐标

APRILTAG_BASE_ROLL = -180 * PI / 180   # Tag的姿态（Roll）
APRILTAG_BASE_PITCH = 3.4 * PI / 180   # Tag的姿态（Pitch）
APRILTAG_BASE_YAW = -155 * PI / 180    # Tag的姿态（Yaw）
```

**用途**：
- ❌ **不是**动作的目标位置
- ✅ **仅用于**计算夹爪的正确姿态
- 📐 提供面板的法向量信息

---

### 2. Target 位置（实际动作目标）
```python
TARGET_X = 0.410  # 按钮/旋钮的实际X坐标 ⬅️ 这是机械臂要去的位置！
TARGET_Y = 0.045  # 按钮/旋钮的实际Y坐标
TARGET_Z = 0.030  # 按钮/旋钮的实际Z坐标
```

**用途**：
- ✅ **是**动作的目标位置
- ✅ 机械臂移动到这个位置
- ✅ 在这个位置执行按压/插入/旋转等动作

---

## 🎯 实际使用逻辑

### 代码中的处理（所有动作通用）：

```python
# 步骤2: 移动到目标位置
if APRILTAG_REFERENCE_POSE_BASE is not None:
    # 1. 使用TARGET位置（按钮的实际位置）
    target_position = np.array([TARGET_X, TARGET_Y, TARGET_Z])  # ✅ 用的是TARGET
    
    # 2. 使用AprilTag姿态计算夹爪姿态
    R_gripper = get_gripper_approach_rotation('perpendicular')  # 从APRILTAG_BASE_ROLL/PITCH/YAW计算
    
    # 3. 组合：TARGET的位置 + AprilTag计算的姿态
    targetT = create_aligned_target_pose(target_position, R_gripper)
    
    # 结果：
    # - 位置 = (TARGET_X, TARGET_Y, TARGET_Z)  ← 按钮位置
    # - 姿态 = 从AprilTag计算的夹爪姿态    ← 正确的垂直姿态
```

---

## 📊 典型场景示例

### 场景：按压一个按钮

**面板信息**（通过AprilTag获取）：
```python
# AprilTag在面板中心
APRILTAG_BASE_X = 0.400    # Tag在面板中心
APRILTAG_BASE_Y = 0.050
APRILTAG_BASE_Z = 0.020
APRILTAG_BASE_ROLL = -180°  # Tag的朝向
APRILTAG_BASE_PITCH = 3.4°
APRILTAG_BASE_YAW = -155°
```

**按钮位置**（与Tag不同）：
```python
# 按钮在Tag右上方5cm处
TARGET_X = 0.410    # 比Tag X大1cm
TARGET_Y = 0.045    # 比Tag Y小0.5cm
TARGET_Z = 0.030    # 比Tag Z高1cm
```

**执行结果**：
- ✅ 机械臂移动到 **(0.410, 0.045, 0.030)** ← 按钮位置
- ✅ 姿态为 **(160°, 3.4°, -155°)** ← 从Tag姿态计算（Roll取反）
- ✅ 垂直于面板按压按钮

---

## ✅ 验证方法

### 查看控制台输出：

```
步骤2: MoveIt规划到目标位姿...
  ✓ 使用AprilTag参考姿态计算的目标位姿
  接近位置: (0.410, 0.045, 0.030)  ← 应该是TARGET_X/Y/Z，不是APRILTAG_BASE_X/Y/Z
  目标姿态: Roll=160.0°, Pitch=3.4°, Yaw=-155.0°
  末端Z轴: (-0.xxx, -0.xxx, -0.xxx)
  姿态模式: 垂直于面板 (perpendicular) ✓
```

### 重要检查点：
1. ✅ "接近位置" 显示的是 **TARGET_X/Y/Z**
2. ✅ "目标姿态" 中Roll应该是 **-APRILTAG_BASE_ROLL**
3. ✅ Pitch和Yaw应该与 **APRILTAG_BASE_PITCH/YAW** 相同

---

## 🔧 配置建议

### 步骤1：标定AprilTag
```python
# 运行视觉检测，观察AprilTag位置和姿态
# 更新这些值：
APRILTAG_BASE_X = ...     # 从 [Tag in Base] 获取
APRILTAG_BASE_Y = ...
APRILTAG_BASE_Z = ...
APRILTAG_BASE_ROLL = ...  # 从 [Tag in Base] 获取
APRILTAG_BASE_PITCH = ...
APRILTAG_BASE_YAW = ...
```

### 步骤2：设置按钮位置
```python
# 根据按钮相对于Tag的偏移量设置
# 例如：按钮在Tag右上方5cm
TARGET_X = APRILTAG_BASE_X + 0.01   # 向右1cm
TARGET_Y = APRILTAG_BASE_Y - 0.005  # 向前0.5cm
TARGET_Z = APRILTAG_BASE_Z + 0.01   # 向上1cm
```

### 步骤3：运行动作
```python
# 代码会自动：
# 1. 使用TARGET_X/Y/Z作为目标位置
# 2. 使用APRILTAG姿态计算正确的夹爪姿态
# 3. 组合两者生成最终的目标位姿
```

---

## 🎉 总结

| 参数 | 含义 | 用途 |
|------|------|------|
| `APRILTAG_BASE_X/Y/Z` | AprilTag标签位置 | 仅用于姿态参考（不是目标位置）|
| `APRILTAG_BASE_ROLL/PITCH/YAW` | AprilTag标签姿态 | 计算夹爪的正确姿态 |
| `TARGET_X/Y/Z` | 按钮/旋钮位置 | **实际动作的目标位置** ✅ |

**代码逻辑**：
```
位置来源 = TARGET_X/Y/Z  ← 按钮位置
姿态来源 = APRILTAG_BASE_ROLL/PITCH/YAW  ← Tag姿态（自动转换）
最终目标 = 组合两者
```

---

**更新日期**: 2025-12-03  
**适用动作**: Push, Plugin, Toggle, Knob（全部统一）
