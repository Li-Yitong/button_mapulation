# 🎯 统一6D位姿支持 - 所有动作类型

## ✅ 更新内容

已为所有四种按钮操作类型添加AprilTag 6D位姿支持：

### 1. **Push** (按压按钮) ✅
- 已支持AprilTag姿态
- 使用 `perpendicular` 模式（垂直按压）

### 2. **Plugin** (插拔连接器) ✅ 新增
- 现已支持AprilTag姿态
- 使用 `perpendicular` 模式（垂直插入）

### 3. **Toggle** (拨动开关) ✅ 新增
- 现已支持AprilTag姿态
- 使用 `perpendicular` 模式（垂直接近后拨动）

### 4. **Knob** (旋转旋钮) ✅ 新增
- 现已支持AprilTag姿态
- 使用 `perpendicular` 模式（垂直插入后旋转）

## 📐 统一逻辑

所有动作在"步骤2: 移动到目标位置"时，都会：

```python
if APRILTAG_REFERENCE_POSE_BASE is not None:
    # 使用AprilTag参考姿态
    button_xyz = np.array([TARGET_X, TARGET_Y, TARGET_Z])
    R_gripper = get_gripper_approach_rotation('perpendicular')
    targetT = create_aligned_target_pose(button_xyz, R_gripper)
    
    # 打印详细的姿态信息
    print(f"  目标姿态: Roll={...}°, Pitch={...}°, Yaw={...}°")
    print(f"  姿态模式: 垂直于面板 (perpendicular) ✓")
else:
    # 回退到传统方式
    targetT = create_target_transform(
        TARGET_X, TARGET_Y, TARGET_Z,
        TARGET_ROLL, TARGET_PITCH, TARGET_YAW,
        USE_6D_POSE
    )
```

## 🎨 姿态计算

**关键转换关系**（垂直按压时）：
```python
# AprilTag姿态（Tag本身）
APRILTAG_BASE_ROLL = -160° 
APRILTAG_BASE_PITCH = 3.4°
APRILTAG_BASE_YAW = 155°

# 夹爪目标姿态（perpendicular模式）
gripper_roll  = -APRILTAG_BASE_ROLL  = 160°  (Roll取反)
gripper_pitch =  APRILTAG_BASE_PITCH = 3.4°  (Pitch相同)
gripper_yaw   =  APRILTAG_BASE_YAW   = 155°  (Yaw相同)
```

## 📊 调试信息

现在所有动作都会打印详细的姿态信息：

### 初始化时：
```
🏷️  AprilTag基座标系位姿:
   位置: (0.413, 0.042, 0.024)
   姿态: Roll=-160.0°, Pitch=3.4°, Yaw=155.0°

🤖 夹爪垂直按压时的目标姿态:
   Roll=160.0° (= -Tag_Roll)
   Pitch=3.4° (= Tag_Pitch)
   Yaw=155.0° (= Tag_Yaw)
```

### 动作执行时：
```
步骤2: 移动到目标位置...
  ✓ 使用AprilTag参考姿态计算的目标位姿
  接近位置: (0.413, 0.042, 0.024)
  目标姿态: Roll=160.0°, Pitch=3.4°, Yaw=155.0°
  末端Z轴: (-0.xxx, -0.xxx, -0.xxx)
  姿态模式: 垂直于面板 (perpendicular) ✓
```

### MoveIt执行后：
```
  [Push接近位姿] 实际到达: XYZ=(0.413, 0.042, 0.024), 误差=0.05cm
  [Push接近位姿] 实际姿态: Roll=160.1°, Pitch=3.5°, Yaw=155.2°
  [Push接近位姿] 目标姿态: Roll=160.0°, Pitch=3.4°, Yaw=155.0°
```

### 笛卡尔精调时（如果触发）：
```
  [Push笛卡尔精调] 当前误差: 0.05cm
  [Push笛卡尔精调] 当前姿态: Roll=160.1°, Pitch=3.5°, Yaw=155.2°
  [Push笛卡尔精调] 目标姿态: Roll=160.0°, Pitch=3.4°, Yaw=155.0°
  [Push笛卡尔精调] 姿态差异: 0.0234
  [Push笛卡尔精调] 生成 30 个waypoints（位置+姿态同步插值）
```

## 🎯 关键改进点

### 1. 姿态保持（解决"强行转正"问题）
- 之前：笛卡尔精调时只插值位置，姿态突然改变
- 现在：位置和姿态同步插值（SLERP），平滑过渡
- 配置：`CARTESIAN_ORIENTATION_INTERPOLATION = True`

### 2. 统一接口
- 所有动作都使用相同的姿态计算逻辑
- 自动处理Tag姿态到夹爪姿态的转换
- 无需手动计算Roll取反

### 3. 调试可见性
- 每个阶段都打印当前和目标姿态
- 可以清楚看到姿态是否正确
- 方便排查"强行转正"等问题

## 📝 使用方法

### 必需配置：
```python
# 1. 设置AprilTag姿态（从视觉检测获取）
APRILTAG_BASE_X = 0.413
APRILTAG_BASE_Y = 0.042
APRILTAG_BASE_Z = 0.024
APRILTAG_BASE_ROLL = -160 * PI / 180
APRILTAG_BASE_PITCH = 3.4 * PI / 180
APRILTAG_BASE_YAW = 155 * PI / 180

# 2. 设置按钮位置
TARGET_X = 0.413
TARGET_Y = 0.042
TARGET_Z = 0.024

# 3. 启用姿态插值（推荐）
CARTESIAN_ORIENTATION_INTERPOLATION = True
```

### 运行动作：
```python
# 所有动作都会自动使用AprilTag姿态
python3 button_actions.py  # ACTION_TYPE = 'push' / 'plugin' / 'toggle' / 'knob'
```

## 🔍 故障排查

### 如果仍然"强行转正"：
1. **检查实际到达的姿态**
   - 查看控制台输出的"实际姿态"
   - 对比"目标姿态"，看是否一致

2. **检查IK求解**
   - IK可能选择了错误的关节配置
   - 尝试调整 `MOVEIT_MAX_REPLAN_ATTEMPTS`

3. **检查姿态插值**
   - 确认 `CARTESIAN_ORIENTATION_INTERPOLATION = True`
   - 查看是否打印"位置+姿态同步插值"

4. **检查AprilTag姿态设置**
   - 运行 `python3 test_apriltag_transform.py`
   - 验证Tag姿态到夹爪姿态的转换是否正确

---

**更新日期**: 2025-12-03  
**状态**: ✅ 所有四种动作已统一支持6D位姿
