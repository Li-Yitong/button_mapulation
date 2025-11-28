# 按钮操作执行器 - 6D位姿与RViz可视化使用指南

## ✨ 新增功能

### 1. 支持6D位姿输入
现在可以指定末端执行器的完整位姿（位置+姿态），而不仅仅是XYZ坐标。

### 2. RViz轨迹可视化
可以在RViz中实时查看MoveIt规划的轨迹。

---

## 📐 6D位姿配置

### 配置文件位置
编辑 `button_actions.py` 顶部的配置区域：

```python
# === 目标位姿配置 (基座坐标系) ===
# 位置 (单位：米)
TARGET_X = 0.25  # X坐标
TARGET_Y = 0.00  # Y坐标
TARGET_Z = 0.20  # Z坐标

# 姿态 (单位：弧度)
TARGET_ROLL = 0.0    # 绕X轴旋转 (翻滚)
TARGET_PITCH = 0.0   # 绕Y轴旋转 (俯仰)
TARGET_YAW = 0.0     # 绕Z轴旋转 (偏航)

# 姿态模式选择
USE_6D_POSE = True   # True=使用6D位姿, False=默认朝前
```

### 姿态角度说明

**Roll (翻滚)**: 绕X轴旋转
- 正值：末端向右倾斜
- 负值：末端向左倾斜
- 示例：`TARGET_ROLL = 0.5` (约28.6°)

**Pitch (俯仰)**: 绕Y轴旋转
- 正值：末端向上抬起
- 负值：末端向下低头
- 示例：`TARGET_PITCH = -0.3` (约-17.2°)

**Yaw (偏航)**: 绕Z轴旋转
- 正值：末端逆时针旋转
- 负值：末端顺时针旋转
- 示例：`TARGET_YAW = 1.57` (约90°)

### 度数与弧度转换

```python
import math

# 度数转弧度
TARGET_ROLL = math.radians(30)   # 30度
TARGET_PITCH = math.radians(-15) # -15度
TARGET_YAW = math.radians(45)    # 45度

# 或者直接计算
TARGET_ROLL = 30 * 3.14159 / 180
```

---

## 🎨 RViz可视化使用

### 启动方式

**启用RViz（推荐用于调试）**:
```bash
./start_button_action.sh --rviz
./start_button_action.sh -r --strategy cartesian --action push
```

**不启用RViz（更快）**:
```bash
./start_button_action.sh
./start_button_action.sh --strategy incremental --action push
```

### RViz界面说明

启动后会自动打开RViz窗口，显示：
- ✅ **机器人模型**: 当前机械臂状态
- ✅ **规划轨迹**: 绿色/橙色线条显示规划路径
- ✅ **目标位姿**: 坐标系显示目标位置
- ✅ **障碍物**: 如果配置了场景

### RViz操作技巧

1. **旋转视角**: 按住鼠标左键拖动
2. **平移视角**: 按住鼠标中键拖动
3. **缩放**: 滚动鼠标滚轮
4. **复位视角**: 在左侧面板选择 `Views` → `Reset`

---

## 🚀 完整使用示例

### 示例1: 斜向按压按钮

```python
# 在 button_actions.py 中配置
TARGET_X = 0.25
TARGET_Y = 0.05    # 向左偏移5cm
TARGET_Z = 0.20
TARGET_ROLL = 0.0
TARGET_PITCH = 0.2  # 向上倾斜约11.5°
TARGET_YAW = 0.0
USE_6D_POSE = True
ACTION_TYPE = 'push'
```

```bash
# 运行
./start_button_action.sh --rviz --strategy cartesian --action push
```

### 示例2: 旋转45度插入连接器

```python
TARGET_X = 0.23
TARGET_Y = 0.00
TARGET_Z = 0.18
TARGET_ROLL = 0.0
TARGET_PITCH = 0.0
TARGET_YAW = 0.785  # 45度 = 0.785弧度
USE_6D_POSE = True
ACTION_TYPE = 'plugin'
```

```bash
./start_button_action.sh -r -s hybrid -a plugin
```

### 示例3: 侧向拨动开关

```python
TARGET_X = 0.26
TARGET_Y = -0.03   # 向右偏移3cm
TARGET_Z = 0.22
TARGET_ROLL = -0.5  # 向右倾斜约28.6°
TARGET_PITCH = 0.0
TARGET_YAW = 0.0
USE_6D_POSE = True
ACTION_TYPE = 'toggle'
TOGGLE_DIRECTION = 'right'
```

```bash
./start_button_action.sh --rviz --action toggle
```

---

## ⚙️ 高级配置

### MoveIt规划参数

在代码中可以调整MoveIt的规划参数：

```python
# 在 main() 函数的 MoveIt初始化部分
move_group.set_planning_time(5.0)                    # 规划时间 (秒)
move_group.set_max_velocity_scaling_factor(1.0)      # 速度缩放 (0.0-1.0)
move_group.set_max_acceleration_scaling_factor(1.0)  # 加速度缩放 (0.0-1.0)
move_group.set_goal_position_tolerance(0.01)         # 位置容差 (米)
move_group.set_goal_orientation_tolerance(0.01)      # 姿态容差 (弧度)
```

### 轨迹可视化颜色

在RViz中自定义显示颜色：
1. 左侧面板找到 `MotionPlanning` → `Planned Path`
2. 展开 `Loop Animation`
3. 修改 `Trail` 和 `Robot` 的颜色

---

## �� 故障排除

### 问题1: RViz没有显示轨迹

**解决方法**:
1. 检查终端输出是否有 "✓ 轨迹已发布到RViz"
2. 在RViz左侧确认 `MotionPlanning` 插件已加载
3. 重启RViz: `killall -9 rviz && ./start_button_action.sh --rviz`

### 问题2: 6D位姿IK求解失败

**解决方法**:
1. 减小姿态角度，从小角度开始测试
2. 调整位置更靠近机械臂中心
3. 使用 `USE_6D_POSE = False` 切换回默认姿态

### 问题3: 轨迹在RViz中闪烁

**解决方法**:
- 增加发布队列大小（已默认设置为20）
- 减慢轨迹执行速度观察

---

## 📊 参数对照表

| 参数 | 类型 | 单位 | 建议范围 | 说明 |
|------|------|------|----------|------|
| TARGET_X | float | 米 | 0.20-0.28 | 前后距离 |
| TARGET_Y | float | 米 | -0.15-0.15 | 左右距离 |
| TARGET_Z | float | 米 | 0.15-0.30 | 上下高度 |
| TARGET_ROLL | float | 弧度 | -0.5-0.5 | 翻滚角 |
| TARGET_PITCH | float | 弧度 | -0.5-0.5 | 俯仰角 |
| TARGET_YAW | float | 弧度 | -π-π | 偏航角 |
| USE_6D_POSE | bool | - | True/False | 是否使用姿态 |

---

## 💡 最佳实践

1. **首次测试**: 使用默认姿态（USE_6D_POSE=False）验证位置
2. **逐步调整**: 先调位置，再调姿态，每次只改一个参数
3. **启用RViz**: 调试时always使用 `--rviz` 查看规划结果
4. **保存配置**: 成功配置记录到文件中方便复用
5. **安全第一**: 大幅度姿态变化前先降低速度测试

---

## 📞 快速参考

```bash
# 完整命令格式
./start_button_action.sh [-s STRATEGY] [-a ACTION] [-r]

# 参数说明
-s, --strategy : incremental | cartesian | hybrid
-a, --action   : toggle | plugin | push | knob
-r, --rviz     : 启用RViz可视化
-h, --help     : 显示帮助

# 常用组合
./start_button_action.sh -r                    # 默认+RViz
./start_button_action.sh -s cartesian -r       # 笛卡尔+RViz
./start_button_action.sh -a push -r            # 按压+RViz
```

---

祝你使用愉快！🎉
