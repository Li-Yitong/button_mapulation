# 预设零位配置说明

## 概述
机械臂的"零位"（home position）已从原来的全0位置改为预设的安全位置。

## 预设零位角度

| 关节 | 角度(度) | SDK单位 | 弧度 |
|------|---------|---------|------|
| Joint 1 | 1.411° | 1411 | 0.0246 rad |
| Joint 2 | 8.211° | 8211 | 0.1433 rad |
| Joint 3 | -39.652° | -39652 | -0.6921 rad |
| Joint 4 | -8.139° | -8139 | -0.1421 rad |
| Joint 5 | 30.782° | 30782 | 0.5372 rad |
| Joint 6 | 6.237° | 6237 | 0.1089 rad |

## 修改的文件

### 1. `button_actions.py`
- 在全局常量区添加了 `PRESET_ZERO_J1` ~ `PRESET_ZERO_J6` 配置
- 添加了 `PRESET_ZERO_JOINTS` 列表（弧度单位）
- 修改了所有 `action_*()` 函数中的回零操作（共5处）：
  - `action_plugin()` - 步骤7
  - `action_toggle()` - 步骤8
  - `action_push()` - 步骤6
  - `action_knob()` - 步骤7
  - `main()` - 初始回零
- 所有 `joints_zero = [0, 0, 0, 0, 0, 0]` 改为 `joints_zero = PRESET_ZERO_JOINTS.copy()`

### 2. `test_go_zero.py`
- 添加了预设零位配置常量
- 修改 `JointCtrl()` 命令，使用预设角度而非全0
- 添加了目标位置的打印输出

### 3. `vision_button_action.py`
- 无需修改（它调用 `button_actions.py` 的函数，会自动使用新的零位）

## 单位说明

### SDK单位（千分度）
- SDK的 `GetArmJointMsgs()` 返回的关节角度单位是**千分度**（即：实际度数 × 1000）
- 例如：SDK返回 `joint_1 = 1411` 表示 1.411°
- `JointCtrl()` 接收的参数也是千分度单位

### button_actions.py 使用弧度
- `button_actions.py` 内部使用**弧度**作为单位
- 转换公式：弧度 = 度数 × π / 180

### 获取当前位置
如果想让当前位置成为新的预设零位，运行：
```bash
python3 -c "
from piper_sdk import *
import time
piper = C_PiperInterface_V2('can0')
piper.ConnectPort()
time.sleep(1)
msg = piper.GetArmJointMsgs()
print('当前关节角度（SDK千分度单位）:')
print(f'J1={msg.joint_state.joint_1}, J2={msg.joint_state.joint_2}, J3={msg.joint_state.joint_3}')
print(f'J4={msg.joint_state.joint_4}, J5={msg.joint_state.joint_5}, J6={msg.joint_state.joint_6}')
print('\\n转换为度数:')
print(f'J1={msg.joint_state.joint_1*0.001:.3f}°, J2={msg.joint_state.joint_2*0.001:.3f}°, J3={msg.joint_state.joint_3*0.001:.3f}°')
print(f'J4={msg.joint_state.joint_4*0.001:.3f}°, J5={msg.joint_state.joint_5*0.001:.3f}°, J6={msg.joint_state.joint_6*0.001:.3f}°')
"
```

## 如何修改预设零位

如果需要调整预设零位，只需修改两个文件中的配置：

### `button_actions.py` (第26-31行)
```python
PRESET_ZERO_J1 = 1.411      # 修改这里（单位：度）
PRESET_ZERO_J2 = 8.211      # 修改这里（单位：度）
PRESET_ZERO_J3 = -39.652    # 修改这里（单位：度）
PRESET_ZERO_J4 = -8.139     # 修改这里（单位：度）
PRESET_ZERO_J5 = 30.782     # 修改这里（单位：度）
PRESET_ZERO_J6 = 6.237      # 修改这里（单位：度）
```

### `test_go_zero.py` (第13-18行)
```python
PRESET_ZERO_J1_RAW = 1411      # 修改这里（单位：千分度 = 度数×1000）
PRESET_ZERO_J2_RAW = 8211      # 修改这里
PRESET_ZERO_J3_RAW = -39652    # 修改这里
PRESET_ZERO_J4_RAW = -8139     # 修改这里
PRESET_ZERO_J5_RAW = 30782     # 修改这里
PRESET_ZERO_J6_RAW = 6237      # 修改这里
```
注意：`test_go_zero.py` 使用 SDK 的原始千分度单位

## 测试

### 测试回零功能
```bash
cd /home/robot/button/V4.0/project2
python3 test_go_zero.py
```

### 测试按钮动作（会在开始和结束时回到预设零位）
```bash
# SDK模式
./start_button_action.sh

# MoveIt模式  
./start_press_moveit.sh
```

## 注意事项

1. **安全性**: 预设零位应该是一个安全的姿态，机械臂在这个位置不会碰撞到工作台或其他物体

2. **一致性**: 所有动作（toggle, plugin, push, knob）都会从预设零位开始，并在结束时回到预设零位

3. **ROS环境**: 确保在使用前已经source ROS环境：
   ```bash
   source /opt/ros/noetic/setup.bash
   ```

4. **关节限制**: 预设零位应该在机械臂的关节限制范围内

## 验证配置

运行以下命令验证配置转换是否正确：
```bash
cd /home/robot/button/V4.0/project2
python3 -c "
import math
PI = math.pi
PRESET_ZERO_J1 = 1.411
factor = 1000 * 180 / PI
print(f'J1: {PRESET_ZERO_J1}° = {int(PRESET_ZERO_J1 * factor)} (SDK单位)')
"
```

期望输出：`J1: 1.411° = 1411 (SDK单位)`
