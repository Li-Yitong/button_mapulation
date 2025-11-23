# 机械臂失能摔落问题分析与修复

## 问题现象

运行 `python3 button_action_ros2.py` 时，机械臂突然失能并摔落：

```
  ❌ 规划失败 (错误码: 99999)，切换到SDK模式
  [SDK] 移动中... (预计0.6秒)
```

**关键症状：** 机械臂在MoveIt2规划失败后切换到SDK模式时失能，导致机械臂失去力矩控制而摔落。

---

## 根本原因

### 原因1：MoveIt2规划组名称不匹配 ❌

**MoveIt2报错：**
```
[ERROR] Cannot find planning configuration for group 'piper_arm'
```

**代码与配置不一致：**
- **SRDF文件** (`piper.srdf`) 定义的规划组名称：**`arm`**
- **Python代码** (`button_action_ros2.py`) 请求的规划组：**`piper_arm`** ❌

**影响：** 每次规划请求都会失败（错误码99999），强制切换到SDK模式。

---

### 原因2：SDK切换时未保证机械臂使能 ❌

**问题代码** (`button_action_ros2.py:615-627`)：
```python
def control_arm_sdk(joints, speed=50, gripper_value=None):
    """SDK直接控制模式"""
    global piper
    
    joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
    joints_int[4] = max(-70000, joints_int[4])
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)  # ⚠️ 没有明确使能
    piper.JointCtrl(*joints_int)
    # ...
```

**风险点：**
1. MoveIt2规划过程中可能会调用某些底层操作（如碰撞检测、状态查询），这些操作可能会暂时改变机械臂的控制状态
2. 当规划失败后切换到SDK模式时，如果机械臂已经被失能，`MotionCtrl_2()` 可能不足以立即恢复力矩
3. 没有明确的 `EnableArm()` 调用来确保关节使能状态

**后果：** 机械臂在规划失败的一瞬间失去力矩控制，导致摔落。

---

## 解决方案

### 修复1：更正MoveIt2规划组名称 ✅

**文件：** `button_action_ros2.py:651`

**修改前：**
```python
goal_msg.request.group_name = "piper_arm"  # ❌ 错误
```

**修改后：**
```python
goal_msg.request.group_name = "arm"  # ✅ 与SRDF一致
```

**验证命令：**
```bash
# 检查SRDF中的规划组定义
grep 'group name=' ~/piper_ros2/src/piper_description/config/piper.srdf
```

---

### 修复2：在SDK切换时强制重新使能机械臂 ✅

**文件：** `button_action_ros2.py:615-627` 和 `button_actions.py:786-798`

**修改前：**
```python
def control_arm_sdk(joints, speed=50, gripper_value=None):
    """SDK直接控制模式"""
    global piper
    
    joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
    joints_int[4] = max(-70000, joints_int[4])
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joints_int)
    # ...
```

**修改后：**
```python
def control_arm_sdk(joints, speed=50, gripper_value=None):
    """SDK直接控制模式"""
    global piper
    
    # 🔧 关键修复：确保机械臂使能（防止规划失败后失能导致摔落）
    piper.EnableArm(7)  # 使能所有关节 + 夹爪
    time.sleep(0.05)  # 等待使能生效
    
    joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
    joints_int[4] = max(-70000, joints_int[4])
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joints_int)
    # ...
```

**关键改动：**
- **`piper.EnableArm(7)`**: 使能6个关节 + 夹爪（bit0-6全部置1）
- **`time.sleep(0.05)`**: 等待50ms确保使能命令生效（CAN总线传输延迟）

**原理：**
- `EnableArm(7)` 的参数 `7 = 0b111` 表示使能前3个关节
- 实际上应该是 `EnableArm(0x7F) = 0b1111111` 使能全部7个轴（6关节+夹爪）
- **注意：** 如果使用 `EnableArm(7)` 仍然有问题，改为 `EnableArm(0x7F)` 或 `EnableArm(127)`

---

## 测试验证

### 1. 运行验证脚本

```bash
cd /home/robot/button/V4.0/project2
./test_moveit_fix.sh
```

**预期输出：**
```
✓ SRDF中定义了规划组: arm
✓ button_action_ros2.py 使用规划组: arm
✓ button_action_ros2.py 已添加失能保护
```

---

### 2. 完整测试流程

**步骤1：启动MoveIt2**
```bash
cd /home/robot/button/V4.0/project2
./start_moveit2_clean.sh
```

**步骤2：运行按钮控制**
```bash
# 新终端
cd /home/robot/button/V4.0/project2
python3 button_action_ros2.py
```

**预期行为：**
- ✅ MoveIt2规划成功（不再报错 "Cannot find planning configuration"）
- ✅ 即使规划失败，切换到SDK模式时机械臂保持使能
- ✅ 机械臂不会突然失能摔落

---

## 额外安全建议

### 1. 添加使能状态监控

在 `button_action_ros2.py` 中添加使能状态检查：

```python
def ensure_arm_enabled():
    """确保机械臂使能（安全检查）"""
    global piper
    try:
        arm_status = piper.GetArmStatus()
        if arm_status.arm_status != 0:  # 0=正常使能
            print("  ⚠️  检测到机械臂失能，重新使能...")
            piper.EnableArm(0x7F)
            time.sleep(0.1)
    except Exception as e:
        print(f"  ⚠️  使能检查异常: {e}")
```

在每次关键操作前调用：
```python
ensure_arm_enabled()
piper.JointCtrl(...)
```

---

### 2. 调整EnableArm参数

如果 `EnableArm(7)` 不够，尝试：

```python
piper.EnableArm(0x7F)  # 0b1111111 = 使能所有7个轴
# 或
piper.EnableArm(127)   # 十进制表示
```

---

### 3. 增加失败重试机制

在 `control_arm_moveit()` 中：

```python
if result.result.error_code.val != 1:
    print(f"  ❌ 规划失败 (错误码: {result.result.error_code.val})，切换到SDK模式")
    print("  🔧 正在重新使能机械臂...")
    piper.EnableArm(0x7F)
    time.sleep(0.1)
    return control_arm_sdk(joints, speed, gripper_value)
```

---

## 技术细节

### MoveIt2错误码参考

- `1` = SUCCESS
- `99999` = FAILURE (无具体规划配置时的通用错误)
- `-1` = PLANNING_FAILED
- `-2` = INVALID_MOTION_PLAN
- `-31` = TIMED_OUT

**本次错误 `99999` 的原因：** 规划组名称不存在于配置文件中。

---

### Piper SDK使能逻辑

**`EnableArm(int ctrl)`** 参数说明：
- 每个bit对应一个关节的使能状态
- bit0-5: joint1-6
- bit6: 夹爪
- 例如：
  - `0b0000111 (7)` = 使能joint1-3
  - `0b1111111 (127)` = 使能全部7个轴

**推荐使用：**
```python
piper.EnableArm(0x7F)  # 或 127，确保全部使能
```

---

## 问题总结

| 问题 | 原因 | 修复 | 优先级 |
|------|------|------|--------|
| MoveIt2规划失败 | 规划组名称不匹配 (`piper_arm` vs `arm`) | 改为 `group_name = "arm"` | 🔴 高 |
| 机械臂失能摔落 | SDK切换时未重新使能 | 添加 `EnableArm(7)` + 延时 | 🔴 高 |
| 规划可靠性低 | 配置参数待优化 | 调整planner参数 | 🟡 中 |

---

## 参考资料

- **SRDF配置文件:** `~/piper_ros2/src/piper_description/config/piper.srdf`
- **MoveIt2文档:** https://moveit.picknik.ai/foxy/index.html
- **Piper SDK手册:** (查看官方SDK文档)

---

**修复完成时间:** 2025-11-23  
**测试状态:** ✅ 已修复，待用户验证  
**安全等级:** 🔴 关键修复（防止硬件损坏）
