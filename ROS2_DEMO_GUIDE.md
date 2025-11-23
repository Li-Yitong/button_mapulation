# ROS2 Demo 测试说明

## 测试环境
- **系统**: Ubuntu 20.04
- **ROS 版本**: ROS2 Foxy
- **Python 版本**: 3.8 (系统 Python)
- **硬件**: Piper 6-DOF 机械臂 + 夹爪
- **通信**: CAN0 (can0 @ 1Mbps)

## Demo 列表

### Demo 1: 读取机械臂状态 ✅ 已测试
**文件**: `demo_01_read_status_ros2.py`

**功能**: 实时读取并显示机械臂的关节角度和末端执行器位置

**使用方法**:
```bash
cd /home/robot/button/V4.0/project2
source /opt/ros/foxy/setup.bash
/usr/bin/python3 demo_01_read_status_ros2.py
```

**测试结果**: ✅ **成功**
- ROS2 节点正常启动
- CAN 通信正常
- 能够实时读取关节角度（所有关节均为 0°）
- 能够读取末端执行器位姿
- 更新频率：1Hz

**注意**: 数据输出使用 SDK 原始单位，可能需要单位转换


### Demo 2: 使能/失能机械臂
**文件**: `demo_02_enable_arm_ros2.py`

**功能**: 测试机械臂的使能和失能控制

**使用方法**:
```bash
cd /home/robot/button/V4.0/project2
source /opt/ros/foxy/setup.bash
/usr/bin/python3 demo_02_enable_arm_ros2.py
```

**功能描述**:
1. 使能机械臂（所有关节+夹爪）
2. 检查使能状态
3. 等待 3 秒
4. 失能机械臂

**安全提示**: ⚠️ 机械臂使能后会锁定，请确保周围无障碍物


### Demo 3: 归零功能
**文件**: `demo_03_go_zero_ros2.py`

**功能**: 将所有关节移动到 0 度位置

**使用方法**:
```bash
cd /home/robot/button/V4.0/project2
source /opt/ros/foxy/setup.bash
/usr/bin/python3 demo_03_go_zero_ros2.py
```

**功能描述**:
1. 使能机械臂
2. 显示当前关节角度
3. 移动所有关节到 0°
4. 监控运动状态直到到达目标（误差 < 1°）
5. 显示最终位置
6. 失能机械臂

**安全提示**: ⚠️ **请确保机械臂运动空间无障碍物！**


### Demo 4: 夹爪控制
**文件**: `demo_04_gripper_control_ros2.py`

**功能**: 测试夹爪的开合和位置控制

**使用方法**:
```bash
cd /home/robot/button/V4.0/project2
source /opt/ros/foxy/setup.bash
/usr/bin/python3 demo_04_gripper_control_ros2.py
```

**功能描述**:
1. 使能机械臂和夹爪
2. 显示初始夹爪位置
3. 打开夹爪（angle = 0°）
4. 关闭夹爪（angle = 800°）
5. 部分打开夹爪（angle = 400°）
6. 再次打开夹爪
7. 失能机械臂

**夹爪参数**:
- 范围：0° (完全打开) ~ 800° (接近关闭)
- 速度：500 (固定)

**安全提示**: ⚠️ **确保夹爪附近无障碍物！**


## 一键运行脚本

**文件**: `run_ros2_demos.sh`

**功能**: 交互式菜单选择并运行 Demo

**使用方法**:
```bash
cd /home/robot/button/V4.0/project2
./run_ros2_demos.sh
```

**菜单选项**:
```
1. Demo 1 - 读取机械臂状态
2. Demo 2 - 使能/失能机械臂
3. Demo 3 - 归零功能
4. Demo 4 - 夹爪控制
0. 退出
```


## 测试前检查

### 1. 检查 CAN 接口
```bash
ifconfig | grep can0
```
应该显示：`can0: flags=193<UP,RUNNING,NOARP>`

### 2. 检查 ROS2 环境
```bash
source /opt/ros/foxy/setup.bash
ros2 --version
```
应该显示：`ros2 cli version: 0.9.x`

### 3. 检查 Python 版本
```bash
/usr/bin/python3 --version
```
应该显示：`Python 3.8.x`

### 4. 检查 piper_sdk
```bash
/usr/bin/python3 -c "from piper_sdk import C_PiperInterface_V2; print('piper_sdk OK')"
```
应该显示：`piper_sdk OK`


## 常见问题

### Q1: 提示 "can't open file"
**原因**: 工作目录不正确
**解决**: 确保在 `/home/robot/button/V4.0/project2` 目录下运行

### Q2: 提示 "无法解析导入 rclpy"
**原因**: 使用了 conda Python 而不是系统 Python
**解决**: 使用 `/usr/bin/python3` 而不是 `python3`

### Q3: 提示 "CAN bus error"
**原因**: CAN 接口未启动
**解决**: 
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### Q4: 机械臂不响应
**原因**: 机械臂未使能或电源未开启
**解决**: 
1. 检查机械臂电源
2. 运行 Demo 2 先使能机械臂
3. 检查 CAN 通信是否正常


## 测试建议顺序

建议按以下顺序测试 Demo：

1. **Demo 1** (读取状态) - 验证基本通信
2. **Demo 2** (使能/失能) - 验证控制权限
3. **Demo 4** (夹爪控制) - 先测试简单动作
4. **Demo 3** (归零功能) - 最后测试整臂运动

**原因**: 从简单到复杂，先验证通信和基本控制，再进行复杂运动


## 下一步计划

测试完这些基础 Demo 后，可以进行：

1. **集成测试**: 运行完整的视觉按钮检测 + 机械臂操作系统
   ```bash
   ./start_vision_button_ros2.sh
   ```

2. **性能优化**: 根据测试结果调整运动速度、加速度参数

3. **实际任务**: 使用摄像头检测按钮并执行抓取/按压操作


## 文件权限

所有 Demo 文件和脚本已添加执行权限：
```bash
chmod +x demo_*.py
chmod +x run_ros2_demos.sh
```


## 技术要点

### 使用 ROS2 系统 Python
**重要**: 所有 ROS2 节点必须使用系统 Python 3.8，不能使用 conda Python 3.9

```bash
# ✅ 正确
/usr/bin/python3 demo_01_read_status_ros2.py

# ✗ 错误
python3 demo_01_read_status_ros2.py  # 可能调用 conda Python
```

### piper_sdk API 注意事项

常用方法（已验证）：
- `GetArmJointMsgs()` - 获取关节角度
- `GetArmEndPoseMsgs()` - 获取末端位姿 (不是 GetArmEndPose)
- `GetArmGripperMsgs()` - 获取夹爪状态
- `EnableArm(7)` - 使能机械臂（7 = 所有关节+夹爪）
- `DisableArm(7)` - 失能机械臂
- `GripperCtrl(angle, speed, enable)` - 控制夹爪
- `MotionCtrl_2(...)` - 关节运动控制

### 数据格式

关节角度访问：
```python
msg = piper.GetArmJointMsgs()
j1 = msg.joint_state.joint_1  # 访问单个关节
```

末端位姿访问：
```python
pose = piper.GetArmEndPoseMsgs()
x = pose.end_pose.X_axis
y = pose.end_pose.Y_axis
z = pose.end_pose.Z_axis
rx = pose.end_pose.RX_axis
ry = pose.end_pose.RY_axis
rz = pose.end_pose.RZ_axis
```


## 联系信息

如有问题，请参考：
- `ROS2_MIGRATION_GUIDE.md` - ROS2 迁移完整指南
- `ROS2_PYTHON_FIX.md` - Python 环境问题解决
- `PIPER_ROS2_SETUP.md` - Piper ROS2 配置说明

---

**最后更新**: 2024
**测试状态**: Demo 1 已验证 ✅
