# ROS2 系统快速测试指南

## 🚀 测试前准备

### 1. 环境检查
```bash
# 进入项目目录
cd /home/robot/button/V4.0/project2

# 检查 ROS2 环境
source /opt/ros/foxy/setup.bash
source ~/ros2_foxy_ws/install/setup.bash
echo $ROS_DISTRO  # 应该输出 foxy

# 检查 Python 依赖
python3 -c "
import rclpy
import piper_sdk
import pyrealsense2
from ultralytics import YOLO
print('✓ 所有依赖已安装')
"
```

### 2. 硬件检查
```bash
# 检查 CAN 接口
ip link show can0
ip link show can1

# 检查 RealSense 相机
rs-enumerate-devices | head -10

# 检查 YOLO 模型
ls -lh yolo_button.pt
```

---

## 🧪 分步测试

### 测试 1: Piper TF 发布器

**目的**: 验证机械臂 TF 坐标变换发布是否正常

```bash
# 终端1: 启动 TF 发布器
source /opt/ros/foxy/setup.bash
cd /home/robot/button/V4.0/project2
python3 piper_tf_publisher_ros2.py
```

**预期输出**:
```
======================================================================
Piper TF Publisher - ROS2 版本
======================================================================
✓ Piper SDK 初始化成功
✓ PiperArm 运动学模块初始化成功
✓ TF 发布器已启动 (10Hz)
======================================================================
```

**验证命令** (终端2):
```bash
# 查看节点
ros2 node list
# 应该看到: /piper_tf_publisher

# 查看 TF
ros2 run tf2_ros tf2_echo arm_base link6
# 应该输出实时的 TF 变换

# 查看 TF 树
ros2 run tf2_tools view_frames
# 会生成 frames.pdf
evince frames.pdf
```

**成功标准**: ✅ 能看到完整的 TF 树 (arm_base → link1 → ... → link6 → camera)

---

### 测试 2: RealSense 相机节点

**目的**: 验证 RealSense ROS2 驱动是否正常工作

```bash
# 终端1: 启动相机
source /opt/ros/foxy/setup.bash
source ~/ros2_foxy_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
    depth_module.depth_profile:=640x480x30 \
    rgb_camera.color_profile:=640x480x30
```

**预期输出**:
```
[INFO] [realsense2_camera_node]: RealSense ROS v4.56.4
[INFO] [realsense2_camera_node]: Device with serial number 405622076497 was found
[INFO] [realsense2_camera_node]: Device Name: Intel RealSense D435I
```

**验证命令** (终端2):
```bash
# 查看话题
ros2 topic list | grep camera
# 应该看到:
# /camera/camera/color/image_raw
# /camera/camera/depth/image_rect_raw
# /camera/camera/color/camera_info
# ...

# 查看图像帧率
ros2 topic hz /camera/camera/color/image_raw
# 应该约 30Hz

# 查看图像（需要安装 rqt_image_view）
ros2 run rqt_image_view rqt_image_view
```

**成功标准**: ✅ 话题正常发布，图像显示正常

---

### 测试 3: 按钮检测器

**目的**: 验证 YOLO 按钮检测和用户交互是否正常

```bash
# 终端1: 保持相机运行（测试2）

# 终端2: 启动按钮检测器
source /opt/ros/foxy/setup.bash
cd /home/robot/button/V4.0/project2
python3 realsense_yolo_button_interactive_ros2.py
```

**预期输出**:
```
======================================================================
交互式按钮检测器 - ROS2 版本
======================================================================
✓ YOLO 模型加载成功: yolo_button.pt
✓ 发布器已创建
✓ RealSense 相机已启动
======================================================================
```

**操作步骤**:
1. 将按钮放在相机视野内
2. 检测窗口应该显示蓝色边框的按钮
3. 用鼠标点击一个按钮（边框变绿色）
4. 按 ENTER 确认选择

**验证命令** (终端3):
```bash
# 监听按钮位置
ros2 topic echo /object_point

# 监听按钮类型
ros2 topic echo /button_type
```

**成功标准**: 
- ✅ 检测窗口正常显示
- ✅ 能用鼠标选择按钮
- ✅ ENTER 后发布 3D 坐标和类型到话题

---

### 测试 4: 动作执行器

**目的**: 验证完整的检测→执行流程

```bash
# 终端1: 保持相机运行
# 终端2: 保持按钮检测器运行

# 终端3: 启动 TF 发布器
source /opt/ros/foxy/setup.bash
cd /home/robot/button/V4.0/project2
python3 piper_tf_publisher_ros2.py

# 终端4: 启动动作执行器
source /opt/ros/foxy/setup.bash
cd /home/robot/button/V4.0/project2
python3 vision_button_action_ros2.py
```

**预期输出**:
```
======================================================================
视觉按钮操作整合器 - ROS2 版本
======================================================================
[1/2] 初始化 Piper SDK...
  ✓ Piper SDK 初始化成功
[2/2] 初始化 PiperArm...
  ✓ PiperArm 初始化成功
✓ 硬件初始化完成
✓ ROS2 节点已初始化
✓ 等待接收按钮信息...
```

**操作步骤**:
1. 在按钮检测器窗口点击选择按钮
2. 按 ENTER 确认
3. 观察机械臂是否执行动作

**成功标准**: 
- ✅ 接收到按钮信息
- ✅ 坐标转换成功
- ✅ 机械臂移动到按钮位置
- ✅ 执行对应的按钮操作（按压/拨动/插拔/旋转）

---

## 🎯 完整系统测试

### 使用启动脚本测试

```bash
# 1. 进入项目目录
cd /home/robot/button/V4.0/project2

# 2. 确保环境正确
source /opt/ros/foxy/setup.bash
source ~/ros2_foxy_ws/install/setup.bash

# 3. 运行启动脚本
./start_vision_button_ros2.sh
```

**验证步骤**:

1. **检查所有节点是否启动**:
```bash
ros2 node list
# 应该看到:
# /piper_tf_publisher
# /camera/camera
# /button_detector_ros2
# /vision_button_action_ros2
```

2. **检查所有话题**:
```bash
ros2 topic list
# 关键话题:
# /object_point
# /button_type
# /camera/camera/color/image_raw
# /camera/camera/depth/image_rect_raw
# /tf
# /tf_static
```

3. **端到端测试**:
   - 在检测窗口点击按钮
   - 按 ENTER 确认
   - 观察机械臂执行动作
   - 验证动作完成后状态

---

## ⚠️ 常见问题排查

### 问题 1: TF 发布器启动失败
```
错误: 无法连接到 CAN 接口
```
**解决**:
```bash
# 检查 CAN 接口状态
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 问题 2: 相机节点无法启动
```
错误: Device or resource busy
```
**解决**:
```bash
# 重置相机
sudo bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/unbind'
sleep 2
sudo bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/bind'
```

### 问题 3: 按钮检测器无图像
```
错误: pyrealsense2 import 失败
```
**解决**:
- 选项A: 安装 pyrealsense2: `pip3 install pyrealsense2`
- 选项B: 修改检测器订阅 ROS2 话题而非直接读取相机

### 问题 4: 机械臂不移动
```
错误: Piper SDK 初始化失败
```
**解决**:
```bash
# 检查 can0 是否启动
ip link show can0

# 手动启动
sudo ip link set can0 up
```

### 问题 5: 坐标转换错误
```
错误: 机械臂移动到错误位置
```
**解决**:
- 检查手眼标定数据是否正确（`piper_arm.py` 中的参数）
- 检查 TCP 偏移参数是否正确（`vision_button_action_ros2.py` 中的 `TCP_OFFSET_LOCAL`）

---

## 📊 性能指标

| 指标 | 目标值 | 验证方法 |
|------|--------|---------|
| TF 发布频率 | 10 Hz | `ros2 topic hz /tf` |
| 相机图像帧率 | 30 Hz | `ros2 topic hz /camera/camera/color/image_raw` |
| 检测延迟 | <100ms | 点击到发布话题的时间 |
| 动作执行时间 | <5s | 从确认到完成动作的时间 |
| 定位精度 | ±5mm | 测量实际到达位置与目标的偏差 |

---

## ✅ 测试通过标准

### 基础功能测试
- [ ] TF 发布器正常运行
- [ ] 相机节点正常发布图像
- [ ] 按钮检测器能识别按钮
- [ ] 用户能点击选择按钮
- [ ] 按钮位置和类型正确发布到话题

### 集成测试
- [ ] 动作执行器能接收按钮信息
- [ ] 坐标转换计算正确
- [ ] 机械臂能移动到按钮位置
- [ ] 能执行正确的按钮操作

### 性能测试
- [ ] 所有节点稳定运行 >5分钟
- [ ] 无异常崩溃或错误
- [ ] 帧率和频率符合预期
- [ ] 定位精度符合要求

---

## 📝 测试记录模板

```
测试日期: ___________
测试人员: ___________

环境信息:
- ROS2 版本: Foxy
- Ubuntu 版本: 20.04
- Python 版本: 3.8

硬件状态:
- Piper 机械臂: [ ] 正常  [ ] 异常
- RealSense 相机: [ ] 正常  [ ] 异常
- CAN 接口: [ ] 正常  [ ] 异常

测试结果:
- 测试 1 (TF发布): [ ] 通过  [ ] 失败
- 测试 2 (相机): [ ] 通过  [ ] 失败
- 测试 3 (检测器): [ ] 通过  [ ] 失败
- 测试 4 (执行器): [ ] 通过  [ ] 失败
- 完整系统: [ ] 通过  [ ] 失败

问题记录:
1. _______________
2. _______________

备注:
_______________
_______________
```

---

## 🎯 下一步

测试通过后：
1. 记录测试结果
2. 优化性能参数
3. 创建用户文档
4. 准备演示视频
