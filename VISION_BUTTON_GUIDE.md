# 视觉按钮操作系统 - 使用指南

## 📁 文件清单

### 核心文件
1. **`realsense_yolo_button_interactive.py`** (新建, ~450行)
   - 交互式按钮检测器
   - 使用 `yolo_button.pt` 模型检测按钮
   - 支持鼠标点击选择目标按钮
   - 发布按钮位置和类型

2. **`vision_button_action.py`** (新建, ~280行)
   - 视觉按钮操作整合器
   - 接收按钮位置和类型
   - 坐标转换：相机坐标系 → 基座坐标系
   - 自动调用 `button_actions.py` 的动作函数

3. **`start_vision_button.sh`** (新建, ~250行)
   - 一键启动脚本
   - 自动配置和启动所有节点
   - 支持命令行参数配置

### 复用文件
- **`button_actions.py`** (已有) - 提供四种按钮操作动作
- **`piper_tf_publisher.py`** (已有) - TF 坐标变换发布

---

## 🚀 快速启动

### 基础使用（推荐）
```bash
cd /home/robot/button/V4.0/project2
./start_vision_button.sh
```

### 高级选项
```bash
# 使用增量运动策略
./start_vision_button.sh --strategy incremental

# 启动 RViz 可视化
./start_vision_button.sh --rviz

# 仅使用 SDK 控制（不用 MoveIt）
./start_vision_button.sh --no-moveit

# 查看帮助
./start_vision_button.sh --help
```

---

## 📖 使用流程

### 步骤 1: 启动系统
```bash
./start_vision_button.sh
```
系统会自动启动 6 个终端窗口：
- 窗口1: roscore
- 窗口2: piper_control
- 窗口3: MoveIt move_group
- 窗口4: button_detector (相机+YOLO) ← **主要交互窗口**
- 窗口5: tf_publisher
- 窗口6: vision_button_action

### 步骤 2: 在检测器窗口中选择按钮

**窗口布局:**
```
┌─────────────────────────────────────────────┐
│  Step 1: Click on a button to select       │
│  Step 2: Press ENTER to confirm | ESC...   │
├─────────────────────────────────────────────┤
│                                             │
│  #0: toggle (0.89) ← 蓝色边框              │
│  ┌──────────┐                               │
│  │          │                               │
│  └──────────┘                               │
│                                             │
│           #1: push (0.92) ← 绿色边框(选中)  │
│           ┌──────────┐                      │
│           │          │                      │
│           └──────────┘                      │
│                                             │
└─────────────────────────────────────────────┘
```

**操作说明:**
1. **鼠标点击** 你想要操作的按钮 → 边框变绿色
2. **按 ENTER** 确认选择 → 机械臂开始执行
3. **按 ESC** 取消选择
4. **按 Q** 退出程序

### 步骤 3: 自动执行动作

系统会自动：
1. ✅ 识别按钮类型（toggle/plugin/push/knob）
2. ✅ 计算3D位置（相机坐标系）
3. ✅ 转换到基座坐标系
4. ✅ 调用对应的动作函数
5. ✅ 机械臂执行操作

### 步骤 4: 继续下一个按钮

操作完成后，可以继续在检测器窗口中选择下一个按钮。

---

## 🎯 支持的按钮类型

| 类型 | 描述 | 动作函数 |
|------|------|----------|
| **toggle** | 拨动开关 | `action_toggle()` |
| **plugin** | 插拔连接器 | `action_plugin()` |
| **push** | 按压按钮 | `action_push()` |
| **knob** | 旋转旋钮 | `action_knob()` |

---

## 📡 ROS 话题通信

### 发布的话题
```
button_detector 节点:
  /object_point        (geometry_msgs/PointStamped)  - 按钮3D位置
  /button_type         (std_msgs/String)             - 按钮类型
  /object_center_marker (visualization_msgs/Marker)  - 可视化标记

vision_button_action 节点:
  /target_button_base  (visualization_msgs/Marker)   - 基座坐标系标记
```

### 订阅的话题
```
vision_button_action 节点:
  /object_point        - 接收按钮位置
  /button_type         - 接收按钮类型
```

---

## ⚙️ 配置参数

### 运动策略选择

在 `start_vision_button.sh` 中可以选择三种运动策略：

```bash
# 1. 增量运动 (incremental) - 快速，逐步接近
./start_vision_button.sh --strategy incremental

# 2. 笛卡尔路径 (cartesian) - 丝滑，直线运动 (默认)
./start_vision_button.sh --strategy cartesian

# 3. 混合策略 (hybrid) - 平衡速度和精度
./start_vision_button.sh --strategy hybrid
```

### MoveIt 开关

```bash
# 使用 MoveIt (默认)
./start_vision_button.sh

# 仅使用 SDK
./start_vision_button.sh --no-moveit
```

---

## 🔧 故障排查

### 问题1: 相机被占用
```
RuntimeError: Device or resource busy
```
**解决方案:**
```bash
# 关闭占用的进程
ps aux | grep realsense | awk '{print $2}' | xargs kill -9

# 或使用重置脚本
./reset_camera.sh
```

### 问题2: YOLO 模型未找到
```
FileNotFoundError: yolo_button.pt
```
**解决方案:**
```bash
# 确保模型文件在当前目录
ls -lh yolo_button.pt

# 或修改代码使用其他模型
# 编辑 realsense_yolo_button_interactive.py:
model = YOLO("yolo11n.pt")  # 使用通用模型
```

### 问题3: 无法导入 button_actions
```
ModuleNotFoundError: No module named 'button_actions'
```
**解决方案:**
```bash
# 确保在正确的目录
cd /home/robot/button/V4.0/project2

# 检查文件是否存在
ls -lh button_actions.py
```

### 问题4: 坐标转换错误
```
AttributeError: 'PiperArm' object has no attribute 'link6_q_camera'
```
**解决方案:**
- 确保已正确标定相机到末端的变换
- 检查 `piper_arm.py` 中是否有标定数据

---

## 📊 性能优化

### 检测速度优化
在 `realsense_yolo_button_interactive.py` 中调整：
```python
# 降低置信度阈值（检测更多）
YOLODetection(image, conf_threshold=0.3)

# 提高置信度阈值（检测更准确）
YOLODetection(image, conf_threshold=0.7)
```

### 点云采样优化
```python
# 增加采样步长（更快，但精度降低）
step = 10  # 默认 5

# 减小采样步长（更精确，但更慢）
step = 3
```

---

## 🎓 代码结构

### 核心逻辑流程
```python
# realsense_yolo_button_interactive.py
相机采集 → YOLO检测 → 用户选择 → 计算3D位置 → 发布话题

# vision_button_action.py
订阅话题 → 获取关节角 → 坐标转换 → 调用动作函数 → 执行操作
```

### 坐标转换链
```
按钮(相机坐标系) 
  → link6_T_cam (相机到末端) 
  → base_T_link6 (末端到基座, FK)
  → 按钮(基座坐标系)
```

---

## 💡 扩展功能建议

### 1. 添加新按钮类型
在 `button_actions.py` 中添加新动作：
```python
def action_new_button():
    """新按钮类型的动作"""
    # 实现动作逻辑
    pass
```

在 `vision_button_action.py` 中注册：
```python
action_map = {
    'toggle': action_toggle,
    'plugin': action_plugin,
    'push': action_push,
    'knob': action_knob,
    'new_button': action_new_button,  # 添加新类型
}
```

### 2. 训练自定义 YOLO 模型
- 收集按钮图像数据集
- 使用 Ultralytics 工具训练
- 替换 `yolo_button.pt`

### 3. 添加安全检查
在 `vision_button_action.py` 中添加：
```python
# 检查按钮位置是否在工作空间内
if button_base[0] < 0.2 or button_base[0] > 0.6:
    print("警告: 按钮位置超出安全范围")
    return False
```

---

## 📞 技术支持

如遇问题，请检查：
1. ROS 节点状态: `rosnode list`
2. 话题数据: `rostopic echo /object_point`
3. TF 树: `rosrun tf view_frames`
4. 日志信息: 查看各终端窗口的输出

---

## ✅ 测试清单

启动系统后，验证以下功能：

- [ ] 相机图像正常显示
- [ ] YOLO 能检测到按钮（显示蓝色边框）
- [ ] 鼠标点击可选择按钮（边框变绿）
- [ ] 按 ENTER 后发布话题
- [ ] vision_button_action 接收到数据
- [ ] 坐标转换正确（基座坐标系）
- [ ] 机械臂执行对应动作
- [ ] 动作完成后可继续选择

---

## 🎉 完成！

现在您可以使用交互式按钮操作系统了！

启动命令:
```bash
cd /home/robot/button/V4.0/project2
./start_vision_button.sh
```

祝使用愉快！🚀
