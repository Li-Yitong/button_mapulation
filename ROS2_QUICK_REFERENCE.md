# ROS2 快速参考

## 🚀 快速启动

```bash
# 1. 确保 ROS2 环境已配置
source /opt/ros/foxy/setup.bash

# 2. 启动系统
cd /home/robot/button/V4.0/project2
./start_vision_button_ros2.sh

# 3. 带可视化启动
./start_vision_button_ros2.sh --rviz
```

## 📁 ROS2 文件列表

| 文件名 | 说明 |
|--------|------|
| `vision_button_action_ros2.py` | 视觉按钮操作执行器 |
| `realsense_yolo_button_interactive_ros2.py` | 交互式按钮检测器 |
| `piper_tf_publisher_ros2.py` | TF2 发布器 |
| `start_vision_button_ros2.sh` | 启动脚本 |
| `install_ros2_dependencies.sh` | 依赖安装脚本 |
| `ROS2_MIGRATION_GUIDE.md` | 完整迁移指南 |

## 🔧 常用命令

### 话题 (Topics)
```bash
# 列出所有话题
ros2 topic list

# 查看话题数据
ros2 topic echo /object_point
ros2 topic echo /button_type

# 查看话题频率
ros2 topic hz /object_point

# 查看话题信息
ros2 topic info /object_point
```

### 节点 (Nodes)
```bash
# 列出所有节点
ros2 node list

# 查看节点信息
ros2 node info /vision_button_action_node

# 查看节点日志
ros2 node info /realsense_yolo_button_interactive
```

### TF (Transforms)
```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看特定 TF 变换
ros2 run tf2_ros tf2_echo arm_base link6

# 列出所有 TF
ros2 topic echo /tf
```

### 调试
```bash
# 查看系统状态
ros2 wtf

# 记录数据包
ros2 bag record -a

# 播放数据包
ros2 bag play <bag_file>
```

## 🔄 ROS1 vs ROS2 对照

| 功能 | ROS1 | ROS2 |
|-----|------|------|
| 启动核心 | `roscore` | ❌ 不需要 |
| 运行节点 | `rosrun pkg node` | `ros2 run pkg node` |
| 启动文件 | `roslaunch` | `ros2 launch` |
| 话题列表 | `rostopic list` | `ros2 topic list` |
| 节点列表 | `rosnode list` | `ros2 node list` |
| TF 查看 | `rosrun tf view_frames` | `ros2 run tf2_tools view_frames` |
| 参数服务器 | `rosparam` | `ros2 param` |

## 📡 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    ROS2 DDS Network                      │
│            (无需 roscore，节点直接通信)                  │
└─────────────────────────────────────────────────────────┘
           ↓                 ↓                  ↓
  ┌────────────────┐  ┌──────────────┐  ┌────────────────┐
  │ Button         │  │ TF2          │  │ Vision Button  │
  │ Detector       │  │ Publisher    │  │ Action         │
  │ (ROS2)         │  │ (ROS2)       │  │ (ROS2)         │
  └────────────────┘  └──────────────┘  └────────────────┘
       ↓ 发布             ↓ 发布             ↓ 订阅
  /object_point      /tf                /object_point
  /button_type       /tf_static         /button_type
                                             ↓
                                       执行动作
                                             ↓
                                     ┌──────────────┐
                                     │ Piper Arm    │
                                     │ (CAN SDK)    │
                                     └──────────────┘
```

## 🛠️ 故障排除

### 问题 1: 找不到 rclpy
```bash
# 解决方案
source /opt/ros/foxy/setup.bash
```

### 问题 2: 节点无法通信
```bash
# 检查节点是否运行
ros2 node list

# 检查话题
ros2 topic list

# 检查 DDS 配置
echo $ROS_DOMAIN_ID
```

### 问题 3: TF 不显示
```bash
# 检查 TF 话题
ros2 topic echo /tf --once

# 查看 TF 树
ros2 run tf2_tools view_frames
```

## 📦 依赖安装

```bash
# 运行依赖安装脚本
./install_ros2_dependencies.sh

# 或手动安装
sudo apt install ros-foxy-rclpy \
                 ros-foxy-std-msgs \
                 ros-foxy-geometry-msgs \
                 ros-foxy-visualization-msgs \
                 ros-foxy-tf2-ros
```

## 📚 更多信息

- 完整迁移指南: `ROS2_MIGRATION_GUIDE.md`
- ROS2 官方文档: https://docs.ros.org/en/foxy/
- 项目主文档: `README.md`

---

**提示**: 原 ROS1 文件（不带 `_ros2` 后缀）保持不变，可继续在 ROS1 环境中使用。
