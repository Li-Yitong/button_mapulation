# Piper ROS2 配置完成说明

## ✅ 已完成的配置

### 1. 安装的依赖

#### Python 包
```bash
✓ python-can
✓ scipy
✓ piper_sdk
```

#### ROS2 包
```bash
✓ ros-foxy-ros2-control
✓ ros-foxy-ros2-controllers
✓ ros-foxy-controller-manager
✓ ros-foxy-joint-state-publisher-gui
✓ ros-foxy-robot-state-publisher
✓ ros-foxy-xacro
✓ python3-colcon-common-extensions
```

### 2. 编译状态

```
✅ piper (Python 控制节点)
✅ piper_description (URDF 模型)
✅ piper_msgs (自定义消息)
```

---

## 🚀 使用方法

### 1. Source 环境

```bash
cd /home/robot/button/V4.0/project2/piper_ros
source install/setup.bash
```

### 2. 配置 CAN 接口

#### 单个机械臂（最简单）

```bash
# 激活 can0 接口，波特率 1000000
bash can_activate.sh can0 1000000
```

#### 多个机械臂

参考 README.MD 中的配置说明

### 3. 启动节点

#### 方式 A: 使用 launch 文件（推荐）

```bash
# 启动控制节点（不带 RViz）
ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false gripper_exist:=true

# 启动控制节点 + RViz 可视化
ros2 launch piper start_single_piper_rviz.launch.py can_port:=can0 auto_enable:=false gripper_exist:=true
```

#### 方式 B: 直接运行节点

```bash
ros2 run piper piper_single_ctrl --ros-args \
    -p can_port:=can0 \
    -p auto_enable:=false \
    -p gripper_exist:=true \
    -p rviz_ctrl_flag:=true
```

### 4. 使能机械臂

```bash
# 方式 1: 使用服务
ros2 service call /enable_srv piper_msgs/srv/Enable "{enable_request: true}"

# 方式 2: 发布话题
ros2 topic pub /enable_flag std_msgs/msg/Bool "{data: true}"
```

### 5. 控制机械臂

```bash
# 发送关节位置命令
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, \
name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'], \
position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01], \
velocity: [0,0,0,0,0,0,10], \
effort: [0,0,0,0,0,0,0.5]}"
```

---

## 📡 ROS2 话题和服务

### 话题 (Topics)

```bash
/arm_status              # 机械臂状态反馈
/enable_flag             # 使能标志位
/end_pose                # 末端位姿状态反馈
/joint_states            # 关节命令输入
/joint_states_single     # 关节状态反馈
/pos_cmd                 # 末端控制消息
```

### 服务 (Services)

```bash
/enable_srv              # 机械臂使能服务
```

---

## 🔧 参数说明

| 参数 | 类型 | 说明 | 默认值 |
|-----|------|------|-------|
| `can_port` | string | CAN 接口名称 | can0 |
| `auto_enable` | bool | 是否自动使能 | false |
| `gripper_exist` | bool | 是否有夹爪 | true |
| `rviz_ctrl_flag` | bool | 是否接收 RViz 控制 | true |

---

## 🎯 集成到你的项目

### 在启动脚本中使用

如果需要在 `start_vision_button_ros2.sh` 中集成 Piper ROS2 控制：

```bash
# 在启动脚本中添加
gnome-terminal --tab --title="piper_ros2" -- bash -c \
    "source /opt/ros/foxy/setup.bash && \
    source /home/robot/button/V4.0/project2/piper_ros/install/setup.bash && \
    ros2 launch piper start_single_piper.launch.py; exec bash"
```

### 在 Python 节点中使用

你的 ROS2 Python 节点可以直接订阅/发布 Piper 的话题：

```python
# 在 vision_button_action_ros2.py 中
from sensor_msgs.msg import JointState

# 创建发布者
self.joint_pub = self.create_publisher(
    JointState,
    '/joint_states',
    10
)

# 发送关节命令
msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'piper_single'
msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
msg.position = [0.2, 0.2, -0.2, 0.3, -0.2, 0.5, 0.01]
msg.velocity = [0, 0, 0, 0, 0, 0, 10]
msg.effort = [0, 0, 0, 0, 0, 0, 0.5]

self.joint_pub.publish(msg)
```

---

## ⚠️ 注意事项

### Python 环境

- **编译时**: 必须使用系统 Python 3.8（不能在 conda 环境中）
- **运行时**: 也必须使用系统 Python 3.8
- **已提供**: `build_piper_ros2.sh` 脚本自动处理环境问题

### CAN 接口

1. **必须先激活 CAN 接口**才能控制机械臂
2. 波特率必须设置为 **1000000**
3. 如果遇到连接问题，拔插 USB 并重启机械臂

### 使能状态

- 如果 `auto_enable:=false`，需要手动使能机械臂
- 程序中断后，机械臂保持上次的使能状态

---

## 📚 参考文档

- **Piper ROS2 官方文档**: `README.MD`
- **可视化调试**: 使用 `start_single_piper_rviz.launch.py`
- **自定义消息**: 查看 `src/piper_msgs/msg/` 和 `src/piper_msgs/srv/`

---

## 🎉 完成！

Piper ROS2 已成功配置，可以在 ROS2 环境中使用了。

**下一步**：
1. 连接机械臂
2. 激活 CAN 接口
3. 启动控制节点
4. 集成到你的视觉按钮操作系统

---

**配置日期**: 2025-11-21  
**ROS2 版本**: Foxy  
**状态**: ✅ 配置完成
