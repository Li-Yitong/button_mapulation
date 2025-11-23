# ROS1 到 ROS2 迁移指南

## 📋 概述

本文档记录了项目从 ROS1 Noetic 迁移到 ROS2 Foxy 的主要变化和使用说明。

**迁移日期**: 2025-11-21  
**目标版本**: ROS2 Foxy (Ubuntu 20.04)

---

## 🎯 主要变化

### 1. **核心库替换**

| ROS1 | ROS2 | 说明 |
|------|------|------|
| `rospy` | `rclpy` | Python 客户端库 |
| `roslaunch` | `ros2 launch` | 启动文件系统 |
| `rosrun` | `ros2 run` | 运行节点 |
| `roscore` | ❌ 不需要 | ROS2 使用 DDS 通信 |
| `rostopic` | `ros2 topic` | 话题工具 |
| `rosnode` | `ros2 node` | 节点工具 |
| `rosservice` | `ros2 service` | 服务工具 |

### 2. **节点初始化**

#### ROS1:
```python
import rospy
rospy.init_node('my_node', anonymous=True)
```

#### ROS2:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

rclpy.init(args=args)
node = MyNode()
rclpy.spin(node)
```

### 3. **发布者 (Publisher)**

#### ROS1:
```python
pub = rospy.Publisher('/topic_name', String, queue_size=10)
msg = String()
msg.data = "hello"
pub.publish(msg)
```

#### ROS2:
```python
self.pub = self.create_publisher(String, '/topic_name', 10)
msg = String()
msg.data = "hello"
self.pub.publish(msg)
```

### 4. **订阅者 (Subscriber)**

#### ROS1:
```python
def callback(msg):
    print(msg.data)

rospy.Subscriber('/topic_name', String, callback, queue_size=10)
```

#### ROS2:
```python
def callback(self, msg):
    self.get_logger().info(msg.data)

self.sub = self.create_subscription(
    String,
    '/topic_name',
    self.callback,
    10
)
```

### 5. **定时器 (Timer)**

#### ROS1:
```python
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # do something
    rate.sleep()
```

#### ROS2:
```python
# 在 __init__ 中创建定时器
self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

def timer_callback(self):
    # do something
    pass
```

### 6. **日志输出**

#### ROS1:
```python
rospy.loginfo("Info message")
rospy.logwarn("Warning message")
rospy.logerr("Error message")
```

#### ROS2:
```python
self.get_logger().info("Info message")
self.get_logger().warn("Warning message")
self.get_logger().error("Error message")
```

### 7. **TF 广播**

#### ROS1:
```python
import tf
br = tf.TransformBroadcaster()
br.sendTransform(translation, rotation, time, child, parent)
```

#### ROS2:
```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

self.tf_broadcaster = TransformBroadcaster(self)

t = TransformStamped()
t.header.stamp = self.get_clock().now().to_msg()
t.header.frame_id = parent
t.child_frame_id = child
t.transform.translation.x = translation[0]
t.transform.translation.y = translation[1]
t.transform.translation.z = translation[2]
t.transform.rotation.x = rotation[0]
t.transform.rotation.y = rotation[1]
t.transform.rotation.z = rotation[2]
t.transform.rotation.w = rotation[3]

self.tf_broadcaster.sendTransform(t)
```

### 8. **时间戳**

#### ROS1:
```python
now = rospy.Time.now()
msg.header.stamp = now
```

#### ROS2:
```python
now = self.get_clock().now()
msg.header.stamp = now.to_msg()
```

---

## 📁 新增的 ROS2 文件

本次迁移创建了以下新文件（原 ROS1 文件保持不变）：

### Python 节点文件

1. **`vision_button_action_ros2.py`**
   - 视觉按钮操作整合器（ROS2 版本）
   - 订阅按钮位置和类型，执行相应动作
   - 使用面向对象的 Node 类

2. **`realsense_yolo_button_interactive_ros2.py`**
   - 交互式按钮检测器（ROS2 版本）
   - YOLO 检测 + 用户点击选择
   - 发布按钮信息到 ROS2 话题

3. **`piper_tf_publisher_ros2.py`**
   - TF2 发布器（ROS2 版本）
   - 发布机械臂的 TF 变换
   - 使用 TransformBroadcaster

### 启动脚本

4. **`start_vision_button_ros2.sh`**
   - ROS2 系统启动脚本
   - 不需要 roscore
   - 启动所有 ROS2 节点

---

## 🚀 使用方法

### 前置条件

1. **安装 ROS2 Foxy**
   ```bash
   # 已在之前安装完成
   source /opt/ros/foxy/setup.bash
   ```

2. **确保环境正确**
   ```bash
   echo $ROS_DISTRO  # 应该输出: foxy
   echo $ROS_VERSION  # 应该输出: 2
   ```

3. **安装 Python 依赖**
   ```bash
   conda activate button
   pip install rclpy geometry-msgs std-msgs visualization-msgs
   ```

### 启动系统

#### 方式 1: 使用启动脚本（推荐）

```bash
cd /home/robot/button/V4.0/project2
./start_vision_button_ros2.sh
```

可选参数：
- `--rviz`: 启动 RViz2 可视化
- `--help`: 显示帮助信息

#### 方式 2: 手动启动各节点

```bash
# 终端 1: 按钮检测器
conda activate button
python3 realsense_yolo_button_interactive_ros2.py

# 终端 2: TF 发布器
conda activate button
python3 piper_tf_publisher_ros2.py

# 终端 3: 视觉按钮操作执行器
conda activate button
python3 vision_button_action_ros2.py
```

---

## 🔧 调试工具

### ROS2 命令行工具

```bash
# 查看所有话题
ros2 topic list

# 查看话题数据
ros2 topic echo /object_point
ros2 topic echo /button_type

# 查看话题信息
ros2 topic info /object_point

# 查看节点列表
ros2 node list

# 查看节点信息
ros2 node info /vision_button_action_node

# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看 TF 变换
ros2 run tf2_ros tf2_echo arm_base link6
```

---

## 📡 ROS2 话题说明

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/object_point` | `geometry_msgs/PointStamped` | 按钮3D位置（相机坐标系） |
| `/button_type` | `std_msgs/String` | 按钮类型 (toggle/plugin/push/knob) |
| `/target_button_base` | `visualization_msgs/Marker` | 可视化标记（基座坐标系） |
| `/object_center_marker` | `visualization_msgs/Marker` | 按钮中心标记 |

---

## ⚙️ 配置差异

### 1. 环境变量

**ROS1**:
```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

**ROS2**:
```bash
source /opt/ros/foxy/setup.bash
# 不需要 ROS_MASTER_URI (使用 DDS)
```

### 2. 通信机制

- **ROS1**: 使用 XML-RPC + TCP/UDP，需要 roscore 作为中心节点
- **ROS2**: 使用 DDS (Data Distribution Service)，节点直接通信，无需中心节点

### 3. Launch 文件

**ROS1**: XML 格式 (`.launch`)
```xml
<launch>
  <node pkg="my_package" type="my_node" name="my_node"/>
</launch>
```

**ROS2**: Python 格式 (`.launch.py`)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node'
        )
    ])
```

---

## 🔄 兼容性说明

### 保持不变的部分

1. **硬件接口**
   - `piper_sdk` (CAN 总线通信)
   - `piper_arm` (运动学计算)
   - `button_actions` (动作执行逻辑)

2. **视觉处理**
   - RealSense SDK
   - YOLO 模型
   - OpenCV 图像处理

3. **数学库**
   - NumPy
   - 坐标变换函数

### 暂不支持的功能

1. **MoveIt2**
   - ROS2 版本暂时使用 SDK 直接控制
   - MoveIt2 for Foxy 可后续集成

2. **RViz 配置文件**
   - ROS1 的 `.rviz` 文件需要在 RViz2 中重新配置

---

## 📝 迁移检查清单

- [x] 替换 `rospy` 为 `rclpy`
- [x] 使用面向对象的 Node 类
- [x] 更新 Publisher/Subscriber 语法
- [x] 更新 TF 广播器为 TF2
- [x] 移除 roscore 依赖
- [x] 更新时间戳获取方式
- [x] 更新日志输出方式
- [x] 创建 ROS2 启动脚本
- [x] 测试节点通信
- [ ] 集成 MoveIt2 (待后续)
- [ ] 转换 launch 文件为 Python 格式 (可选)

---

## 🐛 常见问题

### Q1: 找不到 rclpy 模块

**A**: 确保已 source ROS2 环境：
```bash
source /opt/ros/foxy/setup.bash
```

### Q2: 节点无法通信

**A**: 检查 DDS 配置和网络设置：
```bash
# 检查环境变量
echo $ROS_DOMAIN_ID

# 查看节点列表
ros2 node list
```

### Q3: ROS1 和 ROS2 混用问题

**A**: 不要在同一个终端同时 source ROS1 和 ROS2：
```bash
# 错误示范
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash  # 会导致冲突

# 正确做法：在不同终端中使用
```

### Q4: TF 变换不显示

**A**: 确保 TF 发布器正在运行：
```bash
ros2 topic list | grep /tf
ros2 topic echo /tf --once
```

---

## 📚 参考资源

1. **ROS2 官方文档**
   - [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/)
   - [Migrating from ROS1](https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html)

2. **ROS2 教程**
   - [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
   - [Understanding ROS2 nodes](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

3. **TF2 教程**
   - [Writing a tf2 broadcaster (Python)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html)

---

## 💡 下一步计划

1. **集成 MoveIt2**
   - 安装 MoveIt2 for Foxy
   - 配置机械臂描述文件
   - 测试轨迹规划

2. **创建 Python Launch 文件**
   - 替代 bash 启动脚本
   - 统一管理所有节点

3. **性能优化**
   - 调整 DDS QoS 设置
   - 优化话题通信频率

4. **添加服务接口**
   - 提供更灵活的控制接口
   - 支持同步调用

---

## 📞 联系方式

如有问题或建议，请参考项目 README.md 或联系项目维护者。

---

**最后更新**: 2025-11-21  
**作者**: GitHub Copilot  
**项目**: Piper 按钮操作系统
