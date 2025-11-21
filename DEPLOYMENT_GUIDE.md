# 项目部署指南

本文档说明如何在新机器上完整部署本项目。

## 目录结构

```
project2/
├── piper_ros/          # 松灵机械臂 Piper ROS 包（需要编译）
├── piper_sdk/          # 松灵机械臂 Piper SDK（需要安装）
├── config/             # 配置文件
├── launch/             # ROS launch 文件
├── *.py                # Python 脚本
├── *.sh                # Shell 启动脚本
└── requirements        # Python 依赖
```

## 一、前置条件

### 1. 操作系统
- Ubuntu 20.04 LTS (推荐)
- ROS Noetic

### 2. 硬件要求
- Intel RealSense D435i 深度相机
- 松灵机械臂 Piper (带夹爪)
- CAN 接口（用于机械臂通信）

## 二、系统依赖安装

### 1. 安装 ROS Noetic

```bash
# 设置 sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 设置密钥
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 安装 ROS Noetic 完整版
sudo apt update
sudo apt install ros-noetic-desktop-full

# 初始化 rosdep
sudo rosdep init
rosdep update

# 设置环境变量（添加到 ~/.bashrc）
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 安装 MoveIt

```bash
sudo apt install ros-noetic-moveit
```

### 3. 安装 RealSense SDK

```bash
# 添加 Intel 仓库
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# 安装 SDK
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# 验证安装
realsense-viewer
```

### 4. 安装其他系统依赖

```bash
sudo apt install -y \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    git \
    can-utils
```

## 三、项目部署

### 1. 克隆/复制项目

将整个 `project2` 文件夹复制到目标机器，建议路径：
```bash
# 可以是任意路径，项目已经使用相对路径
mkdir -p ~/robot_workspace
cd ~/robot_workspace
# 将 project2 文件夹放在这里
```

**重要提示**：项目已经配置为使用相对路径，可以放在任意目录。

### 2. 配置 piper_sdk

`piper_sdk` 是松灵官方提供的 Python SDK。

#### 方式一：从本项目安装（推荐）

```bash
cd project2/piper_sdk
sudo python3 setup.py install
```

#### 方式二：从官方仓库安装

```bash
git clone https://github.com/agilexrobotics/piper_sdk.git
cd piper_sdk
sudo python3 setup.py install
```

验证安装：
```bash
python3 -c "from piper_sdk import *; print('piper_sdk installed successfully')"
```

### 3. 编译 piper_ros

`piper_ros` 是松灵官方提供的 ROS 包，包含 MoveIt 配置。

#### 方式一：使用本项目的 piper_ros（推荐）

本项目已包含编译好的 `piper_ros`，但需要在新机器上重新编译：

```bash
cd project2/piper_ros

# 清理旧的编译文件（如果有）
rm -rf build devel

# 重新编译
catkin_make

# 验证编译
source devel/setup.bash
rospack list | grep piper
```

#### 方式二：从官方仓库获取

```bash
# 在 project2 目录外的临时位置
git clone https://github.com/agilexrobotics/piper_ros.git
cd piper_ros
catkin_make

# 然后将编译好的 piper_ros 复制到 project2/
```

**重要说明**：
- `piper_ros/build/` 和 `piper_ros/devel/` 是编译生成的，不应该直接复制
- 每台新机器必须重新编译 `piper_ros`
- 编译后会生成机器相关的配置文件

### 4. 配置 CAN 接口

松灵机械臂通过 CAN 总线通信，需要配置 CAN 接口：

```bash
cd project2/piper_sdk

# 查找 CAN 端口
./piper_sdk/find_all_can_port.sh

# 激活 CAN 接口（假设是 can0）
sudo ./piper_sdk/can_activate.sh can0

# 或使用自动查找并配置
sudo ./piper_sdk/can_find_and_config.sh
```

**注意**：每次重启后需要重新激活 CAN 接口。

### 5. 创建 Conda 环境

项目使用 conda 环境来隔离 Python 依赖（特别是视觉相关的库）：

```bash
# 安装 Miniconda（如果还没有）
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# 创建 conda 环境
conda create -n button python=3.8 -y
conda activate button

# 安装依赖
cd project2
pip install -r requirements

# 安装额外的依赖
pip install pyrealsense2 opencv-python ultralytics
```

**requirements 文件内容示例**：
```
numpy
opencv-python
pyrealsense2
ultralytics
matplotlib
scipy
```

### 6. 下载 YOLO 模型

项目使用 YOLO 进行按钮检测：

```bash
cd project2

# 如果有自定义训练的模型
# 将 yolo_button.pt 放在 project2/ 目录下

# 或下载通用模型作为替代
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.pt
```

## 四、验证安装

### 1. 测试系统

```bash
cd project2
./test_system.sh
```

这会检查：
- ROS 环境
- MoveIt 安装
- Conda 环境
- 相机连接
- CAN 接口

### 2. 测试相机

```bash
conda activate button
cd project2
python3 test_realsense.py
```

### 3. 测试机械臂

```bash
cd project2
python3 test_go_zero.py
```

## 五、运行项目

### 1. 基本运行（视觉按钮操作系统）

```bash
cd project2
./start_vision_button.sh
```

这会启动：
- roscore
- piper_control (机械臂控制)
- MoveIt move_group (轨迹规划)
- 相机视觉检测
- TF 发布器
- 视觉按钮操作执行器

### 2. 运行选项

```bash
# 使用默认配置（MoveIt + cartesian 策略）
./start_vision_button.sh

# 使用不同的运动策略
./start_vision_button.sh --strategy incremental

# 启用 RViz 可视化
./start_vision_button.sh --rviz

# 不使用 MoveIt（仅 SDK 控制）
./start_vision_button.sh --no-moveit
```

### 3. 其他启动脚本

```bash
# 启动完整的 MoveIt 系统
./start_all_moveit.sh

# 启动 6D 位姿控制
./start_6d_pose_control.sh

# 重启 MoveIt
./restart_moveit.sh
```

## 六、关键配置说明

### 1. 相对路径设计

项目所有脚本都使用相对路径，通过以下方式获取项目根目录：

**Shell 脚本**：
```bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
```

**Python 脚本**：
```python
import os
project_root = os.path.dirname(os.path.abspath(__file__))
piper_ros_path = os.path.join(project_root, "piper_ros")
```

### 2. piper_ros 和 piper_sdk 的关系

- **piper_sdk**：底层 Python SDK，提供机械臂控制 API
  - 安装到系统 Python 环境
  - 可以在 conda 环境中使用

- **piper_ros**：ROS 包，提供 MoveIt 配置和高级功能
  - 包含 URDF、SRDF、MoveIt 配置
  - 必须在 ROS 工作空间中编译
  - 使用系统 Python（不是 conda）

### 3. Python 环境说明

项目使用两套 Python 环境：

1. **系统 Python 3.8**（ROS 相关）：
   - 用于运行 MoveIt 相关脚本
   - `button_actions.py`, `vision_button_action.py`, `move_a_to_b.py`

2. **Conda 环境 'button'**（视觉相关）：
   - 用于运行相机和视觉检测
   - `realsense_yolo_button_interactive.py`, `piper_tf_publisher.py`

## 七、常见问题

### 1. MoveIt 无法找到 robot_description

**问题**：`RobotCommander` 初始化失败

**解决**：
```bash
# 检查 ROS_PACKAGE_PATH
echo $ROS_PACKAGE_PATH

# 确保包含 piper_ros/src
cd project2/piper_ros
source devel/setup.bash
```

### 2. 相机被锁定

**问题**：`Camera Locked: YES`

**解决**：
```bash
sudo rmmod uvcvideo
sudo modprobe uvcvideo
```

或使用：
```bash
./reset_camera.sh
```

### 3. CAN 接口未激活

**问题**：无法连接到机械臂

**解决**：
```bash
cd piper_sdk
sudo ./piper_sdk/can_activate.sh can0
```

### 4. conda 环境冲突

**问题**：MoveIt 在 conda 环境中无法运行

**解决**：
- MoveIt 相关脚本使用系统 Python
- 启动脚本会自动处理环境切换

## 八、目录说明

### 可以删除/忽略的目录

1. **piper_ros/build/** 和 **piper_ros/devel/**
   - 编译生成的，每台机器重新编译
   - 不应该提交到版本控制

2. **__pycache__/** 和 **trajectory/**
   - Python 缓存和运行时生成的轨迹文件
   - 可以安全删除

3. **piper_ros/build/atomic_configure/**
   - 临时编译文件
   - 包含旧版本的绝对路径（可忽略）

### 必须保留的目录

1. **piper_ros/src/**
   - ROS 包源代码
   - MoveIt 配置文件

2. **piper_sdk/**
   - SDK 源代码
   - 安装脚本

3. **config/** 和 **launch/**
   - 配置和启动文件

## 九、版本控制建议

### .gitignore 推荐

```gitignore
# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/

# ROS
piper_ros/build/
piper_ros/devel/
piper_ros/.catkin_workspace

# 运行时生成
trajectory/
*.log

# 模型文件（太大）
*.pt

# 临时文件
/tmp/
*.swp
*.swo
```

### 版本控制策略

**应该提交**：
- 所有 Python 脚本
- 所有 Shell 脚本
- config/ 和 launch/
- piper_ros/src/
- piper_sdk/ 源代码
- 文档（*.md）
- requirements 文件

**不应该提交**：
- piper_ros/build/ 和 piper_ros/devel/
- __pycache__/
- trajectory/
- 大型模型文件（提供下载链接）

## 十、项目迁移清单

从一台机器迁移到另一台机器时，按照以下清单操作：

### 源机器（导出）

- [ ] 确认项目完整性
- [ ] 记录 YOLO 模型路径
- [ ] 记录自定义配置
- [ ] 打包整个 project2 文件夹（除了 build/devel）

### 目标机器（部署）

- [ ] 安装 ROS Noetic
- [ ] 安装 MoveIt
- [ ] 安装 RealSense SDK
- [ ] 安装 Conda
- [ ] 复制 project2 文件夹
- [ ] 安装 piper_sdk：`cd piper_sdk && sudo python3 setup.py install`
- [ ] 编译 piper_ros：`cd piper_ros && catkin_make`
- [ ] 创建 conda 环境：`conda create -n button python=3.8`
- [ ] 安装 Python 依赖：`pip install -r requirements`
- [ ] 配置 CAN 接口
- [ ] 运行测试脚本：`./test_system.sh`
- [ ] 启动系统：`./start_vision_button.sh`

## 十一、技术支持

如有问题，请检查：
1. 所有日志输出
2. ROS 话题：`rostopic list`
3. ROS 节点：`rosnode list`
4. 相机状态：`rs-enumerate-devices`
5. CAN 接口：`ifconfig | grep can`

---

**最后更新**：2025-11-21  
**项目版本**：V4.0
