# 项目路径迁移总结

## 修改内容

### 1. Shell 脚本（已全部修改为相对路径）

所有 Shell 脚本都添加了以下代码来获取项目根目录：

```bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
```

**已修改的文件**：
- ✅ `start_vision_button.sh` - 视觉按钮操作系统启动脚本
- ✅ `start_all_moveit.sh` - MoveIt 完整系统启动脚本
- ✅ `start_all.sh` - 基础系统启动脚本
- ✅ `start_press_moveit.sh` - 按压程序启动脚本
- ✅ `restart_moveit.sh` - MoveIt 重启脚本
- ✅ `run_move_a_to_b.sh` - move_a_to_b 运行脚本

### 2. Python 文件（已全部修改为相对路径）

所有 Python 文件中涉及 `piper_ros` 路径的地方都修改为：

```python
import os
project_root = os.path.dirname(os.path.abspath(__file__))
piper_ros_path = os.path.join(project_root, "piper_ros")
```

**已修改的文件**：
- ✅ `button_actions.py` (line 1799)
- ✅ `move_a_to_b.py` (line 316)
- ✅ `grasp_action_copy.py` (line 357)

### 3. 新增文件

- ✅ `DEPLOYMENT_GUIDE.md` - 完整的项目部署指南
- ✅ `.gitignore` - Git 版本控制忽略文件

## 关于 piper_ros 和 piper_sdk

### piper_sdk（松灵官方 Python SDK）

**性质**：Python 包，提供底层机械臂控制 API

**部署方式**：
```bash
# 方式一：从本项目安装（推荐）
cd project2/piper_sdk
sudo python3 setup.py install

# 方式二：从官方仓库安装
git clone https://github.com/agilexrobotics/piper_sdk.git
cd piper_sdk
sudo python3 setup.py install
```

**说明**：
- 安装到系统 Python 环境（`/usr/local/lib/python3.8/dist-packages/`）
- 可以在 conda 环境中使用
- 只需安装一次，不需要每次编译
- 本项目已包含完整的 `piper_sdk/` 目录，可以直接使用

### piper_ros（松灵官方 ROS 包）

**性质**：ROS 工作空间，包含 MoveIt 配置和 ROS 节点

**部署方式**：
```bash
cd project2/piper_ros
catkin_make
source devel/setup.bash
```

**说明**：
- 必须在每台新机器上重新编译
- `build/` 和 `devel/` 目录是编译生成的，不应该复制
- 包含 URDF、SRDF、MoveIt 配置等
- 本项目已包含完整的 `piper_ros/src/` 源代码

## 项目迁移步骤

### 最简步骤（推荐）

1. **复制整个 project2 文件夹**到目标机器
   ```bash
   # 可以放在任意路径
   cp -r project2 ~/your_workspace/
   ```

2. **安装 piper_sdk**
   ```bash
   cd ~/your_workspace/project2/piper_sdk
   sudo python3 setup.py install
   ```

3. **编译 piper_ros**
   ```bash
   cd ~/your_workspace/project2/piper_ros
   catkin_make
   ```

4. **创建 conda 环境并安装依赖**
   ```bash
   conda create -n button python=3.8 -y
   conda activate button
   pip install -r ~/your_workspace/project2/requirements
   ```

5. **运行测试**
   ```bash
   cd ~/your_workspace/project2
   ./start_vision_button.sh
   ```

### 版本控制建议

**应该提交到 Git 的**：
- ✅ 所有 Python 脚本（`*.py`）
- ✅ 所有 Shell 脚本（`*.sh`）
- ✅ `piper_sdk/` 完整目录
- ✅ `piper_ros/src/` 源代码
- ✅ `config/`、`launch/`、`utils/` 等配置目录
- ✅ 文档文件（`*.md`）
- ✅ `requirements` 依赖文件

**不应该提交的**：
- ❌ `piper_ros/build/` - 编译生成
- ❌ `piper_ros/devel/` - 编译生成
- ❌ `__pycache__/` - Python 缓存
- ❌ `trajectory/` - 运行时生成
- ❌ `*.pt`、`*.pth` - 大型模型文件（提供下载链接）

已创建 `.gitignore` 文件来自动忽略这些文件。

## 需要其他人自行安装的内容

### 1. 系统依赖（必须）

```bash
# ROS Noetic
sudo apt install ros-noetic-desktop-full

# MoveIt
sudo apt install ros-noetic-moveit

# RealSense SDK
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# 其他工具
sudo apt install python3-pip python3-catkin-tools can-utils
```

### 2. Python 环境（必须）

```bash
# 安装 Miniconda/Anaconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# 创建 conda 环境
conda create -n button python=3.8 -y
conda activate button
pip install -r requirements
```

### 3. piper_sdk（必须，从项目中安装）

```bash
cd project2/piper_sdk
sudo python3 setup.py install
```

### 4. piper_ros（必须，编译）

```bash
cd project2/piper_ros
catkin_make
```

## 配置检查清单

迁移完成后，检查以下内容：

- [ ] ROS 环境已安装：`roscore &`
- [ ] MoveIt 已安装：`python3 -c "import moveit_commander"`
- [ ] RealSense SDK 已安装：`realsense-viewer`
- [ ] Conda 环境已创建：`conda env list | grep button`
- [ ] piper_sdk 已安装：`python3 -c "from piper_sdk import *"`
- [ ] piper_ros 已编译：`ls project2/piper_ros/devel/setup.bash`
- [ ] CAN 接口已配置：`ifconfig | grep can`
- [ ] 所有脚本可执行：`chmod +x project2/*.sh`

## 常见问题

### Q1: 为什么不直接把 piper_ros/piper_sdk 作为 submodule？

**答**：
- `piper_ros` 需要在每台机器上编译，编译产物不能跨机器复制
- `piper_sdk` 需要安装到系统 Python 环境
- 包含在项目中更方便，减少依赖管理复杂度
- 松灵官方库更新不频繁

### Q2: 需要保留 piper_ros/build 和 devel 吗？

**答**：不需要。这些是编译生成的，每台机器需要重新编译：
```bash
cd piper_ros
rm -rf build devel
catkin_make
```

### Q3: 如何让其他人获取 YOLO 模型？

**答**：模型文件（`yolo_button.pt`）太大，不应该提交到 Git。建议：
- 上传到云盘（如 Google Drive、百度网盘）
- 提供下载链接
- 或者在 README 中说明如何训练/下载模型

### Q4: 项目可以放在任意路径吗？

**答**：是的！所有脚本都已经修改为使用相对路径。你可以把 `project2` 放在任意位置：
- `/home/user/workspace/project2` ✅
- `/opt/robot/project2` ✅
- `~/my_robot/project2` ✅

## 参考文档

详细部署步骤请参考：
- 📖 `DEPLOYMENT_GUIDE.md` - 完整部署指南
- 📖 `README.md` - 项目说明
- 📖 `VISION_BUTTON_GUIDE.md` - 视觉按钮系统使用指南

---

**最后更新**：2025-11-21  
**修改版本**：V4.0
