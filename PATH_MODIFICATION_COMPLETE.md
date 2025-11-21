# 项目路径配置修改完成

## ✅ 已完成的工作

### 1. Shell 脚本修改（6个文件）

所有启动脚本都已修改为使用相对路径：

| 文件名 | 说明 | 状态 |
|--------|------|------|
| `start_vision_button.sh` | 视觉按钮操作系统启动脚本 | ✅ 已修改 |
| `start_all_moveit.sh` | MoveIt 完整系统启动脚本 | ✅ 已修改 |
| `start_all.sh` | 基础系统启动脚本 | ✅ 已修改 |
| `start_press_moveit.sh` | 按压程序启动脚本 | ✅ 已修改 |
| `restart_moveit.sh` | MoveIt 重启脚本 | ✅ 已修改 |
| `run_move_a_to_b.sh` | move_a_to_b 运行脚本 | ✅ 已修改 |

**修改方式**：使用 `PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"` 获取项目根目录

### 2. Python 文件修改（4个文件）

所有 Python 脚本中的绝对路径都已修改为相对路径：

| 文件名 | 修改位置 | 状态 |
|--------|----------|------|
| `button_actions.py` | Line 1799 | ✅ 已修改 |
| `button_actions copy.py` | Line 1356 | ✅ 已修改 |
| `move_a_to_b.py` | Line 316 | ✅ 已修改 |
| `grasp_action_copy.py` | Line 357 | ✅ 已修改 |
| `moveit_6d_pose_control.py` | Line 540 | ✅ 已修改 |

**修改方式**：使用 `os.path.dirname(os.path.abspath(__file__))` 获取脚本所在目录

### 3. 新增文档（3个文件）

| 文件名 | 说明 |
|--------|------|
| `DEPLOYMENT_GUIDE.md` | 完整的项目部署指南，包含系统依赖、安装步骤、常见问题 |
| `PATH_MIGRATION_SUMMARY.md` | 路径迁移总结，关于 piper_ros/piper_sdk 的说明 |
| `.gitignore` | Git 版本控制忽略文件 |

## 📋 关于 piper_ros 和 piper_sdk

### piper_sdk（松灵官方 Python SDK）

**安装方式**：
```bash
cd project2/piper_sdk
sudo python3 setup.py install
```

**说明**：
- ✅ 本项目已包含完整源码
- ✅ 只需在新机器上安装一次
- ✅ 安装到系统 Python 环境
- ✅ 可以在 conda 环境中使用

### piper_ros（松灵官方 ROS 包）

**编译方式**：
```bash
cd project2/piper_ros
catkin_make
```

**说明**：
- ✅ 本项目已包含完整源码（`piper_ros/src/`）
- ⚠️ 必须在每台新机器上重新编译
- ❌ `build/` 和 `devel/` 不应复制到新机器
- ✅ 编译后会生成机器相关的配置

## 🚀 快速部署步骤

### 1. 复制项目
```bash
# 可以放在任意路径
cp -r project2 ~/your_workspace/
cd ~/your_workspace/project2
```

### 2. 安装 piper_sdk
```bash
cd piper_sdk
sudo python3 setup.py install
```

### 3. 编译 piper_ros
```bash
cd ../piper_ros
catkin_make
```

### 4. 创建 conda 环境
```bash
conda create -n button python=3.8 -y
conda activate button
pip install -r ../requirements
```

### 5. 运行测试
```bash
cd ..
./start_vision_button.sh
```

## 📦 Git 版本控制建议

### 应该提交的内容

```
project2/
├── *.py                    # 所有 Python 脚本
├── *.sh                    # 所有 Shell 脚本
├── *.md                    # 所有文档
├── requirements            # Python 依赖
├── config/                 # 配置文件
├── launch/                 # ROS launch 文件
├── utils/                  # 工具函数
├── piper_sdk/             # SDK 源码（完整）
└── piper_ros/src/         # ROS 包源码（完整）
```

### 不应该提交的内容（已在 .gitignore 中）

```
__pycache__/               # Python 缓存
piper_ros/build/          # 编译生成
piper_ros/devel/          # 编译生成
trajectory/               # 运行时生成
*.pt                      # 大型模型文件
*.log                     # 日志文件
```

## ⚠️ 注意事项

### 1. 项目可以放在任意路径
由于所有脚本都使用相对路径，项目可以放在任意位置：
- ✅ `/home/user/workspace/project2`
- ✅ `/opt/robot/project2`
- ✅ `~/my_robot/project2`

### 2. piper_ros 必须重新编译
每台新机器都需要重新编译 piper_ros：
```bash
cd piper_ros
rm -rf build devel  # 清理旧文件
catkin_make         # 重新编译
```

### 3. 模型文件需要单独下载
`yolo_button.pt` 模型文件太大，建议：
- 上传到云盘（Google Drive、百度网盘等）
- 在 README 中提供下载链接
- 或说明如何训练模型

### 4. CAN 接口需要每次重启后激活
```bash
cd piper_sdk
sudo ./piper_sdk/can_activate.sh can0
```

## 📚 参考文档

| 文档 | 说明 |
|------|------|
| `DEPLOYMENT_GUIDE.md` | 完整的部署指南，包含系统依赖、安装步骤、常见问题 |
| `PATH_MIGRATION_SUMMARY.md` | 路径迁移总结，piper_ros/piper_sdk 说明 |
| `VISION_BUTTON_GUIDE.md` | 视觉按钮系统使用指南 |
| `README.md` | 项目说明 |

## ✅ 检查清单

在新机器上部署后，请检查以下内容：

- [ ] ROS Noetic 已安装：`roscore &`
- [ ] MoveIt 已安装：`python3 -c "import moveit_commander"`
- [ ] RealSense SDK 已安装：`realsense-viewer`
- [ ] Conda 环境已创建：`conda env list | grep button`
- [ ] piper_sdk 已安装：`python3 -c "from piper_sdk import *"`
- [ ] piper_ros 已编译：`ls piper_ros/devel/setup.bash`
- [ ] CAN 接口已激活：`ifconfig | grep can`
- [ ] 所有脚本可执行：`chmod +x *.sh`
- [ ] 项目可以正常运行：`./start_vision_button.sh`

## 🎯 总结

✅ **所有绝对路径已修改为相对路径**  
✅ **项目可以在任意路径下运行**  
✅ **piper_ros 和 piper_sdk 说明已完善**  
✅ **部署文档已创建**  
✅ **Git 版本控制已配置**

现在可以安全地将项目复制到其他机器，或提交到 Git 仓库！

---

**修改日期**：2025-11-21  
**项目版本**：V4.0  
**修改人**：AI Assistant
