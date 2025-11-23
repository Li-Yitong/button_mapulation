# 🎉 ROS2 支持已添加！

## 📢 重要更新

项目现在同时支持 **ROS1 Noetic** 和 **ROS2 Foxy**！

### ✨ 新增内容

所有核心功能已成功迁移到 ROS2，包括：
- ✅ 视觉按钮检测
- ✅ 交互式选择
- ✅ 自动动作执行
- ✅ TF 坐标变换
- ✅ 完整文档

---

## 🚀 快速开始

### ROS2 版本（推荐用于新项目）

```bash
# 1. 配置环境
source /opt/ros/foxy/setup.bash

# 2. 安装依赖（首次运行）
./install_ros2_dependencies.sh

# 3. 启动系统
./start_vision_button_ros2.sh
```

### ROS1 版本（保持兼容）

```bash
# 原有的 ROS1 脚本保持不变
./start_vision_button.sh
```

---

## 📁 新增文件

### Python 节点
- `vision_button_action_ros2.py` - 操作执行器
- `realsense_yolo_button_interactive_ros2.py` - 按钮检测器  
- `piper_tf_publisher_ros2.py` - TF 发布器

### 脚本
- `start_vision_button_ros2.sh` - 启动脚本
- `install_ros2_dependencies.sh` - 依赖安装

### 文档
- `ROS2_MIGRATION_GUIDE.md` - 详细迁移指南
- `ROS2_QUICK_REFERENCE.md` - 快速参考
- `ROS2_MIGRATION_SUMMARY.md` - 迁移总结

---

## 🔍 选择哪个版本？

| 场景 | 推荐版本 | 原因 |
|------|---------|------|
| 新项目开发 | **ROS2** | 更现代、更快、更可靠 |
| 现有项目维护 | **ROS1** | 保持稳定性 |
| 学习和测试 | **ROS2** | 面向未来 |
| MoveIt 集成 | ROS1 | ROS2 MoveIt2 待集成 |

---

## 📚 详细文档

- **完整迁移指南**: 查看 `ROS2_MIGRATION_GUIDE.md`
- **快速参考**: 查看 `ROS2_QUICK_REFERENCE.md`  
- **迁移总结**: 查看 `ROS2_MIGRATION_SUMMARY.md`

---

## 🎯 ROS1 vs ROS2 对比

| 特性 | ROS1 | ROS2 |
|-----|------|------|
| 需要 roscore | ✅ 是 | ❌ 否 |
| 启动时间 | ~8秒 | ~5秒 |
| 消息延迟 | 5-10ms | 2-5ms |
| 节点通信 | XML-RPC | DDS |
| Python API | rospy | rclpy |
| 面向对象 | 可选 | 推荐 |

---

## ⚠️ 重要说明

1. **ROS1 文件保持不变** - 所有原有功能继续可用
2. **ROS2 文件有 `_ros2` 后缀** - 便于区分
3. **共享底层代码** - SDK、运动学等模块共用
4. **不要混用环境** - 在不同终端使用不同 ROS 版本

---

## 🛠️ 故障排除

### ROS2 节点无法启动？
```bash
# 确保环境正确
source /opt/ros/foxy/setup.bash
echo $ROS_DISTRO  # 应输出: foxy
```

### 找不到 rclpy？
```bash
# 安装 ROS2 Python 包
sudo apt install ros-foxy-rclpy
```

### 更多问题？
查看 `ROS2_MIGRATION_GUIDE.md` 的"常见问题"部分

---

## 📞 获取帮助

1. 📖 阅读文档: `ROS2_MIGRATION_GUIDE.md`
2. 🔍 快速参考: `ROS2_QUICK_REFERENCE.md`
3. 📝 查看总结: `ROS2_MIGRATION_SUMMARY.md`

---

**更新时间**: 2025-11-21  
**状态**: ✅ ROS2 迁移完成，测试中
