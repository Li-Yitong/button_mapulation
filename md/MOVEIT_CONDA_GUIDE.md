# Conda 环境与 MoveIt 兼容性指南

## 📋 问题说明

### 为什么 Conda 环境下无法使用 MoveIt？

**核心原因**：库版本冲突

```
系统环境（ROS Noetic）          Conda 环境（button）
├─ Python 3.8                  ├─ Python 3.9
├─ libffi 7.x                  ├─ libffi 8.x (conda 自带)
├─ 系统库路径                   ├─ conda 库路径（优先级高）
└─ MoveIt 链接系统库            └─ 加载 conda 库 → 符号冲突
```

**错误信息**：
```
ImportError: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0
```

## 🎯 解决方案

### 方案 1：使用 Conda 环境 + 原始 SDK（推荐）

**特点**：
- ✅ 环境隔离
- ✅ 包管理方便
- ✅ 基本功能完整
- ❌ 无 MoveIt 轨迹规划

**使用方法**：
```bash
./start_all.sh
# 或
conda activate button
python3 grasp_action.py
```

**适用场景**：
- 日常开发和调试
- 简单的点到点运动
- 不需要复杂轨迹规划

---

### 方案 2：使用系统 Python + MoveIt（高级功能）

**特点**：
- ✅ MoveIt 完整功能
- ✅ 轨迹规划和碰撞检测
- ❌ 系统环境可能被污染

**使用方法**：
```bash
./start_all_moveit.sh
# 或
python3 grasp_action.py  # 不激活 conda
```

**适用场景**：
- 需要复杂轨迹规划
- 需要碰撞检测
- 需要笛卡尔路径规划

---

### 方案 3：混合模式（灵活切换）

**智能检测**：代码会自动检测 MoveIt 是否可用

- Conda 环境 → 自动使用 SDK 模式
- 系统 Python → 尝试使用 MoveIt 模式

**输出示例**：

Conda 环境：
```
⚠️  MoveIt 模块加载失败
  将使用原始SDK控制模式
抓取模式: 原始SDK点到点
```

系统 Python：
```
✓ MoveIt 模块加载成功
抓取模式: MoveIt轨迹规划
```

## 📊 功能对比表

| 功能特性 | SDK 模式<br>(Conda) | MoveIt 模式<br>(系统 Python) |
|---------|-------------------|---------------------------|
| **基础运动** | ✅ | ✅ |
| **视觉抓取** | ✅ | ✅ |
| **点到点运动** | ✅ | ✅ |
| **旋转功能** | ✅ | ✅ |
| **夹爪控制** | ✅ | ✅ |
| **轨迹规划** | ❌ 直接运动 | ✅ OMPL/RRT* |
| **路径平滑** | ❌ | ✅ 自动插值 |
| **碰撞检测** | ❌ | ✅ 实时检测 |
| **笛卡尔路径** | ❌ | ✅ 直线运动 |
| **避障功能** | ❌ | ✅ 场景管理 |
| **环境隔离** | ✅ | ❌ |
| **包管理** | ✅ Conda | ⚠️ 系统级 |

## 🔧 高级：修复 Conda 环境（不推荐）

如果必须在 Conda 环境中使用 MoveIt：

```bash
conda activate button

# 方法1：移除 conda 的 libffi
conda remove libffi --force

# 方法2：设置库路径优先级
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# 测试
python3 -c "import moveit_commander; print('成功')"
```

⚠️ **警告**：
- 可能破坏其他依赖
- 不保证稳定性
- 需要每次激活环境时设置

## 💡 最佳实践建议

### 日常开发（推荐）
```bash
conda activate button
./start_all.sh
```
- 使用 Conda 环境
- 原始 SDK 控制
- 快速、稳定

### 需要高级功能时
```bash
./start_all_moveit.sh
```
- 使用系统 Python
- MoveIt 轨迹规划
- 功能完整

### 依赖安装

**Conda 环境依赖**：
```bash
conda activate button
pip install numpy opencv-python pyrealsense2 ultralytics
```

**系统 Python 依赖（MoveIt）**：
```bash
sudo apt install ros-noetic-moveit
pip3 install numpy opencv-python  # 系统级安装
```

## 🎓 技术解释

### 为什么会有这个问题？

1. **编译时链接**：MoveIt 在编译时链接到系统的共享库（如 libffi）
2. **运行时加载**：Python 运行时通过 `LD_LIBRARY_PATH` 查找库
3. **Conda 优先**：Conda 环境的路径优先级高于系统路径
4. **版本不匹配**：MoveIt 期望 libffi 7.x，但 Conda 提供 8.x
5. **符号冲突**：函数签名或 ABI 不兼容导致导入失败

### 类似问题

这个问题不仅存在于 MoveIt，类似情况包括：
- OpenCV with CUDA
- TensorFlow with GPU
- Qt 应用程序
- 任何使用系统编译库的 ROS 包

## 📚 参考资源

- [ROS Noetic 文档](http://wiki.ros.org/noetic)
- [MoveIt 官方教程](https://ros-planning.github.io/moveit_tutorials/)
- [Conda 环境管理](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html)

---

**总结**：对于您的项目，推荐使用 **Conda 环境 + SDK 模式**，功能已经足够使用。如需 MoveIt 高级功能，使用 `start_all_moveit.sh` 切换到系统 Python 模式。
