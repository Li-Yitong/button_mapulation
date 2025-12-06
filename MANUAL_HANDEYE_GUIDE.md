# 手眼标定 - 手动模式快速指南

## 🎯 为什么用手动模式？

easy_handeye2 的 GUI 在 ROS2 Foxy 上有兼容性问题。手动模式更可靠，并且你可以：
- ✅ 实时看到相机画面和棋盘格检测
- ✅ 完全控制采样过程
- ✅ 更好地理解标定原理

---

## 🚀 快速开始

### **准备工作（3个终端）**

```bash
# 终端1：启动相机
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true

# 终端2：启动机械臂TF发布
cd /home/unitree/manipulation/button/vba
python3 piper_tf_publisher_ros2.py

# 终端3：启动手动标定工具
cd /home/unitree/manipulation/button/vba
./start_manual_handeye_calibration.sh
```

---

## 📸 标定流程

### **1. 启动后会出现两个窗口**
- **图像窗口**: 显示相机画面和棋盘格检测结果
  - 绿色文字 "Chessboard Detected!" = 检测成功 ✅
  - 红色文字 "Searching..." = 未检测到 ❌
- **终端窗口**: 显示提示信息，等待你的命令

### **2. 采集样本（关键步骤）**

#### **移动机械臂到第一个姿态**
- 确保图像窗口显示 "Chessboard Detected!"
- 棋盘格角点应该清晰可见（绿色标记）

#### **采集样本**
在终端输入：
```
s
```
按 Enter

你会看到：
```
✓ 样本 #1 采集成功
  机器人位置: [x, y, z]
  棋盘格距离: 0.xxx m
```

#### **重复 12-20 次**
- 每次移动机械臂到**不同姿态**
- 确保棋盘格仍被检测到
- 输入 `s` + Enter 采集

### **3. 计算标定**

采集足够样本后（至少 12 个），输入：
```
c
```
按 Enter

程序会自动计算并显示结果：
```
✓ 标定计算完成！
标定结果 (link6 → camera_color_optical_frame):
  平移: [x, y, z]
  四元数: [w, x, y, z]
  
✓ 标定结果已保存到: ~/.ros/easy_handeye/piper_realsense_handeye.yaml
```

### **4. 退出**
输入：
```
q
```

---

## 💡 **重要提示**

### **姿态多样化的关键**
标定精度取决于姿态的多样性。需要：

✅ **旋转末端** (最重要！)
- 绕 X 轴旋转（俯仰）：-45° 到 +45°
- 绕 Y 轴旋转（偏航）：-45° 到 +45°  
- 绕 Z 轴旋转（滚转）：-45° 到 +45°

❌ **不要只平移**
- 仅仅移动机械臂位置意义不大
- 必须改变相机朝向

### **好的采样策略**
```
姿态 1-4:   正视棋盘格，从 4 个不同角度
姿态 5-8:   倾斜 30°，从 4 个方向
姿态 9-12:  倾斜 -30°，从 4 个方向
姿态 13-16: 旋转末端，从 4 个方向
```

### **如何判断姿态是否足够好？**
- ✅ 棋盘格完整可见（所有角点都检测到）
- ✅ 距离适中（0.3-0.8m）
- ✅ 与上一个姿态有明显差异
- ❌ 棋盘格部分遮挡
- ❌ 距离太近/太远
- ❌ 与上一个姿态几乎相同

---

## 🔧 **命令参考**

| 命令 | 功能 |
|------|------|
| `s` | 采集当前姿态样本 |
| `c` | 计算标定（需 ≥12 个样本） |
| `q` | 退出标定工具 |

---

## 📝 **标定完成后**

### **1. 读取结果**
```bash
cd /home/unitree/manipulation/button/vba
python3 read_calibration_result.py
```

### **2. 更新 piper_arm.py**
复制输出的代码段，替换 `piper_arm.py` 中的：
```python
self.link6_q_camera = np.array([...])  # 旧值
self.link6_t_camera = [...]            # 旧值
```

### **3. 验证标定**
```bash
# 发布标定结果
ros2 launch /home/unitree/manipulation/button/vba/launch/handeye_publish.launch.py

# 运行验证脚本
python3 verify_handeye_calibration.py
```

### **4. 在 RViz 中查看**
```bash
rviz2
```
- 添加 TF 显示
- 应该能看到 `link6 → camera_color_optical_frame` 的变换

---

## ❓ **常见问题**

### **Q: 图像窗口一直显示 "Searching..."**
**A:** 
- 检查棋盘格是否在相机视野内
- 确保光照充足
- 棋盘格应该平整，无折痕
- 尝试调整机械臂位置

### **Q: 采集样本时提示 "TF 查询失败"**
**A:**
- 确保 `piper_tf_publisher_ros2.py` 在运行
- 确保棋盘格被检测到（图像窗口有绿色提示）
- 运行 `ros2 topic echo /tf` 检查 TF 发布

### **Q: 计算标定时提示 "样本不足"**
**A:**
- 至少需要 3 个样本（推荐 12-20 个）
- 输入 `s` + Enter 采集更多样本

### **Q: 标定结果看起来不对**
**A:**
- 重新标定，确保姿态多样化
- 检查棋盘格尺寸设置是否正确（默认 5x7, 30mm）
- 确保每个样本的棋盘格检测都是准确的

---

## 🎓 **原理简介**

手眼标定本质上是求解以下方程：

```
对于每个样本 i:
  A_i · X = X · B_i

其中:
  A_i = base → link6 (机械臂正运动学)
  B_i = camera → checkerboard (视觉检测)
  X   = link6 → camera (要标定的变换)
```

采集多个姿态后，使用 Tsai-Lenz 算法求解 X。

---

**祝标定顺利！** 🚀

如有问题，请查看日志或运行 `ros2 topic list` 检查话题。
