# 手眼标定完整指南

## 📋 概述
本指南用于 Piper 机械臂的眼在手上（eye-in-hand）手眼标定，使用棋盘格作为标定板。

## 🔧 环境要求
- Ubuntu 20.04
- ROS2 Foxy
- RealSense D435i 相机
- Piper 机械臂

## 📂 文件结构
```
~/handeye_ws/src/
├── easy_handeye2/              # 手眼标定核心包
└── button_mapulation/          # 你的项目

/home/unitree/manipulation/button/vba/
├── launch/
│   ├── handeye_calibrate.launch.py    # 标定启动文件
│   └── handeye_publish.launch.py      # 发布标定结果
├── chessboard_pose_publisher.py       # 棋盘格检测节点
├── piper_tf_publisher_ros2.py         # 机械臂TF发布器
├── read_calibration_result.py         # 读取标定结果
├── verify_handeye_calibration.py      # 验证标定
└── piper_arm.py                       # 存放标定参数
```

## 🚀 标定流程

### 方式 A：使用一键脚本（推荐）

1. **准备工作**
   ```bash
   # 终端1：启动相机
   source /opt/ros/foxy/setup.bash
   ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true
   
   # 终端2：启动机械臂TF发布
   cd /home/unitree/manipulation/button/vba
   python3 piper_tf_publisher_ros2.py
   ```

2. **运行标定脚本**
   ```bash
   # 终端3：运行一键标定脚本
   cd /home/unitree/manipulation/button/vba
   ./start_handeye_calibration.sh
   ```

3. **在 GUI 中采集数据**
   - 移动机械臂到不同姿态（12-20个样本）
   - 每个姿态点击 "Take Sample"
   - 确保姿态多样化（不同角度、距离、旋转）
   - 点击 "Compute Calibration"
   - 检查重投影误差（应 < 5 像素）
   - 点击 "Save Calibration"

4. **更新标定参数到代码**
   ```bash
   # 读取标定结果
   python3 read_calibration_result.py
   
   # 复制输出的代码片段到 piper_arm.py 中
   # 替换 self.link6_t_camera 和 self.link6_q_camera
   ```

5. **验证标定**
   ```bash
   # 终端4：发布标定结果
   source ~/handeye_ws/install/setup.bash
   ros2 launch /home/unitree/manipulation/button/vba/launch/handeye_publish.launch.py
   
   # 终端5：运行验证脚本
   python3 verify_handeye_calibration.py
   
   # 或在 RViz 中查看 TF
   rviz2
   ```

---

### 方式 B：手动逐步执行

1. **启动相机**
   ```bash
   source /opt/ros/foxy/setup.bash
   ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true
   ```

2. **启动机械臂TF发布**
   ```bash
   cd /home/unitree/manipulation/button/vba
   python3 piper_tf_publisher_ros2.py
   ```

3. **启动棋盘格检测节点**
   ```bash
   # 终端3：启动棋盘格检测节点
   python3 chessboard_pose_publisher.py --ros-args \
     -p image_topic:=/camera/color/image_raw \
     -p camera_info_topic:=/camera/color/camera_info \
     -p pattern_rows:=5 -p pattern_cols:=7 -p square_size:=0.03 \
     -p camera_frame:=camera_color_optical_frame -p board_frame:=checkerboard
   ```

4. **启动手眼标定GUI**
   ```bash
   source ~/handeye_ws/install/setup.bash
   ros2 launch /home/unitree/manipulation/button/vba/launch/handeye_calibrate.launch.py
   ```

5. **采集数据**（同上）

6. **更新代码**（同上）

7. **验证标定**（同上）

---

## 📊 标定质量指标

- ✅ 优秀：重投影误差 < 2 像素
- ⚠️ 可接受：2-5 像素
- ❌ 需重新标定：> 5 像素

## 🔍 常见问题

### 1. 找不到棋盘格
- 检查光照是否充足
- 确保棋盘格平整无折痕
- 调整相机焦距

### 2. TF 缺失错误
```
Transform from camera_color_optical_frame to checkerboard not available
```
**解决方案：**
- 确保棋盘格在相机视野内
- 检查 `chessboard_pose_publisher.py` 是否运行
- 运行 `ros2 topic echo /tf` 查看是否有 checkerboard TF

### 3. 标定结果不准确
- 增加样本数量（推荐 15-20 个）
- 确保姿态多样化
- 检查相机内参是否正确
- 重新测量棋盘格方格尺寸

### 4. 重投影误差过大
- 检查棋盘格尺寸设置是否正确
- 确保相机标定准确
- 重新采集样本

## 📝 标定参数说明

### 棋盘格参数
```python
pattern_rows: 5      # 内角点行数
pattern_cols: 7      # 内角点列数
square_size: 0.03    # 方格边长（米）
```

**重要：标定配置参数**
- `name`: `piper_realsense_handeye` (标定结果文件名)
- `calibration_type`: `eye_in_hand` (眼在手上)
- 标定结果保存在: `~/.ros/easy_handeye/piper_realsense_handeye.yaml`

**如何数内角点：**
```
○---○---○---○---○---○---○
|   |   |   |   |   |   |
○---●---●---●---●---●---○
|   | 1 | 2 | 3 | 4 |   |
○---●---●---●---●---●---○
|   | 5 | 6 | 7 | 8 |   |
○---●---●---●---●---●---○
|   |   |   |   |   |   |
○---○---○---○---○---○---○

内角点: 5 行 × 7 列 = 35 个
（不包括外围的 ○）
```

### TF 坐标系关系
```
eye-in-hand (眼在手上):
    base_link
       ↓
    link6 (末端)
       ↓
    camera_color_optical_frame (相机光学坐标系，随末端移动)
       ↓
    checkerboard (棋盘格，固定在桌面)

完整的 TF 树:
    world
      ↓
    base_link
      ↓ (正运动学)
    link1 → link2 → link3 → link4 → link5 → link6
      ↓ (手眼标定: link6_T_camera)
    camera_color_optical_frame  ← RealSense 光学坐标系
      ↓ (RealSense 驱动)
    camera_color_frame
      ↓
    camera_link

注意：
- ✅ 使用 camera_color_optical_frame (RealSense 标准)
- ❌ 不要使用 camera_link (物理框架，非成像坐标系)
- 光学坐标系定义: X右, Y下, Z前(光轴)
```

## 🔄 重新标定

如果需要重新标定：

1. 删除旧标定结果
   ```bash
   rm ~/.ros/easy_handeye/piper_realsense_handeye.yaml
   ```

2. 重新运行标定流程

3. 备份标定结果
   ```bash
   cp ~/.ros/easy_handeye/piper_realsense_handeye.yaml \
      ~/manipulation/button/vba/config/handeye_backup_$(date +%Y%m%d).yaml
   ```

## 📚 参考资料

- [easy_handeye2 文档](https://github.com/marcoesposito1988/easy_handeye2)
- [手眼标定原理](https://en.wikipedia.org/wiki/Hand_eye_calibration_problem)
- [OpenCV 棋盘格检测](https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html)

## ✅ 完成标定后

标定完成后，在实际应用中：

1. **始终运行 TF 发布节点**
   ```bash
   # 方式1：独立运行
   ros2 launch /home/unitree/manipulation/button/vba/launch/handeye_publish.launch.py
   
   # 方式2：集成到主launch文件中
   # 在你的主launch文件中 include handeye_publish.launch.py
   ```

2. **在代码中使用标定结果**
   - TF 树中会有 `link6 → camera_color_optical_frame` 变换
   - 视觉检测到的物体位置会自动变换到机械臂坐标系
   - 无需手动计算坐标转换

3. **定期验证标定精度**
   ```bash
   python3 verify_handeye_calibration.py
   ```

---

**祝标定顺利！如有问题请参考上述故障排除部分。** 🚀
