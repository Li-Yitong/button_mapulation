# 笛卡尔路径规划快速参考

## 🔄 ROS1 → ROS2 API 速查表

| 功能 | ROS1 | ROS2 |
|------|------|------|
| **四元数** | `tf.transformations` | `utils_math.rotation_matrix_to_quaternion()` |
| **时间戳** | `.to_sec()` | `.nanoseconds * 1e-9` |
| **时间** | `rospy.Time.now()` | `time.time()` |
| **睡眠** | `rospy.sleep(sec)` | `time.sleep(sec)` |
| **频率** | `rospy.Rate(hz)` | `time.sleep(1.0/hz)` |
| **笛卡尔** | `compute_cartesian_path(wp, step, avoid_coll)` | `compute_cartesian_path(wp, step, jump_th)` |

## 📝 代码模板

### 导入

```python
from geometry_msgs.msg import Pose
from utils.utils_math import rotation_matrix_to_quaternion
import time
```

### 四元数转换

```python
# ROS1
import tf.transformations as tft
quat = tft.quaternion_from_matrix(matrix_4x4)

# ROS2
from utils.utils_math import rotation_matrix_to_quaternion
quat = rotation_matrix_to_quaternion(matrix_3x3)  # 注意：只传 3x3 旋转矩阵
```

### 时间戳转换

```python
# ROS1
time_sec = duration.to_sec()
elapsed = (rospy.Time.now() - start_time).to_sec()

# ROS2
time_sec = duration.nanoseconds * 1e-9
elapsed = time.time() - start_time
```

### 频率控制

```python
# ROS1
rate = rospy.Rate(80)  # 80 Hz
while running:
    # 执行命令
    rate.sleep()

# ROS2
interval = 1.0 / 80  # 80 Hz
while running:
    # 执行命令
    time.sleep(interval)
```

### 笛卡尔路径规划

```python
# ROS1
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints,
    0.01,   # eef_step
    True    # avoid_collisions
)

# ROS2
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints,
    0.01,   # eef_step
    0.0     # jump_threshold (0.0 = disabled)
)
```

## 🧪 测试命令

```bash
# 运行自动化测试
python3 test_cartesian_planning.py

# 验证语法
python3 -m py_compile button_actions.py

# 测试四元数转换
python3 -c "from utils.utils_math import rotation_matrix_to_quaternion; import numpy as np; print(rotation_matrix_to_quaternion(np.eye(3)))"
```

## ⚙️ 常用参数

```python
# Waypoints 密度
num_steps = max(5, int(abs(distance) * 100))  # 每厘米 5-10 点

# 笛卡尔步长
eef_step = 0.01  # 1cm（可调整为 0.005 提高精度）

# 规划覆盖率阈值
if fraction < 0.95:  # 95% 成功率
    # 回退到 IK
```

## 📚 相关文档

- **详细文档**: `MOVEIT2_CARTESIAN_MIGRATION.md`
- **完成总结**: `MOVEIT2_CARTESIAN_MIGRATION_SUMMARY.md`
- **任务清单**: `MOVEIT2_CARTESIAN_MIGRATION_CHECKLIST.md`

---
更新时间: 2025-11-22
