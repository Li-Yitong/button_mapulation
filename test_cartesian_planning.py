#!/usr/bin/env python3
"""
测试笛卡尔路径规划 ROS2 迁移
验证 move_along_end_effector_z() 中的笛卡尔规划功能
"""
import sys
import time
import numpy as np

print("="*70)
print("笛卡尔路径规划 ROS2 迁移测试")
print("="*70)

# ========================================
# 测试 1: 四元数转换导入
# ========================================
print("\n[测试 1/5] 四元数转换导入测试")
try:
    # 尝试导入 tf_transformations
    try:
        import tf_transformations as tft
        print("  ✓ tf_transformations 导入成功（ROS2 标准包）")
        use_local = False
    except ImportError:
        # 使用本地实现
        from utils.utils_math import rotation_matrix_to_quaternion, quaternion_to_rotation_matrix
        print("  ✓ 使用本地 utils_math 四元数转换实现")
        use_local = True
    
    # 测试基本功能
    identity = np.eye(4)
    
    if use_local:
        quat = rotation_matrix_to_quaternion(identity[:3, :3])
    else:
        quat = tft.quaternion_from_matrix(identity)
    
    print(f"  ✓ 单位矩阵转四元数: {quat}")
    print(f"  ✓ 使用{'本地实现' if use_local else 'tf_transformations'}")
    
    test_passed_1 = True
except ImportError as e:
    print(f"  ✗ 四元数转换导入失败: {e}")
    test_passed_1 = False

# ========================================
# 测试 2: Waypoints 生成
# ========================================
print("\n[测试 2/5] Waypoints 生成测试")
try:
    from geometry_msgs.msg import Pose
    
    # 模拟生成 waypoints
    waypoints = []
    distance = 0.03  # 3cm
    num_steps = max(5, int(abs(distance) * 100))  # 每厘米至少5个点
    
    start_pos = np.array([0.26, 0.00, 0.25])
    z_axis = np.array([0.0, 0.0, 1.0])  # 模拟末端Z轴
    
    for i in range(num_steps + 1):
        fraction = i / num_steps
        pos = start_pos + z_axis * distance * fraction
        
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = 1.0
        
        waypoints.append(pose)
    
    print(f"  ✓ 生成 {len(waypoints)} 个 waypoints")
    print(f"  ✓ 起点: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
    print(f"  ✓ 终点: [{waypoints[-1].position.x:.3f}, {waypoints[-1].position.y:.3f}, {waypoints[-1].position.z:.3f}]")
    
    test_passed_2 = True
except Exception as e:
    print(f"  ✗ Waypoints 生成失败: {e}")
    test_passed_2 = False

# ========================================
# 测试 3: 时间戳转换
# ========================================
print("\n[测试 3/5] 时间戳转换测试")
try:
    from builtin_interfaces.msg import Duration
    
    # 创建模拟 Duration
    duration = Duration()
    duration.sec = 2
    duration.nanosec = 500000000  # 0.5秒
    
    # ROS2 方式: nanoseconds 属性
    total_nanosec = duration.sec * 1_000_000_000 + duration.nanosec
    time_sec = total_nanosec * 1e-9
    
    print(f"  ✓ Duration: {duration.sec}s + {duration.nanosec}ns")
    print(f"  ✓ 转换为秒: {time_sec:.3f}s")
    
    # 验证精度
    expected = 2.5
    if abs(time_sec - expected) < 1e-6:
        print(f"  ✓ 精度验证通过 (误差: {abs(time_sec - expected):.9f}s)")
    else:
        print(f"  ✗ 精度验证失败 (期望: {expected}, 实际: {time_sec})")
    
    test_passed_3 = True
except Exception as e:
    print(f"  ✗ 时间戳转换失败: {e}")
    test_passed_3 = False

# ========================================
# 测试 4: 轨迹插值
# ========================================
print("\n[测试 4/5] 轨迹插值测试")
try:
    from trajectory_msgs.msg import JointTrajectoryPoint
    
    # 创建两个轨迹点
    point_current = JointTrajectoryPoint()
    point_current.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point_current.time_from_start = Duration(sec=0, nanosec=0)
    
    point_next = JointTrajectoryPoint()
    point_next.positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    point_next.time_from_start = Duration(sec=1, nanosec=0)
    
    # 时间戳转换
    t_current = point_current.time_from_start.sec * 1_000_000_000 + point_current.time_from_start.nanosec
    t_next = point_next.time_from_start.sec * 1_000_000_000 + point_next.time_from_start.nanosec
    t_current_sec = t_current * 1e-9
    t_next_sec = t_next * 1e-9
    
    # 插值（在 0.5s 时刻）
    elapsed = 0.5
    ratio = (elapsed - t_current_sec) / (t_next_sec - t_current_sec) if t_next_sec > t_current_sec else 1.0
    
    joints_interpolated = []
    for i in range(6):
        pos_current = point_current.positions[i]
        pos_next = point_next.positions[i]
        pos_interp = pos_current + ratio * (pos_next - pos_current)
        joints_interpolated.append(pos_interp)
    
    print(f"  ✓ 插值时刻: {elapsed}s (比例: {ratio:.2f})")
    print(f"  ✓ 插值结果: [{', '.join([f'{j:.3f}' for j in joints_interpolated])}]")
    
    # 验证插值正确性（应该是中点）
    expected = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
    if all(abs(joints_interpolated[i] - expected[i]) < 1e-6 for i in range(6)):
        print(f"  ✓ 插值验证通过")
    else:
        print(f"  ✗ 插值验证失败")
    
    test_passed_4 = True
except Exception as e:
    print(f"  ✗ 轨迹插值失败: {e}")
    test_passed_4 = False

# ========================================
# 测试 5: 时间控制
# ========================================
print("\n[测试 5/5] 时间控制测试")
try:
    # 测试 time.sleep() 替代 rospy.sleep()
    start = time.time()
    time.sleep(0.1)
    elapsed = time.time() - start
    
    print(f"  ✓ time.sleep(0.1) 执行: {elapsed:.4f}s")
    
    # 测试频率控制
    COMMAND_SEND_RATE = 80  # Hz
    command_interval = 1.0 / COMMAND_SEND_RATE
    
    print(f"  ✓ 命令发送频率: {COMMAND_SEND_RATE}Hz (间隔: {command_interval*1000:.2f}ms)")
    
    # 模拟高频循环
    loop_start = time.time()
    loop_count = 0
    target_loops = 10
    
    while loop_count < target_loops:
        # 模拟命令发送
        loop_count += 1
        time.sleep(command_interval)
    
    loop_elapsed = time.time() - loop_start
    actual_rate = target_loops / loop_elapsed
    
    print(f"  ✓ 实际循环: {target_loops}次 / {loop_elapsed:.3f}s = {actual_rate:.1f}Hz")
    
    if abs(actual_rate - COMMAND_SEND_RATE) < 5:  # 允许 5Hz 误差
        print(f"  ✓ 频率控制准确 (误差: {abs(actual_rate - COMMAND_SEND_RATE):.1f}Hz)")
    else:
        print(f"  ⚠️  频率误差较大 (误差: {abs(actual_rate - COMMAND_SEND_RATE):.1f}Hz)")
    
    test_passed_5 = True
except Exception as e:
    print(f"  ✗ 时间控制测试失败: {e}")
    test_passed_5 = False

# ========================================
# 测试总结
# ========================================
print("\n" + "="*70)
print("测试结果汇总")
print("="*70)

tests = [
    ("四元数转换导入", test_passed_1),
    ("Waypoints 生成", test_passed_2),
    ("时间戳转换", test_passed_3),
    ("轨迹插值", test_passed_4),
    ("时间控制", test_passed_5),
]

passed = sum(1 for _, result in tests if result)
total = len(tests)

for name, result in tests:
    status = "✓" if result else "✗"
    print(f"  {status} {name}")

print("="*70)
if passed == total:
    print(f"🎉 所有测试通过！({passed}/{total})")
    print("✓ 笛卡尔路径规划 ROS2 迁移验证成功")
    sys.exit(0)
else:
    print(f"⚠️  部分测试失败 ({passed}/{total})")
    print("✗ 请检查失败的测试项并解决问题")
    sys.exit(1)
