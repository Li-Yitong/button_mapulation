# #!/usr/bin/env python3
# """
# 按钮操作执行器 - 独立版本
# 支持四种按钮操作类型：Toggle, Plug-in, Push, Knob
# 所有参数通过宏定义配置，无需视觉检测
# """
# from piper_sdk import *
# import rospy
# import time
# import numpy as np
# import math
# from piper_arm import PiperArm

# # ========================================
# # 宏定义 - 用户配置区
# # ========================================

# # === 目标位置配置 (基座坐标系，单位：米) ===
# # 注意：请确保目标位置在机械臂工作范围内
# # Piper 机械臂典型工作范围: X: 0.15-0.45, Y: -0.25-0.25, Z: 0.02-0.40
# TARGET_X = 0.35  # X坐标 (前后方向，建议 0.2-0.45)
# TARGET_Y = 0.00  # Y坐标 (左右方向，建议 -0.2-0.2)
# TARGET_Z = 0.20  # Z坐标 (高度方向，建议 0.05-0.35)

# # === 动作类型选择 ===
# # 'toggle' - 拨动开关
# # 'plugin' - 插拔连接器
# # 'push'   - 按压按钮
# # 'knob'   - 旋转旋钮
# ACTION_TYPE = 'plug'

# # === 控制模式 ===
# USE_MOVEIT = True  # True=使用MoveIt规划, False=使用SDK直接控制（推荐先用SDK测试）

# # === Toggle (拨动开关) 配置 ===
# TOGGLE_DIRECTION = 'left'      # 拨动方向: 'up'/'down'/'left'/'right'
# TOGGLE_PUSH_DISTANCE = 0.01  # 拨动行程 (米)
# TOGGLE_GRIPPER_OPEN = 70000  # 夹爪打开宽度 (0.001mm)
# TOGGLE_APPROACH_OFFSET = 0.05  # 接近偏移 (米)
# TOGGLE_APPROACH_SPEED = 20   # 接近速度
# TOGGLE_PUSH_SPEED = 40       # 推动速度
# TOGGLE_HOLD_TIME = 2       # 保持时间 (秒)

# # === Plug-in (插拔连接器) 配置 ===
# PLUGIN_ACTION = 'plug'       # 'plug'=插入, 'unplug'=拔出
# PLUGIN_INSERT_DEPTH = 0.04   # 插入深度 (米)
# PLUGIN_CONNECTOR_DIAMETER = 0.015  # 连接器直径 (米)
# PLUGIN_GRIPPER_HOLD = 30000  # 夹持宽度 (0.001mm)
# PLUGIN_GRIPPER_RELEASE = 50000  # 松开宽度 (0.001mm)
# PLUGIN_LIFT_HEIGHT = 0.05    # 抬起高度 (米)
# PLUGIN_APPROACH_OFFSET = 0.10  # 接近偏移 (米)
# PLUGIN_INSERT_SPEED = 15     # 插入速度
# PLUGIN_EXTRACT_SPEED = 20    # 拔出速度
# PLUGIN_SOCKET_OFFSET_X = 0.10  # 插座相对连接器的X偏移 (米)

# # === Push (按压按钮) 配置 ===
# PUSH_PRESS_DEPTH = 0.01      # 按压深度 (米)
# PUSH_HOLD_TIME = 2.0         # 保持按压时间 (秒)
# PUSH_GRIPPER_CLOSE = 0       # 夹爪闭合值 (0=完全闭合)
# PUSH_APPROACH_OFFSET = 0.08  # 接近偏移 (米) - 按钮前方8cm开始接近
# PUSH_PRESS_SPEED = 30        # 按压速度

# # === Knob (旋转旋钮) 配置 ===
# KNOB_ROTATION_ANGLE = 90     # 旋转角度 (度)
# KNOB_ROTATION_DIRECTION = 'cw'  # 'cw'=顺时针, 'ccw'=逆时针
# KNOB_DIAMETER = 0.020        # 旋钮直径 (米)
# KNOB_GRIPPER_OFFSET = 5000   # 夹爪比旋钮大的余量 (0.001mm)
# KNOB_APPROACH_OFFSET = 0.05  # 接近偏移 (米)
# KNOB_ROTATION_SPEED = 40     # 旋转速度
# KNOB_HOLD_TIME = 0.5         # 保持时间 (秒)
# KNOB_MAX_SINGLE_ROTATION = 180  # 单次最大旋转角度 (度)

# # === 通用速度配置 ===
# APPROACH_SPEED = 30  # 接近速度（慢速）
# NORMAL_SPEED = 60    # 正常速度
# FAST_SPEED = 80      # 快速（返回安全位置）

# # ========================================
# # 全局常量
# # ========================================
# PI = math.pi
# factor = 1000 * 180 / PI

# # ========================================
# # MoveIt 配置 (可选)
# # ========================================
# MOVEIT_AVAILABLE = False
# move_group = None
# try:
#     if USE_MOVEIT:
#         import moveit_commander
#         from moveit_msgs.msg import DisplayTrajectory
#         MOVEIT_AVAILABLE = True
#         print("✓ MoveIt已加载")
# except ImportError:
#     print("⚠️  MoveIt未加载，将使用SDK模式")

# # 全局变量
# piper = None
# piper_arm = None


# # ========================================
# # 控制函数
# # ========================================

# def control_arm_sdk(joints, speed=50, gripper_value=None):
#     """SDK 直接控制模式"""
#     global piper
    
#     joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
#     joints_int[4] = max(-70000, joints_int[4])
    
#     piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
#     piper.JointCtrl(*joints_int)
    
#     if gripper_value is not None:
#         gripper_int = int(gripper_value * 1000000)
#         piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
#     elif len(joints) > 6:
#         gripper_int = int(joints[6] * 1000000)
#         piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    
#     return True


# def control_arm_moveit(joints, speed=50, gripper_value=None):
#     """MoveIt 规划控制模式"""
#     global piper, move_group
    
#     if move_group is None:
#         return control_arm_sdk(joints, speed, gripper_value)
    
#     try:
#         move_group.clear_pose_targets()
#         move_group.stop()
        
#         target_joints = joints[:6] if len(joints) > 6 else joints
#         move_group.set_joint_value_target(target_joints)
        
#         plan = move_group.plan()
#         if isinstance(plan, tuple):
#             success, trajectory = plan[0], plan[1]
#         else:
#             success, trajectory = True, plan
        
#         if not success or not trajectory.joint_trajectory.points:
#             print("  ❌ MoveIt规划失败，切换到SDK模式")
#             return control_arm_sdk(joints, speed, gripper_value)
        
#         traj_points = trajectory.joint_trajectory.points
        
#         piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
#         rate = rospy.Rate(50)
        
#         sample_indices = np.linspace(0, len(traj_points) - 1, min(20, len(traj_points)), dtype=int)
#         for idx in sample_indices:
#             point = traj_points[idx]
#             joints_int = [int(point.positions[i] * factor) for i in range(6)]
#             joints_int[4] = max(-70000, joints_int[4])
#             piper.JointCtrl(*joints_int)
#             if idx < len(sample_indices) - 1:
#                 rate.sleep()
        
#         if gripper_value is not None:
#             gripper_int = int(gripper_value * 1000000)
#             piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
        
#         return True
#     except Exception as e:
#         print(f"  ❌ MoveIt执行失败: {e}，切换到SDK模式")
#         return control_arm_sdk(joints, speed, gripper_value)


# def control_arm(joints, speed=50, use_moveit=False):
#     """统一控制接口"""
#     gripper_value = joints[6] if len(joints) > 6 else None
    
#     if use_moveit and MOVEIT_AVAILABLE and move_group is not None:
#         return control_arm_moveit(joints[:6], speed, gripper_value)
#     else:
#         return control_arm_sdk(joints, speed, gripper_value)


# # ========================================
# # 四种按钮操作函数
# # ========================================

# def action_toggle():
#     """拨动开关操作"""
#     global piper_arm
    
#     print("="*70)
#     print("动作类型: Toggle (拨动开关)")
#     print("="*70)
#     print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
#     print(f"拨动方向: {TOGGLE_DIRECTION}, 行程: {TOGGLE_PUSH_DISTANCE*100:.1f}cm")
    
#     # 方向向量映射
#     direction_vectors = {
#         'up':    [0, 0, TOGGLE_PUSH_DISTANCE],
#         'down':  [0, 0, -TOGGLE_PUSH_DISTANCE],
#         'left':  [0, TOGGLE_PUSH_DISTANCE, 0],
#         'right': [0, -TOGGLE_PUSH_DISTANCE, 0]
#     }
    
#     if TOGGLE_DIRECTION not in direction_vectors:
#         print(f"❌ 无效的拨动方向: {TOGGLE_DIRECTION}")
#         return False
    
#     # 步骤1: 夹爪完全打开
#     print("\n步骤1: 夹爪完全打开...")
#     piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
#     time.sleep(0.8)
    
#     # 步骤2: 移动到拨片前方
#     print(f"\n步骤2: 移动到拨片前方 {TOGGLE_APPROACH_OFFSET*100:.0f}cm...")
#     targetT_pre = np.eye(4)
#     targetT_pre[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
#     targetT_pre[0, 3] = TARGET_X - TOGGLE_APPROACH_OFFSET
#     targetT_pre[1, 3] = TARGET_Y
#     targetT_pre[2, 3] = TARGET_Z
    
#     joints_pre = piper_arm.inverse_kinematics(targetT_pre)
#     if not joints_pre:
#         print("❌ 预接近位置IK失败")
#         return False
    
#     joints_pre.append(TOGGLE_GRIPPER_OPEN / 1000000)
#     if not control_arm(joints_pre, NORMAL_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(1.0)
    
#     # 步骤3: 接触拨片
#     print("\n步骤3: 接触拨片...")
#     targetT_contact = targetT_pre.copy()
#     targetT_contact[0, 3] = TARGET_X
    
#     joints_contact = piper_arm.inverse_kinematics(targetT_contact)
#     if not joints_contact:
#         print("❌ 接触位置IK失败")
#         return False
    
#     joints_contact.append(TOGGLE_GRIPPER_OPEN / 1000000)
#     if not control_arm(joints_contact, TOGGLE_APPROACH_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(0.5)
    
#     # 步骤4: 推动拨片
#     print(f"\n步骤4: 推动拨片 ({TOGGLE_DIRECTION})...")
#     direction_offset = direction_vectors[TOGGLE_DIRECTION]
#     targetT_push = targetT_contact.copy()
#     targetT_push[0, 3] += direction_offset[0]
#     targetT_push[1, 3] += direction_offset[1]
#     targetT_push[2, 3] += direction_offset[2]
    
#     joints_push = piper_arm.inverse_kinematics(targetT_push)
#     if not joints_push:
#         print("❌ 推动位置IK失败")
#         return False
    
#     joints_push.append(TOGGLE_GRIPPER_OPEN / 1000000)
#     if not control_arm(joints_push, TOGGLE_PUSH_SPEED, USE_MOVEIT):
#         return False
    
#     # 步骤5: 保持
#     print(f"\n步骤5: 保持 {TOGGLE_HOLD_TIME}秒...")
#     time.sleep(TOGGLE_HOLD_TIME)
    
#     # 步骤6: 回退
#     print("\n步骤6: 回退到预接近位置...")
#     if not control_arm(joints_pre, TOGGLE_PUSH_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(0.5)
    
#     # 步骤7: 返回安全位置
#     print("\n步骤7: 返回安全位置...")
#     joints_safe = [0, 0, -0.4, 0, 0, 0, TOGGLE_GRIPPER_OPEN / 1000000]
#     if not control_arm(joints_safe, FAST_SPEED, USE_MOVEIT):
#         return False
    
#     print("="*70)
#     print("✓✓✓ Toggle 操作完成！✓✓✓")
#     print("="*70)
#     return True


# def action_plugin():
#     """插拔连接器操作"""
#     global piper_arm
    
#     print("="*70)
#     print(f"动作类型: Plug-in ({'插入' if PLUGIN_ACTION == 'plug' else '拔出'}连接器)")
#     print("="*70)
#     print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
#     print(f"插入深度: {PLUGIN_INSERT_DEPTH*100:.1f}cm")
    
#     if PLUGIN_ACTION == 'plug':
#         # === 插入流程 ===
#         print("\n步骤1: 夹爪打开...")
#         piper.GripperCtrl(PLUGIN_GRIPPER_RELEASE, 1000, 0x01, 0)
#         time.sleep(0.8)
        
#         print(f"\n步骤2: 移动到连接器上方 {PLUGIN_APPROACH_OFFSET*100:.0f}cm...")
#         targetT_above = np.eye(4)
#         targetT_above[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
#         targetT_above[0, 3] = TARGET_X
#         targetT_above[1, 3] = TARGET_Y
#         targetT_above[2, 3] = TARGET_Z + PLUGIN_APPROACH_OFFSET
        
#         joints_above = piper_arm.inverse_kinematics(targetT_above)
#         if not joints_above:
#             print("❌ 上方位置IK失败")
#             return False
        
#         joints_above.append(PLUGIN_GRIPPER_RELEASE / 1000000)
#         if not control_arm(joints_above, NORMAL_SPEED, USE_MOVEIT):
#             return False
#         time.sleep(1.0)
        
#         print("\n步骤3: 下降并夹持连接器...")
#         targetT_grasp = targetT_above.copy()
#         targetT_grasp[2, 3] = TARGET_Z
        
#         joints_grasp = piper_arm.inverse_kinematics(targetT_grasp)
#         if not joints_grasp:
#             print("❌ 夹持位置IK失败")
#             return False
        
#         joints_grasp.append(PLUGIN_GRIPPER_HOLD / 1000000)
#         if not control_arm(joints_grasp, PLUGIN_INSERT_SPEED, USE_MOVEIT):
#             return False
#         time.sleep(1.0)
        
#         print(f"\n步骤4: 抬起连接器 {PLUGIN_LIFT_HEIGHT*100:.0f}cm...")
#         targetT_lift = targetT_grasp.copy()
#         targetT_lift[2, 3] += PLUGIN_LIFT_HEIGHT
        
#         joints_lift = piper_arm.inverse_kinematics(targetT_lift)
#         if not joints_lift:
#             print("❌ 抬起位置IK失败")
#             return False
        
#         joints_lift.append(PLUGIN_GRIPPER_HOLD / 1000000)
#         if not control_arm(joints_lift, APPROACH_SPEED, USE_MOVEIT):
#             return False
#         time.sleep(0.8)
        
#         print("\n步骤5: 移动到插座上方...")
#         targetT_socket = targetT_lift.copy()
#         targetT_socket[0, 3] += PLUGIN_SOCKET_OFFSET_X
        
#         joints_socket = piper_arm.inverse_kinematics(targetT_socket)
#         if not joints_socket:
#             print("❌ 插座位置IK失败")
#             return False
        
#         joints_socket.append(PLUGIN_GRIPPER_HOLD / 1000000)
#         if not control_arm(joints_socket, NORMAL_SPEED, USE_MOVEIT):
#             return False
#         time.sleep(1.0)
        
#         print(f"\n步骤6: 垂直插入 (深度{PLUGIN_INSERT_DEPTH*100:.1f}cm)...")
#         targetT_insert = targetT_socket.copy()
#         targetT_insert[2, 3] -= PLUGIN_INSERT_DEPTH
        
#         joints_insert = piper_arm.inverse_kinematics(targetT_insert)
#         if not joints_insert:
#             print("❌ 插入位置IK失败")
#             return False
        
#         joints_insert.append(PLUGIN_GRIPPER_HOLD / 1000000)
#         if not control_arm(joints_insert, PLUGIN_INSERT_SPEED, USE_MOVEIT):
#             return False
#         time.sleep(1.0)
        
#         print("\n步骤7: 松开夹爪...")
#         joints_insert[6] = PLUGIN_GRIPPER_RELEASE / 1000000
#         control_arm_sdk(joints_insert, 10)
#         time.sleep(0.5)
        
#         print("\n步骤8: 垂直上升...")
#         if not control_arm(joints_socket, APPROACH_SPEED, USE_MOVEIT):
#             return False
        
#     else:  # unplug
#         # === 拔出流程 ===
#         print("\n步骤1: 夹爪打开...")
#         piper.GripperCtrl(PLUGIN_GRIPPER_RELEASE, 1000, 0x01, 0)
#         time.sleep(0.8)
        
#         print("\n步骤2: 移动到连接器位置...")
#         targetT_connector = np.eye(4)
#         targetT_connector[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
#         targetT_connector[0, 3] = TARGET_X
#         targetT_connector[1, 3] = TARGET_Y
#         targetT_connector[2, 3] = TARGET_Z
        
#         joints_connector = piper_arm.inverse_kinematics(targetT_connector)
#         if not joints_connector:
#             print("❌ 连接器位置IK失败")
#             return False
        
#         joints_connector.append(PLUGIN_GRIPPER_HOLD / 1000000)
#         if not control_arm(joints_connector, PLUGIN_INSERT_SPEED, USE_MOVEIT):
#             return False
#         time.sleep(1.0)
        
#         print(f"\n步骤3: 垂直拔出 (行程{PLUGIN_INSERT_DEPTH*100:.1f}cm)...")
#         targetT_extract = targetT_connector.copy()
#         targetT_extract[2, 3] += PLUGIN_INSERT_DEPTH
        
#         joints_extract = piper_arm.inverse_kinematics(targetT_extract)
#         if not joints_extract:
#             print("❌ 拔出位置IK失败")
#             return False
        
#         joints_extract.append(PLUGIN_GRIPPER_HOLD / 1000000)
#         if not control_arm(joints_extract, PLUGIN_EXTRACT_SPEED, USE_MOVEIT):
#             return False
#         time.sleep(1.0)
        
#         print("\n步骤4: 松开连接器...")
#         joints_extract[6] = PLUGIN_GRIPPER_RELEASE / 1000000
#         control_arm_sdk(joints_extract, 10)
#         time.sleep(0.5)
    
#     # 返回安全位置
#     print("\n返回安全位置...")
#     joints_safe = [0, 0, -0.4, 0, 0, 0, PLUGIN_GRIPPER_RELEASE / 1000000]
#     if not control_arm(joints_safe, FAST_SPEED, USE_MOVEIT):
#         return False
    
#     print("="*70)
#     print(f"✓✓✓ Plug-in ({'插入' if PLUGIN_ACTION == 'plug' else '拔出'}) 操作完成！✓✓✓")
#     print("="*70)
#     return True


# def action_push():
#     """按压按钮操作"""
#     global piper_arm
    
#     print("="*70)
#     print("动作类型: Push (按压按钮)")
#     print("="*70)
#     print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
#     print(f"按压深度: {PUSH_PRESS_DEPTH*100:.1f}cm, 保持时间: {PUSH_HOLD_TIME}秒")
    
#     # 步骤1: 夹爪闭合
#     print("\n步骤1: 夹爪闭合（形成按压面）...")
#     piper.GripperCtrl(PUSH_GRIPPER_CLOSE, 1000, 0x01, 0)
#     time.sleep(0.8)
    
#     # 步骤2: 移动到按钮上方
#     print(f"\n步骤2: 移动到按钮上方 {PUSH_APPROACH_OFFSET*100:.0f}cm...")
#     targetT_above = np.eye(4)
#     targetT_above[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
#     targetT_above[0, 3] = TARGET_X - PUSH_APPROACH_OFFSET
#     targetT_above[1, 3] = TARGET_Y
#     targetT_above[2, 3] = TARGET_Z
    
#     joints_above = piper_arm.inverse_kinematics(targetT_above)
#     if not joints_above:
#         print("❌ 上方位置IK失败")
#         return False
    
#     joints_above.append(PUSH_GRIPPER_CLOSE / 1000000)
#     if not control_arm(joints_above, NORMAL_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(1.0)
    
#     # 步骤3: 接近按钮表面
#     print("\n步骤3: 缓慢接近按钮表面...")
#     targetT_surface = targetT_above.copy()
#     targetT_surface[0, 3] = TARGET_X
    
#     joints_surface = piper_arm.inverse_kinematics(targetT_surface)
#     if not joints_surface:
#         print("❌ 表面位置IK失败")
#         return False
    
#     joints_surface.append(PUSH_GRIPPER_CLOSE / 1000000)
#     if not control_arm(joints_surface, PUSH_PRESS_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(0.5)
    
#     # 步骤4: 按压
#     print(f"\n步骤4: 按压到指定深度 {PUSH_PRESS_DEPTH*100:.1f}cm...")
#     targetT_press = targetT_surface.copy()
#     targetT_press[0, 3] += PUSH_PRESS_DEPTH
    
#     joints_press = piper_arm.inverse_kinematics(targetT_press)
#     if not joints_press:
#         print("❌ 按压位置IK失败")
#         return False
    
#     joints_press.append(PUSH_GRIPPER_CLOSE / 1000000)
#     if not control_arm(joints_press, PUSH_PRESS_SPEED, USE_MOVEIT):
#         return False
    
#     # 步骤5: 保持按压
#     print(f"\n步骤5: 保持按压 {PUSH_HOLD_TIME}秒...")
#     time.sleep(PUSH_HOLD_TIME)
    
#     # 步骤6: 释放
#     print("\n步骤6: 释放按压...")
#     if not control_arm(joints_surface, PUSH_PRESS_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(0.5)
    
#     # 步骤7: 返回上方
#     print("\n步骤7: 返回上方位置...")
#     if not control_arm(joints_above, NORMAL_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(0.5)
    
#     # 步骤8: 返回安全位置
#     print("\n步骤8: 返回安全位置...")
#     joints_safe = [0, 0, -0.4, 0, 0, 0, PUSH_GRIPPER_CLOSE / 1000000]
#     if not control_arm(joints_safe, FAST_SPEED, USE_MOVEIT):
#         return False
    
#     print("="*70)
#     print("✓✓✓ Push 操作完成！✓✓✓")
#     print("="*70)
#     return True


# def action_knob():
#     """旋转旋钮操作"""
#     global piper_arm
    
#     gripper_width = int(KNOB_DIAMETER * 1000000) + KNOB_GRIPPER_OFFSET
    
#     print("="*70)
#     print("动作类型: Knob (旋转旋钮)")
#     print("="*70)
#     print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
#     print(f"旋转: {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})")
#     print(f"夹持宽度: {gripper_width/1000:.1f}mm")
    
#     # 步骤1: 夹爪打开
#     print(f"\n步骤1: 夹爪打开到 {gripper_width/1000:.1f}mm...")
#     piper.GripperCtrl(gripper_width + 5000, 1000, 0x01, 0)
#     time.sleep(0.8)
    
#     # 步骤2: 移动到旋钮上方
#     print(f"\n步骤2: 移动到旋钮上方 {KNOB_APPROACH_OFFSET*100:.0f}cm...")
#     targetT_above = np.eye(4)
#     targetT_above[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
#     targetT_above[0, 3] = TARGET_X
#     targetT_above[1, 3] = TARGET_Y
#     targetT_above[2, 3] = TARGET_Z + KNOB_APPROACH_OFFSET
    
#     joints_above = piper_arm.inverse_kinematics(targetT_above)
#     if not joints_above:
#         print("❌ 上方位置IK失败")
#         return False
    
#     joints_above.append((gripper_width + 5000) / 1000000)
#     if not control_arm(joints_above, NORMAL_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(1.0)
    
#     # 步骤3: 下降到旋钮高度
#     print("\n步骤3: 下降到旋钮高度...")
#     targetT_knob = targetT_above.copy()
#     targetT_knob[2, 3] = TARGET_Z
    
#     joints_knob = piper_arm.inverse_kinematics(targetT_knob)
#     if not joints_knob:
#         print("❌ 旋钮位置IK失败")
#         return False
    
#     joints_knob.append(gripper_width / 1000000)
#     if not control_arm(joints_knob, APPROACH_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(1.0)
    
#     # 步骤4: 旋转
#     direction_sign = 1 if KNOB_ROTATION_DIRECTION == 'cw' else -1
#     total_rotation = KNOB_ROTATION_ANGLE * direction_sign
    
#     if abs(total_rotation) > KNOB_MAX_SINGLE_ROTATION:
#         print(f"\n步骤4: 分段旋转 (总角度{KNOB_ROTATION_ANGLE}°)...")
#         segments = int(np.ceil(abs(total_rotation) / KNOB_MAX_SINGLE_ROTATION))
#         angle_per_segment = total_rotation / segments
        
#         for i in range(segments):
#             print(f"  旋转段 {i+1}/{segments}: {angle_per_segment:.1f}°")
#             joints_knob[5] += angle_per_segment * PI / 180
#             if not control_arm(joints_knob, KNOB_ROTATION_SPEED, USE_MOVEIT):
#                 return False
#             time.sleep(KNOB_HOLD_TIME)
#     else:
#         print(f"\n步骤4: 旋转夹爪 {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})...")
#         joints_knob[5] += total_rotation * PI / 180
#         if not control_arm(joints_knob, KNOB_ROTATION_SPEED, USE_MOVEIT):
#             return False
#         time.sleep(KNOB_HOLD_TIME)
    
#     # 步骤5: 松开夹爪
#     print("\n步骤5: 松开夹爪...")
#     joints_knob[6] = (gripper_width + 5000) / 1000000
#     control_arm_sdk(joints_knob, 10)
#     time.sleep(0.5)
    
#     # 步骤6: 上升
#     print("\n步骤6: 上升...")
#     if not control_arm(joints_above, APPROACH_SPEED, USE_MOVEIT):
#         return False
#     time.sleep(0.5)
    
#     # 步骤7: 返回安全位置
#     print("\n步骤7: 返回安全位置...")
#     joints_safe = [0, 0, -0.4, 0, 0, 0, 40000 / 1000000]
#     if not control_arm(joints_safe, FAST_SPEED, USE_MOVEIT):
#         return False
    
#     print("="*70)
#     print("✓✓✓ Knob 操作完成！✓✓✓")
#     print("="*70)
#     return True


# # ========================================
# # 主程序
# # ========================================

# def main():
#     global piper, piper_arm, move_group
    
#     print("="*70)
#     print("按钮操作执行器 - 独立版本")
#     print("="*70)
#     print(f"\n📍 目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
#     print(f"🎯 动作类型: {ACTION_TYPE.upper()}")
#     print(f"🔧 控制模式: {'MoveIt' if USE_MOVEIT and MOVEIT_AVAILABLE else 'SDK'}")
    
#     # 显示动作特定参数
#     if ACTION_TYPE == 'toggle':
#         print(f"\n拨动开关配置:")
#         print(f"  方向: {TOGGLE_DIRECTION}")
#         print(f"  行程: {TOGGLE_PUSH_DISTANCE*100:.1f}cm")
#     elif ACTION_TYPE == 'plugin':
#         print(f"\n插拔连接器配置:")
#         print(f"  动作: {'插入' if PLUGIN_ACTION == 'plug' else '拔出'}")
#         print(f"  深度: {PLUGIN_INSERT_DEPTH*100:.1f}cm")
#     elif ACTION_TYPE == 'push':
#         print(f"\n按压按钮配置:")
#         print(f"  深度: {PUSH_PRESS_DEPTH*100:.1f}cm")
#         print(f"  保持: {PUSH_HOLD_TIME}秒")
#     elif ACTION_TYPE == 'knob':
#         print(f"\n旋转旋钮配置:")
#         print(f"  角度: {KNOB_ROTATION_ANGLE}°")
#         print(f"  方向: {KNOB_ROTATION_DIRECTION}")
    
#     print("="*70)
    
#     # 初始化硬件
#     print("\n初始化机械臂...")
#     piper = C_PiperInterface_V2("can0")
#     piper.ConnectPort()
#     piper.EnableArm(7)
    
#     # 使能所有关节
#     for i in range(7):
#         piper.EnableArm(i + 1)
#         time.sleep(0.1)
    
#     piper.GripperCtrl(70000, 1000, 0x01, 0)
#     print("  ✓ 硬件初始化完成")
    
#     # 初始化 ROS
#     print("\n初始化ROS...")
#     rospy.init_node('button_action_node', anonymous=True)
    
#     # 初始化 MoveIt (如果需要)
#     if USE_MOVEIT and MOVEIT_AVAILABLE:
#         try:
#             import os
#             piper_ros_path = "/home/robot/button/V4.0/project2/piper_ros"
#             src_path = os.path.join(piper_ros_path, 'src')
#             current_path = os.environ.get('ROS_PACKAGE_PATH', '')
#             if src_path not in current_path:
#                 os.environ['ROS_PACKAGE_PATH'] = f"{src_path}:{current_path}"
            
#             moveit_commander.roscpp_initialize([])
#             robot = moveit_commander.RobotCommander()
#             move_group = moveit_commander.MoveGroupCommander("arm")
#             move_group.set_planning_time(5.0)
#             move_group.set_max_velocity_scaling_factor(1.0)
#             move_group.set_max_acceleration_scaling_factor(1.0)
#             print("  ✓ MoveIt初始化完成")
#         except Exception as e:
#             print(f"  ⚠️  MoveIt初始化失败: {e}")
#             print("  将使用SDK模式")
    
#     # 初始化 Piper Arm
#     piper_arm = PiperArm()
    
#     # 回零位
#     print("\n回零位...")
#     joints_zero = [0, 0, 0, 0, 0, 0, 0]
#     control_arm_sdk(joints_zero, 100)
#     time.sleep(2)
#     print("  ✓ 已回零位")
    
#     print("\n="*70)
#     print("开始执行动作...")
#     print("="*70)
    
#     # 执行对应动作
#     action_functions = {
#         'toggle': action_toggle,
#         'plugin': action_plugin,
#         'push': action_push,
#         'knob': action_knob
#     }
    
#     if ACTION_TYPE not in action_functions:
#         print(f"❌ 未知动作类型: {ACTION_TYPE}")
#         print(f"   支持的类型: {list(action_functions.keys())}")
#         return
    
#     try:
#         success = action_functions[ACTION_TYPE]()
#         if success:
#             print("\n✓ 动作执行成功！")
#         else:
#             print("\n❌ 动作执行失败")
#     except Exception as e:
#         print(f"\n❌ 动作执行异常: {e}")
#         import traceback
#         traceback.print_exc()
    
#     # 清理资源
#     if MOVEIT_AVAILABLE:
#         moveit_commander.roscpp_shutdown()
    
#     print("\n程序结束")


# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3
"""
按钮操作执行器 - 独立版本
支持四种按钮操作类型：Toggle, Plug-in, Push, Knob
所有参数通过宏定义配置，无需视觉检测
"""
from piper_sdk import *
import rospy
import time
import numpy as np
import math
from piper_arm import PiperArm

# ========================================
# 宏定义 - 用户配置区
# ========================================

# === 目标位置配置 (基座坐标系，单位：米) ===
TARGET_X = 0.40  # X坐标
TARGET_Y = 0.00  # Y坐标
TARGET_Z = 0.20  # Z坐标

# === 动作类型选择 ===
ACTION_TYPE = 'push'  # 'toggle'/'plugin'/'push'/'knob'

# === 控制模式 ===
USE_MOVEIT = True  # True=使用MoveIt规划, False=使用SDK直接控制

# === Plugin (插拔连接器) 配置 ===
PLUGIN_GRIPPER_OPEN = 70000     # 张开宽度 (0.001mm)
PLUGIN_INSERT_DEPTH = 0.03      # 插入深度 (米)
PLUGIN_GRIPPER_HOLD = 30000     # 闭合夹持宽度 (0.001mm)
PLUGIN_INSERT_SPEED = 100        # 插入速度
PLUGIN_EXTRACT_SPEED = 100       # 拔出速度

# === Toggle (拨动开关) 配置 ===
TOGGLE_GRIPPER_OPEN = 70000     # 张开宽度 (0.001mm)
TOGGLE_JOINT4_ROTATE = 90       # joint4旋转角度 (度)
TOGGLE_INSERT_DEPTH = 0.03      # 插入深度 (米)
TOGGLE_GRIPPER_HOLD = 30000     # 闭合夹持宽度 (0.001mm)
TOGGLE_JOINT3_ANGLE = 30        # joint3拨动角度 (度)
TOGGLE_DIRECTION = 'left'       # 拨动方向: 'left'(左拨) / 'right'(右拨)
TOGGLE_INSERT_SPEED = 20        # 插入速度
TOGGLE_TOGGLE_SPEED = 30        # 拨动速度

# === Push (按压按钮) 配置 ===
PUSH_GRIPPER_CLOSE = 0          # 夹爪闭合值
PUSH_INSERT_DEPTH = 0.01        # 按压深度 (米)
PUSH_HOLD_TIME = 2            # 保持时间 (秒)
PUSH_PRESS_SPEED = 30           # 按压速度

# === Knob (旋转旋钮) 配置 ===
KNOB_GRIPPER_OPEN = 70000       # 张开宽度 (0.001mm)
KNOB_INSERT_DEPTH = 0.01        # 插入深度 (米)
KNOB_GRIPPER_HOLD = 25000       # 闭合夹持宽度 (0.001mm)
KNOB_ROTATION_ANGLE = 90        # 旋转角度 (度)
KNOB_ROTATION_DIRECTION = 'cw'  # 'cw'=顺时针(右旋), 'ccw'=逆时针(左旋)
KNOB_INSERT_SPEED = 20          # 插入速度
KNOB_ROTATION_SPEED = 40        # 旋转速度

# === 通用速度配置 ===
NORMAL_SPEED = 60    # 正常移动速度
FAST_SPEED = 80      # 快速移动速度

# ========================================
# 全局常量
# ========================================
PI = math.pi
factor = 1000 * 180 / PI

# ========================================
# MoveIt 配置 (可选)
# ========================================
# 轨迹执行频率控制
TRAJECTORY_PUBLISH_RATE = 10   # 轨迹发布到RViz的频率 (Hz)
TRAJECTORY_EXECUTE_RATE = 50   # SDK执行轨迹的频率 (Hz) - 建议50Hz
TRAJECTORY_SAMPLE_POINTS = 20  # 轨迹采样点数

MOVEIT_AVAILABLE = False
move_group = None
try:
    if USE_MOVEIT:
        import moveit_commander
        from moveit_msgs.msg import DisplayTrajectory
        MOVEIT_AVAILABLE = True
        print("✓ MoveIt已加载")
except ImportError:
    print("⚠️  MoveIt未加载，将使用SDK模式")

# 全局变量
piper = None
piper_arm = None


# ========================================
# 控制函数
# ========================================

def control_arm_sdk(joints, speed=50, gripper_value=None):
    """SDK 直接控制模式"""
    global piper
    
    joints_int = [int(joints[i] * factor) for i in range(min(6, len(joints)))]
    joints_int[4] = max(-70000, joints_int[4])
    
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joints_int)
    
    if gripper_value is not None:
        gripper_int = int(gripper_value)
        piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    elif len(joints) > 6:
        gripper_int = int(joints[6] * 1000000)
        piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
    
    return True


def control_arm_moveit(joints, speed=50, gripper_value=None):
    """MoveIt 规划控制模式"""
    global piper, move_group
    
    if move_group is None:
        return control_arm_sdk(joints, speed, gripper_value)
    
    try:
        move_group.clear_pose_targets()
        move_group.stop()
        
        target_joints = joints[:6] if len(joints) > 6 else joints
        move_group.set_joint_value_target(target_joints)
        
        # MoveIt 规划
        print("  [MoveIt] 规划轨迹...")
        plan = move_group.plan()
        if isinstance(plan, tuple):
            success, trajectory = plan[0], plan[1]
        else:
            success, trajectory = True, plan
        
        if not success or not trajectory.joint_trajectory.points:
            print("  ❌ 规划失败，切换到SDK模式")
            return control_arm_sdk(joints, speed, gripper_value)
        
        traj_points = trajectory.joint_trajectory.points
        print(f"  ✓ 规划成功 (轨迹点: {len(traj_points)})")
        
        # SDK 执行轨迹（采样）
        sample_indices = np.linspace(0, len(traj_points)-1, min(TRAJECTORY_SAMPLE_POINTS, len(traj_points)), dtype=int)
        sample_points = [traj_points[i] for i in sample_indices]
        
        print(f"  [SDK] 执行轨迹 (采样点: {len(sample_points)}, 速度: {speed}, 频率: {TRAJECTORY_EXECUTE_RATE}Hz)")
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        
        # 使用ROS Rate控制执行频率
        rate = rospy.Rate(TRAJECTORY_EXECUTE_RATE)
        for idx, point in enumerate(sample_points):
            joints_int = [int(point.positions[i] * factor) for i in range(6)]
            joints_int[4] = max(-70000, joints_int[4])
            piper.JointCtrl(*joints_int)
            
            # 按照指定频率执行
            if idx < len(sample_points) - 1:  # 最后一个点不sleep
                rate.sleep()
        
        # 到达最终位置并等待稳定
        final_joints = [int(traj_points[-1].positions[i] * factor) for i in range(6)]
        final_joints[4] = max(-70000, final_joints[4])
        piper.JointCtrl(*final_joints)
        rospy.sleep(0.5)  # 等待到达
        
        # 控制夹爪
        if gripper_value is not None:
            gripper_int = int(gripper_value)
            piper.GripperCtrl(abs(gripper_int), 1000, 0x01, 0)
        
        print(f"  ✓ 执行完成")
        return True
    except Exception as e:
        print(f"  ❌ MoveIt执行失败: {e}，切换到SDK模式")
        return control_arm_sdk(joints, speed, gripper_value)


def control_arm(joints, speed=50, use_moveit=False, gripper_value=None):
    """统一控制接口"""
    if gripper_value is None:
        gripper_value = joints[6] * 1000000 if len(joints) > 6 else None
    
    if use_moveit and MOVEIT_AVAILABLE and move_group is not None:
        return control_arm_moveit(joints[:6], speed, gripper_value)
    else:
        return control_arm_sdk(joints, speed, gripper_value)


def get_current_joints():
    """获取当前关节角度"""
    global piper
    msg = piper.GetArmJointMsgs()
    return [
        msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_6 * 1e-3 * PI / 180.0,
    ]


def move_along_end_effector_z(current_joints, distance, speed=20):
    """
    沿末端执行器z轴方向移动
    
    参数:
        current_joints: 当前关节角度 (弧度)
        distance: 移动距离 (米)，正值=前进，负值=后退
        speed: 移动速度
    
    返回:
        新的关节角度
    """
    global piper_arm
    
    # 获取当前末端位姿
    current_T = piper_arm.forward_kinematics(current_joints)
    
    # 沿末端z轴移动（末端坐标系的z轴是 current_T 的第一列，即 [0,0,1] 在末端坐标系中）
    # 在基坐标系中，末端z轴方向是 current_T[:3, 0]
    z_axis = current_T[:3, 0]
    
    # 计算新的目标位置
    target_T = current_T.copy()
    target_T[:3, 3] += z_axis * distance
    
    # 逆运动学求解
    target_joints = piper_arm.inverse_kinematics(target_T)
    if not target_joints:
        print(f"  ❌ 移动距离{distance*100:.1f}cm的IK求解失败")
        return None
    
    # 执行运动
    if not control_arm(target_joints, speed, USE_MOVEIT):
        return None
    
    return target_joints


# ========================================
# 四种按钮操作函数（重写版）
# ========================================

def action_plugin():
    """
    插拔连接器操作
    流程: 张开 → 到达 → 插入(z轴前进) → 闭合 → 拔出(z轴后退) → 张开 → 回零闭合
    """
    global piper_arm
    
    print("="*70)
    print("动作类型: Plugin (插拔连接器)")
    print("="*70)
    print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"插入深度: {PLUGIN_INSERT_DEPTH*100:.1f}cm")
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(PLUGIN_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    targetT = np.eye(4)
    targetT[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])  # 夹爪朝前
    targetT[0, 3] = TARGET_X
    targetT[1, 3] = TARGET_Y
    targetT[2, 3] = TARGET_Z
    
    joints_target = piper_arm.inverse_kinematics(targetT)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    if not control_arm(joints_target, NORMAL_SPEED, USE_MOVEIT, PLUGIN_GRIPPER_OPEN):
        return False
    time.sleep(1.0)
    
    # 步骤3: 沿末端z轴插入
    print(f"\n步骤3: 沿末端z轴插入 {PLUGIN_INSERT_DEPTH*100:.1f}cm...")
    joints_insert = move_along_end_effector_z(joints_target, PLUGIN_INSERT_DEPTH, PLUGIN_INSERT_SPEED)
    if not joints_insert:
        return False
    time.sleep(0.5)
    
    # 步骤4: 夹爪闭合
    print(f"\n步骤4: 夹爪闭合到 {PLUGIN_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(PLUGIN_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(1.0)
    
    # 步骤5: 沿末端z轴拔出
    print(f"\n步骤5: 沿末端z轴拔出 {PLUGIN_INSERT_DEPTH*100:.1f}cm...")
    joints_extract = move_along_end_effector_z(joints_insert, -PLUGIN_INSERT_DEPTH, PLUGIN_EXTRACT_SPEED)
    if not joints_extract:
        return False
    time.sleep(0.5)
    
    # 步骤6: 夹爪张开
    print("\n步骤6: 夹爪张开...")
    piper.GripperCtrl(PLUGIN_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤7: 回零位
    print("\n步骤7: 回零位...")
    joints_zero = [0, 0, 0, 0, 0, 0]
    if not control_arm(joints_zero, FAST_SPEED, USE_MOVEIT):
        return False
    time.sleep(1.0)
    
    # 步骤8: 夹爪闭合
    print("\n步骤8: 夹爪闭合...")
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.5)
    
    print("="*70)
    print("✓✓✓ Plugin 操作完成！✓✓✓")
    print("="*70)
    return True


def action_toggle():
    """
    拨动开关操作
    流程: 张开 → 到达 → joint4旋转90° → 插入(z轴前进) → 闭合 → joint3拨动 → 张开 → 回零闭合
    """
    global piper_arm
    
    print("="*70)
    print("动作类型: Toggle (拨动开关)")
    print("="*70)
    print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"joint4旋转: {TOGGLE_JOINT4_ROTATE}°, 插入: {TOGGLE_INSERT_DEPTH*100:.1f}cm")
    print(f"joint3拨动: {TOGGLE_JOINT3_ANGLE}° ({TOGGLE_DIRECTION})")
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    targetT = np.eye(4)
    targetT[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    targetT[0, 3] = TARGET_X
    targetT[1, 3] = TARGET_Y
    targetT[2, 3] = TARGET_Z
    
    joints_target = piper_arm.inverse_kinematics(targetT)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    if not control_arm(joints_target, NORMAL_SPEED, USE_MOVEIT, TOGGLE_GRIPPER_OPEN):
        return False
    time.sleep(1.0)
    
    # 步骤3: joint4旋转90度
    print(f"\n步骤3: joint4旋转 {TOGGLE_JOINT4_ROTATE}°...")
    joints_rotate = joints_target.copy()
    joints_rotate[3] += TOGGLE_JOINT4_ROTATE * PI / 180
    if not control_arm(joints_rotate, NORMAL_SPEED, USE_MOVEIT, TOGGLE_GRIPPER_OPEN):
        return False
    time.sleep(1.0)
    
    # 步骤4: 沿末端z轴插入
    print(f"\n步骤4: 沿末端z轴插入 {TOGGLE_INSERT_DEPTH*100:.1f}cm...")
    joints_insert = move_along_end_effector_z(joints_rotate, TOGGLE_INSERT_DEPTH, TOGGLE_INSERT_SPEED)
    if not joints_insert:
        return False
    time.sleep(0.5)
    
    # 步骤5: 夹爪闭合
    print(f"\n步骤5: 夹爪闭合到 {TOGGLE_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(TOGGLE_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(1.0)
    
    # 步骤6: joint3拨动
    direction_sign = -1 if TOGGLE_DIRECTION == 'left' else 1
    print(f"\n步骤6: joint3 {TOGGLE_DIRECTION}拨 {TOGGLE_JOINT3_ANGLE}°...")
    joints_toggle = joints_insert.copy()
    joints_toggle[2] += direction_sign * TOGGLE_JOINT3_ANGLE * PI / 180
    if not control_arm(joints_toggle, TOGGLE_TOGGLE_SPEED, USE_MOVEIT, TOGGLE_GRIPPER_HOLD):
        return False
    time.sleep(1.0)
    
    # 步骤7: 夹爪张开
    print("\n步骤7: 夹爪张开...")
    piper.GripperCtrl(TOGGLE_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤8: 回零位
    print("\n步骤8: 回零位...")
    joints_zero = [0, 0, 0, 0, 0, 0]
    if not control_arm(joints_zero, FAST_SPEED, USE_MOVEIT):
        return False
    time.sleep(1.0)
    
    # 步骤9: 夹爪闭合
    print("\n步骤9: 夹爪闭合...")
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.5)
    
    print("="*70)
    print("✓✓✓ Toggle 操作完成！✓✓✓")
    print("="*70)
    return True


def action_push():
    """
    按压按钮操作
    流程: 闭合 → 到达 → 插入(z轴前进) → 保持0.5s → 返回 → 回零
    """
    global piper_arm
    
    print("="*70)
    print("动作类型: Push (按压按钮)")
    print("="*70)
    print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"按压深度: {PUSH_INSERT_DEPTH*100:.1f}cm, 保持: {PUSH_HOLD_TIME}秒")
    
    # 步骤1: 夹爪闭合
    print("\n步骤1: 夹爪闭合...")
    piper.GripperCtrl(PUSH_GRIPPER_CLOSE, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    targetT = np.eye(4)
    targetT[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    targetT[0, 3] = TARGET_X
    targetT[1, 3] = TARGET_Y
    targetT[2, 3] = TARGET_Z
    
    joints_target = piper_arm.inverse_kinematics(targetT)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    if not control_arm(joints_target, NORMAL_SPEED, USE_MOVEIT, PUSH_GRIPPER_CLOSE):
        return False
    time.sleep(1.0)
    
    # 步骤3: 沿末端z轴插入（按压）
    print(f"\n步骤3: 沿末端z轴按压 {PUSH_INSERT_DEPTH*100:.1f}cm...")
    joints_press = move_along_end_effector_z(joints_target, PUSH_INSERT_DEPTH, PUSH_PRESS_SPEED)
    if not joints_press:
        return False
    
    # 步骤4: 保持按压
    print(f"\n步骤4: 保持按压 {PUSH_HOLD_TIME}秒...")
    time.sleep(PUSH_HOLD_TIME)
    
    # 步骤5: 返回到目标位置
    print("\n步骤5: 返回目标位置...")
    if not control_arm(joints_target, PUSH_PRESS_SPEED, USE_MOVEIT, PUSH_GRIPPER_CLOSE):
        return False
    time.sleep(0.5)
    
    # 步骤6: 回零位
    print("\n步骤6: 回零位...")
    joints_zero = [0, 0, 0, 0, 0, 0]
    if not control_arm(joints_zero, FAST_SPEED, USE_MOVEIT, PUSH_GRIPPER_CLOSE):
        return False
    time.sleep(1.0)
    
    print("="*70)
    print("✓✓✓ Push 操作完成！✓✓✓")
    print("="*70)
    return True


def action_knob():
    """
    旋转旋钮操作
    流程: 张开 → 到达 → 插入(z轴前进) → 闭合 → 旋转 → 回零闭合
    """
    global piper_arm
    
    print("="*70)
    print("动作类型: Knob (旋转旋钮)")
    print("="*70)
    print(f"目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"插入: {KNOB_INSERT_DEPTH*100:.1f}cm, 旋转: {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})")
    
    # 步骤1: 夹爪张开
    print("\n步骤1: 夹爪张开...")
    piper.GripperCtrl(KNOB_GRIPPER_OPEN, 1000, 0x01, 0)
    time.sleep(0.8)
    
    # 步骤2: 移动到目标位置
    print("\n步骤2: 移动到目标位置...")
    targetT = np.eye(4)
    targetT[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    targetT[0, 3] = TARGET_X
    targetT[1, 3] = TARGET_Y
    targetT[2, 3] = TARGET_Z
    
    joints_target = piper_arm.inverse_kinematics(targetT)
    if not joints_target:
        print("❌ 目标位置IK失败")
        return False
    
    if not control_arm(joints_target, NORMAL_SPEED, USE_MOVEIT, KNOB_GRIPPER_OPEN):
        return False
    time.sleep(1.0)
    
    # 步骤3: 沿末端z轴插入
    print(f"\n步骤3: 沿末端z轴插入 {KNOB_INSERT_DEPTH*100:.1f}cm...")
    joints_insert = move_along_end_effector_z(joints_target, KNOB_INSERT_DEPTH, KNOB_INSERT_SPEED)
    if not joints_insert:
        return False
    time.sleep(0.5)
    
    # 步骤4: 夹爪闭合
    print(f"\n步骤4: 夹爪闭合到 {KNOB_GRIPPER_HOLD/1000:.1f}mm...")
    piper.GripperCtrl(KNOB_GRIPPER_HOLD, 1000, 0x01, 0)
    time.sleep(1.0)
    
    # 步骤5: 旋转joint6
    direction_sign = 1 if KNOB_ROTATION_DIRECTION == 'cw' else -1
    print(f"\n步骤5: 旋转 {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})...")
    joints_rotate = joints_insert.copy()
    joints_rotate[5] += direction_sign * KNOB_ROTATION_ANGLE * PI / 180
    if not control_arm(joints_rotate, KNOB_ROTATION_SPEED, USE_MOVEIT, KNOB_GRIPPER_HOLD):
        return False
    time.sleep(1.0)
    
    # 步骤6: 回零位
    print("\n步骤6: 回零位...")
    joints_zero = [0, 0, 0, 0, 0, 0]
    if not control_arm(joints_zero, FAST_SPEED, USE_MOVEIT):
        return False
    time.sleep(1.0)
    
    # 步骤7: 夹爪闭合
    print("\n步骤7: 夹爪闭合...")
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.5)
    
    print("="*70)
    print("✓✓✓ Knob 操作完成！✓✓✓")
    print("="*70)
    return True


# ========================================
# 主程序
# ========================================

def main():
    global piper, piper_arm, move_group
    
    print("="*70)
    print("按钮操作执行器 - 独立版本")
    print("="*70)
    print(f"\n📍 目标位置: ({TARGET_X:.3f}, {TARGET_Y:.3f}, {TARGET_Z:.3f})")
    print(f"🎯 动作类型: {ACTION_TYPE.upper()}")
    print(f"🔧 控制模式: {'MoveIt' if USE_MOVEIT and MOVEIT_AVAILABLE else 'SDK'}")
    
    # 显示动作特定参数
    if ACTION_TYPE == 'plugin':
        print(f"\n插拔连接器配置:")
        print(f"  插入深度: {PLUGIN_INSERT_DEPTH*100:.1f}cm")
        print(f"  夹持宽度: {PLUGIN_GRIPPER_HOLD/1000:.1f}mm")
    elif ACTION_TYPE == 'toggle':
        print(f"\n拨动开关配置:")
        print(f"  joint4旋转: {TOGGLE_JOINT4_ROTATE}°")
        print(f"  插入深度: {TOGGLE_INSERT_DEPTH*100:.1f}cm")
        print(f"  joint3拨动: {TOGGLE_JOINT3_ANGLE}° ({TOGGLE_DIRECTION})")
    elif ACTION_TYPE == 'push':
        print(f"\n按压按钮配置:")
        print(f"  按压深度: {PUSH_INSERT_DEPTH*100:.1f}cm")
        print(f"  保持时间: {PUSH_HOLD_TIME}秒")
    elif ACTION_TYPE == 'knob':
        print(f"\n旋转旋钮配置:")
        print(f"  插入深度: {KNOB_INSERT_DEPTH*100:.1f}cm")
        print(f"  旋转角度: {KNOB_ROTATION_ANGLE}° ({KNOB_ROTATION_DIRECTION})")
    
    print("="*70)
    
    # 初始化硬件
    print("\n初始化机械臂...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    
    for i in range(7):
        piper.EnableArm(i + 1)
        time.sleep(0.1)
    
    piper.GripperCtrl(70000, 1000, 0x01, 0)
    print("  ✓ 硬件初始化完成")
    
    # 初始化 ROS
    print("\n初始化ROS...")
    rospy.init_node('button_action_node', anonymous=True)
    
    # 初始化 MoveIt (如果需要)
    if USE_MOVEIT and MOVEIT_AVAILABLE:
        try:
            import os
            piper_ros_path = "/home/robot/button/V4.0/project2/piper_ros"
            src_path = os.path.join(piper_ros_path, 'src')
            current_path = os.environ.get('ROS_PACKAGE_PATH', '')
            if src_path not in current_path:
                os.environ['ROS_PACKAGE_PATH'] = f"{src_path}:{current_path}"
            
            moveit_commander.roscpp_initialize([])
            robot = moveit_commander.RobotCommander()
            move_group = moveit_commander.MoveGroupCommander("arm")
            move_group.set_planning_time(5.0)
            move_group.set_max_velocity_scaling_factor(1.0)
            move_group.set_max_acceleration_scaling_factor(1.0)
            print("  ✓ MoveIt初始化完成")
        except Exception as e:
            print(f"  ⚠️  MoveIt初始化失败: {e}")
            print("  将使用SDK模式")
    
    # 初始化 Piper Arm
    piper_arm = PiperArm()
    
    # 回零位
    print("\n回零位...")
    joints_zero = [0, 0, 0, 0, 0, 0]
    control_arm_sdk(joints_zero, 100)
    time.sleep(2)
    print("  ✓ 已回零位")
    
    print("\n="*70)
    print("开始执行动作...")
    print("="*70)
    
    # 执行对应动作
    action_functions = {
        'plugin': action_plugin,
        'toggle': action_toggle,
        'push': action_push,
        'knob': action_knob
    }
    
    if ACTION_TYPE not in action_functions:
        print(f"❌ 未知动作类型: {ACTION_TYPE}")
        print(f"   支持的类型: {list(action_functions.keys())}")
        return
    
    try:
        success = action_functions[ACTION_TYPE]()
        if success:
            print("\n✓ 动作执行成功！")
        else:
            print("\n❌ 动作执行失败")
    except Exception as e:
        print(f"\n❌ 动作执行异常: {e}")
        import traceback
        traceback.print_exc()
    
    # 清理资源
    if MOVEIT_AVAILABLE:
        moveit_commander.roscpp_shutdown()
    
    print("\n程序结束")


if __name__ == "__main__":
    main()