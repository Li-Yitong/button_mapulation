#!/usr/bin/env python3
"""
坐标转换诊断脚本
用于验证 camera → base 坐标转换的每个环节
"""
import numpy as np
from piper_sdk import C_PiperInterface_V2
from piper_arm import PiperArm
from utils.utils_math import quaternion_to_rotation_matrix
from utils.utils_piper import enable_fun
import math

PI = math.pi

def main():
    print("\n" + "="*70)
    print("坐标转换诊断工具")
    print("="*70)
    
    # 初始化硬件
    print("\n[1/3] 初始化 Piper SDK...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    print("  ✓ Piper SDK 初始化成功")
    
    print("\n[2/3] 初始化 PiperArm...")
    piper_arm = PiperArm()
    print("  ✓ PiperArm 初始化成功")
    
    # 获取当前关节角度
    print("\n[3/3] 获取当前关节角度...")
    msg = piper.GetArmJointMsgs()
    current_joints = [
        msg.joint_state.joint_1 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_2 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_3 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_4 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_5 * 1e-3 * PI / 180.0,
        msg.joint_state.joint_6 * 1e-3 * PI / 180.0,
    ]
    print(f"  关节角度 (度): {np.array(current_joints) * 180 / PI}")
    
    # 显示手眼标定参数
    print("\n" + "="*70)
    print("手眼标定参数")
    print("="*70)
    print(f"link6_t_camera = {piper_arm.link6_t_camera}")
    print(f"link6_q_camera = {piper_arm.link6_q_camera}  # (w, x, y, z)")
    
    # 转换为旋转矩阵
    R_link6_camera = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    print(f"\nlink6到camera的旋转矩阵 R_link6_camera:")
    print(R_link6_camera)
    
    # 构建变换矩阵
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = R_link6_camera
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera
    print(f"\nlink6到camera的变换矩阵 link6_T_cam:")
    print(link6_T_cam)
    
    # 获取基座到link6的变换
    base_T_link6 = piper_arm.forward_kinematics(current_joints)
    print(f"\n基座到link6的变换矩阵 base_T_link6:")
    print(base_T_link6)
    print(f"末端位置 (基座系): ({base_T_link6[0, 3]:.4f}, {base_T_link6[1, 3]:.4f}, {base_T_link6[2, 3]:.4f})")
    
    # 完整变换链
    base_T_camera = base_T_link6 @ link6_T_cam
    print(f"\n基座到camera的完整变换矩阵 base_T_camera:")
    print(base_T_camera)
    print(f"相机位置 (基座系): ({base_T_camera[0, 3]:.4f}, {base_T_camera[1, 3]:.4f}, {base_T_camera[2, 3]:.4f})")
    
    # 测试样例：从用户日志中的按钮位置
    print("\n" + "="*70)
    print("测试案例：用户报告的按钮位置")
    print("="*70)
    button_camera = np.array([0.4556, -0.0110, 1.2350])  # 相机坐标系
    print(f"按钮位置 (相机系): ({button_camera[0]:.4f}, {button_camera[1]:.4f}, {button_camera[2]:.4f})")
    
    button_cam_h = np.array([button_camera[0], button_camera[1], button_camera[2], 1.0])
    button_base = base_T_camera @ button_cam_h
    
    print(f"按钮位置 (基座系): ({button_base[0]:.4f}, {button_base[1]:.4f}, {button_base[2]:.4f})")
    print(f"\n注意：从日志看期望目标是 (1.238, -0.422, 0.117)")
    print(f"      但实际到达的是  (0.573, -0.193, 0.198)")
    print(f"      计算结果是:      ({button_base[0]:.4f}, {button_base[1]:.4f}, {button_base[2]:.4f})")
    
    # 分析坐标系差异
    print("\n" + "="*70)
    print("坐标系定义检查")
    print("="*70)
    print("ROS1 camera_color_optical_frame (光学坐标系):")
    print("  X轴: 图像右方向")
    print("  Y轴: 图像下方向")
    print("  Z轴: 相机前方 (深度方向)")
    print("")
    print("ROS2 realsense2_camera 默认发布:")
    print("  /camera/camera/color/image_raw")
    print("  frame_id: camera_color_optical_frame")
    print("")
    print("PiperArm link6 (末端法兰坐标系):")
    print("  需要确认Z轴方向是否与夹爪轴线一致")
    
    # 提供调试建议
    print("\n" + "="*70)
    print("调试建议")
    print("="*70)
    print("1. 确认相机发布的坐标系与手眼标定时使用的坐标系一致")
    print("2. 检查 button_camera 是否使用了正确的轴向定义")
    print("3. 如果差异巨大 (>50cm)，可能需要:")
    print("   - 重新进行手眼标定")
    print("   - 或添加坐标系转换矩阵 (optical → standard)")
    print("4. 可以尝试添加以下转换:")
    print("   optical_T_standard = np.array([[0, 0, 1, 0],")
    print("                                   [-1, 0, 0, 0],")
    print("                                   [0, -1, 0, 0],")
    print("                                   [0, 0, 0, 1]])")
    print("   button_standard = optical_T_standard @ button_optical_h")
    
    print("\n" + "="*70)
    print("诊断完成")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
