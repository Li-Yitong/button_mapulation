#!/usr/bin/env python3
"""
夹爪姿态约束策略 - 使用指南
===============================

本文档说明如何让夹爪以不同姿态接近面板上的按钮

前提条件：
1. 已在HOME位姿观察AprilTag
2. 已记录面板参考姿态（APRILTAG_REFERENCE_POSE_BASE不为None）
3. 已获取按钮在基座系的位置 [TARGET_X, TARGET_Y, TARGET_Z]

"""

import numpy as np
from button_actions import (
    create_aligned_target_pose,
    get_gripper_approach_rotation,
    PI
)

# ========================================
# 示例1：按压按钮（Push/Plugin）
# ========================================
def example_press_button():
    """垂直按压 - 最常用的按压姿态"""
    
    # 按钮位置（来自视觉检测）
    button_xyz = np.array([0.4, 0.0, 0.2])
    
    # 使用垂直接近模式
    R_offset = get_gripper_approach_rotation('perpendicular')
    target_pose = create_aligned_target_pose(button_xyz, R_offset)
    
    # 使用target_pose进行运动规划...
    print("垂直按压姿态已构建")
    print("夹爪将垂直于面板接近按钮")


# ========================================
# 示例2：侧面滑动（Toggle）
# ========================================
def example_side_slide():
    """平行滑动 - 用于拨动开关"""
    
    button_xyz = np.array([0.4, 0.0, 0.2])
    
    # 使用平行模式（默认）
    R_offset = get_gripper_approach_rotation('parallel')
    # 或者直接传None
    target_pose = create_aligned_target_pose(button_xyz, None)
    
    print("平行姿态已构建")
    print("夹爪将与面板保持平行")


# ========================================
# 示例3：两阶段接近（安全策略）
# ========================================
def example_two_stage_approach():
    """先倾斜接近，再垂直按压"""
    
    button_xyz = np.array([0.4, 0.0, 0.2])
    
    # 阶段1：预备位置（倾斜15°，后退5cm）
    prep_xyz = button_xyz.copy()
    prep_xyz[0] -= 0.05  # 沿X轴后退5cm
    
    R_prep = get_gripper_approach_rotation('tilted_15')
    prep_pose = create_aligned_target_pose(prep_xyz, R_prep)
    
    print("阶段1: 预备位置（倾斜15°）")
    # 执行运动到prep_pose...
    
    # 阶段2：最终按压（垂直）
    R_final = get_gripper_approach_rotation('perpendicular')
    final_pose = create_aligned_target_pose(button_xyz, R_final)
    
    print("阶段2: 最终按压（垂直）")
    # 执行运动到final_pose...


# ========================================
# 示例4：旋转按钮（Knob）
# ========================================
def example_rotate_knob():
    """旋转操作 - 需要特殊姿态"""
    
    knob_xyz = np.array([0.4, 0.0, 0.2])
    
    # 方法1：垂直接近
    R_offset = get_gripper_approach_rotation('perpendicular')
    target_pose = create_aligned_target_pose(knob_xyz, R_offset)
    
    # 方法2：自定义旋转（例如绕Z轴旋转45°）
    from button_actions import euler_to_rotation_matrix
    R_custom = euler_to_rotation_matrix(0, 0, PI/4)  # 绕Z轴45°
    target_pose = create_aligned_target_pose(knob_xyz, R_custom)
    
    print("旋钮操作姿态已构建")


# ========================================
# 完整的动作流程示例
# ========================================
def complete_action_example():
    """完整的按压动作流程"""
    
    from button_actions import (
        piper, piper_arm,
        get_current_joints,
        control_arm,
        APRILTAG_REFERENCE_POSE_BASE
    )
    
    # 1. 检查前提条件
    if APRILTAG_REFERENCE_POSE_BASE is None:
        print("❌ 错误：未设置面板参考姿态")
        return False
    
    # 2. 获取按钮位置（假设已从视觉检测获取）
    button_xyz = np.array([0.4, 0.0, 0.2])
    
    # 3. 构建垂直按压姿态
    R_offset = get_gripper_approach_rotation('perpendicular')
    target_pose = create_aligned_target_pose(button_xyz, R_offset)
    
    # 4. 求解逆运动学
    ik_result = piper_arm.inverse_kinematics_search(
        target_pose,
        seed_joints=get_current_joints(),
        max_attempts=100,
        max_time=3.0
    )
    
    if not ik_result.success:
        print("❌ IK求解失败")
        return False
    
    # 5. 执行运动
    print("✓ 移动到按压位置...")
    success = control_arm(
        ik_result.joint_positions,
        speed=50,
        use_moveit=True
    )
    
    if success:
        print("✓✓✓ 按压动作完成！")
    
    return success


# ========================================
# 姿态模式对比表
# ========================================
APPROACH_MODES = {
    'parallel': {
        'name': '平行于面板',
        'rotation': None,
        'use_case': 'Toggle/Knob侧面操作',
        'description': '夹爪与面板保持平行，适合滑动、旋转'
    },
    'perpendicular': {
        'name': '垂直于面板',
        'rotation': 'euler(0, π, 0)',
        'use_case': 'Push/Plugin按压操作 ⭐推荐',
        'description': '夹爪垂直于面板，直接按压'
    },
    'tilted_15': {
        'name': '倾斜15°',
        'rotation': 'euler(0, 165°, 0)',
        'use_case': '预备姿态',
        'description': '轻微倾斜，用于安全接近'
    },
    'tilted_30': {
        'name': '倾斜30°',
        'rotation': 'euler(0, 150°, 0)',
        'use_case': '预备姿态（大角度）',
        'description': '较大倾斜，用于避障接近'
    }
}


def print_mode_comparison():
    """打印所有姿态模式对比"""
    print("\n" + "="*70)
    print("姿态模式对比表")
    print("="*70)
    
    for mode, info in APPROACH_MODES.items():
        print(f"\n模式: {mode}")
        print(f"  名称: {info['name']}")
        print(f"  旋转: {info['rotation']}")
        print(f"  适用: {info['use_case']}")
        print(f"  说明: {info['description']}")
    
    print("\n" + "="*70)


if __name__ == "__main__":
    print("="*70)
    print("夹爪姿态约束策略 - 使用指南")
    print("="*70)
    
    print("\n运行示例演示:")
    print_mode_comparison()
    
    print("\n提示:")
    print("1. 在实际代码中导入: from button_actions import ...")
    print("2. 修改现有动作函数: 添加 get_gripper_approach_rotation() 调用")
    print("3. 测试不同模式: 观察夹爪姿态是否符合预期")
