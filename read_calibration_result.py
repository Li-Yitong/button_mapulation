#!/usr/bin/env python3
"""
读取easy_handeye2标定结果并生成piper_arm.py配置代码

使用方法：
1. 完成easy_handeye2标定
2. 运行此脚本：python3 read_calibration_result.py
3. 复制输出的代码到piper_arm.py中替换对应的标定参数
"""
import yaml
import os
from pathlib import Path


def read_calibration_result(namespace='piper_realsense_handeye'):
    """
    读取easy_handeye2标定结果
    
    Args:
        namespace: 标定名称（与标定时的name参数一致，不是ROS命名空间）
    
    Returns:
        dict: 包含translation和rotation的字典
    """
    # 标定结果路径
    calib_file = Path.home() / '.ros' / 'easy_handeye' / f'{namespace}.yaml'
    
    if not calib_file.exists():
        print(f"❌ 标定文件不存在: {calib_file}")
        print(f"   请确保已完成标定并保存结果")
        return None
    
    # 读取YAML文件
    with open(calib_file, 'r') as f:
        data = yaml.safe_load(f)
    
    # 提取标定结果
    calib = data.get('calibration', {})
    translation = calib.get('translation', {})
    rotation = calib.get('rotation', {})
    
    return {
        'translation': [translation['x'], translation['y'], translation['z']],
        'rotation': [rotation['x'], rotation['y'], rotation['z'], rotation['w']]  # x,y,z,w
    }


def generate_piper_arm_config(calib_result):
    """
    生成piper_arm.py配置代码（camera_color_optical_frame版本）
    
    Args:
        calib_result: 标定结果字典
    
    Returns:
        str: 配置代码字符串
    """
    if not calib_result:
        return None
    
    t = calib_result['translation']
    q = calib_result['rotation']  # [x, y, z, w]
    
    # 转换为 [w, x, y, z] 格式（piper_arm.py使用的格式）
    q_wxyz = [q[3], q[0], q[1], q[2]]
    
    config_code = f"""
# ========================================
# 手眼标定参数 (link6 → camera_color_optical_frame)
# 由 easy_handeye2 标定生成
# ========================================

# ✅ 当前使用的标定结果
self.link6_q_camera = np.array([{q_wxyz[0]:.16f}, {q_wxyz[1]:.16f}, {q_wxyz[2]:.16f}, {q_wxyz[3]:.16f}])  # [w, x, y, z]
self.link6_t_camera = [{t[0]:.16f}, {t[1]:.16f}, {t[2]:.16f}]  # [x, y, z] 单位：米
"""
    
    return config_code


def print_calibration_result(calib_result):
    """
    打印标定结果
    
    Args:
        calib_result: 标定结果字典
    """
    t = calib_result['translation']
    q = calib_result['rotation']
    
    print("="*70)
    print("Easy_handeye2 标定结果 (link6 → camera_color_optical_frame)")
    print("="*70)
    print(f"平移向量 (link6 → camera_color_optical_frame):")
    print(f"  X: {t[0]:+.6f} m")
    print(f"  Y: {t[1]:+.6f} m")
    print(f"  Z: {t[2]:+.6f} m")
    print()
    print(f"旋转四元数 (x, y, z, w):")
    print(f"  x: {q[0]:+.6f}")
    print(f"  y: {q[1]:+.6f}")
    print(f"  z: {q[2]:+.6f}")
    print(f"  w: {q[3]:+.6f}")
    print("="*70)
    print()
    print("请将以下代码复制到 piper_arm.py 的 __init__ 方法中:")
    print("="*70)
    
    config_code = generate_piper_arm_config(calib_result)
    print(config_code)
    print("="*70)
    print()
    print("⚠️  注意：")
    print("   - easy_handeye2输出的四元数顺序是 (x, y, z, w)")
    print("   - piper_arm.py使用的顺序是 (w, x, y, z)")
    print("   - 上述代码已自动转换顺序")
    print()


def main():
    """主函数"""
    print("\n🔍 正在读取easy_handeye2标定结果...\n")
    
    # 读取标定结果
    calib_result = read_calibration_result()
    
    if calib_result is None:
        print("\n❌ 读取失败！\n")
        print("📋 故障排查步骤:")
        print("   1. 确认已完成easy_handeye2标定")
        print("   2. 确认已点击'Save Calibration'按钮")
        print("   3. 检查文件是否存在:")
        print(f"      ls -la {Path.home() / '.ros' / 'easy_handeye' / 'piper_realsense_handeyeye.yaml'}")
        return
    
    # 生成配置代码
    generate_piper_arm_config(calib_result)
    
    print("✅ 完成！\n")


if __name__ == '__main__':
    main()
