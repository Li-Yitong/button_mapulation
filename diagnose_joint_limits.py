#!/usr/bin/env python3
"""
关节限位诊断工具
功能: 实时监控关节角度，检测是否接近限位边界
作者: GitHub Copilot
日期: 2025-12-04
"""

import sys
import os
import time
import math
import numpy as np

# 添加路径
sys.path.append('/home/robot/button/V4.0/project2/piper_sdk')
from piper_sdk import C_PiperInterface_V2

# ANSI颜色代码
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
RESET = '\033[0m'

# 关节限位（弧度）- 从XACRO提取
JOINT_LIMITS = {
    'joint1': {'lower': -2.618, 'upper': 2.168, 'name': 'J1 (Base)'},
    'joint2': {'lower': 0.0,    'upper': 3.14,  'name': 'J2 (Shoulder)'},
    'joint3': {'lower': -2.967, 'upper': 0.0,   'name': 'J3 (Elbow)'},
    'joint4': {'lower': -1.745, 'upper': 1.745, 'name': 'J4 (Wrist Roll)'},
    'joint5': {'lower': -1.22,  'upper': 1.22,  'name': 'J5 (Wrist Pitch)'},
    'joint6': {'lower': -2.094, 'upper': 2.094, 'name': 'J6 (Wrist Yaw)'},
}

# 警告阈值（距离限位边界的百分比）
WARNING_THRESHOLD = 0.15  # 15%以内警告
DANGER_THRESHOLD = 0.05   # 5%以内危险


def get_joint_status(joint_value, lower, upper):
    """
    计算关节状态
    返回: (距离下限%, 距离上限%, 状态颜色, 状态文字)
    """
    range_size = upper - lower
    dist_to_lower = joint_value - lower
    dist_to_upper = upper - joint_value
    
    dist_lower_pct = dist_to_lower / range_size
    dist_upper_pct = dist_to_upper / range_size
    
    # 判断状态
    if dist_lower_pct < DANGER_THRESHOLD or dist_upper_pct < DANGER_THRESHOLD:
        return dist_lower_pct, dist_upper_pct, RED, "⚠️ DANGER"
    elif dist_lower_pct < WARNING_THRESHOLD or dist_upper_pct < WARNING_THRESHOLD:
        return dist_lower_pct, dist_upper_pct, YELLOW, "⚠️ WARN"
    else:
        return dist_lower_pct, dist_upper_pct, GREEN, "✓ OK"


def format_progress_bar(percentage, length=20):
    """生成进度条"""
    filled = int(length * percentage)
    bar = '█' * filled + '░' * (length - filled)
    return bar


def main():
    print(f"{BLUE}{'='*80}{RESET}")
    print(f"{BLUE}🔍 关节限位实时诊断工具{RESET}")
    print(f"{BLUE}{'='*80}{RESET}")
    print(f"{CYAN}功能: 监控关节角度，检测是否接近限位边界{RESET}")
    print(f"{CYAN}警告阈值: {WARNING_THRESHOLD*100:.0f}% (黄色) | 危险阈值: {DANGER_THRESHOLD*100:.0f}% (红色){RESET}")
    print(f"{CYAN}按 Ctrl+C 退出{RESET}")
    print(f"{BLUE}{'='*80}{RESET}\n")
    
    # 初始化机械臂
    print(f"{CYAN}正在连接机械臂...{RESET}")
    try:
        piper = C_PiperInterface_V2("can0")
        piper.ConnectPort()
        time.sleep(0.5)
        print(f"{GREEN}✓ 机械臂已连接{RESET}\n")
    except Exception as e:
        print(f"{RED}❌ 连接失败: {e}{RESET}")
        return
    
    try:
        while True:
            # 读取关节角度
            msg = piper.GetArmJointMsgs()
            joints_deg = [
                msg.joint_state.joint_1 * 1e-3,
                msg.joint_state.joint_2 * 1e-3,
                msg.joint_state.joint_3 * 1e-3,
                msg.joint_state.joint_4 * 1e-3,
                msg.joint_state.joint_5 * 1e-3,
                msg.joint_state.joint_6 * 1e-3,
            ]
            joints_rad = [math.radians(deg) for deg in joints_deg]
            
            # 清屏（保留标题）
            print("\033[H\033[J", end="")  # 清屏
            
            # 打印标题
            print(f"{BLUE}{'='*80}{RESET}")
            print(f"{BLUE}🔍 关节限位实时诊断 - {time.strftime('%H:%M:%S')}{RESET}")
            print(f"{BLUE}{'='*80}{RESET}\n")
            
            # 统计状态
            total_warnings = 0
            total_dangers = 0
            
            # 逐个关节显示
            for i, (joint_name, limits) in enumerate(JOINT_LIMITS.items()):
                joint_rad = joints_rad[i]
                joint_deg = joints_deg[i]
                
                lower = limits['lower']
                upper = limits['upper']
                name = limits['name']
                
                dist_lower_pct, dist_upper_pct, color, status = get_joint_status(joint_rad, lower, upper)
                
                # 统计
                if status == "⚠️ DANGER":
                    total_dangers += 1
                elif status == "⚠️ WARN":
                    total_warnings += 1
                
                # 进度条（当前位置在范围内的百分比）
                range_size = upper - lower
                position_pct = (joint_rad - lower) / range_size
                progress_bar = format_progress_bar(position_pct, length=30)
                
                # 打印关节信息
                print(f"{color}{name:20s}{RESET} | {color}{status}{RESET}")
                print(f"  当前角度: {joint_deg:+7.2f}° ({joint_rad:+.3f} rad)")
                print(f"  限位范围: {math.degrees(lower):+7.2f}° ~ {math.degrees(upper):+7.2f}° "
                      f"({lower:+.3f} ~ {upper:+.3f} rad)")
                print(f"  位置: [{progress_bar}] {position_pct*100:.1f}%")
                
                # 显示距离限位的百分比
                if dist_lower_pct < dist_upper_pct:
                    print(f"  {color}⚠️  距离下限: {dist_lower_pct*100:.1f}% ({abs(joint_rad - lower):.3f} rad){RESET}")
                else:
                    print(f"  {color}⚠️  距离上限: {dist_upper_pct*100:.1f}% ({abs(upper - joint_rad):.3f} rad){RESET}")
                
                print()
            
            # 总体状态
            print(f"{BLUE}{'='*80}{RESET}")
            if total_dangers > 0:
                print(f"{RED}❌ 总体状态: 危险 - {total_dangers} 个关节接近极限位置！{RESET}")
            elif total_warnings > 0:
                print(f"{YELLOW}⚠️  总体状态: 警告 - {total_warnings} 个关节接近限位边界{RESET}")
            else:
                print(f"{GREEN}✓ 总体状态: 正常 - 所有关节在安全范围内{RESET}")
            print(f"{BLUE}{'='*80}{RESET}\n")
            
            print(f"{CYAN}💡 提示: 手动移动关节可实时查看限位状态{RESET}")
            print(f"{CYAN}按 Ctrl+C 退出{RESET}")
            
            time.sleep(0.2)  # 更新频率 5Hz
    
    except KeyboardInterrupt:
        print(f"\n\n{GREEN}✓ 用户退出{RESET}")
    except Exception as e:
        print(f"\n{RED}❌ 错误: {e}{RESET}")


if __name__ == '__main__':
    main()
