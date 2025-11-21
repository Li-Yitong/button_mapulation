#!/usr/bin/env python3
"""
测试6D位姿发布器
用于测试不同的目标位姿是否可达
"""

import rospy
from std_msgs.msg import Float64MultiArray
import time

def publish_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """
    发布6D目标位姿
    
    Args:
        x, y, z: 位置 (m)
        roll, pitch, yaw: 欧拉角 (rad)
    """
    rospy.init_node('test_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/target_6d_pose', Float64MultiArray, queue_size=1)
    
    # 等待连接建立
    rospy.sleep(1)
    
    msg = Float64MultiArray()
    msg.data = [x, y, z, roll, pitch, yaw]
    
    print("="*70)
    print("发布目标位姿:")
    print(f"  位置: ({x:.3f}, {y:.3f}, {z:.3f}) m")
    print(f"  姿态: Roll={roll:.3f}, Pitch={pitch:.3f}, Yaw={yaw:.3f} rad")
    print(f"        Roll={roll*57.3:.1f}°, Pitch={pitch*57.3:.1f}°, Yaw={yaw*57.3:.1f}°")
    print("="*70)
    
    pub.publish(msg)
    print("✓ 位姿已发布到 /target_6d_pose")
    
    rospy.sleep(1)


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 4:
        print("用法: python3 test_pose_publisher.py <x> <y> <z> [roll] [pitch] [yaw]")
        print("")
        print("示例:")
        print("  # 只指定位置")
        print("  python3 test_pose_publisher.py 0.3 0.0 0.2")
        print("")
        print("  # 指定位置和姿态")
        print("  python3 test_pose_publisher.py 0.3 0.0 0.2 0.0 0.0 0.0")
        print("")
        print("推荐的测试位姿:")
        print("  1. 正前方近距离: 0.25 0.0 0.20")
        print("  2. 正前方中距离: 0.30 0.0 0.25")
        print("  3. 正前方高位置: 0.25 0.0 0.30")
        print("  4. 左前方: 0.25 0.10 0.25")
        print("  5. 右前方: 0.25 -0.10 0.25")
        sys.exit(1)
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    
    roll = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    
    try:
        publish_pose(x, y, z, roll, pitch, yaw)
    except rospy.ROSInterruptException:
        pass
