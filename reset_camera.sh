#!/bin/bash

# RealSense 相机重置脚本
# 用于解决 "Device or resource busy" 错误

echo "======================================================"
echo "RealSense 相机重置工具"
echo "======================================================"

# 1. 关闭所有使用相机的进程
echo "[1/3] 查找并关闭占用相机的进程..."

REALSENSE_PIDS=$(ps aux | grep -E "realsense|pyrealsense2" | grep -v grep | awk '{print $2}')
if [ ! -z "$REALSENSE_PIDS" ]; then
    echo "  找到以下进程:"
    ps aux | grep -E "realsense|pyrealsense2" | grep -v grep
    echo ""
    echo "  正在关闭这些进程..."
    echo "$REALSENSE_PIDS" | xargs kill -9 2>/dev/null
    sleep 1
    echo "  ✓ 进程已关闭"
else
    echo "  ✓ 没有发现占用相机的进程"
fi

# 2. 检查相机状态
echo ""
echo "[2/3] 检查相机设备状态..."

if command -v rs-enumerate-devices &>/dev/null; then
    echo "  相机设备列表:"
    rs-enumerate-devices 2>/dev/null || echo "  ⚠️  无法枚举设备"
    
    # 检查是否锁定
    if rs-enumerate-devices 2>/dev/null | grep -q "Camera Locked.*YES"; then
        echo ""
        echo "  ⚠️  检测到相机被锁定，需要重置驱动"
        
        # 3. 重置相机驱动
        echo ""
        echo "[3/3] 重置相机驱动..."
        echo "  (可能需要输入密码)"
        
        sudo rmmod uvcvideo 2>/dev/null
        sudo modprobe uvcvideo 2>/dev/null
        sleep 2
        
        echo "  ✓ 驱动已重置"
    else
        echo "  ✓ 相机未锁定"
    fi
else
    echo "  ⚠️  rs-enumerate-devices 命令未找到"
    echo "  安装方法: sudo apt install librealsense2-utils"
fi

# 4. 最终检查
echo ""
echo "======================================================"
echo "重置完成！"
echo "======================================================"
echo ""
echo "现在可以重新启动相机程序:"
echo "  python3 realsense_yolo_pc_roi.py"
echo ""
echo "或重新运行启动脚本:"
echo "  ./start_all_moveit.sh"
echo ""
