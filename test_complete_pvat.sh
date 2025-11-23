#!/bin/bash
# Test script: Complete PVAT trajectory recording from zero to zero
# This script demonstrates the complete action sequence with accumulated trajectory data

echo "========================================="
echo "Testing Complete PVAT Trajectory Recording"
echo "========================================="
echo ""
echo "This test will:"
echo "1. Clear all trajectory records"
echo "2. Execute complete PUSH action (zero → target → press → zero)"
echo "3. Accumulate ALL planning and execution data"
echo "4. Generate complete PVAT charts showing the entire sequence"
echo ""
echo "Expected behavior:"
echo "- Timeline starts at 0.0s"
echo "- All planning steps are accumulated (not overwritten)"
echo "- PVAT chart shows complete trajectory from start to finish"
echo ""
echo "========================================="
echo ""

# Kill any existing processes
pkill -f button_actions.py
sleep 1

# Start ROS2 environment
echo "Setting up ROS2 environment..."
source /opt/ros/foxy/setup.bash
source ~/ros2_foxy_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# Start MoveIt2
echo "Starting MoveIt2 (in background)..."
ros2 launch piper_with_gripper_moveit piper_with_gripper_moveit.launch.py &
MOVEIT_PID=$!
sleep 5

# Start TF publisher
echo "Starting TF publisher..."
python3 piper_tf_publisher_ros2.py &
TF_PID=$!
sleep 2

# Run button actions
echo ""
echo "========================================="
echo "Running button action (PUSH)..."
echo "========================================="
python3 button_actions.py

# Wait for completion
sleep 2

# Check results
echo ""
echo "========================================="
echo "Checking results..."
echo "========================================="

if [ -d "trajectory" ]; then
    echo "✓ Trajectory directory exists"
    
    # Find latest PVAT files
    LATEST_PVAT_DATA=$(ls -t trajectory/pvat_data_*.pkl 2>/dev/null | head -1)
    LATEST_PVAT_CHART=$(ls -t trajectory/gripper_pvat_analysis_*.png 2>/dev/null | head -1)
    
    if [ -n "$LATEST_PVAT_DATA" ]; then
        echo "✓ PVAT data saved: $LATEST_PVAT_DATA"
        
        # Analyze PVAT data
        python3 << EOF
import pickle
import sys

try:
    with open('$LATEST_PVAT_DATA', 'rb') as f:
        data = pickle.load(f)
    
    print("\n📊 PVAT Data Summary:")
    print(f"  Planned points: {len(data['planned_points'])}")
    print(f"  Execution records: {len(data['execution_records'])}")
    print(f"  Total time: {data['total_time']:.3f}s")
    
    # Check timeline
    if len(data['execution_records']) > 0:
        first_time = data['execution_records'][0][0]
        last_time = data['execution_records'][-1][0]
        print(f"  Timeline: {first_time:.3f}s ~ {last_time:.3f}s")
        
        if first_time < 0.1:
            print("  ✓ Timeline starts near zero (correct)")
        else:
            print("  ⚠️  Timeline does not start at zero")
    
except Exception as e:
    print(f"❌ Error analyzing PVAT data: {e}")
    sys.exit(1)
EOF
    else
        echo "❌ No PVAT data file found"
    fi
    
    if [ -n "$LATEST_PVAT_CHART" ]; then
        echo "✓ PVAT chart saved: $LATEST_PVAT_CHART"
    else
        echo "❌ No PVAT chart file found"
    fi
else
    echo "❌ Trajectory directory not found"
fi

# Cleanup
echo ""
echo "Cleaning up..."
kill $MOVEIT_PID $TF_PID 2>/dev/null
pkill -f piper_tf_publisher_ros2
pkill -f moveit

echo ""
echo "========================================="
echo "Test complete!"
echo "========================================="
