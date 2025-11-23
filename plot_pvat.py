#!/usr/bin/env python3
"""
PVAT (Position-Velocity-Acceleration-Time) Analysis Tool
Plots and analyzes MoveIt2 trajectory planning vs execution comparison
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import os
import sys


def plot_pvat_analysis(pvat_data, save_dir="trajectory"):
    """
    Plot PVAT analysis charts for gripper end-effector trajectory
    
    Args:
        pvat_data: Dictionary containing:
            - planned_points: MoveIt generated trajectory points (JointTrajectoryPoint list)
            - execution_records: Actual execution records [(time, joints, xyz, velocities), ...]
            - total_time: Total execution time (seconds)
        save_dir: Directory to save the chart
    
    Returns:
        Path to the saved chart file
    """
    
    # Import PiperArm (for FK computation)
    from piper_arm import PiperArm
    piper_arm = PiperArm()
    
    # Extract data
    planned_points = pvat_data['planned_points']
    execution_records = pvat_data['execution_records']
    total_time = pvat_data['total_time']
    
    print(f"\n{'='*80}")
    print(f"📊 Generating Gripper End-Effector PVAT Chart...")
    print(f"{'='*80}")
    print(f"  Planned points: {len(planned_points)}")
    print(f"  Execution records: {len(execution_records)}")
    print(f"  Total time: {total_time:.3f}s")
    
    # Extract planned end-effector XYZ trajectory (via FK)
    planned_xyz = []  # (N, 3)
    planned_times = []  # (N,)
    
    for point in planned_points:
        joints_rad = [point.positions[i] for i in range(6)]
        T = piper_arm.forward_kinematics(joints_rad)
        xyz = T[:3, 3]
        planned_xyz.append(xyz.copy())
        
        # Extract timestamp (ROS2 time format)
        if hasattr(point.time_from_start, 'sec'):
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        else:
            t = point.time_from_start  # Assume float
        planned_times.append(t)
    
    planned_xyz = np.array(planned_xyz)  # (N, 3)
    planned_times = np.array(planned_times)  # (N,)
    
    # Extract executed end-effector XYZ trajectory
    exec_xyz = []  # (M, 3)
    exec_times = []  # (M,)
    
    for record in execution_records:
        time, joints, xyz, velocities = record
        exec_xyz.append(xyz.copy())
        exec_times.append(time)
    
    exec_xyz = np.array(exec_xyz)  # (M, 3)
    exec_times = np.array(exec_times)  # (M,)
    
    print(f"  ✓ Extracted end-effector XYZ data:")
    print(f"    Planned: {planned_xyz.shape[0]} points, time range: {planned_times[0]:.3f}s ~ {planned_times[-1]:.3f}s")
    print(f"    Executed: {exec_xyz.shape[0]} points, time range: {exec_times[0]:.3f}s ~ {exec_times[-1]:.3f}s")
    
    # Compute end-effector velocities and accelerations (numerical differentiation)
    exec_velocities_xyz = np.zeros_like(exec_xyz)  # (M, 3)
    exec_accelerations_xyz = np.zeros_like(exec_xyz)  # (M, 3)
    
    for i in range(1, len(exec_xyz)):
        dt = exec_times[i] - exec_times[i-1]
        if dt > 0:
            exec_velocities_xyz[i] = (exec_xyz[i] - exec_xyz[i-1]) / dt
    
    for i in range(1, len(exec_velocities_xyz)):
        dt = exec_times[i] - exec_times[i-1]
        if dt > 0:
            exec_accelerations_xyz[i] = (exec_velocities_xyz[i] - exec_velocities_xyz[i-1]) / dt
    
    # Compute planned trajectory velocities and accelerations
    if len(planned_xyz) > 0:
        planned_velocities_xyz = np.zeros_like(planned_xyz)
        planned_accelerations_xyz = np.zeros_like(planned_xyz)
        
        for i in range(1, len(planned_xyz)):
            dt = planned_times[i] - planned_times[i-1]
            if dt > 0:
                planned_velocities_xyz[i] = (planned_xyz[i] - planned_xyz[i-1]) / dt
        
        for i in range(1, len(planned_velocities_xyz)):
            dt = planned_times[i] - planned_times[i-1]
            if dt > 0:
                planned_accelerations_xyz[i] = (planned_velocities_xyz[i] - planned_velocities_xyz[i-1]) / dt
    else:
        planned_velocities_xyz = np.array([])
        planned_accelerations_xyz = np.array([])
    
    # Create figure (2 rows x 3 columns)
    fig = plt.figure(figsize=(18, 10))
    gs = GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    axis_names = ['X', 'Y', 'Z']
    colors_planned = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green
    colors_exec = ['#d62728', '#9467bd', '#8c564b']     # Red, Purple, Brown
    
    # ========== Row 1: Position vs Time (X, Y, Z) ==========
    for axis_idx in range(3):
        ax = fig.add_subplot(gs[0, axis_idx])
        
        # Planned trajectory
        if len(planned_xyz) > 0:
            ax.plot(planned_times, planned_xyz[:, axis_idx], '--', 
                   color=colors_planned[axis_idx], linewidth=2.5, 
                   label=f'{axis_names[axis_idx]} Planned', alpha=0.7)
        
        # Executed trajectory
        ax.plot(exec_times, exec_xyz[:, axis_idx], '-', 
               color=colors_exec[axis_idx], linewidth=2, 
               label=f'{axis_names[axis_idx]} Executed', alpha=0.9)
        
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel(f'{axis_names[axis_idx]} Position (m)', fontsize=11)
        ax.set_title(f'Gripper {axis_names[axis_idx]} Position vs Time', fontsize=12, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3, linestyle='--')
    
    # ========== Row 2 Left: Velocity vs Time (X, Y, Z) ==========
    ax_vel = fig.add_subplot(gs[1, 0])
    for axis_idx in range(3):
        if len(planned_velocities_xyz) > 0:
            ax_vel.plot(planned_times, planned_velocities_xyz[:, axis_idx], '--', 
                       color=colors_planned[axis_idx], linewidth=2, 
                       label=f'{axis_names[axis_idx]} Plan', alpha=0.6)
        ax_vel.plot(exec_times, exec_velocities_xyz[:, axis_idx], '-', 
                   color=colors_exec[axis_idx], linewidth=1.5, 
                   label=f'{axis_names[axis_idx]} Exec', alpha=0.8)
    
    ax_vel.set_xlabel('Time (s)', fontsize=11)
    ax_vel.set_ylabel('Velocity (m/s)', fontsize=11)
    ax_vel.set_title('Gripper Velocity vs Time', fontsize=12, fontweight='bold')
    ax_vel.legend(fontsize=9, ncol=2)
    ax_vel.grid(True, alpha=0.3, linestyle='--')
    ax_vel.axhline(y=0, color='k', linestyle=':', linewidth=0.8)
    
    # ========== Row 2 Center: Acceleration vs Time (X, Y, Z) ==========
    ax_acc = fig.add_subplot(gs[1, 1])
    for axis_idx in range(3):
        if len(planned_accelerations_xyz) > 0:
            ax_acc.plot(planned_times, planned_accelerations_xyz[:, axis_idx], '--', 
                       color=colors_planned[axis_idx], linewidth=2, 
                       label=f'{axis_names[axis_idx]} Plan', alpha=0.6)
        ax_acc.plot(exec_times, exec_accelerations_xyz[:, axis_idx], '-', 
                   color=colors_exec[axis_idx], linewidth=1.5, 
                   label=f'{axis_names[axis_idx]} Exec', alpha=0.8)
    
    ax_acc.set_xlabel('Time (s)', fontsize=11)
    ax_acc.set_ylabel('Acceleration (m/s²)', fontsize=11)
    ax_acc.set_title('Gripper Acceleration vs Time', fontsize=12, fontweight='bold')
    ax_acc.legend(fontsize=9, ncol=2)
    ax_acc.grid(True, alpha=0.3, linestyle='--')
    ax_acc.axhline(y=0, color='k', linestyle=':', linewidth=0.8)
    
    # ========== Row 2 Right: 3D Trajectory Comparison ==========
    ax_3d = fig.add_subplot(gs[1, 2], projection='3d')
    
    # Planned trajectory
    if len(planned_xyz) > 0:
        ax_3d.plot(planned_xyz[:, 0], planned_xyz[:, 1], planned_xyz[:, 2], 
                  'b--', linewidth=2.5, label='Planned Path', alpha=0.6)
    
    # Executed trajectory
    ax_3d.plot(exec_xyz[:, 0], exec_xyz[:, 1], exec_xyz[:, 2], 
              'r-', linewidth=2, label='Executed Path', alpha=0.9)
    
    # Start and end markers
    ax_3d.scatter(exec_xyz[0, 0], exec_xyz[0, 1], exec_xyz[0, 2], 
                 c='green', s=150, marker='o', label='Start', edgecolors='k', linewidths=1.5)
    ax_3d.scatter(exec_xyz[-1, 0], exec_xyz[-1, 1], exec_xyz[-1, 2], 
                 c='orange', s=150, marker='s', label='End', edgecolors='k', linewidths=1.5)
    
    ax_3d.set_xlabel('X (m)', fontsize=10)
    ax_3d.set_ylabel('Y (m)', fontsize=10)
    ax_3d.set_zlabel('Z (m)', fontsize=10)
    ax_3d.set_title('Gripper 3D Trajectory', fontsize=12, fontweight='bold')
    ax_3d.legend(fontsize=9)
    ax_3d.grid(True, alpha=0.2)
    
    # Overall title
    fig.suptitle('Gripper End-Effector PVAT Analysis (Position-Velocity-Acceleration-Time)', 
                fontsize=14, fontweight='bold', y=0.98)
    
    # Compute and display statistics
    max_vel = np.max(np.sqrt(np.sum(exec_velocities_xyz**2, axis=1)))  # Velocity magnitude
    max_acc = np.max(np.sqrt(np.sum(exec_accelerations_xyz**2, axis=1)))  # Acceleration magnitude
    
    if len(planned_xyz) > 0:
        final_pos_error = np.linalg.norm(planned_xyz[-1] - exec_xyz[-1])
        interpolation_ratio = len(execution_records) / len(planned_points)
        stats_text = f"Planned: {len(planned_points)} pts | Executed: {len(execution_records)} cmds | Interp Ratio: {interpolation_ratio:.1f}x | "
        stats_text += f"Total Time: {total_time:.2f}s | Max Vel: {max_vel:.3f}m/s | Max Acc: {max_acc:.3f}m/s² | Final Error: {final_pos_error*1000:.2f}mm"
    else:
        stats_text = f"Executed: {len(execution_records)} cmds | Total Time: {total_time:.2f}s | "
        stats_text += f"Max Vel: {max_vel:.3f}m/s | Max Acc: {max_acc:.3f}m/s²"
    
    fig.text(0.5, 0.02, stats_text, ha='center', fontsize=10, 
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    
    # Save chart
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{save_dir}/gripper_pvat_analysis_{timestamp}.png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"\n  ✓ Gripper PVAT chart saved: {filename}")
    print(f"  📊 Statistics: {stats_text}")
    print(f"{'='*80}")
    
    plt.close()
    return filename


if __name__ == "__main__":
    # Load PVAT data from pickle file
    import pickle
    
    if len(sys.argv) > 1:
        pvat_file = sys.argv[1]
    else:
        pvat_file = "trajectory/pvat_data.pkl"
    
    if not os.path.exists(pvat_file):
        print(f"❌ PVAT data file not found: {pvat_file}")
        print(f"Usage: python3 plot_pvat.py [pvat_data.pkl]")
        sys.exit(1)
    
    print(f"📂 Loading PVAT data from: {pvat_file}")
    with open(pvat_file, 'rb') as f:
        pvat_data = pickle.load(f)
    
    # Plot PVAT analysis
    plot_pvat_analysis(pvat_data)
