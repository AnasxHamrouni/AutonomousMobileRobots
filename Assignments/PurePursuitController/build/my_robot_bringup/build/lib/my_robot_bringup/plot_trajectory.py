#!/usr/bin/env python3

"""
Visualization script for robot trajectory data
Generates four required plots:
1. XY Trajectory (actual vs expected)
2. X vs Time
3. Y vs Time
4. Heading (Yaw) vs Time
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import sys
from pathlib import Path


def load_trajectory_data(file_path):
    """Load trajectory data from JSON file."""
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data
    except FileNotFoundError:
        print(f"Error: Data file not found at {file_path}")
        print("Make sure the waypoint_controller has been run and saved data.")
        return None


def plot_trajectories(data):
    """Create visualization plots."""
    time = np.array(data['time'])
    x = np.array(data['x'])
    y = np.array(data['y'])
    yaw = np.array(data['yaw'])
    waypoints = np.array(data['waypoints'])
    
    # Create a figure with 2x2 subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Differential Drive Robot Control - Trajectory Analysis', fontsize=16, fontweight='bold')
    
    # Plot 1: XY Trajectory
    ax1 = axes[0, 0]
    ax1.plot(x, y, 'b-', linewidth=2, label='Actual Trajectory')
    ax1.plot(waypoints[:, 0], waypoints[:, 1], 'ro', markersize=10, label='Waypoints')
    ax1.plot(x[0], y[0], 'gs', markersize=12, label='Start')
    ax1.plot(x[-1], y[-1], 'r*', markersize=15, label='End')
    
    # Connect waypoints with dashed line
    for i in range(len(waypoints) - 1):
        ax1.plot(waypoints[i:i+2, 0], waypoints[i:i+2, 1], 'r--', alpha=0.3, linewidth=1)
    
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X (meters)', fontsize=11)
    ax1.set_ylabel('Y (meters)', fontsize=11)
    ax1.set_title('XY Trajectory: Actual vs Expected', fontsize=12, fontweight='bold')
    ax1.legend(loc='best')
    ax1.axis('equal')
    
    # Plot 2: X vs Time
    ax2 = axes[0, 1]
    ax2.plot(time, x, 'b-', linewidth=2, label='Actual X')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel('Time (seconds)', fontsize=11)
    ax2.set_ylabel('X (meters)', fontsize=11)
    ax2.set_title('X Position vs Time', fontsize=12, fontweight='bold')
    ax2.legend(loc='best')
    
    # Plot 3: Y vs Time
    ax3 = axes[1, 0]
    ax3.plot(time, y, 'g-', linewidth=2, label='Actual Y')
    ax3.grid(True, alpha=0.3)
    ax3.set_xlabel('Time (seconds)', fontsize=11)
    ax3.set_ylabel('Y (meters)', fontsize=11)
    ax3.set_title('Y Position vs Time', fontsize=12, fontweight='bold')
    ax3.legend(loc='best')
    
    # Plot 4: Heading (Yaw) vs Time
    ax4 = axes[1, 1]
    yaw_degrees = np.degrees(yaw)
    ax4.plot(time, yaw_degrees, 'm-', linewidth=2, label='Heading (Yaw)')
    ax4.grid(True, alpha=0.3)
    ax4.set_xlabel('Time (seconds)', fontsize=11)
    ax4.set_ylabel('Heading (degrees)', fontsize=11)
    ax4.set_title('Robot Heading vs Time', fontsize=12, fontweight='bold')
    ax4.legend(loc='best')
    
    plt.tight_layout()
    return fig, (ax1, ax2, ax3, ax4)


def print_statistics(data):
    """Print trajectory statistics."""
    x = np.array(data['x'])
    y = np.array(data['y'])
    time = np.array(data['time'])
    waypoints = np.array(data['waypoints'])
    
    print("\n" + "="*60)
    print("TRAJECTORY STATISTICS")
    print("="*60)
    
    # Calculate distances to each waypoint at end
    final_x, final_y = x[-1], y[-1]
    print(f"\nFinal Robot Position: ({final_x:.3f}, {final_y:.3f})")
    
    # Total distance traveled
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    total_distance = np.sum(distances)
    print(f"Total Distance Traveled: {total_distance:.3f} meters")
    
    # Time taken
    total_time = time[-1]
    print(f"Total Time: {total_time:.2f} seconds")
    
    # Average speed
    avg_speed = total_distance / total_time if total_time > 0 else 0
    print(f"Average Speed: {avg_speed:.3f} m/s")
    
    # Waypoint analysis
    print(f"\nWaypoint Analysis ({len(waypoints)} waypoints):")
    print(f"{'Waypoint':<12} {'Target':<20} {'Distance':<15}")
    print("-" * 47)
    
    for i, wp in enumerate(waypoints):
        dist_to_wp = np.sqrt((final_x - wp[0])**2 + (final_y - wp[1])**2)
        print(f"T{i+1:<10} ({wp[0]:.1f}, {wp[1]:.1f})       {dist_to_wp:.3f} m")
    
    print("="*60 + "\n")


def main():
    """Main function to generate and display plots."""
    
    data_file = '/tmp/robot_trajectory.json'
    
    # Try alternative paths if default doesn't exist
    if not Path(data_file).exists():
        home_path = Path.home() / '.ros' / 'robot_trajectory.json'
        if not home_path.exists():
            print("Usage: python plot_trajectory.py [data_file_path]")
            if len(sys.argv) > 1:
                data_file = sys.argv[1]
            else:
                print(f"\nDefault location checked: {data_file}")
                print("Please run the waypoint_controller node first to generate data.")
                sys.exit(1)
        else:
            data_file = str(home_path)
    
    # Load data
    data = load_trajectory_data(data_file)
    if data is None:
        sys.exit(1)
    
    # Check if data is not empty
    if not data.get('x') or len(data['x']) < 2:
        print("Error: Not enough data points. Make sure the robot moved.")
        sys.exit(1)
    
    # Print statistics
    print_statistics(data)
    
    # Plot trajectories
    fig, axes = plot_trajectories(data)
    
    # Save figure
    output_path = '/tmp/robot_trajectory_plots.png'
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Plots saved to: {output_path}")
    
    # Also save as PDF for report
    pdf_path = '/tmp/robot_trajectory_plots.pdf'
    fig.savefig(pdf_path, bbox_inches='tight')
    print(f"PDF saved to: {pdf_path}")
    
    # Display plots
    plt.show()


if __name__ == '__main__':
    main()
