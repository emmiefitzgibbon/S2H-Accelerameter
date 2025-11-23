#!/usr/bin/env python
# encoding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import glob
import argparse

def plot_accelerometer_data(csv_file=None, start_time=None, end_time=None):
    """
    Plot time series data from accelerometer CSV file.
    One line per sensor, showing acc_x, acc_y, acc_z over time.
    
    Args:
        csv_file: Path to CSV file. If None, will use the most recent acc_data*.csv file.
        start_time: Start time in seconds (optional, filters data to this time and later)
        end_time: End time in seconds (optional, filters data up to this time)
    """
    
    # If no file specified, find the most recent acc_data CSV file
    if csv_file is None:
        csv_files = glob.glob('acc_data*.csv')
        if not csv_files:
            # Try parent directory
            csv_files = glob.glob('../acc_data*.csv')
        if not csv_files:
            print("Error: No acc_data*.csv files found in current or parent directory")
            print("Usage: python plot_accelerometer_data.py [path_to_csv_file]")
            return
        
        # Get the most recent file
        csv_file = max(csv_files, key=os.path.getmtime)
        print(f"Using most recent file: {csv_file}")
    
    # Check if file exists
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found")
        return
    
    # Read CSV file
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Check required columns
    required_cols = ['sensor', 'time', 'acc_x', 'acc_y', 'acc_z']
    missing_cols = [col for col in required_cols if col not in df.columns]
    if missing_cols:
        print(f"Error: Missing required columns: {missing_cols}")
        return
    
    # Filter by time window if specified
    original_count = len(df)
    if start_time is not None:
        df = df[df['time'] >= start_time]
        print(f"Filtered to start time >= {start_time}s")
    if end_time is not None:
        df = df[df['time'] <= end_time]
        print(f"Filtered to end time <= {end_time}s")
    
    if len(df) == 0:
        print("Error: No data in specified time window")
        return
    
    print(f"Plotting {len(df)} data points (from {original_count} total)")
    
    # Get time range for display
    time_min = df['time'].min()
    time_max = df['time'].max()
    
    # Get unique sensors
    sensors = df['sensor'].unique()
    print(f"Found {len(sensors)} sensor(s): {', '.join(sensors)}")
    print(f"Time range: {time_min:.3f}s to {time_max:.3f}s")
    
    # Create figure with subplots for each axis
    # Increase DPI for higher resolution rendering
    # Disable path simplification to ensure ALL points are plotted
    plt.rcParams['path.simplify'] = False
    plt.rcParams['path.simplify_threshold'] = 0.0
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), dpi=100)
    
    # Create title with time window info
    title = f'Accelerometer Time Series Data - {os.path.basename(csv_file)}'
    if start_time is not None or end_time is not None:
        time_info = f' (Time: {time_min:.3f}s - {time_max:.3f}s)'
        title += time_info
    fig.suptitle(title, fontsize=14)
    
    axis_names = ['X', 'Y', 'Z']
    axis_cols = ['acc_x', 'acc_y', 'acc_z']
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown']
    
    # Calculate y-axis limits based on Z axis data (across all sensors)
    z_min = df['acc_z'].min()
    z_max = df['acc_z'].max()
    # Add some padding (5% on each side)
    z_range = z_max - z_min
    y_min = z_min - 0.05 * z_range
    y_max = z_max + 0.05 * z_range
    
    # Plot each sensor - plot ALL points with no downsampling
    for sensor_idx, sensor in enumerate(sensors):
        sensor_data = df[df['sensor'] == sensor].copy()
        sensor_data = sensor_data.sort_values('time')  # Ensure sorted by time
        
        color = colors[sensor_idx % len(colors)]
        
        # Get raw numpy arrays to ensure all points are plotted
        time_values = sensor_data['time'].values
        acc_x_values = sensor_data['acc_x'].values
        acc_y_values = sensor_data['acc_y'].values
        acc_z_values = sensor_data['acc_z'].values
        
        # Plot each axis - explicitly plot all points
        axes[0].plot(
            time_values,
            acc_x_values,
            label=f'{sensor}',
            color=color,
            linewidth=0.5,
            alpha=0.7,
            rasterized=False,
            drawstyle='default'  # Use default drawing style, no downsampling
        )
        axes[1].plot(
            time_values,
            acc_y_values,
            label=f'{sensor}',
            color=color,
            linewidth=0.5,
            alpha=0.7,
            rasterized=False,
            drawstyle='default'
        )
        axes[2].plot(
            time_values,
            acc_z_values,
            label=f'{sensor}',
            color=color,
            linewidth=0.5,
            alpha=0.7,
            rasterized=False,
            drawstyle='default'
        )
        # Set labels and formatting for each axis
        axes[0].set_ylabel('Acceleration X (m/s²)')
        axes[1].set_ylabel('Acceleration Y (m/s²)')
        axes[2].set_ylabel('Acceleration Z (m/s²)')
        
        for axis_idx in range(3):
            axes[axis_idx].grid(True, alpha=0.3)
            axes[axis_idx].legend()
            # Set same y-axis limits for all plots (based on Z axis)
            axes[axis_idx].set_ylim(y_min, y_max)
    
    # Set x-axis label on bottom plot only
    axes[-1].set_xlabel('Time (s)')
    
    # Adjust layout
    plt.tight_layout()
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Plot accelerometer time series data from CSV file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python plot_accelerometer_data.py
  python plot_accelerometer_data.py acc_data.csv
  python plot_accelerometer_data.py acc_data.csv --start 100 --end 200
  python plot_accelerometer_data.py acc_data.csv --start 1000
  python plot_accelerometer_data.py acc_data.csv --end 500
        """
    )
    parser.add_argument('csv_file', nargs='?', default=None,
                       help='Path to CSV file (default: most recent acc_data*.csv)')
    parser.add_argument('--start', type=float, default=None,
                       help='Start time in seconds (filter data from this time onwards)')
    parser.add_argument('--end', type=float, default=None,
                       help='End time in seconds (filter data up to this time)')
    
    args = parser.parse_args()
    
    plot_accelerometer_data(csv_file=args.csv_file, start_time=args.start, end_time=args.end)

