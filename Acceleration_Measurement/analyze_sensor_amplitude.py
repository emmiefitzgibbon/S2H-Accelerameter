"""
Analyze accelerometer data from DriveXMeasureY subfolders.
Calculates average amplitude (magnitude) for sensor 1 vs sensor 2.
Groups files within each DriveXMeasureY folder and averages them.
"""

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import re
from pathlib import Path
from collections import defaultdict

def calculate_amplitude(acc_x, acc_y, acc_z):
    """Calculate the amplitude using only Z axis."""
    return np.abs(acc_z)

def analyze_csv_file(csv_path):
    """Analyze a single CSV file and return average amplitudes for each sensor."""
    try:
        # Read CSV file
        df = pd.read_csv(csv_path)
        
        if len(df) == 0:
            print(f"  Warning: {csv_path} is empty, skipping...")
            return None
        
        # Remove DC offset for each sensor by normalizing each axis to have mean of 0
        # Group by sensor and subtract the mean from each axis
        df_normalized = df.copy()
        
        for sensor in df['sensor'].unique():
            sensor_mask = df['sensor'] == sensor
            sensor_data = df[sensor_mask]
            
            # Calculate mean for each axis for this sensor
            mean_x = sensor_data['acc_x'].mean()
            mean_y = sensor_data['acc_y'].mean()
            mean_z = sensor_data['acc_z'].mean()
            
            # Subtract the mean (DC offset) from each axis
            df_normalized.loc[sensor_mask, 'acc_x'] = sensor_data['acc_x'] - mean_x
            df_normalized.loc[sensor_mask, 'acc_y'] = sensor_data['acc_y'] - mean_y
            df_normalized.loc[sensor_mask, 'acc_z'] = sensor_data['acc_z'] - mean_z
        
        # Calculate amplitude for each row using normalized data
        df_normalized['amplitude'] = calculate_amplitude(
            df_normalized['acc_x'], 
            df_normalized['acc_y'], 
            df_normalized['acc_z']
        )
        
        # Filter out periods where either sensor shows very low amplitude (vibration hasn't started)
        # Thresholds: Sensor 1 (0x1D) < 5 m/s², Sensor 2 (0x53) < 0.7 m/s²
        AMPLITUDE_THRESHOLD_S1 = 5.0  # m/s²
        AMPLITUDE_THRESHOLD_S2 = 0.7  # m/s²
        
        # Group by time to find periods where both sensors have valid data
        # Use a time window approach: group by time bins and check if both sensors are present and above threshold
        # Use smaller bins for more granular analysis
        BIN_SIZE = 10000  # 10ms bins
        MIN_FILTER_DURATION = 200000  # Minimum duration (200ms in microseconds) for filtering
        
        df_normalized['time_bin'] = (df_normalized['time'] // BIN_SIZE) * BIN_SIZE
        
        # For each time bin, check if BOTH sensors have significant variation/amplitude
        # Check both mean amplitude and standard deviation to avoid including regions with DC offset but small variation
        time_bin_valid_raw = {}
        for time_bin in sorted(df_normalized['time_bin'].unique()):
            bin_data = df_normalized[df_normalized['time_bin'] == time_bin]
            s1_has_valid = False
            s2_has_valid = False
            s1_present = False
            s2_present = False
            
            for sensor in bin_data['sensor'].unique():
                sensor_data = bin_data[bin_data['sensor'] == sensor]
                if '0x1D' in sensor:
                    # Sensor 1: check if mean amplitude AND std dev indicate real vibration
                    s1_present = True
                    if len(sensor_data) > 0:
                        mean_amp = sensor_data['amplitude'].mean()
                        std_amp = sensor_data['amplitude'].std()
                        # Require both mean and std to be above threshold to avoid DC offset issues
                        s1_has_valid = (mean_amp >= AMPLITUDE_THRESHOLD_S1 * 0.5) and (std_amp >= AMPLITUDE_THRESHOLD_S1 * 0.3)
                elif '0x53' in sensor:
                    # Sensor 2: check if mean amplitude AND std dev indicate real vibration
                    s2_present = True
                    if len(sensor_data) > 0:
                        mean_amp = sensor_data['amplitude'].mean()
                        std_amp = sensor_data['amplitude'].std()
                        # Require both mean and std to be above threshold to avoid DC offset issues
                        s2_has_valid = (mean_amp >= AMPLITUDE_THRESHOLD_S2 * 0.5) and (std_amp >= AMPLITUDE_THRESHOLD_S2 * 0.3)
            
            # Mark bin as invalid if either sensor is missing or doesn't show real vibration
            time_bin_valid_raw[time_bin] = s1_present and s2_present and s1_has_valid and s2_has_valid
        
        # Apply temporal smoothing: only filter if invalid bins persist for a minimum duration
        # This prevents momentary filtering in the middle of trials
        sorted_bins = sorted(time_bin_valid_raw.keys())
        time_bin_valid = {}
        
        # Find continuous invalid regions that are long enough to filter
        # Use shorter minimum duration for beginning/end regions
        MIN_FILTER_DURATION_EDGES = 100000  # 100ms for beginning/end
        invalid_regions = []
        current_invalid_start = None
        first_bin = sorted_bins[0] if sorted_bins else None
        last_bin = sorted_bins[-1] if sorted_bins else None
        
        for i, time_bin in enumerate(sorted_bins):
            is_valid = time_bin_valid_raw[time_bin]
            
            if not is_valid:
                # Start or continue an invalid region
                if current_invalid_start is None:
                    current_invalid_start = time_bin
            else:
                # End of invalid region
                if current_invalid_start is not None:
                    invalid_duration = time_bin - current_invalid_start
                    # Use shorter threshold if at beginning, longer for middle
                    is_at_start = current_invalid_start == first_bin
                    min_duration = MIN_FILTER_DURATION_EDGES if is_at_start else MIN_FILTER_DURATION
                    
                    if invalid_duration >= min_duration:
                        invalid_regions.append((current_invalid_start, time_bin))
                    current_invalid_start = None
        
        # Handle case where invalid region extends to the end
        if current_invalid_start is not None:
            invalid_duration = sorted_bins[-1] - current_invalid_start + BIN_SIZE
            # Use shorter threshold if invalid region starts at beginning or extends to end
            is_at_edge = (current_invalid_start == first_bin) or (current_invalid_start + invalid_duration >= last_bin)
            min_duration = MIN_FILTER_DURATION_EDGES if is_at_edge else MIN_FILTER_DURATION
            
            if invalid_duration >= min_duration:
                invalid_regions.append((current_invalid_start, sorted_bins[-1] + BIN_SIZE))
        
        # Mark bins: valid if raw check says valid AND not in a long invalid region
        for time_bin in sorted_bins:
            raw_valid = time_bin_valid_raw[time_bin]
            
            # Check if this bin is in a long enough invalid region
            is_in_long_invalid_region = False
            for invalid_start, invalid_end in invalid_regions:
                if invalid_start <= time_bin < invalid_end:
                    is_in_long_invalid_region = True
                    break
            
            # Bin is valid if: raw check says valid OR it's not in a long invalid region
            # (i.e., only filter if in a long invalid region)
            time_bin_valid[time_bin] = raw_valid or not is_in_long_invalid_region
        
        # Filter to keep only rows in valid time bins
        df_normalized['time_bin_valid'] = df_normalized['time_bin'].map(time_bin_valid)
        df_filtered = df_normalized[df_normalized['time_bin_valid'] == True].copy()
        
        if len(df_filtered) == 0:
            print(f"  Warning: {csv_path} has no valid data after amplitude filtering, skipping...")
            return None
        
        # Group by sensor and calculate average amplitude
        sensor_amplitudes = df_filtered.groupby('sensor')['amplitude'].mean()
        
        # Get sensor names
        sensors = sensor_amplitudes.index.tolist()
        
        # Map sensors to sensor 1 and sensor 2
        # Sensor_0x1D is typically sensor 1, Sensor_0x53 is typically sensor 2
        sensor1_avg = None
        sensor2_avg = None
        sensor1_name = None
        sensor2_name = None
        
        for sensor in sensors:
            if '0x1D' in sensor:
                sensor1_avg = sensor_amplitudes[sensor]
                sensor1_name = sensor
            elif '0x53' in sensor:
                sensor2_avg = sensor_amplitudes[sensor]
                sensor2_name = sensor
        
        results = {
            'file': os.path.basename(csv_path),
            'path': csv_path,
            'sensor1_name': sensor1_name,
            'sensor1_avg_amplitude': sensor1_avg,
            'sensor2_name': sensor2_name,
            'sensor2_avg_amplitude': sensor2_avg,
            'ratio': sensor2_avg / sensor1_avg if (sensor1_avg and sensor1_avg != 0) else None,
            'difference': sensor2_avg - sensor1_avg if (sensor1_avg and sensor2_avg) else None,
            'total_rows': len(df_filtered),
            'df_normalized': df_normalized,  # Include full normalized data with filtering info
            'df_filtered': df_filtered  # Include filtered data
        }
        
        return results
        
    except Exception as e:
        print(f"  Error analyzing {csv_path}: {str(e)}")
        return None

def parse_drive_measure(folder_name):
    """Extract Drive and Measure numbers from folder name like 'Drive1Measure2'."""
    match = re.search(r'Drive(\d+)Measure(\d+)', folder_name, re.IGNORECASE)
    if match:
        return int(match.group(1)), int(match.group(2))
    return None, None

def main():
    # Base directory containing the data
    script_dir = Path(__file__).parent
    base_dir = script_dir.parent / "Accelerometer Data Sunday Dec 7"
    
    # Find all CSV files in DriveXMeasureY subfolders
    csv_files = []
    for root, dirs, files in os.walk(base_dir):
        # Check if this is a DriveXMeasureY folder
        if 'Drive' in root and 'Measure' in root:
            for file in files:
                if file.endswith('.csv'):
                    csv_files.append(os.path.join(root, file))
    
    print(f"Found {len(csv_files)} CSV files to analyze")
    print("=" * 80)
    
    # Analyze each file
    all_results = []
    for csv_file in sorted(csv_files):
        result = analyze_csv_file(csv_file)
        if result:
            all_results.append(result)
    
    if not all_results:
        print("No valid results to analyze.")
        return
    
    # Group by DriveXMeasureY folder
    folder_results = defaultdict(list)
    for result in all_results:
        # Extract folder name from path
        path_parts = Path(result['path']).parts
        folder_name = None
        for part in path_parts:
            if 'Drive' in part and 'Measure' in part:
                folder_name = part
                break
        if folder_name is None:
            folder_name = Path(result['path']).parent.name
        
        folder_results[folder_name].append(result)
    
    # Group by Drive and Measure, and calculate statistics
    drive_measure_data = defaultdict(lambda: {
        'sensor1_trials': [],
        'sensor2_trials': [],
        'folder_name': None
    })
    
    for folder_name, results in folder_results.items():
        drive, measure = parse_drive_measure(folder_name)
        if drive is not None and measure is not None:
            key = (drive, measure)
            drive_measure_data[key]['folder_name'] = folder_name
            
            # Collect all trial values for this Drive/Measure combo
            for result in results:
                if result['sensor1_avg_amplitude'] is not None:
                    drive_measure_data[key]['sensor1_trials'].append(result['sensor1_avg_amplitude'])
                if result['sensor2_avg_amplitude'] is not None:
                    drive_measure_data[key]['sensor2_trials'].append(result['sensor2_avg_amplitude'])
    
    # Print summary statistics
    print("\n" + "=" * 80)
    print("SUMMARY STATISTICS (Averaged by Drive/Measure)")
    print("=" * 80)
    
    # Group by Drive for printing
    drives = defaultdict(list)
    for (drive, measure), data in sorted(drive_measure_data.items()):
        drives[drive].append((measure, data))
    
    for drive in sorted(drives.keys()):
        print(f"\nDrive {drive}:")
        for measure, data in sorted(drives[drive]):
            s1_trials = data['sensor1_trials']
            s2_trials = data['sensor2_trials']
            
            if s1_trials:
                s1_mean = np.mean(s1_trials)
                s1_std = np.std(s1_trials)
                s1_se = s1_std / np.sqrt(len(s1_trials)) if len(s1_trials) > 1 else 0
                print(f"  Measure {measure}:")
                print(f"    Sensor 1 - Mean: {s1_mean:.4f} m/s², Std: {s1_std:.4f} m/s², SE: {s1_se:.4f} m/s² ({len(s1_trials)} trials)")
            
            if s2_trials:
                s2_mean = np.mean(s2_trials)
                s2_std = np.std(s2_trials)
                s2_se = s2_std / np.sqrt(len(s2_trials)) if len(s2_trials) > 1 else 0
                print(f"    Sensor 2 - Mean: {s2_mean:.4f} m/s², Std: {s2_std:.4f} m/s², SE: {s2_se:.4f} m/s² ({len(s2_trials)} trials)")
            
            if s1_trials and s2_trials:
                ratio = s2_mean / s1_mean
                print(f"    Ratio (S2/S1): {ratio:.4f}")
    
    # Save detailed results to CSV
    summary_data = []
    for (drive, measure), data in sorted(drive_measure_data.items()):
        s1_trials = data['sensor1_trials']
        s2_trials = data['sensor2_trials']
        
        summary_data.append({
            'drive': drive,
            'measure': measure,
            'folder': data['folder_name'],
            'sensor1_mean': np.mean(s1_trials) if s1_trials else None,
            'sensor1_std': np.std(s1_trials) if s1_trials else None,
            'sensor1_se': np.std(s1_trials) / np.sqrt(len(s1_trials)) if len(s1_trials) > 1 else 0,
            'sensor1_n_trials': len(s1_trials),
            'sensor2_mean': np.mean(s2_trials) if s2_trials else None,
            'sensor2_std': np.std(s2_trials) if s2_trials else None,
            'sensor2_se': np.std(s2_trials) / np.sqrt(len(s2_trials)) if len(s2_trials) > 1 else 0,
            'sensor2_n_trials': len(s2_trials),
            'ratio': np.mean(s2_trials) / np.mean(s1_trials) if (s1_trials and s2_trials) else None
        })
    
    results_df = pd.DataFrame(summary_data)
    output_file = base_dir / "sensor_amplitude_analysis_results.csv"
    results_df.to_csv(output_file, index=False)
    print(f"\nDetailed results saved to: {output_file}")
    
    # Create plots
    create_plots(drive_measure_data, base_dir)
    
    # Create filtering visualization plots for each trial
    create_filtering_plots(all_results, base_dir)

def create_filtering_plots(all_results, base_dir):
    """Create grid plots showing filtered vs kept data for all trials in one figure."""
    print("\nGenerating filtering visualization plots...")
    
    AMPLITUDE_THRESHOLD_S1 = 5.0  # m/s²
    AMPLITUDE_THRESHOLD_S2 = 3.0  # m/s²
    
    # Filter results that have data
    valid_results = [r for r in all_results if 'df_normalized' in r and r['df_normalized'] is not None]
    
    if len(valid_results) == 0:
        print("  No valid results to plot.")
        return
    
    # Create one figure with all trials stacked
    n_trials = len(valid_results)
    fig, axes = plt.subplots(n_trials, 1, figsize=(14, 4 * n_trials), dpi=100)
    
    # If only one trial, axes won't be an array
    if n_trials == 1:
        axes = [axes]
    
    fig.suptitle('Filtering Visualization: All Trials (Z-axis only)', fontsize=16, fontweight='bold')
    
    # Get unique sensors (should be same for all)
    sensors = valid_results[0]['df_normalized']['sensor'].unique()
    colors = {'Sensor_0x1D': '#2E86AB', 'Sensor_0x53': '#A23B72'}
    
    # Plot each trial
    for trial_idx, result in enumerate(valid_results):
        df_normalized = result['df_normalized']
        file_name = result['file']
        ax = axes[trial_idx]
        
        # Convert time from microseconds to seconds for display
        df_normalized = df_normalized.copy()
        df_normalized['time_sec'] = df_normalized['time'] / 1e6
        
        # Get time range for y-axis limits
        all_times = df_normalized['time_sec'].values
        all_z_values = df_normalized['acc_z'].values
        y_min = np.min(all_z_values)
        y_max = np.max(all_z_values)
        y_range = y_max - y_min
        y_padding = y_range * 0.05
        
        # Plot each sensor - plot ALL data in sensor colors
        for sensor in sensors:
            sensor_data = df_normalized[df_normalized['sensor'] == sensor].copy()
            sensor_data = sensor_data.sort_values('time_sec')
            
            color = colors.get(sensor, 'gray')
            sensor_label = 'Sensor 1 (0x1D)' if '0x1D' in sensor else 'Sensor 2 (0x53)'
            
            # Plot all data in sensor color
            ax.plot(
                sensor_data['time_sec'],
                sensor_data['acc_z'],
                color=color,
                alpha=0.8,
                linewidth=0.8,
                label=sensor_label if trial_idx == 0 else ''
            )
        
        # Identify filtered time regions and add red transparent overlays
        # Group by time bins to find filtered regions
        time_bins = sorted(df_normalized['time_bin'].unique())
        filtered_regions = []
        
        for time_bin in time_bins:
            bin_data = df_normalized[df_normalized['time_bin'] == time_bin]
            is_valid = bin_data['time_bin_valid'].iloc[0] if len(bin_data) > 0 else False
            
            if not is_valid:
                # This is a filtered region
                bin_times = bin_data['time_sec'].values
                if len(bin_times) > 0:
                    filtered_regions.append((bin_times.min(), bin_times.max()))
        
        # Merge adjacent filtered regions
        if filtered_regions:
            merged_regions = []
            filtered_regions = sorted(filtered_regions)
            current_start, current_end = filtered_regions[0]
            
            for start, end in filtered_regions[1:]:
                if start <= current_end:
                    # Merge with current region
                    current_end = max(current_end, end)
                else:
                    # Save current region and start new one
                    merged_regions.append((current_start, current_end))
                    current_start, current_end = start, end
            merged_regions.append((current_start, current_end))
            
            # Add red transparent overlays for each filtered region
            for start, end in merged_regions:
                ax.axvspan(start, end, alpha=0.3, color='red', zorder=0)
        
        # Set labels and title for this trial
        ax.set_ylabel('Acceleration Z (m/s²)', fontsize=10)
        ax.set_title(f'{file_name}', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        if trial_idx == 0:
            ax.legend(loc='best', fontsize=9)
        
        # Add text annotation showing filtering stats for this trial
        total_points = len(df_normalized)
        kept_points = len(df_normalized[df_normalized['time_bin_valid'] == True])
        filtered_points = total_points - kept_points
        kept_pct = (kept_points / total_points * 100) if total_points > 0 else 0
        
        stats_text = f'Kept: {kept_points:,} ({kept_pct:.1f}%) | Filtered: {filtered_points:,} ({100-kept_pct:.1f}%)'
        ax.text(0.99, 0.02, stats_text, transform=ax.transAxes, ha='right', fontsize=8,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Set x-axis label on bottom plot only
    axes[-1].set_xlabel('Time (s)', fontsize=10)
    
    plt.tight_layout()
    
    # Save the combined plot
    plot_file = base_dir / "filtering_visualization_all_trials.png"
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"  Combined filtering plot saved to: {plot_file}")
    
    plt.close()
    
    print("Filtering visualization plot generated!")

def create_plots(drive_measure_data, base_dir):
    """Create visualization plots for the sensor amplitude analysis."""
    print("\nGenerating plots...")
    
    # Group by Drive
    drives = defaultdict(list)
    for (drive, measure), data in drive_measure_data.items():
        drives[drive].append((measure, data))
    
    # Create one plot per Drive
    for drive in sorted(drives.keys()):
        fig, ax = plt.subplots(figsize=(10, 6))
        
        measures = []
        s1_means = []
        s1_stds = []
        s1_ses = []
        s2_means = []
        s2_stds = []
        s2_ses = []
        s1_all_trials = []
        s2_all_trials = []
        
        for measure, data in sorted(drives[drive]):
            measures.append(measure)
            
            s1_trials = data['sensor1_trials']
            s2_trials = data['sensor2_trials']
            
            if s1_trials:
                s1_mean = np.mean(s1_trials)
                s1_std = np.std(s1_trials)
                s1_se = s1_std / np.sqrt(len(s1_trials)) if len(s1_trials) > 1 else 0
                s1_means.append(s1_mean)
                s1_stds.append(s1_std)
                s1_ses.append(s1_se)
                s1_all_trials.append(s1_trials)
            else:
                s1_means.append(0)
                s1_stds.append(0)
                s1_ses.append(0)
                s1_all_trials.append([])
            
            if s2_trials:
                s2_mean = np.mean(s2_trials)
                s2_std = np.std(s2_trials)
                s2_se = s2_std / np.sqrt(len(s2_trials)) if len(s2_trials) > 1 else 0
                s2_means.append(s2_mean)
                s2_stds.append(s2_std)
                s2_ses.append(s2_se)
                s2_all_trials.append(s2_trials)
            else:
                s2_means.append(0)
                s2_stds.append(0)
                s2_ses.append(0)
                s2_all_trials.append([])
        
        # Set up x positions
        x = np.arange(len(measures))
        width = 0.35
        
        # Plot bars with error bars
        bars1 = ax.bar(x - width/2, s1_means, width, yerr=s1_ses, 
                      label='Sensor 1 (0x1D)', alpha=0.8, color='#2E86AB', 
                      capsize=5, edgecolor='black', linewidth=1.5)
        bars2 = ax.bar(x + width/2, s2_means, width, yerr=s2_ses,
                      label='Sensor 2 (0x53)', alpha=0.8, color='#A23B72',
                      capsize=5, edgecolor='black', linewidth=1.5)
        
        # Plot individual trial points
        for i, (s1_trials, s2_trials) in enumerate(zip(s1_all_trials, s2_all_trials)):
            x_pos_s1 = x[i] - width/2
            x_pos_s2 = x[i] + width/2
            
            # Add jitter to x positions for better visibility
            if s1_trials:
                jitter_s1 = np.random.normal(0, width/8, len(s1_trials))
                ax.scatter(x_pos_s1 + jitter_s1, s1_trials, 
                          s=60, alpha=0.6, color='#1a5f7a', 
                          edgecolors='black', linewidth=0.5, zorder=10)
            
            if s2_trials:
                jitter_s2 = np.random.normal(0, width/8, len(s2_trials))
                ax.scatter(x_pos_s2 + jitter_s2, s2_trials,
                          s=60, alpha=0.6, color='#7a2d5a',
                          edgecolors='black', linewidth=0.5, zorder=10)
        
        # Customize the plot
        ax.set_xlabel('Measure Motor', fontsize=12, fontweight='bold')
        ax.set_ylabel('Average Amplitude (m/s²)', fontsize=12, fontweight='bold')
        ax.set_title(f'Drive {drive}: Sensor Amplitude Comparison', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels([f'Measure {m}' for m in measures], fontsize=10)
        ax.legend(fontsize=10, loc='best')
        ax.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        
        # Save the plot
        plot_file = base_dir / f"sensor_amplitude_drive{drive}_plot.png"
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        print(f"  Plot saved to: {plot_file}")
        
        plt.close()
    
    print("All plots generated successfully!")

if __name__ == "__main__":
    main()
