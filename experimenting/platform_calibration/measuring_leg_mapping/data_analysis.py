#!/usr/bin/env python3
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def static_plot(df, title=None):
    """
    Plots the motor and linear slider data on a single static plot
    
    Args:
        df: DataFrame containing 'motor_timestamp', 'motor_position',
            'slider_timestamp', and 'slider_position'.
    """
   
    plt.figure(figsize=(10, 6))
    plt.plot(df['motor_timestamp'], df['motor_position'],
             label="Motor Position (rev)", color='blue')
    plt.plot(df['slider_timestamp'],
             df['slider_position'],
             label="Linear Slider Position", color='red')#), linestyle='None', marker='.', alpha=0.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    if title:
        plt.title(title)
    else:
        plt.title(f"Actuator Data vs. Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def align_data_to_zero(df, seconds_at_zero=1.0):
    """
    Align the motor and linear slider data by assuming the first 'seconds_at_zero' seconds are at '0'.
    
    Args:
        df: DataFrame containing 'motor_timestamp', 'motor_position',
            'slider_timestamp', and 'slider_position'.
        seconds_at_zero: Float number of seconds to assume are at '0'.
    
    Returns:
        aligned_df: DataFrame with the aligned timestamps.
    """
    # Take the average of the first 'seconds_at_zero' seconds of data.
    motor_offset = df['motor_position'][df['motor_timestamp'] <= seconds_at_zero].mean()
    slider_offset = df['slider_position'][df['slider_timestamp'] <= seconds_at_zero].mean()

    print(f"Homed Length: {slider_offset:.4f} mm")

    # Subtract the offsets from the position data.
    aligned_df = df.copy()
    aligned_df['motor_position'] = aligned_df['motor_position'] - motor_offset
    aligned_df['slider_position'] = aligned_df['slider_position'] - slider_offset
    
    return aligned_df

def simple_scale_data(df, time_offset=0.0):
    """
    Scale the motor position data to match the linear slider data.
    
    Args:
        df: DataFrame containing 'motor_position' and 'slider_position'.
        time_offset: Float number of seconds to offset the linear slider position data by (if needed)
    
    Returns:
        scaled_df: DataFrame with the scaled linear slider data.
    """
    # Compute the scaling factor, in mm/rev
    scaling_factor = df['slider_position'].max() / df['motor_position'].max()

    # Scale the motor position data.
    scaled_df = df.copy()
    scaled_df['motor_position'] = scaled_df['motor_position'] * scaling_factor

    # Offset the linear slider position data by 'time_offset' seconds.
    # Do this by simply subtracting the offset from the timestamps.
    scaled_df['slider_timestamp'] = scaled_df['slider_timestamp'] + time_offset
    
    return scaled_df, scaling_factor

def scale_data_based_on_all_extended_lengths(df, peak_times, peak_duration, time_offset=0.0):
    """
    Scale the motor position data to match the linear slider data based on the extended lengths
    measured over all 5 runs.
    ie. take the average position of the motor and linear slider at each peak (which are held for >1s)
    and use these averages to scale the data.
    """

    # Create a list to store the average motor and linear slider positions at each peak
    avg_positions_list = []

    # Loop through each peak time and calculate the average motor and linear slider positions
    for time in peak_times:
        # Get the data for the peak time
        peak_data = df[(df['motor_timestamp'] >= time) & (df['motor_timestamp'] <= time + peak_duration)]
        avg_motor_position = peak_data['motor_position'].mean()
        avg_slider_position = peak_data['slider_position'].mean()

        # Append the average positions to the list
        avg_positions_list.append({'motor_position': avg_motor_position, 'slider_position': avg_slider_position})
    
    # Convert the list to a DataFrame
    avg_positions = pd.DataFrame(avg_positions_list)

    # Compute the scaling factor, in mm/rev
    scaling_factor = avg_positions['slider_position'].mean() / avg_positions['motor_position'].mean()

    # Scale the motor position data.
    scaled_df = df.copy()
    scaled_df['motor_position'] = scaled_df['motor_position'] * scaling_factor

    # Offset the linear slider position data by 'time_offset' seconds.
    # Do this by simply subtracting the offset from the timestamps.
    scaled_df['slider_timestamp'] = scaled_df['slider_timestamp'] + time_offset

    return scaled_df, scaling_factor

def assess_and_plot_error(df, peak_times=None, peak_duration=None, title=None, label=None):
    """
    Assess and plot the error between the motor and linear slider data.
    
    Args:
        df: DataFrame containing 'motor_position' and 'slider_position'.
        
        title: String title for the plot.
        label: String label for the error analysis.
    """
    # Interpolate motor position to the linear slider timestamps.
    motor_interp = np.interp(df['slider_timestamp'], df['motor_timestamp'], df['motor_position'])
    error = motor_interp - df['slider_position']
    error_mean = error.mean()
    error_std = error.std()
    if label:
        print(f"Error Mean ({label}): {error_mean:.4f}")
        print(f"Error Std ({label}): {error_std:.4f}")
    else:
        print(f"Error Mean: {error_mean:.4f}")
        print(f"Error Std: {error_std:.4f}")

    # Create a figure with two subplots: the top for data, bottom for error.
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12), sharex=True)

    # Top plot: plot motor and linear slider data.
    ax1.plot(df['motor_timestamp'], df['motor_position'],
             label="Motor Position", color='blue')
    ax1.plot(df['slider_timestamp'],
             df['slider_position'],
             label="Linear Slider Position", color='red')#, linestyle='None', marker='o', alpha=0.5)
    ax1.set_ylabel("Position (mm)")

    if peak_times and peak_duration:
        # Add vertical lines at the start and end of each peak
        for time in peak_times:
            ax1.axvline(x=time, color='gray', linestyle='--')
            ax1.axvline(x=time + peak_duration, color='gray', linestyle='--')

    if title:
        ax1.set_title(title)
    else:
        ax1.set_title("Motor and Linear Slider Data")
    ax1.legend()
    ax1.grid(True)

    # Bottom plot: plot error between motor and linear slider data using the linear slider timestamps.
    ax2.plot(df['slider_timestamp'], error, label="Error (Motor - Slider)", color='green')
    ax2.axhline(y=0, color='gray', linestyle='--')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Error (mm)")
    ax2.set_title("Error Between Motor and Linear Slider Data")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()

def main():
    """ Step 1: Load and normalize the data """
    # Find the package directory and CSV file.
    file_name = os.path.join('Vertical Rig', '5_data.csv')
    folder_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file_path = os.path.join(folder_dir, 'Data', file_name)
    
    # Read the CSV file.
    df = pd.read_csv(csv_file_path)
    
    # Verify required columns.
    required_cols = ['motor_timestamp', 'motor_position', 'slider_timestamp', 'slider_position']
    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"CSV file is missing required column: {col}")
    
    # Normalize timestamps to start at 0.
    df['motor_timestamp'] = df['motor_timestamp'] - df['motor_timestamp'].iloc[0]
    df['slider_timestamp'] = df['slider_timestamp'] - df['slider_timestamp'].iloc[0]

    # Truncate the data (if desired)
    # If the data is from the vertical rig, truncate the data to the first 140 seconds
    # if 'Vertical Rig' in file_name:
    #     df = df[df['slider_timestamp'] <= 140]

    # Truncate all data to the first 57 seconds, to remove the high-error regions at the end of each test
    df = df[df['slider_timestamp'] <= 57]

    """ Step 2: Align the data (assume first XX seconds are at '0')"""
    print("\n##############################################")
    print(f"Analyzing data from: {file_name}")
    aligned_df = align_data_to_zero(df)

    # Define the times when the actuator is at its peak
    peak_times = [5.00, 16.50, 28.00, 39.50, 51.00]
    peak_duration = 0.75 # seconds

    # Static plot with the aligned data.
    # static_plot(aligned_df, title="Aligned Data")

    """ Step 4: Scale the linear slider data to match the motor data """
    simple_scaled_df, simple_scaling_factor = simple_scale_data(aligned_df)
    averaged_scaled_df, averaged_scaling_factor = scale_data_based_on_all_extended_lengths(aligned_df, peak_times, peak_duration)

    # Static plot with the scaled data.
    print(f"Simple scaling factor: {simple_scaling_factor:.3f} mm/rev, or {1/simple_scaling_factor:.6f} rev/mm")
    print(f"Averaged scaling factor: {averaged_scaling_factor:.3f} mm/rev, or {1/averaged_scaling_factor:.6f} rev/mm")
    # static_plot(scaled_df, title="Scaled Data")


    """ Step 5: Assess and plot the error between the motor and Linear slider data """
    assess_and_plot_error(simple_scaled_df, title=f"Error Analysis for {file_name}, with Simple Scaling Factor", label="Simple Scale")
    assess_and_plot_error(averaged_scaled_df, peak_times, peak_duration, title=f"Error Analysis for {file_name}, with Averaged Scaling Factor", label="Averaged Scale")

    print("##############################################\n")

    plt.show()

if __name__ == '__main__':
    main()
