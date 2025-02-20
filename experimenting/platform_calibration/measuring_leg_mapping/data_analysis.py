#!/usr/bin/env python3
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

def filter_arduino_data(df, window_size, threshold_factor):
    """
    Apply robust outlier filtering to Arduino data using a rolling median and MAD.
    
    Args:
        df: DataFrame containing 'arduino_position'.
        window_size: Integer number of points in the rolling window.
        threshold_factor: Float multiplier for the MAD threshold.
    
    Returns:
        valid_mask: A boolean Series that is True for points considered valid.
    """
    med = df['arduino_position'].rolling(window=window_size, center=True, min_periods=1).median()
    mad = df['arduino_position'].rolling(window=window_size, center=True, min_periods=1).apply(
        lambda x: np.median(np.abs(x - np.median(x))), raw=True
    )
    valid_mask = (np.abs(df['arduino_position'] - med) <= threshold_factor * mad)
    return valid_mask

def slider_window(df):
    """
    Opens an interactive slider window that lets you dynamically adjust the robust
    outlier filtering parameters (window size and threshold factor) for the Arduino data.
    
    Uses filter_arduino_data() to compute the valid mask.
    """
    # Create the figure and primary axis.
    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(left=0.1, bottom=0.25)  # Leave room for sliders

    # Plot raw Arduino data in gray.
    raw_line, = ax.plot(df['arduino_timestamp'], df['arduino_position'],
                        label="Arduino Position (raw)",
                        color='gray', linestyle='None', marker='.')

    # Set initial filter parameters.
    init_window = 7       # initial window size (number of points)
    init_thresh = 3.0     # initial threshold factor (e.g. 3x MAD)

    # Compute initial filtered data.
    valid_mask = filter_arduino_data(df, init_window, init_thresh)
    
    # Plot filtered data in red.
    filtered_line, = ax.plot(df['arduino_timestamp'][valid_mask],
                             df['arduino_position'][valid_mask],
                             label="Arduino Position (filtered)",
                             color='red', linestyle='None', marker='o')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position")
    ax.set_title(f"Actuator Data vs. Time (Window={init_window}, Threshold={init_thresh:.1f})")
    ax.legend()
    ax.grid(True)

    # Create slider axes for window size and threshold factor.
    slider_ax_window = plt.axes([0.15, 0.1, 0.7, 0.03])
    slider_ax_thresh = plt.axes([0.15, 0.05, 0.7, 0.03])
    
    # Create the sliders.
    window_slider = Slider(slider_ax_window, 'Window Size', 3, 21,
                           valinit=init_window, valstep=1, valfmt='%0.0f')
    thresh_slider = Slider(slider_ax_thresh, 'Threshold Factor', 0.0, 5.0,
                           valinit=init_thresh, valfmt='%0.01f')
    
    def update(val):
        # Get current slider values.
        current_window = int(window_slider.val)
        current_thresh = thresh_slider.val
        
        # Recompute valid mask using the standalone filter function.
        valid_mask = filter_arduino_data(df, current_window, current_thresh)
        
        # Update filtered data plot.
        filtered_line.set_xdata(df['arduino_timestamp'][valid_mask])
        filtered_line.set_ydata(df['arduino_position'][valid_mask])
        
        ax.set_title(f"Actuator Data vs. Time (Window={current_window}, Threshold={current_thresh:.2f})")
        fig.canvas.draw_idle()
    
    # Connect the slider events.
    window_slider.on_changed(update)
    thresh_slider.on_changed(update)
    
    plt.show()

def static_plot(df):
    """
    Plots the motor and Arduino data on a single static plot using robust outlier filtering.
    
    The filter is applied via filter_arduino_data().
    
    Args:
        df: DataFrame containing 'motor_timestamp', 'motor_position',
            'arduino_timestamp', and 'arduino_position'.
        mask: Boolean mask indicating valid data points for the Arduino data.
    """
   
    plt.figure(figsize=(10, 6))
    plt.plot(df['motor_timestamp'], df['motor_position'],
             label="Motor Position (rev)", color='blue')
    plt.plot(df['arduino_timestamp'],
             df['arduino_position'],
             label="Arduino Position (filtered)", color='red', linestyle='None', marker='o')
    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    plt.title(f"Filtered Actuator Data vs. Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def align_data_to_zero(df, seconds_at_zero=2.0):
    """
    Align the motor and Arduino data by assuming the first 'seconds_at_zero' seconds are at '0'.
    
    Args:
        df: DataFrame containing 'motor_timestamp', 'motor_position',
            'arduino_timestamp', and 'arduino_position'.
        seconds_at_zero: Float number of seconds to assume are at '0'.
    
    Returns:
        aligned_df: DataFrame with the aligned timestamps.
    """
    # Take the average of the first 'seconds_at_zero' seconds of data.
    motor_offset = df['motor_position'][df['motor_timestamp'] <= seconds_at_zero].mean()
    arduino_offset = df['arduino_position'][df['arduino_timestamp'] <= seconds_at_zero].mean()

    # Subtract the offsets from the position data.
    aligned_df = df.copy()
    aligned_df['motor_position'] = aligned_df['motor_position'] - motor_offset
    aligned_df['arduino_position'] = aligned_df['arduino_position'] - arduino_offset
    
    return aligned_df

def simple_scale_data(df, time_offset=-0.12):
    """
    Scale the motor position data to match the Arduino data.
    
    Args:
        df: DataFrame containing 'motor_position' and 'arduino_position'.
        time_offset: Float number of seconds to offset the Arduino position data by
    
    Returns:
        scaled_df: DataFrame with the scaled Arduino data.
    """
    # Compute the scaling factor, in mm/rev
    scaling_factor = df['arduino_position'].max() / df['motor_position'].max()

    # Scale the motor position data.
    scaled_df = df.copy()
    scaled_df['motor_position'] = scaled_df['motor_position'] * scaling_factor

    # Offset the Arduino position data by 'time_offset' seconds.
    # Do this by simply subtracting the offset from the timestamps.
    scaled_df['arduino_timestamp'] = scaled_df['arduino_timestamp'] + time_offset
    
    return scaled_df, scaling_factor

def advanced_scale_align_auto_offset(df, poly_degree=1, offset_range=(-0.5, 0.5), offset_steps=100):
    """
    Automatically determine the best time offset and forced-polynomial mapping (with zero intercept)
    that maps motor_position (in rev) to arduino_position (in mm) so that the two datasets align.
    
    For poly_degree==1, the mapping is forced linear: f(x)=c*x, which is equivalent to a constant scaling factor.
    For poly_degree>1, the mapping is a forced polynomial:
         f(x) = c1*x + c2*x^2 + ... + c_d*x^d   (with f(0)=0).
    
    The function searches over candidate time offsets (applied to arduino_timestamp) and, for each candidate:
      - Interpolates motor_position at the adjusted Arduino times.
      - Fits a forced polynomial of degree poly_degree to (motor_interp, arduino_position).
      - Computes the mean squared error (MSE) between f(motor_interp) and arduino_position.
    The candidate offset yielding the smallest error is selected.
    
    The best time offset is then applied to arduino_timestamp, and the best polynomial mapping is applied
    to motor_position (thus “scaling” the motor data in the y axis).
    
    Args:
        df: DataFrame with columns 'motor_timestamp', 'motor_position',
            'arduino_timestamp', and 'arduino_position'.
        poly_degree: Degree of the forced polynomial (must be >=1).  
                     Note: For a forced fit with zero intercept, poly_degree==1 produces a constant scaling factor.
        offset_range: Tuple (min_offset, max_offset) in seconds over which to search.
        offset_steps: Number of candidate offsets to try.
        
    Returns:
        new_df: DataFrame with updated columns (motor_position scaled and arduino_timestamp shifted),
                while keeping the same headings.
        poly_func: Function that maps motor_position to the predicted arduino_position.
        poly_deriv: Function for the derivative (local scaling factor).
        best_offset: The time offset (in seconds) that minimizes the error.
    """
    import numpy as np

    if poly_degree < 1:
        raise ValueError("poly_degree must be >= 1.")

    # Helper: forced polynomial fit.
    def forced_poly_fit(x, y, degree):
        # For degree==1, compute the constant scaling factor directly.
        if degree == 1:
            denom = np.sum(x**2)
            if denom == 0:
                return np.array([0.0])
            return np.array([np.sum(x * y) / denom])
        # For higher degrees, build the design matrix.
        X = np.vstack([x**i for i in range(1, degree+1)]).T
        try:
            coeffs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
        except np.linalg.LinAlgError:
            coeffs = np.full(degree, np.nan)
        return coeffs

    # Helper: build polynomial function (forced through zero) and its derivative.
    def build_poly_func(coeffs):
        def f(x):
            result = np.zeros_like(x, dtype=float)
            for i, c in enumerate(coeffs, start=1):
                result += c * (x**i)
            return result
        def f_deriv(x):
            result = np.zeros_like(x, dtype=float)
            for i, c in enumerate(coeffs, start=1):
                result += i * c * (x**(i-1))
            return result
        return f, f_deriv

    # Extract arrays.
    motor_time = df['motor_timestamp'].values
    motor_pos = df['motor_position'].values
    arduino_time = df['arduino_timestamp'].values
    arduino_pos = df['arduino_position'].values

    offsets = np.linspace(offset_range[0], offset_range[1], offset_steps)
    best_error = np.inf
    best_offset = None
    best_coeffs = None

    for offset in offsets:
        adjusted_time = arduino_time + offset
        # Interpolate motor positions at the adjusted Arduino times.
        motor_interp = np.interp(adjusted_time, motor_time, motor_pos)
        # Skip candidate if motor_interp is essentially constant (to avoid singular fit)
        if np.all(np.isclose(motor_interp, motor_interp[0])):
            continue
        coeffs = forced_poly_fit(motor_interp, arduino_pos, poly_degree)
        if np.any(np.isnan(coeffs)):
            continue
        poly_f, _ = build_poly_func(coeffs)
        pred = poly_f(motor_interp)
        error = np.nanmean((pred - arduino_pos) ** 2)
        if error < best_error:
            best_error = error
            best_offset = offset
            best_coeffs = coeffs.copy()

    if best_offset is None:
        raise ValueError("Could not find a suitable time offset; check your data.")

    poly_func, poly_deriv = build_poly_func(best_coeffs)
    new_df = df.copy()
    # Apply the best time offset.
    new_df['arduino_timestamp'] = new_df['arduino_timestamp'] + best_offset
    # Replace motor_position with the fitted values.
    new_df['motor_position'] = poly_func(new_df['motor_position'])

    return new_df, poly_func, poly_deriv, best_offset


def assess_and_plot_error(df):
    """
    Assess and plot the error between the motor and Arduino data.
    
    Args:
        df: DataFrame containing 'motor_position' and 'arduino_position'.
    """
    # Interpolate motor position to the arduino timestamps.
    motor_interp = np.interp(df['arduino_timestamp'], df['motor_timestamp'], df['motor_position'])
    error = motor_interp - df['arduino_position']
    error_mean = error.mean()
    error_std = error.std()
    print(f"Error Mean: {error_mean:.4f}")
    print(f"Error Std: {error_std:.4f}")

    # Create a figure with two subplots: the top for data, bottom for error.
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12), sharex=True)

    # Top plot: plot motor and Arduino data.
    ax1.plot(df['motor_timestamp'], df['motor_position'],
             label="Motor Position", color='blue')
    ax1.plot(df['arduino_timestamp'],
             df['arduino_position'],
             label="Arduino Position", color='red', linestyle='None', marker='o')
    ax1.set_ylabel("Position (mm)")
    ax1.set_title("Motor and Arduino Data")
    ax1.legend()
    ax1.grid(True)

    # Bottom plot: plot error between motor and Arduino data using the Arduino timestamps.
    ax2.plot(df['arduino_timestamp'], error, label="Error (Motor - Arduino)", color='green')
    ax2.axhline(y=0, color='gray', linestyle='--')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Error")
    ax2.set_title("Error Between Motor and Arduino Data")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    """ Step 1: Load and normalize the data """
    # Find the package directory and CSV file.
    folder_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file_path = os.path.join(folder_dir, '0_data_filtered.csv')
    
    # Read the CSV file.
    df = pd.read_csv(csv_file_path)
    
    # Verify required columns.
    required_cols = ['motor_timestamp', 'motor_position', 'arduino_timestamp', 'arduino_position']
    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"CSV file is missing required column: {col}")
    
    # Normalize timestamps to start at 0.
    df['motor_timestamp'] = df['motor_timestamp'] - df['motor_timestamp'].iloc[0]
    df['arduino_timestamp'] = df['arduino_timestamp'] - df['arduino_timestamp'].iloc[0]
    
    """ Step 2: Filter the Arduino data to remove outliers """
    
    # Interactive slider window for dynamically tuning filter parameters.
    # slider_window(df)
    
    # Once the parameters are tuned, generate the mask to filter out the outliers.
    tuned_window = 12
    tuned_thresh = 0.99
    valid_mask = filter_arduino_data(df, tuned_window, tuned_thresh)

    # Apply the mask to the arduino data
    df['arduino_position'] = df['arduino_position'].where(valid_mask)

    # Static plot with the filtered data.
    # static_plot(df)

    """ Step 3: Align the data (assume first XX seconds are at '0')"""
    aligned_df = align_data_to_zero(df, seconds_at_zero=2.0)

    # Static plot with the aligned data.
    # static_plot(aligned_df)

    """ Step 4: Scale the Arduino data to match the motor data """
    scaled_df, scaling_factor = simple_scale_data(aligned_df)

    # Static plot with the scaled data.
    print(f"Scaling factor: {scaling_factor:.3f} mm/rev, or {1/scaling_factor:.6f} rev/mm")
    # static_plot(scaled_df)

    # For a linear forced mapping (constant scaling factor), use poly_degree=1.
    scaled_df, poly_func, poly_deriv, best_offset = advanced_scale_align_auto_offset(
        aligned_df, poly_degree=1, offset_range=(-0.5, 0.5), offset_steps=100
    )

    print("Best time offset (s):", best_offset)
    # When poly_degree==1, poly_func is of the form f(x)=s*x.
    # You can print the scaling factor as:
    scaling_factor = poly_deriv(1)  # This returns the constant s.
    print("Constant scaling factor (mm/rev):", scaling_factor)

    """ Step 5: Assess and plot the error between the motor and Arduino data """
    assess_and_plot_error(scaled_df)


if __name__ == '__main__':
    main()
