'''
Simulate Ball Butler throwing to Jugglebot, in 2D.
Show trajectory of the throw, and colour each trajectory based on throw angle
'''

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm

g = 9.81 # m/s^2

# Ball and Ball Butler Properties
ball_mass = 0.07    # kg
hand_stroke = 0.09 # m, only includes distance for the acceleration phase

def calc_energy_power_accel(throw_vel):
    kinetic_energy = 0.5 * ball_mass * throw_vel**2

    # Calculate the necessary (linear) acceleration for the hand to accelerate the ball
    peak_acceleration = throw_vel**2 / (2 * hand_stroke)

    # Calculate the time it takes to reach the peak acceleration
    acceleration_time = throw_vel / peak_acceleration
    if acceleration_time == 0:
        return np.nan, np.nan
    
    # Power is energy divided by time
    power = kinetic_energy / acceleration_time

    # Optional print
    # print(f'Throw vel: {throw_vel:.2f} m/s, Kinetic Energy: {kinetic_energy:.2f} J, Power: {power:.2f} W, Accel Time: {acceleration_time:.2f} s, Peak Acceleration: {peak_acceleration:.2f} m/s²')
    return kinetic_energy, power, peak_acceleration

def calculate_trajectories(throw_pos, catch_pos, throw_angles):
    """Calculate trajectories for given throw/catch positions and angles"""
    horizontal_range = catch_pos[0] - throw_pos[0] # horizontal distance to cover {m}
    vertical_range = throw_pos[1] - catch_pos[1] # vertical distance to cover {m}
    
    trajectories = []
    trajectory_data = []
    
    for angle in throw_angles:
        v_i = np.sqrt((g * horizontal_range**2) / (2 * math.cos(angle)**2 * (horizontal_range * math.tan(angle) + vertical_range)))

        # Calculate the energy and power required to throw the ball
        E, P, peak_accel = calc_energy_power_accel(v_i)

        # Get components of initial velocity
        v_i_y = v_i * math.sin(angle)
        v_i_x = v_i * math.cos(angle)

        # Calculate flight time
        t_flight = (v_i * math.sin(angle) + np.sqrt((v_i * math.sin(angle))**2 + 2 * g * vertical_range)) / g
        t = np.linspace(0, t_flight, num=100)  # time points

        # Parametric equations for the projectile motion
        x = throw_pos[0] + v_i * np.cos(angle) * t
        y = throw_pos[1] + v_i * np.sin(angle) * t - 0.5 * g * t**2

        # Calculate the landing velocity (assuming no air resistance)
        v_f_y = np.sqrt(v_i_y**2 + 2 * g * vertical_range)
        v_f = np.sqrt(v_i_x**2 + v_f_y**2) 

        # Calculate the landing angle
        landing_angle = np.arctan2(v_f_y, v_i_x)

        # print(f'Angle: {np.degrees(angle):.1f} degrees, v_i_x= {v_i_x:.2f} m/s, v_i_y= {v_i_y:.2f} m/s, Flight Time: {t_flight:.2f} s')
        # print(f'Landing Velocity: {v_f:.2f} m/s, Landing Angle: {np.degrees(landing_angle):.1f} degrees\n')

        trajectories.append((x, y))
        trajectory_data.append({
            'throw_angle': angle,
            'v_i': v_i,
            'peak_accel': peak_accel,
            'power': P,
            'landing_angle': landing_angle,
            'v_f': v_f
        })

    # Find the trajectory with the lowest launch velocity
    min_velocity_index = np.argmin([data['v_i'] for data in trajectory_data])
    min_velocity_trajectory = trajectories[min_velocity_index]
    min_velocity_data = trajectory_data[min_velocity_index]

    return trajectories, trajectory_data, min_velocity_trajectory, min_velocity_data

def plot_trajectories(trajectories, trajectory_data, min_velocity_trajectory, min_velocity_data, figsize=(15, 8), title='Projectile Trajectories'):
    """Plot trajectories with legend"""
   
    # Create a figure
    fig = plt.figure(figsize=figsize)
    colours = cm.rainbow(np.linspace(0, 1, len(trajectories)))

    # Initialize lists to store plot lines and legend data
    lines = []
    legend_labels = []

    # Add the header row with an empty line object
    header = plt.Line2D([0], [0], color='none')
    legend_labels.append("   $\\theta_{i}$,        $v_{i}$,       Peak Accel,  Power  —    $\\theta_{f}$,         $v_{f}$")

    # Plot each trajectory
    for i, ((x, y), traj_data) in enumerate(zip(trajectories, trajectory_data)):
        # If this is the minimum velocity trajectory, make it thicker and black
        line, = plt.plot(x, y, color='black' if traj_data == min_velocity_data else colours[i], linewidth=2 if traj_data == min_velocity_data else 1)

        # Ensure we show a max of 15 legend labels, irrespective of how many datapoints we have
        # Calculate how many lines to skip between each one being added to the legend
        if len(trajectories) <= 15:
            lines_to_skip = 1
        else:
            lines_to_skip = max(2, len(trajectories) // 15 + 1)

        if i % lines_to_skip == 0 or traj_data == min_velocity_data:
            lines.append(line)
            legend_labels.append(
                f"{np.degrees(traj_data['throw_angle']):.1f}°, "
                f"{traj_data['v_i']:.2f} m/s, "
                f"{traj_data['peak_accel']:.2f} m/s², "
                f"{traj_data['power']:.2f} W — "
                f"{np.degrees(traj_data['landing_angle']):.1f}°, "
                f"{traj_data['v_f']:.2f} m/s"
        )

    plt.legend([header] + lines, legend_labels, bbox_to_anchor=(1.01, 1), loc='upper left')
    plt.tight_layout()
    plt.gcf().subplots_adjust(left=0.05, right=0.67, bottom=0.1, top=0.9)
    plt.xlabel('Horizontal Distance (m)')
    plt.ylabel('Vertical Distance (m)')
    plt.title(title)
    plt.grid()
    plt.axis('equal')

    return fig

# Original parameters
throw_pos = (0, 0)  # (x, y) in meters
catch_pos = (1.1, -1.5)  # (x, y) in meters
throw_angle = np.radians(np.linspace(1, 80, 500))  # angles to test

trajectory, trajectory_data, min_velocity_trajectory, min_velocity_data = calculate_trajectories(throw_pos, catch_pos, throw_angle)

# Create the first plot, showing the trajectories
fig1 = plot_trajectories(trajectory, trajectory_data, min_velocity_trajectory, min_velocity_data, title=f'Original Projectile Trajectories \n Throw Position: {throw_pos}, Catch Position: {catch_pos}')

show_velocity_plots = False  # Master toggle for showing velocity plots

if show_velocity_plots:
    # Create a single figure with one plot
    fig2, ax = plt.subplots(figsize=(15, 8))

    # Initial velocities - solid lines
    ax.plot(np.degrees(throw_angle), [data['v_i'] * np.cos(data['throw_angle']) for data in trajectory_data], color='red', label='$v_{i_x}$ (m/s)')
    ax.plot(np.degrees(throw_angle), [data['v_i'] * np.sin(data['throw_angle']) for data in trajectory_data], color='green', label='$v_{i_y}$ (m/s)')
    ax.plot(np.degrees(throw_angle), [data['v_i'] for data in trajectory_data], color='blue', label='$v_{i}$ (m/s)')
    
    # Final velocities - dashed lines
    ax.plot(np.degrees(throw_angle), [data['v_f'] * np.cos(data['landing_angle']) for data in trajectory_data], color='red', linestyle='--', label='$v_{f_x}$ (m/s)')
    ax.plot(np.degrees(throw_angle), [data['v_f'] * np.sin(data['landing_angle']) for data in trajectory_data], color='green', linestyle='--', label='$v_{f_y}$ (m/s)')
    ax.plot(np.degrees(throw_angle), [data['v_f'] for data in trajectory_data], color='blue', linestyle='--', label='$v_{f}$ (m/s)')
    
    # Mark the minimum initial velocity angle
    ax.axvline(np.degrees(min_velocity_data['throw_angle']), color='black', linestyle='--', label='Min $v_{i}$')
    
    ax.set_xlabel('Throw Angle (degrees)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Variation of Initial and Final Velocity Components with Throw Angle')
    ax.legend()
    ax.grid()
    
    plt.tight_layout()


plt.show()