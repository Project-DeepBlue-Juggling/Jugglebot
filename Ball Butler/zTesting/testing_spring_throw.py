import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Constants
ball_mass = 0.25  # kg
throw_height = 0.0  # meters (starting height, assuming throw from ground level)
spring_resting_length = 0.05  # meters
gravity = 9.806  # m/s^2
initial_target_height = 2.0  # meters (initial peak height we want to reach)

def calculate_spring_extension(k, ball_mass, target_height, angle_from_vertical):
    """Calculate required spring extension to achieve throw parameters."""
    # Convert angle from vertical to angle from horizontal
    throw_angle = 90 - angle_from_vertical
    cos_angle = math.cos(math.radians(throw_angle))
    sin_angle = math.sin(math.radians(throw_angle))
    
    # Calculate extension to reach target height at given angle
    extension = math.sqrt(2 * ball_mass * gravity * target_height / (k * sin_angle**2))
    return extension

def calculate_velocity(k, extension, ball_mass):
    """Calculate initial velocity from spring extension."""
    # Energy stored in spring = 0.5 * k * x^2
    # This energy converts to kinetic energy = 0.5 * m * v^2
    v = math.sqrt(k * extension**2 / ball_mass)
    return v

def calculate_trajectory(v0, angle_from_vertical, steps=100, time=None):
    """Calculate the trajectory of the projectile."""
    # Convert angle from vertical to angle from horizontal
    angle = 90 - angle_from_vertical
    angle_rad = math.radians(angle)
    v0x = v0 * math.cos(angle_rad)
    v0y = v0 * math.sin(angle_rad)
    
    if time is None:
        # Calculate time of flight (time to return to initial height)
        # t = (2 * v0y) / g
        time = 2 * v0y / gravity
    
    t = np.linspace(0, time, steps)
    x = v0x * t
    y = v0y * t - 0.5 * gravity * t**2 + throw_height
    
    return x, y, time

# Initialize the figure
fig, ax = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(left=0.25, bottom=0.4)  # Made more room for the new slider

# Initial parameter values
k_value = 100  # Spring constant (N/m)
angle_value = 0  # Throw angle from vertical (degrees)
target_height = initial_target_height  # Target height (meters)

# Calculate initial values
extension = calculate_spring_extension(k_value, ball_mass, target_height, angle_value)
velocity = calculate_velocity(k_value, extension, ball_mass)
x_traj, y_traj, flight_time = calculate_trajectory(velocity, angle_value)

# Create initial trajectory plot
trajectory_line, = ax.plot(x_traj, y_traj, 'b-', lw=2, label='Ball Trajectory')
peak_point, = ax.plot([], [], 'ro', markersize=8, label='Peak Height')

# Target height line (to be updated by slider)
target_line = ax.axhline(y=target_height, color='g', linestyle='--', label=f'Target Height: {target_height}m')

# Set up axis labels and title
ax.set_xlabel('Horizontal Distance (m)')
ax.set_ylabel('Height (m)')
ax.set_title('Ball Trajectory')
ax.grid(True)
ax.set_ylim(0, target_height * 1.5)
ax.set_aspect('equal')  # Set axes to be equal

# Add info text
info_text = fig.text(0.5, 0.95, '', 
                     horizontalalignment='center',
                     verticalalignment='center',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7),
                     transform=fig.transFigure)

plt.subplots_adjust(left=0.25, bottom=0.4, top=0.85)

# Add sliders
ax_k = plt.axes([0.25, 0.20, 0.65, 0.03])
slider_k = Slider(ax_k, 'Spring Constant (N/m)', 10, 500, valinit=k_value, valstep=10)

ax_angle = plt.axes([0.25, 0.15, 0.65, 0.03])
slider_angle = Slider(ax_angle, 'Angle from Vertical (degrees)', 0, 40, valinit=angle_value, valstep=1)

# Add new slider for target height
ax_height = plt.axes([0.25, 0.10, 0.65, 0.03])
slider_height = Slider(ax_height, 'Target Height (m)', 0.5, 5.0, valinit=target_height, valstep=0.1)

def update_plot(k=None, angle=None, height=None):
    """Update trajectory plot with new parameters."""
    k = k if k is not None else slider_k.val
    angle = angle if angle is not None else slider_angle.val
    height = height if height is not None else slider_height.val
    
    # Update target height line
    target_line.set_ydata([height, height])
    target_line.set_label(f'Target Height: {height}m')
    
    # Calculate new values
    extension = calculate_spring_extension(k, ball_mass, height, angle)
    velocity = calculate_velocity(k, extension, ball_mass)
    x_traj, y_traj, flight_time = calculate_trajectory(velocity, angle)
    
    # Update trajectory
    trajectory_line.set_xdata(x_traj)
    trajectory_line.set_ydata(y_traj)
    
    # Find and mark peak height
    max_height_idx = np.argmax(y_traj)
    peak_point.set_xdata([x_traj[max_height_idx]])
    peak_point.set_ydata([y_traj[max_height_idx]])
    
    # Update info text
    info_text.set_text(f'Required Spring Extension: {extension:.3f}m\n'
                       f'Initial Velocity: {velocity:.2f}m/s\n'
                       f'Range: {x_traj[-1]:.2f}m')
    
    # Update axis limits
    ax.set_xlim(0, x_traj[-1] * 1.1)
    ax.set_ylim(0, max(height * 1.2, y_traj[max_height_idx] * 1.2))

    # Calculate peak spring force
    peak_spring_force = k * extension
    info_text.set_text(info_text.get_text() + f'\nPeak Spring Force: {peak_spring_force:.2f} N')
    
    # Ensure axes remain equal
    ax.set_aspect('equal')
    
    # Update legend
    # ax.legend(loc='upper right')
    
    fig.canvas.draw_idle()

def update_k_from_slider(val):
    """Update when spring constant changes via slider."""
    update_plot(k=val)

def update_angle_from_slider(val):
    """Update when throw angle changes via slider."""
    update_plot(angle=val)

def update_height_from_slider(val):
    """Update when target height changes via slider."""
    update_plot(height=val)

# Connect the callbacks
slider_k.on_changed(update_k_from_slider)
slider_angle.on_changed(update_angle_from_slider)
slider_height.on_changed(update_height_from_slider)

# Add a legend
# ax.legend(loc='upper right')

# Initial update
update_plot()

# Add a second figure for spring compression vs spring constant
fig_spring, ax_spring = plt.subplots(figsize=(10, 5))
plt.subplots_adjust(left=0.15, right=0.95, top=0.9, bottom=0.15)

# Spring constant range for plotting
spring_constants = np.linspace(10, 500, 100)
extensions = [calculate_spring_extension(k, ball_mass, target_height, angle_value) for k in spring_constants]

# Create the plot
spring_plot, = ax_spring.plot(spring_constants, extensions, 'r-', lw=2)
current_point, = ax_spring.plot(k_value, extensions[0], 'bo', markersize=8)

# Labels and styling
ax_spring.set_xlabel('Spring Constant (N/m)')
ax_spring.set_ylabel('Required Spring Extension (m)')
ax_spring.set_title('Spring Extension vs Spring Constant')
ax_spring.grid(True)

# Add vertical line for current k
current_k_line = ax_spring.axvline(x=k_value, color='g', linestyle='--')

# Update the spring plot when parameters change
def update_spring_plot():
    # Calculate new extensions for all spring constants
    new_extensions = [calculate_spring_extension(k, ball_mass, slider_height.val, slider_angle.val) for k in spring_constants]
    spring_plot.set_ydata(new_extensions)
    
    # Update current point and vertical line
    current_k = slider_k.val
    current_ext = calculate_spring_extension(current_k, ball_mass, slider_height.val, slider_angle.val)
    current_point.set_xdata([current_k])
    current_point.set_ydata([current_ext])
    current_k_line.set_xdata([current_k, current_k])
    
    # Update y-axis limits if needed
    ax_spring.set_ylim(0, max(new_extensions) * 1.1)
    fig_spring.canvas.draw_idle()

# Update original callbacks to also update the spring plot
def update_k_from_slider(val):
    update_plot(k=val)
    update_spring_plot()

def update_angle_from_slider(val):
    update_plot(angle=val)
    update_spring_plot()

def update_height_from_slider(val):
    update_plot(height=val)
    update_spring_plot()

# Reconnect the callbacks
slider_k.on_changed(update_k_from_slider)
slider_angle.on_changed(update_angle_from_slider)
slider_height.on_changed(update_height_from_slider)

# Initial update for the spring plot
update_spring_plot()

plt.show()

