'''This script is used to load and analyse the results from the platform angle calibration sequence.
The sequence runs the platform through a series of poses, and records the platform's orientation at each pose.

This script plots the measured and commanded orientations at each position in a quiver plot
and calculates the error between the two.

Note that the tilt is as measured from the vertical axis, so a tilt of 0 degrees means the platform is vertical.'''

import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# Define the path to the CSV file
csv_file_path = '/home/jetson/Desktop/angle_calibration_data.csv'

# Initialize lists to store the data
positions = []
measured_orientations = []
commanded_orientations = []

# Read the CSV file and append the data to the lists
with open(csv_file_path, 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        positions.append([float(row['Position X (mm)']), float(row['Position Y (mm)'])])
        measured_orientations.append([float(row['Measured Tilt X (deg)']), float(row['Measured Tilt Y (deg)'])])
        commanded_orientations.append([float(row['Commanded Tilt X (deg)']), float(row['Commanded Tilt Y (deg)'])])

# Convert the lists to numpy arrays
positions = np.array(positions)
measured_orientations = np.array(measured_orientations)
commanded_orientations = np.array(commanded_orientations)

def euler_to_vector(roll, pitch):
    '''Converts the Euler angles to a unit vector pointing in the direction of the angles.'''
    # Convert the Euler angles to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)

    # Convert the Euler angles to a rotation matrix
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    # Combined rotation matrix
    R = Ry @ Rx

    # Calculate the unit vector pointing in the direction of the Euler angles
    vector = R @ np.array([0, 0, 1])

    return vector

def difference_between_angles(measured_angles, commanded_angles):
    '''Calculates the difference between two sets of Euler angles.'''
    # Convert the Euler angles to vectors
    measured_vector = euler_to_vector(measured_angles[0], measured_angles[1])
    commanded_vector = euler_to_vector(commanded_angles[0], commanded_angles[1])

    # Calculate the angle between the vectors
    angle = np.arccos(np.dot(measured_vector, commanded_vector) / (np.linalg.norm(measured_vector) * np.linalg.norm(commanded_vector)))

    return np.degrees(angle)

def multiply_error(measured_angles, commanded_angles, factor=1.0):
    '''Measures the error between measured and commanded angles then multiplies this error by
    the nominated factor before returning the multiplied measured angles
    
    This is useful for plotting, to emphasize the discrepancy between commanded and measured angles'''

    error_x = measured_angles[0] - commanded_angles[0]
    error_y = measured_angles[1] - commanded_angles[1]

    print(f'Error x: {error_x:.2f}, Error y: {error_y:.2f}')

    # Multiply the error by the chosen factor
    error_x *= factor
    error_y *= factor

    # Construct the multiplied "measured" angles
    multiplied_angle_x = commanded_angles[0] + error_x
    multiplied_angle_y = commanded_angles[1] + error_y

    # Return the result
    return [multiplied_angle_x, multiplied_angle_y]


# Plot the results on a 3D quiver plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

arrow_length = 250
multiplication_factor = 10

for i in range(len(positions)):
    # print(f'Position: {positions[i]}')
    print(f'Measured orientation: {measured_orientations[i]}')
    print(f'Commanded orientation: {commanded_orientations[i]}')

    # Calculate the error between the measured and commanded orientations
    error = difference_between_angles(measured_orientations[i], commanded_orientations[i])
    print(f'Error: {error} degrees')

    multiplied_angles = multiply_error(measured_orientations[i], commanded_orientations[i], factor=multiplication_factor)

    # Calculate the direction of the arrows
    dx_meas, dy_meas, dz_meas = euler_to_vector(measured_orientations[i, 0], measured_orientations[i, 1])
    dx_cmd, dy_cmd, dz_cmd = euler_to_vector(commanded_orientations[i, 0], commanded_orientations[i, 1])
    dx_mult, dy_mult, dz_mult = euler_to_vector(multiplied_angles[0], multiplied_angles[1])

    print(f'Measured vector: {dx_meas, dy_meas, dz_meas}')

    # Plot the point
    ax.scatter(positions[i, 0], positions[i, 1], 0, color='b')

    # Plot the arrows
    ax.quiver(positions[i, 0], positions[i, 1], 0, dx_meas, dy_meas, dz_meas, length=arrow_length, color='r')
    ax.quiver(positions[i, 0], positions[i, 1], 0, dx_cmd, dy_cmd, dz_cmd, length=arrow_length, color='g')
    # ax.quiver(positions[i, 0], positions[i, 1], 0, dx_mult, dy_mult, dz_mult, length=arrow_length, color='orange')


ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_title('Platform Orientation Calibration')

# Set the axis limits to be 350 mm
ax.set_xlim(-350, 350)
ax.set_ylim(-350, 350)
ax.set_zlim(-350, 350)

# Remove the tick marks on the z axis
ax.set_zticks([])

plt.show()
    





