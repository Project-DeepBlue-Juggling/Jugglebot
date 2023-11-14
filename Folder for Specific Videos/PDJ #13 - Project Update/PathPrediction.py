# This script was used for the path prediction shown in PDJ 13. #
# It simulates a ball being thrown with a given initial position and velocity, adds
# some noise to each of the "true" datapoints (to mimic noise used in real-world sensors)
# and tests various methods of landing site estimation. This is useful as it allows for
# the testing of multiple methods in various scenarios (eg. differing sensor framerate,
# different amounts of noise etc.) quickly and easily.

# Written on 11/01/23 by Harrison Low

import math
import matplotlib.pyplot as plt
import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D

def plot_coordinates(coordinates, color='b', marker='o', show=False, label='Data'):
  # Check if coordinates is a single tuple or a list of tuples
  if not coordinates:
    return
  if not isinstance(coordinates[0], tuple):
    # If it is a single tuple, wrap it in a list
    coordinates = [coordinates]
  
  # Extract the x, y, and z coordinates
  x = [c[0] for c in coordinates]
  y = [c[1] for c in coordinates]
  z = [c[2] for c in coordinates]
  
  # Create a 3D scatter plot
  # fig = plt.figure()
  # ax = fig.add_subplot(111, projection='3d')
  ax.scatter(x, y, z, c=color, marker=marker, label=label)

  # Add labels, title and legend
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  # ax.set_title('High Noise')
  ax.legend(bbox_to_anchor=(0.3,1))

  # Show the plot
  if show:
    plt.show()


def time_to_ground(initial_position, initial_velocity):
  # Set the acceleration due to gravity
  g = 9.81

  # Calculate the time it takes for the ball to hit the ground
  t = (-initial_velocity[2] - np.sqrt(initial_velocity[2]**2 + 2*g*initial_position[2])) / (-g)

  return t


def calculate_frames(duration, fps):
  # Calculates the number of frames in the simulation based on the 
  # duration of the ball in the air and the framerate
  return int(duration * fps)


def generate_coordinates(initial_position, initial_velocity, num_points):
  # Initial position is a tuple (x0, y0, z0) representing the initial position
  # of the object
  # Initial velocity is a tuple (vx0, vy0, vz0) representing the initial velocity
  # of the object
  # Num_points is the number of points to generate

  # Constants
  g = 9.81  # Acceleration due to gravity (m/s^2)
  t = 0  # Initial time

  # Initialize the list of coordinates
  coordinates = []

  # Extract the initial position and velocity components
  x0, y0, z0 = initial_position
  vx, vy, vz = initial_velocity
  
  # print(initial_velocity)

  # Compute the time at which the object reaches the z = 0 plane
  t_landing = (vz + math.sqrt(vz**2 + 2*g*z0)) / g

  # Compute the interval between each point
  dt = t_landing / (num_points - 1)

  # Generate the coordinates by iterating over the time steps
  for i in range(num_points):
    t = i * dt
    x = x0 + vx * t
    y = y0 + vy * t
    z = z0 + vz * t - 0.5 * g * t**2
    coordinates.append((x, y, z))

  return coordinates, dt


def predict_path(coordinates, finish_height=0, timestep=0.1):
  # Coordinates is a list of 3D coordinates (x, y, z) representing the path of the object so far
  # Finish_height is the height at which the prediction should end (in meters)
  
  # Constants
  g = 9.81  # Acceleration due to gravity (m/s^2)

  # Extract the x, y, and z coordinates
  x = [c[0] for c in coordinates]
  y = [c[1] for c in coordinates]
  z = [c[2] for c in coordinates]

  # Compute the time for each point in the path
  t = [i * timestep for i in range(len(coordinates))]

  # Fit a quadratic curve to the x, y, and z coordinates
  x_coeffs = np.polyfit(t, x, 1)
  y_coeffs = np.polyfit(t, y, 1)
  z_coeffs = np.polyfit(t, z, 2)
  # print("x coeff: ", x_coeffs)
  # print("y coeff: ", y_coeffs)
  # print("z coeff: ", z_coeffs)
  # print()

  # Solve for the time at which the object reaches the desired height
  discriminant = z_coeffs[1]**2 - 4 * z_coeffs[0] * (z_coeffs[2] - finish_height)
  
  if discriminant < 0:
    # No solution
    print("The object does not reach the desired height within the predicted time.")
    return [], []
  else:
    t_finish = (-z_coeffs[1] - math.sqrt(discriminant)) / (2 * z_coeffs[0])
    # print("t finish: ", t_finish)
    if t_finish < 0:
      t_finish = (-z_coeffs[1] + math.sqrt(discriminant)) / (2 * z_coeffs[0])
      # print("t finish, 2: ", t_finish)
    # print("discriminant: ", discriminant)
    
  # Predict the rest of the path up to the desired height
  t_pred = [i * timestep for i in range(len(coordinates), int(t_finish / timestep) + 1)]
  x_pred = np.polyval(x_coeffs, t_pred)
  y_pred = np.polyval(y_coeffs, t_pred)
  z_pred = np.polyval(z_coeffs, t_pred)
  
  if math.isinf(t_finish) or math.isnan(t_finish):
    x_finish = x_pred[-1]
    y_finish = y_pred[-1]
    
  else:
    # Find the x and y coordinates at the time when the ball passes the z = 0 plane
    x_finish = np.polyval(x_coeffs, t_finish)
    y_finish = np.polyval(y_coeffs, t_finish)
  
  # Return the predicted coordinates
  return list(zip(x_pred, y_pred, z_pred)), (x_finish, y_finish, 0)


def add_noise(coordinates, noise_percent = 0.5, noise_val = 0.01):
  # Coordinates is a list of 3D coordinates (x, y, z)
  # Noise_percent is the percentage of points that should have noise added to them
  
  # Add noise to a random subset of the points
  noisy_coordinates = list(coordinates)
  num_points = len(noisy_coordinates)
  num_noisy = int(num_points * noise_percent)
  noisy_points = random.sample(range(num_points), num_noisy)
  
  # Add a random value between -noise_val and noise_val to the x, y, and z coordinates of the noisy points
  for i in noisy_points:
    x, y, z = noisy_coordinates[i]
    noisy_coordinates[i] = (x + random.uniform(-noise_val, noise_val),
                     y + random.uniform(-noise_val, noise_val),
                     z + random.uniform(-noise_val, noise_val))
  
  return noisy_coordinates


def distance_between_points(point1, point2):
  # point1 and point2 are tuples containing the coordinates of the points (x1, y1, z1) and (x2, y2, z2)
  x1, y1, z1 = point1
  x2, y2, z2 = point2
  
  # Compute the distance between the points using the Euclidean distance formula
  distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
  
  return distance


def moving_average(coordinates, k):
  # coordinates is a list of 3D coordinates (x, y, z)
  # k is the size of the moving average window (number of points to average over)

  # Initialize the list of filtered coordinates
  filtered_coordinates = []

  # Iterate over the coordinates
  for i in range(len(coordinates)):
    # Compute the start and end indices of the window
    start = max(0, i - k)
    end = min(len(coordinates), i + k + 1)

    # Average the values within the window
    window = coordinates[start:end]
    x_avg = sum([x for x, y, z in window]) / len(window)
    y_avg = sum([y for x, y, z in window]) / len(window)
    z_avg = sum([z for x, y, z in window]) / len(window)

    # Add the averaged point to the list of filtered coordinates
    filtered_coordinates.append((x_avg, y_avg, z_avg))

  return filtered_coordinates

  
def kalman_filter(coordinates, dt):
  # coordinates is a list of 3D coordinates (x, y, z) representing the position of the ball

  g = 9.81
  # Initialize the state vector (position and velocity in x, y, and z)
  x = np.zeros((6, 1))
  x = np.array([[0], [0], [0], [1], [0], [4]])

  # Initialize the transition matrix (constant acceleration model)
  A = np.array([[1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])
  
  B = np.array([[0, 0, 0],
                [0, 0, 0],
                [0, 0, 0.5*dt**2],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, dt]])
  
  u = np.array([[0], [0], [-g]])

  # Initialize the measurement matrix (measure position only)
  H = np.array([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0]])

  # Initialize the process and measurement noise covariance matrices
  noise_measurement = 0.1 # Should be expected noise of measurements squared
  noise_process = noise_measurement / 100 # Experiment with divisor here.
  Q = np.array([[noise_process, 0, 0, 0, 0, 0],
                [0, noise_process, 0, 0, 0, 0],
                [0, 0, noise_process, 0, 0, 0],
                [0, 0, 0, noise_process, 0, 0],
                [0, 0, 0, 0, noise_process, 0],
                [0, 0, 0, 0, 0, noise_process]])

                
  R = np.array([[noise_measurement, 0, 0],
                [0, noise_measurement, 0],
                [0, 0, noise_measurement]])
                
  P = np.zeros((6, 6))

  # Initialize the Kalman gain
  K = np.zeros((6, 3))

  # Initialize the list of filtered coordinates
  filtered_coordinates = []

  # Iterate over the coordinates
  for i in range(len(coordinates)):
    # Get the current measurement
    z = np.array(coordinates[i]).reshape((3, 1))

    # Predict the next state using the transition matrix and the current state
    x_pred = np.dot(A, x) + np.dot(B, u)

    # Predict the process noise covariance
    P = np.dot(A, np.dot(Q, A.T)) + Q
    # print(Q)

    # Compute the Kalman gain
    K = np.dot(P, np.dot(H.T, np.linalg.inv(np.dot(H, np.dot(P, H.T)) + R)))

    # Update the state estimate using the Kalman gain and the measurement
    x = x_pred + np.dot(K, (z - np.dot(H, x_pred)))

    # Update the process noise covariance
    P = (np.eye(6) - np.dot(K, H)) * P
    # print(x[:3])

    # Add the filtered point to the list of filtered coordinates
    filtered_coordinates.append(tuple(x[:3].flatten()))

  return filtered_coordinates


def predict_landing(ideal_path, noisy_path, prediction_functions, args, colors):
  # Initialize a list to store the predictions for each point in the noisy path
  errors = []

  plot_coordinates(ideal_path, label="Actual Path", color='r', marker='x')
  plot_coordinates(noisy_path, label="Noisy Path", color='m')  
  num_pre_plots = len(plt.gca().collections)

  # Iterate over the points in the noisy path
  for i in range(5, len(noisy_path)):
    path_prediction = {}
    landing_prediction = {}
    error = {}
    
    if len(plt.gca().collections) > num_pre_plots: 
        if i < len(noisy_path):
          for data in range(num_pre_plots,len(plt.gca().collections)):
            ax.collections[num_pre_plots].remove()
    
    # Feed the current point to each prediction function and store the result
    for function in prediction_functions:
      filtered_coords = function(noisy_path[:i], args[function.__name__])
      
      path_prediction[function.__name__], landing_prediction[function.__name__] = predict_path(filtered_coords, timestep=timestep)
      
      if len(path_prediction[function.__name__]) > 0:
        error[function.__name__] = distance_between_points(ideal_path[-1], landing_prediction[function.__name__])
      else:
        error[function.__name__] = np.nan
      
      # Add the point predictions to the list of predictions
      errors.append(error)
      
      plot_coordinates(path_prediction[function.__name__], 
                       label=function.__name__+' - prediction',
                       color=colors[function.__name__][1]
                       )
      
      plot_coordinates(landing_prediction[function.__name__], 
                       label=function.__name__+' - prediction',
                       color=colors[function.__name__][1]
                       )
                       
      plot_coordinates(filtered_coords, 
                       label=function.__name__, 
                       color=colors[function.__name__][0]
                       )
                       
    # print(error)
    plt.show(block=False)
    plt.pause(0.01)

  return errors


def plot_errors(error_data):
  fig2 = plt.figure()
  ax2 = fig2.add_subplot(111)
  ax2.set_title("Error for each estimation method")
  ax2.set_xlabel("Datapoint")
  ax2.set_ylabel("Distance between predicted and actual landing sites (m)")
  ax2.grid(True)
  ax2.set_ylim(0, 0.2)

  # Initialize an empty list to store the labels
  labels = []

  count = 0
  for data_point in errors:
    for label, value in data_point.items():
      # Add the label to the list if it is not already in the list
      if label not in labels:
        labels.append(label)
        
      plt.plot(count, value, label=label, marker='o', color=colors[label][0])
    count += 1
    plt.legend(labels)

  plt.show()


def linear_weighting(values):
  weighted_values = []
  max_value = len(values)

  for i, value in enumerate(values):
    weighted_values.append(value * (max_value - i) / max_value)
    
  return weighted_values 


def compare_methods(data_points, weighting_function):
  # Initialize a dictionary to hold the data for each method
  methods = {}
  
  # Iterate over the data points
  for data_point in data_points:
    # Iterate over the labels and values in the data point
    for label, value in data_point.items():
      # If the label is not in the methods dictionary, add it and initialize the value to be a list
      if label not in methods:
        methods[label] = []
      # If the value is not nan, add it to the list of data for the relevant method
      if not np.isnan(value):
        methods[label].append(value)
  
  # Initialize a dictionary to hold the error of each method
  errors = {}
  
  # Iterate over the labels and data in the methods dictionary
  for label, data in methods.items():
    # Apply the weighting function to the data
    weighted_data = weighting_function(data)
    # Calculate the mean squared error of the weighted data
    mse = np.mean((np.array(weighted_data) - np.mean(weighted_data))**2)
    # Add the error to the errors dictionary
    errors[label] = mse
  
  return errors


# Create the figure and axes objects
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(0,1)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(0,1)

initial_position = (0, 0, 0)  # Initial position at the origin
initial_velocity = (1, 0, 4)  # Initial velocity in the x, y, and z directions
frames_per_second = 100 # Frames per second to sample data at
noise_val = 0.1 # Maximum amount of noise to add to the ideal path (m)


time = time_to_ground(initial_position, initial_velocity)
num_points = calculate_frames(time, frames_per_second)
print(num_points)

path_actual, timestep = generate_coordinates(initial_position, initial_velocity, num_points)

# Add some noise to the prediction points
path_noisy = add_noise(path_actual, noise_percent=1, noise_val=noise_val)

# Prepare for the prediction function
prediction_functions = [moving_average, kalman_filter]
# prediction_functions = [moving_average]

args = {
  "moving_average": 2, # Size of window
  "kalman_filter":timestep
  }
  
colors = {
    "moving_average": ['blue', 'cornflowerblue'],
    "kalman_filter": ['darkgreen', 'mediumspringgreen']
    }

# print(len(path_actual))
# plot_coordinates(path_actual)
# plt.show()

errors = predict_landing(path_actual, path_noisy, prediction_functions, args, colors)

scores = compare_methods(errors, linear_weighting)
print(scores)

plot_errors(errors)
