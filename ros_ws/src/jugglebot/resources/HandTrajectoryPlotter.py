import csv
import matplotlib.pyplot as plt
import numpy as np

# Define the path to the CSV file
csv_file_path = '/home/jetson/Desktop/HandTelemetry/hand_telemetry-from-stream.csv'

# Initialize lists to store the data
timestamps = []
positions = []
velocities = []

linear_gain = 1000 / (np.pi * 10.268) # {rev/m} 10.268 is experimentally-found effective spool diameter

# Read the CSV file
with open(csv_file_path, mode='r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        timestamps.append(float(row['Timestamp']))
        positions.append(float(row['Position']))
        velocities.append(float(row['Velocity']))

# Subtract the timestamp of the first sample to make it relative
timestamps = [timestamp - timestamps[0] for timestamp in timestamps]

# Remove up to sample 28000
# positions = positions[28000:]
# velocities = velocities[28000:]

# Divide by the linear gain to convert to meters
positions = [position / linear_gain for position in positions]
velocities = [velocity / linear_gain for velocity in velocities]

# Plot the data
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

# Plot positions against timestamps
ax1.plot(timestamps, positions, label='Position', color='blue')
ax1.set_ylabel('Position')
ax1.legend()
ax1.grid(True)

# Plot velocities
ax2.plot(timestamps, velocities, label='Velocity', color='red')
ax2.set_ylabel('Velocity')
ax2.set_xlabel('Time (ms)')
ax2.legend()
ax2.grid(True)

# Set the title for the entire figure
fig.suptitle('Motor Feedback')

# Show the plot
plt.show()
