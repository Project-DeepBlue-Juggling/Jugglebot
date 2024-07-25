import csv
import matplotlib.pyplot as plt
import numpy as np

# Define the path to the CSV file
# csv_file_path = '/home/jetson/Desktop/HandTelemetry/throws_with_csv_traj_and_auto_generated_traj.csv'
csv_file_path = '/home/jetson/Desktop/hand_telemetry.csv'


# Initialize lists to store the data
time_telemetry = []
pos_telemetry = []
vel_telemetry = []
time_cmd = []
pos_cmd = []
vel_cmd = []
tor_cmd = []

linear_gain = 1000 / (np.pi * 10.268) # {rev/m} 10.268 is experimentally-found effective spool diameter

# # Read the CSV file and append the telemetry data to the lists.
# # Telemetry data continues until the blank row, after which the command data begins.
with open(csv_file_path, 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        # Append telemetry data if it's not empty
        if row['Time_telemetry']:
            time_telemetry.append(float(row['Time_telemetry']))
        if row['Position_telemetry']:
            pos_telemetry.append(float(row['Position_telemetry']))
        if row['Velocity_telemetry']:
            vel_telemetry.append(float(row['Velocity_telemetry']))

        # Append command data if it's not empty
        if row['Time_cmd']:
            time_cmd.append(float(row['Time_cmd']))
        if row['Position_cmd']:
            pos_cmd.append(float(row['Position_cmd']))
        if row['Velocity_cmd']:
            vel_cmd.append(float(row['Velocity_cmd']))
        if row['Torque_cmd']:
            tor_cmd.append(float(row['Torque_cmd']))

# Subtract the timestamp of the first sample to make it relative
time_telemetry = [timestamp - time_telemetry[0] for timestamp in time_telemetry]
time_cmd = [timestamp - time_cmd[0] for timestamp in time_cmd]

# Remove up to sample 28000
# positions = positions[28000:]
# velocities = velocities[28000:]

# Divide by the linear gain to convert to meters
pos_telemetry = [position / linear_gain for position in pos_telemetry]
vel_telemetry = [velocity / linear_gain for velocity in vel_telemetry]
pos_cmd = [position / linear_gain for position in pos_cmd]
vel_cmd = [velocity / linear_gain for velocity in vel_cmd]

# Plot the data
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

# Plot positions against timestamps
ax1.plot(time_telemetry, pos_telemetry, label='Position', color='blue')
ax1.set_ylabel('Position')
ax1.legend()
ax1.grid(True)

# Plot velocities
ax2.plot(time_telemetry, vel_telemetry, label='Velocity', color='red')
ax2.set_ylabel('Velocity')
ax2.set_xlabel('Time (ms)')
ax2.legend()
ax2.grid(True)

# Set the title for the entire figure
fig.suptitle('Motor Feedback')

# Now on a separate plot, plot the command data
fig2, (ax3, ax4, ax5) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

# Plot positions against timestamps
ax3.plot(time_cmd, pos_cmd, label='Position', color='blue')
ax3.set_ylabel('Position')
ax3.legend()
ax3.grid(True)

# Plot velocities
ax4.plot(time_cmd, vel_cmd, label='Velocity', color='red')
ax4.set_ylabel('Velocity')
ax4.legend()
ax4.grid(True)

# Plot torques
ax5.plot(time_cmd, tor_cmd, label='Torque', color='green')
ax5.set_ylabel('Torque')
ax5.set_xlabel('Time (s)')
ax5.legend()
ax5.grid(True)

# Set the title for the entire figure
fig2.suptitle('Motor Commands')

# Show the plot
plt.show()
