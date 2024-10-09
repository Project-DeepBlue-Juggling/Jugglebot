import csv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Define the path to the CSV file
# csv_file_path = '/home/jetson/Desktop/HandTelemetry/throws_with_csv_traj_and_auto_generated_traj.csv'
csv_file_path = '/home/jetson/Desktop/telemetry.csv'

# Find the row when the leg data begins (first cell of this row will always contain `Leg1_cmd`)
leg_data_row = None
with open(csv_file_path, 'r') as file:
    reader = csv.reader(file)
    for i, row in enumerate(reader):
        if row[0] == 'Leg1_cmd':
            print(f'Found the row at index {i}')
            leg_data_row = i
            break

# Initialize lists to store the data
hand_time_telemetry = []
hand_pos_telemetry = []
hand_vel_telemetry = []
hand_time_cmd = []
hand_pos_cmd = []
hand_vel_cmd = []
hand_tor_cmd = []

leg_cmd = []
leg_telemetry = []

''' HAND DATA '''

linear_gain = 1000 / (np.pi * 10.268) # {rev/m} 10.268 is experimentally-found effective spool diameter

# Read the CSV file and append the telemetry data to the lists. Only read up until the leg data begins.
with open(csv_file_path, 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        # Break if the row is the leg data row
        if reader.line_num == leg_data_row:
            break

        # Append telemetry data if it's not empty
        if row['Hand_time_telemetry']:
            hand_time_telemetry.append(float(row['Hand_time_telemetry']))
        if row['Hand_position_telemetry']:
            hand_pos_telemetry.append(float(row['Hand_position_telemetry']))
        if row['Hand_velocity_telemetry']:
            hand_vel_telemetry.append(float(row['Hand_velocity_telemetry']))

        # Append command data if it's not empty
        if row['Hand_time_cmd']:
            hand_time_cmd.append(float(row['Hand_time_cmd']))
        if row['Hand_position_cmd']:
            hand_pos_cmd.append(float(row['Hand_position_cmd']))
        if row['Hand_velocity_cmd']:
            hand_vel_cmd.append(float(row['Hand_velocity_cmd']))
        if row['Hand_torque_cmd']:
            hand_tor_cmd.append(float(row['Hand_torque_cmd']))

# Subtract the timestamp of the first sample to make it relative
hand_time_telemetry = [timestamp - hand_time_telemetry[0] for timestamp in hand_time_telemetry]
hand_time_cmd = [timestamp - hand_time_cmd[0] for timestamp in hand_time_cmd]

# Remove up to sample 28000
# positions = positions[28000:]
# velocities = velocities[28000:]

# Divide by the linear gain to convert to meters
hand_pos_telemetry = [position / linear_gain for position in hand_pos_telemetry]
hand_vel_telemetry = [velocity / linear_gain for velocity in hand_vel_telemetry]
hand_pos_cmd = [position / linear_gain for position in hand_pos_cmd]
hand_vel_cmd = [velocity / linear_gain for velocity in hand_vel_cmd]

# Plot the data
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

# Plot positions against timestamps
ax1.plot(hand_time_telemetry, hand_pos_telemetry, label='Position', color='blue')
ax1.set_ylabel('Position')
ax1.legend()
ax1.grid(True)

# Plot velocities
ax2.plot(hand_time_telemetry, hand_vel_telemetry, label='Velocity', color='red')
ax2.set_ylabel('Velocity')
ax2.set_xlabel('Time (ms)')
ax2.legend()
ax2.grid(True)

# Set the title for the entire figure
fig.suptitle('Motor Feedback')

# Now on a separate plot, plot the command data
fig2, (ax3, ax4, ax5) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

# Plot positions against timestamps
ax3.plot(hand_time_cmd, hand_pos_cmd, label='Position', color='blue')
ax3.set_ylabel('Position')
ax3.legend()
ax3.grid(True)

# Plot velocities
ax4.plot(hand_time_cmd, hand_vel_cmd, label='Velocity', color='red')
ax4.set_ylabel('Velocity')
ax4.legend()
ax4.grid(True)

# Plot torques
ax5.plot(hand_time_cmd, hand_tor_cmd, label='Torque', color='green')
ax5.set_ylabel('Torque')
ax5.set_xlabel('Time (s)')
ax5.legend()
ax5.grid(True)

# Set the title for the entire figure
fig2.suptitle('Motor Commands')


''' LEG DATA '''

# Read the CSV file and append the telemetry data to the lists. Only look at rows after leg_data_row
# Note that the headings for the leg data are `Leg1_cmd`, `Leg2_cmd`, ..., `Leg6_cmd`, `Leg1_telemetry`, `Leg2_telemetry`, etc.
# And that there's no timestamp associated with this data

df = pd.read_csv(csv_file_path, skiprows=leg_data_row)
leg_cmd = df.filter(regex='Leg\d_cmd').values
leg_telemetry = df.filter(regex='Leg\d_telemetry').values

# Plot the data. For now, plot only the telemtry data, showing all 6 legs on the same plot
# fig3, (ax6, ax7) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
fig3, ax6 = plt.subplots(1, 1, figsize=(10, 8))

# Plot positions against timestamps
for i in range(6):
    ax6.plot(leg_telemetry[:, i], label=f'Leg {i+1}')
ax6.set_ylabel('Position')
ax6.legend()
ax6.grid(True)




# Show the plots
plt.show()
