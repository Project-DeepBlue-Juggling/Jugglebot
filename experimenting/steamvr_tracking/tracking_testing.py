## 2D:

import triad_openvr as vr
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import numpy as np

def calculate_relative_position(origin_pos, origin_quat, tracker_pos, tracker_quat):
    """
    Calculate the tracker's position relative to the origin.
    Both positions and quaternions are given as lists or numpy arrays.
    """
    # Convert lists/tuples to numpy arrays if they aren't already
    origin_pos = np.array(origin_pos)
    tracker_pos = np.array(tracker_pos)
    origin_quat = np.quaternion(*origin_quat)  # The * operator unpacks the quaternion components
    tracker_quat = np.quaternion(*tracker_quat)

    # Calculate the inverse of the origin's rotation
    inverse_origin_rotation = origin_quat.inverse()

    # Convert the tracker's position to a pure quaternion
    tracker_pos_quat = np.quaternion(0.0, *tracker_pos)

    # Apply the inverse rotation of the origin to the tracker's position
    tracker_pos_rotated_quat = inverse_origin_rotation * tracker_pos_quat * inverse_origin_rotation.conjugate()

    # Extract the vector part of the resulting quaternion
    tracker_pos_rotated = np.array([tracker_pos_rotated_quat.x, tracker_pos_rotated_quat.y, tracker_pos_rotated_quat.z])

    # Subtract the origin's position from the rotated tracker's position to get the final relative position
    relative_position = tracker_pos_rotated - origin_pos

    return relative_position

def get_pose_and_quaternion(sample):
    # Gets the pose and quaternion values out of the given sample and puts them in a nice format

    pos = [sample.x[0], sample.y[0], sample.z[0]]
    quat = [sample.r_w[0], sample.r_x[0], sample.r_y[0], sample.r_z[0]]

    return pos, quat

# Initialize VR interface
v = vr.triad_openvr()

# Print the available devices
v.print_discovered_objects()

t1 = v.devices["tracker_1"]
t2 = v.devices["tracker_2"]

# Create a new figure and plot
fig, ax = plt.subplots()
xdata, ydata_x, ydata_y, ydata_z = [], [], [], []
ln_x, = plt.plot([], [], 'r-', label='X', animated=True)
ln_y, = plt.plot([], [], 'g-', label='Y', animated=True)
ln_z, = plt.plot([], [], 'b-', label='Z', animated=True)

def init():
    ax.set_xlim(0, 2)  # Set initial x-axis limits
    ax.set_ylim(-2, 2)  # Set y-axis limits; adjust as needed
    ax.legend()
    return ln_x, ln_y, ln_z,

def update(frame):
    tracker = t1.sample(1, 250)
    origin = t2.sample(1, 250)

    tracker_pos, tracker_quaternion = get_pose_and_quaternion(tracker)
    origin_pos, origin_quaternion = get_pose_and_quaternion(origin)

    relative_pos = calculate_relative_position(origin_pos, origin_quaternion, tracker_pos, tracker_quaternion)

    xdata.append(time.time())  # Use the current time for the x-axis
    ydata_x.append(relative_pos[0])  # Tracker's relative X position
    ydata_y.append(relative_pos[1])  # Tracker's relative Y position
    ydata_z.append(relative_pos[2])  # Tracker's relative Z position

    # Remove the oldest data points after a certain limit to avoid excessive memory usage
    if len(xdata) > 50:
        xdata.pop(0)
        ydata_x.pop(0)
        ydata_y.pop(0)
        ydata_z.pop(0)

    ln_x.set_data(xdata, ydata_x)
    ln_y.set_data(xdata, ydata_y)
    ln_z.set_data(xdata, ydata_z)

    ax.set_xlim(xdata[0], xdata[-1])  # Update x-axis limits to current time frame

    return ln_x, ln_y, ln_z,

ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=50)

plt.show()

# while True:
#     tracker = t1.sample(1, 250)
#     origin = t2.sample(1, 250)

#     tracker_pos, tracker_quaternion = get_pose_and_quaternion(tracker)
#     origin_pos, origin_quaternion = get_pose_and_quaternion(origin)

#     relative_pos = calculate_relative_position(origin_pos, origin_quaternion, tracker_pos, tracker_quaternion)

#     print(relative_pos)

#     time.sleep(0.1)

# # Create figure for plotting
# fig, ax = plt.subplots()
# xdata, ydata = [], []
# ln, = plt.plot([], [], 'r-', animated=True)

# def init():
#     ax.set_xlim(0, 10)  # Set the x-axis limits (you may need to adjust these)
#     ax.set_ylim(-2, 2)  # Set the y-axis limits (you may need to adjust these)
#     return ln,

# def update(frame):
#     data = t1.sample(1, 250)
#     xdata.append(time.time())  # Use time for x-axis
#     ydata.append(data.x)  # Tracker's x position for y-axis
#     ln.set_data(xdata, ydata)
#     ax.set_xlim(xdata[0], xdata[-1])  # Update x-axis limits to current time frame
#     return ln,

# ani = FuncAnimation(fig, update, frames=None, init_func=init, blit=True, interval=50)

# plt.show()


## 3D:

# import triad_openvr as vr
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from mpl_toolkits.mplot3d import Axes3D
# from collections import deque
# import time

# # Initialize VR interface
# v = vr.triad_openvr()

# # Create a figure for plotting and configure 3D axis
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Use a deque for efficient FIFO data handling
# trail_length = 50  # Number of samples in the trail
# xdata = deque(maxlen=trail_length)
# ydata = deque(maxlen=trail_length)
# zdata = deque(maxlen=trail_length)

# def update(frame):
#     data = v.devices["tracker_1"].sample(1, 250)
#     xdata.append(data.x)
#     ydata.append(data.y)
#     zdata.append(data.z)

#     ax.clear()  # Clear the previous plot

#     # Set the axis limits; you might need to adjust these based on your tracking area
#     ax.set_xlim(-2, 2)
#     ax.set_ylim(-2, 2)
#     ax.set_zlim(-2, 2)

#     # Plot the trail of last 50 samples
#     ax.plot(list(xdata), list(ydata), list(zdata), color='blue')  # Trail as a blue line
#     ax.scatter([data.x], [data.y], [data.z], color='red')  # Current position as a red dot

#     # Reset the view angles if you want to maintain a specific perspective
#     # ax.view_init(elev=20., azim=30)  # Uncomment if specific view angle is needed

# ani = FuncAnimation(fig, update, frames=range(100), interval=50)  # Using a range for frames

# plt.show()






# import triad_openvr as vr
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from mpl_toolkits.mplot3d import Axes3D
# from collections import deque
# import numpy as np
# import time

# def euler_to_mat(z=0, y=0, x=0):
#     # Converts euler angles to rotation matrix
#     # Note: z, y, x are yaw, pitch, roll respectively
#     z = np.radians(z)
#     y = np.radians(y)
#     x = np.radians(x)

#     cz = np.cos(z)
#     sz = np.sin(z)
#     cy = np.cos(y)
#     sy = np.sin(y)
#     cx = np.cos(x)
#     sx = np.sin(x)

#     cxcz = cx * cz
#     cxsz = cx * sz
#     sxcz = sx * cz
#     sxsz = sx * sz

#     mat = np.array([
#            [cy * cz, sy * sxcz - cxsz, sy * cxcz + sxsz],
#            [cy * sz, sy * sxsz + cxcz, sy * cxsz - sxcz],
#            [-sy, cy * sx, cy * cx]
#         ])

#     return mat

# # Initialize VR interface
# v = vr.triad_openvr()

# # Create a figure for plotting and configure 3D axis
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Use a deque for efficient FIFO data handling
# trail_length = 50  # Number of samples in the trail
# xdata = deque(maxlen=trail_length)
# ydata = deque(maxlen=trail_length)
# zdata = deque(maxlen=trail_length)

# def update(frame):
#     origin_data = v.devices["tracker_1"].sample(1, 250)
#     tracker_data = v.devices["tracker_2"].sample(1, 250)

#     # Calculate relative position
#     rel_x = tracker_data.x[0] - origin_data.x[0]
#     rel_y = tracker_data.y[0] - origin_data.y[0]
#     rel_z = tracker_data.z[0] - origin_data.z[0]

#     # Construct rotation matrix from the origin's orientation
#     rot_mat = euler_to_mat(-origin_data.yaw[0], -origin_data.pitch[0], -origin_data.roll[0])

#     # Apply rotation matrix to relative position
#     rotated_pos = np.dot(rot_mat, np.array([rel_x, rel_y, rel_z]))

#     xdata.append(rotated_pos[0])
#     ydata.append(rotated_pos[1])
#     zdata.append(rotated_pos[2])

#     ax.clear()  # Clear the previous plot

#     # Set the axis limits; you might need to adjust these based on your tracking area
#     ax.set_xlim(-2, 2)
#     ax.set_ylim(-2, 2)
#     ax.set_zlim(-2, 2)

#     # Plot the trail of last 50 samples
#     ax.plot(list(xdata), list(ydata), list(zdata), color='blue')  # Trail as a blue line
#     ax.scatter([rel_x], [rel_y], [rel_z], color='red')  # Current position as a red dot

#     # Reset the view angles if you want to maintain a specific perspective
#     # ax.view_init(elev=20., azim=30)  # Uncomment if specific view angle is needed

# ani = FuncAnimation(fig, update, frames=range(100), interval=50)  # Using a range for frames

# plt.show()