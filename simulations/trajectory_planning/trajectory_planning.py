import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle, Ellipse
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.image as mpimg


def calc_bezier(p_in, p_out, travel_limits_z, durations, framerate, color='r'):
    # Creates a four-point Bézier curve that ensures tangency with the incoming and outgoing
    # ball paths (ie. smooth position). Then scales the position along the bezier such that

    # the hand velocity is matched with the incoming and outgoing ball velocities.

    # p_in and p_out are vectors of ((pos), (vel))
    # travel_limits_z is the z limit for the movement
    # duration is the amount of time that the path must be completed in {sec}
    # framerate is the rate at which path points will be generated {pts/sec}

    # Desired number of points to have in the final bezier. May be off by a little due to rounding
    desired_num_points = 60

    # Determine if the start of the path is a catch or a throw (z component of v_in will be -ve for a catch)
    catch_or_throw = p_in[1][2] / abs(p_in[1][2])  # -1 for catch, 1 for throw
    # Pick limit for this move (0 is catch, 1 is throw)
    travel_lim_z = travel_limits_z[int((1 + catch_or_throw) / 2)]
    # Pick duration for this move
    duration = durations[int((1 + catch_or_throw) / 2)]

    # Prepare for the final path
    path_x = []
    path_y = []
    path_z = []

    # Set up control points (CPs)
    P0 = p_in[0]   # First CP is the "incoming" point
    P3 = p_out[0]  # Last CP is the "outgoing" point
    P1 = P0 + catch_or_throw * travel_lim_z * np.array((p_in[1][0]/p_in[1][2], p_in[1][1]/p_in[1][2], 1))
    P2 = P3 + catch_or_throw * travel_lim_z * np.array((p_out[1][0]/p_out[1][2], p_out[1][1]/p_out[1][2], 1))

    control_points = np.array((P0, P1, P2, P3))

    coefficients = np.array(([1, 0, 0, 0],
                             [-3, 3, 0, 0],
                             [3, -6, 3, 0],
                             [-1, 3, -3, 1]))  # Coefficients of the bezier matrix

    ################################################################################################
    #                          Calculate the length of the Bézier curve                            #
    ################################################################################################
    path_length = 0  # Initialize the path length
    pt_prev = (0, 0, 0)  # Resolves the warning in the if statement later
    num_points = 50  # Number of points to use to calculate the length of the bezier

    for i in range(num_points + 1):
        t = i / num_points  # Scale t to be 0 < t < 1
        t_matrix = np.array((1, t, t ** 2, t ** 3))  # Create the "time" vector
        pt = np.dot(np.dot(t_matrix, coefficients), control_points)

        if i > 0:  # If on anything but the first point, add the new length to the sum
            path_length += math.dist(pt_prev, pt)

        pt_prev = pt  # Update the "old" point to be the current point

    ################################################################################################
    # Form the velocity equation to ensure a smooth velocity throughout (using quadratic velocity) #
    ################################################################################################
    v_in = np.linalg.norm(p_in[1])
    v_out = np.linalg.norm(p_out[1])

    a = 3*(v_out + v_in - 2*path_length/duration)/(duration**2)
    b = (6*path_length - duration*(2*v_out + 4*v_in))/(duration**2)
    c = v_in
    v = []
    accel = []
    ppath = []  # A vector of how far long the main bezier the hand is at each frame (stands for "percentage along path")

    ################################################################################################
    #                Creating and plotting the position and velocity curves                        #
    #                                 for quadratic velocity                                       #
    ################################################################################################
    num_steps = int(duration * framerate)
    step_size = 1 / framerate

    for t in np.linspace(0, duration, num_steps):
        s = (1/3)*a*t**3 + (1/2)*b*t**2 + c*t # Area under velocity curve is displacement. Velocity curve is quadratic
        ppath.append(s)
        v_temp = a*t**2 + b*t + c
        v.append(v_temp)
        ax2.scatter(t/duration, s  / path_length, color=color)
        ax2.set_title('Parabolic velocity')
        ax2.set_ylabel('Position \n(along path)')
        ax3.scatter(t/duration, v_temp, color=color)
        ax3.set_ylabel('Velocity')

        # Calculate and plot acceleration
        if t > 0:
            accel.append((v[-1] - v[-2]) / step_size)
            ax6.scatter(t/duration, accel[-1], color=color)
            # ax6.set_title('Parabolic velocity - accel')
            ax6.set_ylabel('Accel')
            ax6.set_xlabel('Percentage along path')

    ################################################################################################
    #                             Correct in case of < 0 velocities                                #
    ################################################################################################
    # If the velocity is ever below zero, change the profile to be two linear sections with v = 0 between them
    if min(v) < 0:
        ppath = []  # Clear the old ppath as it will be populated with new values
        v = []      # Clear the old velocity vector so that it's ready for new values
        accel = []  # Clear accel vector

        t1 = path_length / v_in
        t2 = path_length / v_out
        th = duration - (t1 + t2)

        for t in np.linspace(0, duration, int(duration * framerate)):
            if t < t1:
                ppath.append(v_in*(t - t**2 / (2*t1)))
                v.append(v_in*(1 - t/t1))
            elif t < t1 + th:
                ppath.append(ppath[-1])
                v.append(0)
            else:
                ppath.append(v_out*((t**2 - duration**2)/(2*t2) + ((t1 + th)/t2)*(duration - t)) + path_length)
                v.append((v_out/t2) * (t - (t1 + th)))

            ax4.scatter(t / duration, ppath[-1] / path_length, color=color)
            ax4.set_title('Non-linear acceleration')
            ax4.set_ylabel('Position \n(along path)')
            ax5.scatter(t / duration, v[-1], color=color)
            ax5.set_ylabel('Velocity')

            # Calculate and plot acceleration
            if t > 0:
                accel.append((v[-1] - v[-2]) / step_size)
                ax7.scatter(t / duration, accel[-1], color=color)
                ax7.set_ylabel('Accel')
                ax7.set_xlabel('Percentage along path')

    ################################################################################################
    #                Weight point placement to ensure more points in faster regions                #
    ################################################################################################
    # "Weight" acceleration values so that the sum of their magnitudes = 1
    weighted_accel = np.abs(accel / np.sum(np.abs(accel)))
    points_per_segment = np.round(weighted_accel * num_steps).astype(int)

    # Ensure each segment has at least one point attributed to it. Prevents large gaps in the end result
    mid = len(points_per_segment) // 2  # Index of the middle value
    if len(points_per_segment) % 2 == 0:  # If points_per_segment has an even number of values
        if points_per_segment[mid-1] == 0 and points_per_segment[mid] == 0:
            points_per_segment[mid-1] = 1
            points_per_segment[mid] = 1
    else:
        if points_per_segment[mid] == 0:
            points_per_segment[mid] = 1

    # Create new ppath, where the points are placed according to the points_per_segment vector above
    weighted_pts = np.zeros((np.sum(points_per_segment), 1))
    for i in range(len(ppath) - 1):
        for idx, j in enumerate(np.linspace(ppath[i], ppath[i+1], points_per_segment[i] + 1)):
            # Split this section of the path into points_per_segment + 1 points. The "+1" is needed because otherwise
            # there are duplicate points at every exchange between one segment and the next. The if, break resolves this
            if idx == points_per_segment[i]:
                break
            pt_index = idx + np.sum(points_per_segment[:i])
            weighted_pts[pt_index] = j

    # Scale 's' to be between 0 and 1
    weighted_pts = weighted_pts / np.max(weighted_pts)
    weighted_pts = weighted_pts[1:]  # Remove the first element so that there's no overlap between catching and throwing

    # Scale the Bézier curve to ensure a smooth velocity throughout
    i = 0
    if catch_or_throw == 1:
        marker = 'o'
        facecolor = 'none'
    else:
        marker = 'x'
        facecolor = color
    
    for i, _ in enumerate(np.linspace(0, duration, weighted_pts.size)):
        t_matrix = np.array((1, weighted_pts[i][0], weighted_pts[i][0] ** 2, weighted_pts[i][0] ** 3))  # Create the time vector
        path_pt = np.dot(np.dot(t_matrix, coefficients), control_points)
        path_x.append((path_pt[0]))
        path_y.append((path_pt[1]))
        path_z.append((path_pt[2]))

    path = np.column_stack((path_x, path_y, path_z))

    return path, np.sum(points_per_segment)  # Get pts/seg to tell ball path how many points to use


def calc_ball_path(p_throw, duration, pts):
    # Creates the ball path after being thrown.

    # p_throw is a vector of where the ball is being thrown from ((throw pos), (throw vel))
    # duration is how long the ball is in the air for
    # framerate is how many frames/sec the simulation is being run at

    accel = np.array((0, 0, -g))
    ball_path_x = []
    ball_path_y = []
    ball_path_z = []

    for t in np.linspace(0, duration, pts-1):
        pos = p_throw[0] + t * p_throw[1] + 0.5 * t**2 * accel
        ball_path_x.append(pos[0])
        ball_path_y.append(pos[1])
        ball_path_z.append(pos[2])

    ball_path = np.column_stack((ball_path_x, ball_path_y, ball_path_z))
    return ball_path


fig2 = plt.figure()
ax2 = fig2.add_subplot(311)
ax3 = fig2.add_subplot(312)
ax6 = fig2.add_subplot(313)
ax3.grid(True)
plt.subplots_adjust(hspace=0.7, wspace=0.3)

fig3 = plt.figure()
ax4 = fig3.add_subplot(311)
ax5 = fig3.add_subplot(312)
ax7 = fig3.add_subplot(313)

plt.subplots_adjust(hspace=0.7, wspace=0.3)

throw_height = 0.5
hand_xyspan = [0.2, 0]
hand_full_zspan = 0.5  # z distance that the hand moves while holding the ball {m}
hand_empty_zspan = 0.2  # z distance empty hand moves {m}

g = 9.81

vfz = math.sqrt(2*g*throw_height)
tf = 2*vfz/g  # Time of flight {sec}
vfxy = [-xy/tf for xy in hand_xyspan]
vel = math.sqrt(vfz**2 + vfxy[0]**2 + vfxy[1]**2)

# Variables that will be inputs to the function:
catch_pos = np.array((-hand_xyspan[0]/2, -hand_xyspan[1]/2, 0))
catch_vel = np.array((vfxy[0], vfxy[1], -vfz))
throw_pos = np.array((hand_xyspan[0]/2, hand_xyspan[1]/2, 0))
throw_vel = np.array((vfxy[0], vfxy[1], vfz))

p_catch = np.concatenate(([catch_pos], [catch_vel]))
p_throw = np.concatenate(([throw_pos], [throw_vel]))

travel_lims = [hand_full_zspan, hand_empty_zspan]
hold_time = tf * 0.8  # {sec}
empty_time = tf
path_duration = [hold_time, empty_time]
frames_per_second = 200  # {frames/sec}

path_hold, _ = calc_bezier(p_catch, p_throw, travel_lims, path_duration, frames_per_second, color='#ff7f0e')
path_empty, pts_ball_in_air = calc_bezier(p_throw, p_catch, travel_lims, path_duration, frames_per_second, color='#2ca02c')
path_ball = calc_ball_path(p_throw, empty_time, pts_ball_in_air)

path_hold = path_hold[:, [0, 2]]  # Get rid of the y values. Don't need them for plotting.
path_empty = path_empty[:, [0, 2]]
path_ball = path_ball[:, [0, 2]]

path_ball_full = np.vstack((path_ball, path_hold))
path_hand_full = np.vstack((path_empty, path_hold))

print("Ball frames: ", len(path_ball))
print("Empty frames: ", len(path_empty))
print("Hold frames: ", len(path_hold))

# Prepare for plotting

fig, ax = plt.subplots(figsize=(10, 10))

# set the limits
ax.set_xlim(-hand_xyspan[0], hand_xyspan[0])
ax.set_ylim(-hand_full_zspan * 1.05, throw_height * 1.1)

# Plot static image
ax.plot(path_ball[:, 0], path_ball[:, 1], label="Ball in air")
ax.plot(path_hold[:, 0], path_hold[:, 1], label="Ball in hand")
ax.plot(path_empty[:, 0], path_empty[:, 1], label="Empty hand")
ax.legend(prop={'size': 16})
ax.set_axis_off()
fig.patch.set_alpha(0)

# Calculate aspect ratio of the plot
x_scale = ax.get_xlim()[1] - ax.get_xlim()[0]
y_scale = ax.get_ylim()[1] - ax.get_ylim()[0]
aspect_ratio = y_scale / x_scale

# Use Ellipse to create circle with the correct aspect ratio
circle = Ellipse((0, 0), 0.1 / aspect_ratio, 0.1, fill=True, color='r', zorder=3)  # creating a hollow circle

# Load the hand and create an OffsetImage
hand = mpimg.imread('hand.png')  # replace 'icon.png' with the path to your icon
hand = OffsetImage(hand, zoom=0.05)  # adjust zoom level to scale your icon

# Create an AnnotationBox to place the icon at the position of the rectangle
hand_imagebox = AnnotationBbox(hand, (0, 0), box_alignment=(0.5, 0.4), frameon=False, pad=0, zorder=4)
ax.add_artist(hand_imagebox)

ax.add_patch(circle)


def animate(i):
    circle.center = path_ball_full[i, :2]
    hand_imagebox.xybox = path_hand_full[i, :2]
    return circle, hand_imagebox


ani = animation.FuncAnimation(fig, animate, frames=len(path_ball_full), interval=100, blit=False)
# ani.save('animation.gif', writer='imagemagick', dpi=100)
plt.show()
