import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def calc_bezier(p_in, p_out, travel_limits_z):
    # Creates a four-point Bézier curve that ensures tangency with the incoming and outgoing
    # ball paths (ie. smooth position). Then scales the position along the bezier such that

    # the hand velocity is matched with the incoming and outgoing ball velocities.

    # p_in and p_out are vectors of ((pos), (vel))
    # travel_limits_z is the z limit for the movement
    # duration is the amount of time that the path must be completed in {sec}

    # Desired number of points to have in the final bezier. May be off by a little due to rounding
    desired_num_points = 60

    # Determine if the start of the path is a catch or a throw (z component of v_in will be -ve for a catch)
    catch_or_throw = p_in[1][2] / abs(p_in[1][2])  # -1 for catch, 1 for throw
    # Pick limit for this move (0 is catch, 1 is throw)
    travel_lim_z = travel_limits_z[int((1 + catch_or_throw) / 2)]
    # Pick duration for this move
    duration = path_duration[int((1 + catch_or_throw) / 2)]

    # Set up control points (CPs)
    P0 = p_in[0]  # First CP is the "incoming" point
    P3 = p_out[0]  # Last CP is the "outgoing" point
    P1 = P0 + catch_or_throw * travel_lim_z * np.array((p_in[1][0] / p_in[1][2], p_in[1][1] / p_in[1][2], 1))
    P2 = P3 + catch_or_throw * travel_lim_z * np.array((p_out[1][0] / p_out[1][2], p_out[1][1] / p_out[1][2], 1))

    control_points = np.array((P0, P1, P2, P3))

    coefficients = np.array(([1, 0, 0, 0],
                             [-3, 3, 0, 0],
                             [3, -6, 3, 0],
                             [-1, 3, -3, 1]))  # Coefficients of the bezier matrix

    ################################################################################################
    #                          Calculate the length of the Bézier curve                            #
    ################################################################################################
    path_length = 0  # Initialize the path length
    path_x = []
    path_y = []
    path_z = []

    pt_prev = (0, 0, 0)  # Resolves the warning in the if statement later
    num_points = 50  # Number of points to use to calculate the length of the bezier

    for i in range(num_points + 1):
        t = i / num_points  # Scale t to be 0 < t < 1
        t_matrix = np.array((1, t, t ** 2, t ** 3))  # Create the "time" vector
        pt = np.dot(np.dot(t_matrix, coefficients), control_points)

        if i > 0:  # If on anything but the first point, add the new length to the sum
            path_length += math.dist(pt_prev, pt)

        path_x.append(pt[0])
        path_y.append(pt[1])
        path_z.append(pt[2])

        pt_prev = pt  # Update the "old" point to be the current point

    path = np.column_stack((path_x, path_y, path_z))
    return path, control_points


def calc_ball_path(p_throw, framerate=100):
    # Creates the ball path after being thrown.

    # p_throw is a vector of where the ball is being thrown from ((throw pos), (throw vel))
    # duration is how long the ball is in the air for
    # framerate is how many frames/sec the simulation is being run at

    accel = np.array((0, 0, -g))
    duration = tf
    path_x = []
    path_y = []
    path_z = []

    for t in np.linspace(0, duration, int(duration * framerate)):
        pos = p_throw[0] + t * p_throw[1] + 0.5 * t**2 * accel
        path_x.append(pos[0])
        path_y.append(pos[1])
        path_z.append(pos[2])

    path = np.column_stack((path_x, path_y, path_z))
    return path


throw_height = 1.0
hand_xyspan = [0.2, 0]
hand_full_zspan = 0.5  # z distance that the hand moves while holding the ball {m}
hand_empty_zspan = 0.2  # z distance empty hand moves {m}

g = 9.81

vfz = math.sqrt(2*g*throw_height)
tf = 2*vfz/g  # Time of flight {sec}
vfxy = [-xy/tf for xy in hand_xyspan]
vel = math.sqrt(vfz**2 + vfxy[0]**2 + vfxy[1]**2)

# Inputs to the path creator:
catch_pos = np.array((-hand_xyspan[0]/2, -hand_xyspan[1]/2, 0))
catch_vel = np.array((vfxy[0], vfxy[1], -vfz))
throw_pos = np.array((hand_xyspan[0]/2, hand_xyspan[1]/2, 0))
throw_vel = np.array((vfxy[0], vfxy[1], vfz))

p_catch = np.concatenate(([catch_pos], [catch_vel]))
p_throw = np.concatenate(([throw_pos], [throw_vel]))

travel_lims = [hand_full_zspan, hand_empty_zspan]
hold_time = tf  # {sec}
empty_time = tf
path_duration = [hold_time, empty_time]


path_hold, hold_CPs = calc_bezier(p_catch, p_throw, travel_lims)
path_empty, empty_CPs = calc_bezier(p_throw, p_catch, travel_lims)
ball_path = calc_ball_path(p_throw)


hold_col = '#ff7f0e'
empty_col = '#2ca02c'
ball_col = 'b'

fig, ax = plt.subplots()

# set the limits
ax.set_xlim(-hand_xyspan[0], hand_xyspan[0])
ax.set_ylim(-hand_full_zspan * 1.05, throw_height * 1.1)

ax.set_axis_off()

# Plot control points
# ax.scatter(hold_CPs[:, 0], hold_CPs[:, 2], c=hold_col)
# ax.scatter(empty_CPs[:, 0], empty_CPs[:, 2], c=empty_col)
# ax.plot(hold_CPs[:, 0], hold_CPs[:, 2], c=hold_col, linestyle='--')
# ax.plot(empty_CPs[:, 0], empty_CPs[:, 2], c=empty_col, linestyle='--')

# Plot paths
ax.plot(path_hold[:, 0], path_hold[:, 2], c=hold_col, label='Ball in hand')
# ax.plot(path_empty[:, 0], path_empty[:, 2], c=empty_col, label='Empty hand')
ax.plot(ball_path[:, 0], ball_path[:, 2], c=ball_col, label='Ball in air')
ax.legend()

plt.savefig('plot.png', transparent=True)
plt.show()
