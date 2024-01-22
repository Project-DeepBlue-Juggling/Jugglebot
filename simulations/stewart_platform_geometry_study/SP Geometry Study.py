import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# import alphashape


def platform_builder(base_rad, plat_rad, base_small_angle, plat_small_angle):
    """
        Builds a Stewart platform given specified geometric parameters.

        Args:
            base_rad (float): Radius of the base (AU - Arbitrary Units)
            plat_rad (float): Radius of the platform (AU)
            base_small_angle (float): Angle between close base nodes (Gamma2 on main sketch) in deg (from 0 to 60)
            plat_small_angle (float): Angle between close plat nodes (Lambda1 on main sketch) in deg (from 0 to 60)

        Returns:
        a (numpy.ndarray), shape=(3,6): Position vectors of platform leg nodes in platform frame
        B (numpy.ndarray), shape=(3,6): Position vectors of base leg nodes in base frame
        """

    # Angles to connection points of each leg
    gamma0 = 0  # Offset from horizontal (deg)
    gamma2 = base_small_angle  # Angle between close base nodes (deg)
    gamma1 = 120 - gamma2    # Angle between far base nodes (deg)

    lambda1 = plat_small_angle  # Angle between close platform nodes (deg)
    lambda2 = 120 - lambda1  # Angle between far platform nodes (deg)
    lambda0 = (gamma1-lambda1)/2 + gamma0  # Offset from horizontal for platform nodes (deg)
    gamma_node = np.zeros(6)   # Angle to each base node in base frame (deg)
    lambda_node = np.zeros(6)  # Angle to each platform node in platform frame (deg)

    # Create position vectors for all links
    B = np.zeros((3, 6))  # Position vectors of base leg nodes in base frame
    a = np.zeros((3, 6))  # Position vectors of platform leg nodes in platform frame

    # Create nodes for all leg connections
    for i in range(6):
        # These are used to orient the nodes according to the desired angles
        first_angle_index = np.floor((i + 1) / 2)
        second_angle_index = np.ceil((i - 1) / 2)

        # Calculate the actual angles for each node
        gamma_node[i] = gamma0 + gamma1 * first_angle_index + gamma2 * second_angle_index
        lambda_node[i] = lambda0 + lambda1 * first_angle_index + lambda2 * second_angle_index

        B[0, i] = np.cos(np.deg2rad(gamma_node[i]))
        B[1, i] = np.sin(np.deg2rad(gamma_node[i]))
        B[2, i] = 0
        B[:, i] = base_rad*B[:, i]  # Scale B to actual base radius

        a[0, i] = np.cos(np.deg2rad(lambda_node[i]))
        a[1, i] = np.sin(np.deg2rad(lambda_node[i]))
        a[2, i] = 0
        a[:, i] = plat_rad * a[:, i]  # Scale a to actual platform radius

    return a, B


def generate_point_cloud(num_points, short_leg_len, long_leg_len):
    """
    Generates a point cloud within a specified range of values.

    Args:
        num_points (int): Number of points to generate in the point cloud.
        short_leg_len (float): Shortest length of a leg (AU).
        long_leg_len (float):  Longest length of a leg (AU).

    Returns:
        ptlist (numpy.ndarray): List of points in the point cloud.
    """

    # Initialize an empty array to store the points
    ptlist = np.zeros((num_points, 3))

    # Set the ranges to generate the point cloud within
    zMin = 0.8 * short_leg_len  # Min z value (AU)
    zMax = 1.0 * long_leg_len  # Max z value (AU)
    xySpan = 0.6 * long_leg_len  # Span of x/y values from origin (AU)

    # Generate numPoints random points within the specified ranges
    ptlist[:, 0] = -xySpan + 2 * xySpan * np.random.rand(num_points)
    ptlist[:, 1] = -xySpan + 2 * xySpan * np.random.rand(num_points)
    ptlist[:, 2] = zMin + (zMax - zMin) * np.random.rand(num_points)

    return ptlist


def compute_reachability(ptlist, a, B, mov_limits):
    """
    Given a point cloud (ptlist) and a Stewart platform configuration (a, B), this function checks whether
    each point in the point cloud is reachable by the platform, and returns a percentage of the reachable
    points as well as a list of valid points.

    Parameters:
    ptlist (numpy.ndarray): An array of shape (numPoints, 3) containing the 3D coordinates of the points to be
                            checked for reachability.
    a (numpy.ndarray): An array of shape (3, 6) containing the position vectors of the platform leg nodes in the
                       platform frame.
    B (numpy.ndarray): An array of shape (3, 6) containing the position vectors of the base leg nodes in the
                       base frame.
    movLimits (list): A list of three values containing the minimum and maximum length of each leg, and the
                      minimum leg angle in degrees.

    Returns:
    reachRate (float): A percentage of the reachable points.
    validPtList (numpy.ndarray): An array of shape (numValidPoints, 3) containing the 3D coordinates of the valid
                                 points.
    """
    checklist = np.zeros(len(ptlist))  # List of 1's and 0's that correspond to whether a point can be reached or not
    for i in range(len(ptlist)):
        pt = ptlist[i, :]
        Rot = np.eye(3)
        L = pt.reshape((3, 1)) + Rot @ a - B
        length = np.linalg.norm(L, axis=0)

        # check leg angles
        unit_legs = np.zeros((3, 6))  # Unit vector for each leg
        proj_unit_legs = np.zeros((3, 6))  # Projection of each unit vector to the x-y plane
        angles = np.zeros(6)
        for j in range(6):
            unit_legs[:, j] = L[:, j] / length[j]
            # Project the legs to the x-y plane to measure the angles of the legs with the z axis
            proj_unit_legs[:, j] = np.array([unit_legs[0, j], unit_legs[1, j], 0])
            angles[j] = abs(np.arctan2(unit_legs[2, j], np.linalg.norm(proj_unit_legs[:, j])) * 180 / np.pi)

        min_angle = np.min(angles)  # Only care about the min angle; only one impossibly angled leg is enough

        # Check if all leg lengths are within the specified movement limits
        legal_legs = length[(length >= mov_limits[0]) & (length <= mov_limits[1]) & (min_angle >= mov_limits[2])]
        if len(legal_legs) == 6:
            checklist[i] = 1

    reach_rate = np.sum(checklist) / len(ptlist) * 100
    valid_pt_pist = ptlist[checklist == 1, :]

    return reach_rate, valid_pt_pist


def plot_results(ROM_Centroid, plat_nodes, base_nodes, ptlist, valid_pt_list, hemisphere_pts):
    fig = plt.figure()
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122, projection='3d')
    ax1.set_aspect('equal')

    z_offset_mov_area = ROM_Centroid[2] * 1.1

    # plot platform nodes and center of mass
    A = ROM_Centroid + plat_nodes  # To plot the platform at the average position of the reachability cloud
    # A = plat_nodes
    for j in range(6):
        ax1.scatter(A[0, j], A[1, j], A[2, j], c='m')  # platform
        ax1.scatter(ROM_Centroid[0], ROM_Centroid[1], ROM_Centroid[2], c='m', marker='o')  # platform COM
        ax1.scatter(base_nodes[0, j], base_nodes[1, j], base_nodes[2, j], c='r', marker='o')  # base
        ax1.plot([base_nodes[0, j], A[0, j]], [base_nodes[1, j], A[1, j]], [base_nodes[2, j], A[2, j]], c='b')  # legs

    # Filter valid_pt_list, if wanting to show only a slice along the x axis. If not, uncomment next three lines.
    # x_axis_slice_max_min = 2  # Set this threshold to whatever you want (mm)
    # mask_x = np.logical_and(valid_pt_list[:, 0] >= -x_axis_slice_max_min, valid_pt_list[:, 0] <= x_axis_slice_max_min)
    # valid_pt_list = valid_pt_list[mask_x]

    valid_pts_in_hemisphere = points_in_hemisphere(valid_pt_list, [hemisphere_pts[0], hemisphere_pts[1], z_offset_mov_area])
    total_pts_in_hemisphere = points_in_hemisphere(ptlist, [hemisphere_pts[0], hemisphere_pts[1], z_offset_mov_area])

    print(f"Valid points in hemisphere: {len(valid_pts_in_hemisphere)}\nInvalid points in hemisphere: {len(total_pts_in_hemisphere)}")

    print(f"Percentage of points inside truncated hemisphere that are reachable: "
          f"{(len(valid_pts_in_hemisphere) / len(total_pts_in_hemisphere)) * 100:.2f}%")

    # plot point cloud
    # ax1.plot(ptlist[:, 0], ptlist[:, 1], ptlist[:, 2], 'ro', markersize=0.6, alpha=0.5, label='Unreachable points')
    ax1.plot(valid_pt_list[:, 0], valid_pt_list[:, 1], valid_pt_list[:, 2], 'g*', label='Reachable points')

    show_hemisphere = False

    if show_hemisphere:
        ax1.plot(total_pts_in_hemisphere[:, 0],
                total_pts_in_hemisphere[:, 1],
                total_pts_in_hemisphere[:, 2], 'c*', label='Unreachable points inside hemisphere')

        ax1.plot(valid_pts_in_hemisphere[:, 0],
                valid_pts_in_hemisphere[:, 1],
                valid_pts_in_hemisphere[:, 2], 'b*', label='Reachable points inside hemisphere')

    # plot_hemisphere(z_offset=z_offset_mov_area, geometry_pts=hemisphere_pts, ax=ax1)
    ax1.legend(loc='upper right')

    # Mask the data if wanting to plot truncated data
    threshold_xy = 500  # Threshold at which to cut off the x and y data for the plot
    mask = np.logical_and(np.abs(valid_pt_list[:, 0]) < threshold_xy, np.abs(valid_pt_list[:, 1]) < threshold_xy)
    valid_pt_list = valid_pt_list[mask]
    ax2.plot(valid_pt_list[:, 0], valid_pt_list[:, 1], valid_pt_list[:, 2], 'g*')
    # ax2.set_xlim([-250, 250])
    # ax2.set_ylim([-250, 250])


def compute_reachability_of_hemisphere(ROM_Centroid, ptlist, valid_pt_list, hemisphere_pts):
    z_offset_mov_area = ROM_Centroid[2] * 1.1

    valid_pts_in_hemisphere = points_in_hemisphere(valid_pt_list, [hemisphere_pts[0], hemisphere_pts[1], z_offset_mov_area])
    total_pts_in_hemisphere = points_in_hemisphere(ptlist, [hemisphere_pts[0], hemisphere_pts[1], z_offset_mov_area])

    try:
        percentage_reached = (len(valid_pts_in_hemisphere) / len(total_pts_in_hemisphere)) * 100
    except ZeroDivisionError:
        percentage_reached = 0

    # print(f"Valid points in hemisphere: {len(valid_pts_in_hemisphere)}\nInvalid points in hemisphere: {len(total_pts_in_hemisphere)}")
    #
    # print(f"Percentage of points inside truncated hemisphere that are reachable: "
    #       f"{percentage_reached:.2f}%")
    return percentage_reached


def plot_reachable_volume(valid_pts):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=20, azim=-60)

    # Set axis limits
    xy_span = 1.2 * max(abs(valid_pts[:, 0].max()), abs(valid_pts[:, 1].max()))
    ax.set_xlim([-xy_span, xy_span])
    ax.set_ylim([-xy_span, xy_span])
    ax.set_zlim([0, valid_pts[:, 2].max()])

    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.scatter(valid_pts[:, 0], valid_pts[:, 1], valid_pts[:, 2])  # Plot points
    alpha_shape = alphashape.alphashape(valid_pts)
    ax.plot_trisurf(*zip(*alpha_shape.vertices), triangles=alpha_shape.faces)

    plt.show()


def plot_hemisphere(z_offset, geometry_pts, ax):
    z_limit = geometry_pts[0]
    xy_span = geometry_pts[1]
    # Create a meshgrid for the hemisphere
    u = np.linspace(0, np.pi / 2, 100)  # For theta values
    v = np.linspace(0, 2 * np.pi, 100)  # For phi values
    x = xy_span / 2 * np.outer(np.sin(u), np.cos(v))
    y = xy_span / 2 * np.outer(np.sin(u), np.sin(v))
    z = z_offset - z_limit * np.outer(np.cos(u), np.ones_like(v))  # Negative z values for hemisphere below x-y plane

    # Plot the surface
    ax.plot_surface(x, y, z, color='c', alpha=0.5)


def points_in_hemisphere(pts, geometry_points):
    z_limit = geometry_points[0]
    xy_span = geometry_points[1]
    z_offset = geometry_points[2]

    r = xy_span / 2

    # Condition 1 - Points in sphere with radius 'r', offset by z_offset
    condition1 = ((pts[:, 0]) ** 2 + (pts[:, 1]) ** 2 + (pts[:, 2] - z_offset) ** 2) <= r ** 2
    # Condition 2
    condition2 = (pts[:, 2] >= z_offset - z_limit) & (pts[:, 2] <= z_offset)
    # Condition 3 and 4
    condition3 = (pts[:, 0] >= -xy_span / 2) & (pts[:, 0] <= xy_span / 2)
    condition4 = (pts[:, 1] >= -xy_span / 2) & (pts[:, 1] <= xy_span / 2)

    # Combine all conditions
    mask = condition1 & condition2 & condition3 & condition4
    return pts[mask]


def check_necessary_num_pts_in_cloud():
    # Will run the given SP with varying numbers of points in the cloud, then plot the results. Used to know how many
    # points will give a ~stable measure of the actual reach rate

    # 40,000 should do nicely.

    num_points = np.logspace(0, 7, 100).astype(int)
    reach_rate_list = []

    for pt in range(num_points.size):
        cloud = generate_point_cloud(num_points[pt], shortLeg, longLeg)
        RR, _ = compute_reachability(cloud, plat_nodes, base_nodes, movLimits)
        reach_rate_list.append(RR)
        print(pt)

    fig_pt_cloud_pts = plt.figure()
    ax_pt_cloud_pts = fig_pt_cloud_pts.add_subplot(111)
    ax_pt_cloud_pts.plot(num_points, reach_rate_list)
    ax_pt_cloud_pts.set_xlabel("Number of points in cloud")
    ax_pt_cloud_pts.set_ylabel("Reach rate")


##################################################################################
#                          Single-shot testing                                   #
##################################################################################

# Set geometric parameters to test:
# Platform layout
platRad = 229.5  # Radius of the platform (mm)
baseRad = 410.0  # Radius of the base (mm)
baseSmallAngle = 24.0     # Gamma2 on main sketch (deg)  (range from 0 -> 60)
platSmallAngle = 7.49496  # Lambda1 on main sketch (deg) (range from 0 -> 60)

# Leg specifications
shortLeg = 691.49  # Shortest length of leg (mm) # Is correct for Leg_V3_Inline_Motor
extLeg = 280.0   # Extension of leg (mm)
longLeg = shortLeg + extLeg  # Longest length of leg (mm)
# longLeg = 950  # Longest length of leg (mm) [Use either this or above] # Is correct for Leg_V3_Inline_Motor
legAngleLimit = 45  # Limit of how far the ball joints allow the legs to tilt wrt x-y plane (deg) # GUESS

# Parameters for "desired volume" hemisphere
hand_xy_span = 600  # mm
hand_z_span = 200  # mm

# Points in point cloud
numPoints = int(1e6)  # Number of points to check

# Structure input data
movLimits = [shortLeg, longLeg, legAngleLimit]

plat_nodes, base_nodes = platform_builder(baseRad, platRad, baseSmallAngle, platSmallAngle)
pt_cloud = generate_point_cloud(numPoints, shortLeg, longLeg)
reach_rate, valid_pt_list = compute_reachability(pt_cloud, plat_nodes, base_nodes, movLimits)

# check_necessary_num_pts_in_cloud()

ROM_centroid = np.mean(valid_pt_list, axis=0).reshape((3, 1))
plot_results(ROM_centroid, plat_nodes, base_nodes, pt_cloud, valid_pt_list, [hand_z_span, hand_xy_span])
# plot_reachable_volume(valid_pt_list)

reach_rates = []
base_radii = []

testing_base_radius = False  # Do you want to test the effect of varying the base radius?

if testing_base_radius:
    for i, baseRad in enumerate(np.linspace(300, 500, 50)):
        plat_nodes, base_nodes = platform_builder(baseRad, platRad, baseSmallAngle, platSmallAngle)
        reach_rate, valid_pt_list = compute_reachability(pt_cloud, plat_nodes, base_nodes, movLimits)

        ROM_centroid = np.mean(valid_pt_list, axis=0).reshape((3, 1))

        reached_pct = compute_reachability_of_hemisphere(ROM_centroid, pt_cloud, valid_pt_list, [hand_z_span, hand_xy_span])

        reach_rates.append(reached_pct)
        base_radii.append(baseRad)
        print(f"Run: {i}, base radius = {baseRad:.2f}, % reached: {reached_pct:.2f}")

    fig_hemisphere_testing = plt.figure()
    ax_hemisphere_testing = fig_hemisphere_testing.add_subplot(111)

    ax_hemisphere_testing.plot(base_radii, reach_rates)
    ax_hemisphere_testing.set_title("Ability to reach points of interest with varying base radius")
    ax_hemisphere_testing.set_xlabel("Base radius (mm)")
    ax_hemisphere_testing.set_ylabel("% of points in the desired volume that are reachable")

# _ = compute_reachability_of_hemisphere(ROM_centroid, pt_cloud, valid_pt_list,[hand_z_span, hand_xy_span])

print("Reach rate = {:.2f}%".format(reach_rate))

plt.show()

##################################################################################
#                           Optimizer testing                                    #
##################################################################################

run_optimizer = False

# Currently set up to keep either leg lengths or base/platform radius constant, while varying the other.
# Both configurations will be tested to check for agreement.

if run_optimizer:
    pts = 100  # Number of points to vary parameter over
    reach_rate_total = np.zeros((pts, 2))  # RR | rad / leg_len

    # Set geometric parameters to test:
    # Platform layout
    platRad = 100  # Radius of the platform (mm)
    baseRad = platRad  # Radius of the base (mm)
    baseSmallAngle = 12  # Gamma2 on main sketch (deg)  (range from 0 -> 60)
    platSmallAngle = 12  # Lambda1 on main sketch (deg) (range from 0 -> 60)

    # Leg specifications
    shortLeg = np.logspace(1, 3, pts)  # Shortest length of leg (mm)
    extLeg = 0.9 * shortLeg   # Extension of leg (mm)
    legAngleLimit = 20  # Limit of how far the ball joints allow the legs to tilt wrt x-y plane (deg)

    # Points in point cloud
    numPoints = int(1e5)  # Number of points to check

    # Structure input data
    longLeg = shortLeg + extLeg  # Longest length of leg (AU)

    # movLimits = [shortLeg, longLeg, legAngleLimit]

    # Generate point cloud (only generate once so that all conditions are tested equally)

    # pt_cloud = generate_point_cloud(numPoints, shortLeg[-1], longLeg[-1])

    for i in range(pts):
        # Generate Stewart Platform
        plat_nodes, base_nodes = platform_builder(baseRad, platRad, baseSmallAngle, platSmallAngle)

        pt_cloud = generate_point_cloud(numPoints, shortLeg[i], longLeg[i])

        # Determine which points in the cloud are reachable and which aren't
        movLimits = [shortLeg[i], longLeg[i], legAngleLimit]
        reach_rate, valid_pt_list = compute_reachability(pt_cloud, plat_nodes, base_nodes, movLimits)
        rom_centroid = np.mean(valid_pt_list, axis=0)
        reach_rate_total[i, :] = reach_rate, platRad / longLeg[i]
        print(i)

    fig_optimizer = plt.figure()
    ax_optimizer = fig_optimizer.add_subplot(111)
    ax_optimizer.plot(reach_rate_total[:, 0], reach_rate_total[:, 1])
    ax_optimizer.set_xlabel("Plat/Base rad / max leg length")
    ax_optimizer.set_ylabel("Reach rate")

    plt.show()
