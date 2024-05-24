import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
import json
from scipy.spatial import ConvexHull
import time

#####################################################
#       Build the Platform and Generate Poses       #
#####################################################

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

def generate_poses(max_radius, min_radius, num_circles, min_height, max_height, num_height_steps=2, points_per_circle_increment=4):
    poses = []
    deviations = [0, 15]  # Deviations in degrees

    height_steps = (max_height - min_height) / num_height_steps

    for z in np.arange(min_height, max_height + height_steps, height_steps):
        for i in range(num_circles):
            # Interpolate radius for current height
            t = (z - min_height) / (max_height - min_height)
            base_radius = (1 - t) * max_radius + t * min_radius
            radius = (base_radius / num_circles) * i

            num_points = max(1, points_per_circle_increment * i)  # At least one point per circle
            angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
            for angle in angles:
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                pos = (x, y, z)

                for deviation in deviations:
                    if y == 0 and x == 0:
                        # For the point at x=0, y=0, add five sets of rotations with evenly spaced roll and pitch
                        for k in range(5):
                            yaw = 0
                            pitch = deviation * np.cos(2 * np.pi * k / 5)
                            roll = -deviation * np.sin(2 * np.pi * k / 5)
                            orientation = (pitch, roll, yaw)
                            poses.append((pos, orientation))
                    else:
                        roll = -deviation * np.cos(angle)
                        pitch = deviation * np.sin(angle)
                        yaw = 0

                        orientation = (pitch, roll, yaw)
                        poses.append((pos, orientation))

    return poses

def check_and_delete_duplicate_poses(poses):
    seen = set()
    duplicates = []

    for pos, orientation in poses:
        # Create a tuple from the position and orientation to use as a set element
        pos_tuple = (round(pos[0], 5), round(pos[1], 5), round(pos[2], 5))  # Round to avoid floating-point precision issues
        orientation_tuple = (round(orientation[0], 5), round(orientation[1], 5), round(orientation[2], 5))
        pose_tuple = (pos_tuple, orientation_tuple)

        if pose_tuple in seen:
            duplicates.append((pos, orientation))
        else:
            seen.add(pose_tuple)

    # Print whether duplicates were found
    if duplicates:
        print(f'Duplicates found: {len(duplicates)}')
    else:
        print('No duplicates found')

    # Remove duplicates from the original list
    for duplicate in duplicates:
        poses.remove(duplicate)

    # Print the final number of poses
    print(f'Number of poses after removing duplicates: {len(poses)}')

    return poses


#####################################################
#         Compute Jacobian and Reachability         #
#####################################################

def compute_analytical_jacobian(plat_nodes, base_nodes, pose):
    """
    Computes the Jacobian matrix for a given platform geometry and pose.
    
    Args:
        plat_nodes (numpy.ndarray): Position vectors of platform leg nodes in platform frame, shape (3, 6).
        base_nodes (numpy.ndarray): Position vectors of base leg nodes in base frame, shape (3, 6).
        pose (numpy.ndarray): Pose of the platform [(x, y, z), (alpha, beta, gamma)]. Units are in mm and degrees.
    
    Returns:
        J (numpy.ndarray): Jacobian matrix, shape (6, 6).
    """
    # Extract translation and rotation from pose
    r = np.array(pose[0])
    alpha, beta, gamma = pose[1]
    # Convert angles into radians
    alpha, beta, gamma = np.deg2rad(alpha), np.deg2rad(beta), np.deg2rad(gamma)
    R = euler_to_rot_matrix(alpha, beta, gamma)
    
    # Compute leg vectors
    L = np.zeros((3, 6))
    for i in range(6):
        L[:, i] = r + np.dot(R, plat_nodes[:, i]) - base_nodes[:, i]
    
    # Compute the Jacobian matrix
    J = np.zeros((6, 6))
    for i in range(6):
        Li_norm = np.linalg.norm(L[:, i])
        Li = L[:, i] / Li_norm
        Ri_pi = np.dot(R, plat_nodes[:, i])
        skew_Ri_pi = skew_symmetric(Ri_pi)
        J[i, :3] = Li
        J[i, 3:] = np.dot(skew_Ri_pi, Li)

    return J

def check_reachability(pose, plat_nodes, base_nodes, shortLeg, longLeg, legAngleLimit):
    '''
    Places the given geometry at the given pose and checks if the legs are within the reach limits.

    Note that pose is a tuple of ((x, y, z), (alpha, beta, gamma)).
    '''
    # Extract translation and rotation from pose ((x, y, z), (alpha, beta, gamma)):
    r = np.array(pose[0])
    alpha, beta, gamma = pose[1]
    # Convert angles into radians
    alpha, beta, gamma = np.deg2rad(alpha), np.deg2rad(beta), np.deg2rad(gamma)
    R = euler_to_rot_matrix(alpha, beta, gamma)

    # Compute leg vectors
    L = np.zeros((3, 6))
    for i in range(6):
        L[:, i] = r + np.dot(R, plat_nodes[:, i]) - base_nodes[:, i]

    # Check if the legs are within the reach limits
    leg_lengths = np.linalg.norm(L, axis=0)
    short_legs = leg_lengths < shortLeg # Will return True for each leg that is shorter than shortLeg
    long_legs = leg_lengths > longLeg # Will return True for each leg that is longer than longLeg

    # check leg angles
    unit_legs = np.zeros((3, 6))  # Unit vector for each leg
    proj_unit_legs = np.zeros((3, 6))  # Projection of each unit vector to the x-y plane
    angles = np.zeros(6)
    for j in range(6):
        unit_legs[:, j] = L[:, j] / leg_lengths[j]
        # Project the legs to the x-y plane to measure the angles of the legs with the z axis
        proj_unit_legs[:, j] = np.array([unit_legs[0, j], unit_legs[1, j], 0])
        angles[j] = abs(np.arctan2(unit_legs[2, j], np.linalg.norm(proj_unit_legs[:, j])) * 180 / np.pi)

    leg_angles = np.array(angles) < legAngleLimit  # Will return True for each leg that is within the leg angle limit

    # print(f'Short legs: {short_legs}, Long legs: {long_legs}, Leg angle limit: {leg_angles}')
    # print(f'Leg lengths: {leg_lengths}')
    # print(f'shortLeg: {shortLeg}, longLeg: {longLeg}, legAngleLimit: {legAngleLimit}')

    # If any of the legs are outside the reach limits, return False
    if np.any(short_legs) or np.any(long_legs) or np.any(leg_angles):
        return False
    else:
        return True


#####################################################
#                   Scoring                         #
#####################################################

def evaluate_jacobian(J):
    """
    Evaluates how well-balanced the Jacobian matrix is and returns the results.
    
    Args:
        J (numpy.ndarray): Jacobian matrix, shape (6, 6).

    Returns:
        tuple: A tuple containing:
            - condition_number (float): The condition number of the Jacobian matrix.
            - rank_deficiency (int): The rank deficiency of the Jacobian matrix.
            - submatrix_condition_number (float): The condition number of the submatrix (1st, 2nd, 4th, 5th columns).
            - norm_score (float): The ratio of average rotation norms to average translation norms.
    """
    # Compute the condition number. This should ideally be close to 1.
    condition_number = np.linalg.cond(J)
    
    # Compute the rank deficiency (number of linearly dependent columns). This will ideally be 0.
    rank_deficiency = 6 - np.linalg.matrix_rank(J)

    # Compute the condition number for the submatrix of translation and rotation columns
    submatrix = J[:, [0, 1, 3, 4]]
    submatrix_condition_number = np.linalg.cond(submatrix)

    # Compute the ratio between translation and rotation column norms. This should ideally be 1.
    norm_trans_x = np.linalg.norm(J[:, 0])
    norm_trans_y = np.linalg.norm(J[:, 1])
    norm_rot_x = np.linalg.norm(J[:, 3])
    norm_rot_y = np.linalg.norm(J[:, 4])
    # Compute average norms
    avg_norm_trans = (norm_trans_x + norm_trans_y) / 2
    avg_norm_rot = (norm_rot_x + norm_rot_y) / 2

    norm_score = avg_norm_rot / avg_norm_trans
    
    return condition_number, submatrix_condition_number, rank_deficiency, norm_score

def aggregate_scores(scores, unreachable_count):
    """
    Aggregates scores for a given geometry.
    
    Args:
        scores (numpy.ndarray): Scores array of shape (num_poses, 4).
    
    Returns:
        dict: A dictionary with aggregated metrics.
    """
    condition_numbers = scores[:, 0]
    rank_deficiencies = scores[:, 1]
    submatrix_condition_numbers = scores[:, 2]
    norm_scores = scores[:, 3]
    
    aggregated_results = np.array([
        np.mean(condition_numbers), # Mean condition number
        np.std(condition_numbers),  # Standard deviation of condition numbers
        np.sum(rank_deficiencies),  # Total rank deficiencies
        np.max(rank_deficiencies),  # Maximum rank deficiency
        np.mean(rank_deficiencies == 0),      # Percentage of poses with no rank deficiency
        np.mean(submatrix_condition_numbers), # Mean submatrix condition number
        np.std(submatrix_condition_numbers),  # Standard deviation of submatrix condition numbers
        np.mean(norm_scores),       # Mean norm score
        np.std(norm_scores),        # Standard deviation of norm scores
        unreachable_count           # Total number of unreachable poses
    ])
    
    return aggregated_results


#####################################################
#                   Plotting                        #
#####################################################

def plot_poses(poses, arrow_length=5):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    for pos, orientation in poses:
        x, y, z = pos

        # if z != 500:
        #     continue

        roll, pitch, yaw = np.deg2rad(orientation)

        # Calculate the direction of the arrow
        dx, dy, dz = euler_to_vector(roll, pitch, yaw)

        # Plot the point
        ax.scatter(x, y, z, color='b', s=20)
        # Plot the arrow
        ax.quiver(x, y, z, dx, dy, dz, length=arrow_length, color='r')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Poses')
    # plt.show()

def plot_geometry_with_poses(curr_pose, platRad, platAngle, poses, plat_nodes, base_nodes, arrow_length=30, geom_limits=None, normalized_cond_threshold=0.0, hide_unreachable=True):
    ''' 
    Plot the platform geometry at a default pose with the given poses around it. 
    Colour unreachable poses black and use a colour gradient to show the condition number of the Jacobian matrix at each reachable pose.
    Save the resulting plot as a PNG file.
    '''
    
    # Plot the platform
    alpha, beta, gamma = np.deg2rad(curr_pose[1])
    R = euler_to_rot_matrix(alpha, beta, gamma)
    A = np.array(curr_pose[0]).reshape(3, 1) + np.dot(R, plat_nodes)

    # Calculate the direction of the arrow
    dx, dy, dz = euler_to_vector(alpha, beta, gamma)

    # Initialize matplotlib figure
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')

    for j in range(6):
        ax.scatter(A[0, j], A[1, j], A[2, j], c='magenta', marker='o') # Plot platform nodes
        ax.scatter(np.mean(A[0, :]), np.mean(A[1, :]), np.mean(A[2, :]), c='magenta', marker='o') # Plot platform center
        ax.quiver(np.mean(A[0, :]), np.mean(A[1, :]), np.mean(A[2, :]), dx, dy, dz, color='magenta', length=arrow_length, normalize=True) # Plot platform orientation
        ax.scatter(base_nodes[0, j], base_nodes[1, j], base_nodes[2, j], c='red', marker='o') # Plot base nodes
        ax.plot([base_nodes[0, j], A[0, j]], [base_nodes[1, j], A[1, j]], [base_nodes[2, j], A[2, j]], c='blue') # Plot legs

    # Check reachability and compute condition numbers for all poses
    condition_numbers = []
    num_unreachable = 0
    for pos, orientation in poses:
        if check_reachability((pos, orientation), plat_nodes, base_nodes, geom_limits[0], geom_limits[1], geom_limits[2]):
            J = compute_analytical_jacobian(plat_nodes, base_nodes, (pos, orientation))
            condition_number, submatrix_condition_number, rank_deficiency, norm_score = evaluate_jacobian(J)
            condition_numbers.append(condition_number)
        else:
            condition_numbers.append(None)
            num_unreachable += 1

    # Filter out None values for min and max calculation
    filtered_condition_numbers = [cn for cn in condition_numbers if cn is not None]
    min_condition = np.min(filtered_condition_numbers)
    max_condition = np.max(filtered_condition_numbers)

    # Normalize condition numbers
    norm_condition_numbers = [(cn - min_condition) / (max_condition - min_condition) if cn is not None else None for cn in condition_numbers]

    # Plot poses with colors based on condition numbers
    for i, (pos, orientation) in enumerate(poses):
        x, y, z = pos
        roll, pitch, yaw = np.deg2rad(orientation)

        # Calculate the direction of the arrow
        dx, dy, dz = euler_to_vector(roll, pitch, yaw)

        # Determine the color based on the condition number
        if norm_condition_numbers[i] is None or norm_condition_numbers[i] < normalized_cond_threshold:
            # If the pose is unreachable, color it black
            color = 'black'

            if hide_unreachable:
                # If the pose is unreachable, make the point invisible
                x = np.nan
                y = np.nan
                z = np.nan 
        else:
            color = cm.viridis(norm_condition_numbers[i])
            # For figuring out the pose where the condition number is the highest
            # print(f'x: {x}, y: {y}, z: {z}, roll: {roll}, pitch: {pitch}, yaw: {yaw}')

        # Plot scatter points
        ax.scatter(x, y, z, color=color, marker='o')

        # Plot arrows
        if norm_condition_numbers[i] is not None:
            ax.quiver(x, y, z, dx, dy, dz, color=color, length=arrow_length, normalize=True)

    # Add a colorbar with a label
    sm = plt.cm.ScalarMappable(cmap=cm.viridis, norm=plt.Normalize(vmin=min_condition, vmax=max_condition))
    sm._A = []
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label('Condition Number')

    # Set axis labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'3D Poses with Condition Number Gradient\nPlatform Radius = {platRad}mm, Angle = {platAngle} deg\n{num_unreachable} unreachable poses')

def plot_convex_hull_and_poses(convex_hull_points, poses, arrow_length=5):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot convex hull
    hull = ConvexHull(convex_hull_points)
    for simplex in hull.simplices:
        ax.plot(convex_hull_points[simplex, 0], convex_hull_points[simplex, 1], convex_hull_points[simplex, 2], 'k-')

    # Plot poses
    for pos, orientation in poses:
        x, y, z = pos
        roll, pitch, yaw = np.deg2rad(orientation)
        dx, dy, dz = euler_to_vector(roll, pitch, yaw)
        ax.scatter(x, y, z, color='b', s=20)
        ax.quiver(x, y, z, dx, dy, dz, length=arrow_length, color='r')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Convex Hull and 3D Poses')
    plt.show()


#####################################################
#               Helper Functions                    #
#####################################################

def euler_to_vector(roll, pitch, yaw):
    # Convert Euler angles to a rotation matrix
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])

    # Combined rotation matrix
    R = Rz @ Ry @ Rx

    # The direction vector (we'll assume it's initially pointing in the x direction)
    direction = np.array([0, 0, 1])

    # Rotate the direction vector
    rotated_direction = R @ direction

    return rotated_direction

def euler_to_rot_matrix(alpha, beta, gamma):
    """
    Computes the rotation matrix for given Euler angles.
    
    Args:
        alpha, beta, gamma (float): Euler angles in radians.
    
    Returns:
        R (numpy.ndarray): Rotation matrix, shape (3, 3).
    """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(alpha), -np.sin(alpha)],
                    [0, np.sin(alpha), np.cos(alpha)]])
    
    R_y = np.array([[np.cos(beta), 0, np.sin(beta)],
                    [0, 1, 0],
                    [-np.sin(beta), 0, np.cos(beta)]])
    
    R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                    [np.sin(gamma), np.cos(gamma), 0],
                    [0, 0, 1]])
    
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def skew_symmetric(v):
    """
    Computes the skew-symmetric matrix of a vector.
    
    Args:
        v (numpy.ndarray): Input vector, shape (3,).
    
    Returns:
        skew_v (numpy.ndarray): Skew-symmetric matrix, shape (3, 3).
    """
    skew_v = np.array([[0, -v[2], v[1]],
                       [v[2], 0, -v[0]],
                       [-v[1], v[0], 0]])
    return skew_v

def compute_leg_lengths(plat_nodes, base_nodes, pose):
    """
    Computes the lengths of the legs for a given platform geometry and pose.
    
    Args:
        plat_nodes (numpy.ndarray): Position vectors of platform leg nodes in platform frame, shape (3, 6).
        base_nodes (numpy.ndarray): Position vectors of base leg nodes in base frame, shape (3, 6).
        pose (numpy.ndarray): Pose of the platform [x, y, z, alpha, beta, gamma]. Units are in mm and degrees.
    
    Returns:
        lengths (numpy.ndarray): Lengths of the legs, shape (6,).
    """
    x, y, z, alpha, beta, gamma = pose
    r = np.array([x, y, z])
    # Convert angles into radians
    alpha, beta, gamma = np.deg2rad(alpha), np.deg2rad(beta), np.deg2rad(gamma)
    R = euler_to_rot_matrix(alpha, beta, gamma)
    
    lengths = np.zeros(6)
    for i in range(6):
        L = r + np.dot(R, plat_nodes[:, i]) - base_nodes[:, i]
        lengths[i] = np.linalg.norm(L)
    
    return lengths

def compute_pose_error(J, leg_length_error):
    """
    Computes the pose error (translation and rotation) due to a given leg length error.
    
    Args:
        J (numpy.ndarray): Jacobian matrix, shape (6, 6).
        leg_length_error (numpy.ndarray): Leg length errors, shape (6,).
    
    Returns:
        pose_error (numpy.ndarray): Pose error [dx, dy, dz, dalpha, dbeta, dgamma], shape (6,).
    """
    # Compute the inverse of the Jacobian matrix
    J_inv = np.linalg.pinv(J)  # Use pseudoinverse for numerical stability
    
    # Calculate the pose error
    pose_error = np.dot(J_inv, leg_length_error)
    
    return pose_error

def load_convex_hull(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
    points = np.array(data['vertices'])
    return points

# For confirming the analytical Jacobian
def compute_numerical_jacobian(plat_nodes, base_nodes, pose, delta=1e-6):
    """
    Computes the numerical Jacobian matrix for a given platform geometry and pose.
    
    Args:
        plat_nodes (numpy.ndarray): Position vectors of platform leg nodes in platform frame, shape (3, 6).
        base_nodes (numpy.ndarray): Position vectors of base leg nodes in base frame, shape (3, 6).
        pose (numpy.ndarray): Pose of the platform [(x, y, z), (alpha, beta, gamma)]. Units are in mm and degrees.
        delta (float): Small perturbation value for numerical differentiation.
    
    Returns:
        J_numerical (numpy.ndarray): Numerical Jacobian matrix, shape (6, 6).
    """
    # Convert pose into a 6x1 array
    pose = np.array([*pose[0], *pose[1]])

    J_numerical = np.zeros((6, 6))
    original_lengths = compute_leg_lengths(plat_nodes, base_nodes, pose)

    for i in range(6):
        perturbed_pose = np.copy(pose)
        perturbed_pose[i] += delta
        perturbed_lengths = compute_leg_lengths(plat_nodes, base_nodes, perturbed_pose)
        length_diff = perturbed_lengths - original_lengths
        J_numerical[:, i] = length_diff / delta
    
    return J_numerical


#####################################################
#                  Pose Generation                  #
#####################################################

# "aggregated_results_more_points" done with num_points = 120, num_height_steps = 6, points_per_circle_increment = 6, num_circles = 6

num_circles = 6 # Number of concentric circles to generate at each layer height
points_per_circle_increment = 6  # How many points to add per circle
num_height_steps = 6
max_radius = 380
min_radius = 150
min_height = 700 
max_height = 850 
poses = generate_poses(max_radius, min_radius, num_circles, min_height, max_height, num_height_steps, points_per_circle_increment)
poses = check_and_delete_duplicate_poses(poses)

path = 'path_to_convex_hull.json'

### PLOTTING CONVEX HULL OVER POSES ###

# Load convex hull 
# convex_hull_points = load_convex_hull(path + 'convex_hull_points_big.json')
# plot_convex_hull_and_poses(convex_hull_points, poses, arrow_length=25)

# plot_poses(poses, arrow_length=25)

#####################################################
#               Initialize Geometry                 #
#####################################################

# Set fixed geometric parameters
baseRad = 410.0       # Radius of the base (mm)
baseSmallAngle = 24.0 # Gamma2 on main sketch (deg)
shortLeg = 640.5  # Shortest length of leg (mm) # Is actually 640.47
extLeg = 300.0   # Extension of leg (mm). Is actually 309.5mm
longLeg = shortLeg + extLeg  # Longest length of leg (mm)
legAngleLimit = 30  # Limit of how far the  joints allow the legs to tilt wrt x-y plane (deg) # GUESS
geometric_limits = [shortLeg, longLeg, legAngleLimit] # This just makes it easier to pass these limits to the plotting function

running_geometry_study = False

if running_geometry_study:
    start_time = time.time()
    # Set parameters to test
    num_points = 120
    # platRad = np.round(np.linspace(50.0, 500, num_points)) # Radius of the platform (mm)
    # platSmallAngle = np.round(np.linspace(0.0, 120.0, num_points)) # Lambda1 on main sketch (deg)
    platRad = np.arange(50, 501) # Radius of the platform (mm)
    platSmallAngle = np.arange(0, 121) # Lambda1 on main sketch (deg)

    print(f'platRad: {platRad}')
    print(f'platSmallAngle: {platSmallAngle}')

    # Define the dtype for the results array
    results_dtype = np.dtype([
        ('platRad', np.float64),
        ('platSmallAngle', np.float64),
        ('aggregated_results', np.float64, (10,)) # All the outputs of the aggregated results as well as sum of unreachable poses for all geometries
    ])


    # Initialize arrays to store the results
    results = np.zeros((len(platRad), len(platSmallAngle)), dtype=results_dtype)

    for i, rad in enumerate(platRad):
        # Print an update on the progress every 5% of the way
        if i % (num_points // 20) == 0:
            elapsed_time = time.time() - start_time
            hours = int(elapsed_time // 3600)
            minutes = int((elapsed_time % 3600) // 60)
            seconds = int(elapsed_time % 60)
            print(f'Progress: {i / num_points * 100:.0f}%. Time elapsed: {hours:02d}:{minutes:02d}:{seconds:02d}')

        for j, angle in enumerate(platSmallAngle):
            # Initialize a variable to track how many poses are unreachable for this geometry
            sum_unreachable = 0
            temp_results = np.zeros((len(poses), 4))  # Condition number, rank deficiency, submatrix condition number, norm score for each pose

            # Build the platform
            plat_nodes, base_nodes = platform_builder(baseRad, rad, baseSmallAngle, angle)

            # Step through each pose
            for k, pose in enumerate(poses):
                # print(f'Angle: {angle}, Radius: {rad}, Pose: {pose}, Pose[0]: {pose[0]}')

                # Check reachability
                reachable = check_reachability(pose, plat_nodes, base_nodes, shortLeg, longLeg, legAngleLimit)

                if not reachable:
                    sum_unreachable += 1
                    continue # Skip the rest of the loop if the pose is unreachable

                # Compute the Jacobian matrix
                J = compute_analytical_jacobian(plat_nodes, base_nodes, pose)

                # Evaluate the Jacobian matrix
                condition_number, submatrix_condition_number, rank_deficiency, norm_score = evaluate_jacobian(J)

                # Store the results
                temp_results[k] = [condition_number, rank_deficiency, submatrix_condition_number, norm_score]

            # Aggregate the results for this geometry
            aggregated_results = aggregate_scores(temp_results, sum_unreachable)

            # Store the aggregated results
            results[i, j] = (rad, angle, aggregated_results)

            # print(temp_results)
            # print(f'Sum unreachable: {sum_unreachable}')

    save_path = 'path_to_save_results'
    np.save(save_path + 'aggregated_results_even_more_points.npy', results)

# # Strange result:
# platRad = 100.0
# platSmallAngle = 100.0
# plat_nodes, base_nodes = platform_builder(baseRad, platRad, baseSmallAngle, platSmallAngle)
# pose_deg = np.array([0.0, 0.0, 700.0, 0.0, 0.0, 0.0])
# # pose_deg = np.array([143.0, 143.0, 750.0, np.rad2deg(0.185), np.rad2deg(-0.185), 0.0])
# pose_deg = (tuple(pose_deg[:3]), tuple(pose_deg[3:]))  # Convert to tuple for plotting
# # Set the normalized condition number threshold for plotting (1 is the maximum condition number for the given geometry/pose)
# condition_number_threshold = 0.0 # Poses with normalized condition numbers below this threshold will be invisible in the plot
# plot_geometry_with_poses(pose_deg, platRad, platSmallAngle, poses, plat_nodes, base_nodes, arrow_length=50, geom_limits=geometric_limits, normalized_cond_threshold=condition_number_threshold)
# plt.show()
# exit(1)

platRad = baseRad#240.0 # mm
platSmallAngle = 96.0 # deg

plat_nodes, base_nodes = platform_builder(baseRad, platRad, baseSmallAngle, platSmallAngle)

pose_deg = np.array([0.0, 0.0, 700.0, 0.0, 0.0, 0.0])
pose_deg = (tuple(pose_deg[:3]), tuple(pose_deg[3:]))  # Convert to tuple for plotting

reachable = check_reachability(pose_deg, plat_nodes, base_nodes, shortLeg, longLeg, legAngleLimit)

print(f'Reachability: {reachable}')

# Compute the analytical and numerical Jacobians
J_analytical = compute_analytical_jacobian(plat_nodes, base_nodes, pose_deg)
J_numerical = compute_numerical_jacobian(plat_nodes, base_nodes, pose_deg)

np.set_printoptions(precision=2)
print(f"Analytical Jacobian:\n{J_analytical}\n")
# print(f"Numerical Jacobian:\n{J_numerical}\n")
# print(f"Difference:\n{J_analytical - J_numerical}")

print('Sum of differences between analytic and numeric Jacobian:', np.sum(J_analytical - J_numerical))

results = evaluate_jacobian(J_analytical)
print(f"Condition Number: {results[0]:.2f}")
print(f"Submatrix Condition Number: {results[1]:.2f}")
print(f"Rank Deficiency: {results[2]:.2f}")
print(f"Norm Score: {results[3]:.2f}")

plot_geometry_with_poses(pose_deg, platRad, platSmallAngle, poses, plat_nodes, base_nodes, arrow_length=50, geom_limits=geometric_limits, hide_unreachable=False)
plt.show()

np.set_printoptions(precision=8) 
