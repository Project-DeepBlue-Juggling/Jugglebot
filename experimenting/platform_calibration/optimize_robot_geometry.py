import numpy as np
import pandas as pd
from scipy.optimize import least_squares, minimize, differential_evolution
import math
from transforms3d.quaternions import quat2mat  # for converting quaternion to rotation matrix
import matplotlib.pyplot as plt

##############################
# Global Calibration Limits  #
##############################

# Directional tolerances for base nodes (mm) relative to design values:
BASE_NODE_TOL_X = (-10, 10)   # x-axis tolerance
BASE_NODE_TOL_Y = (-10, 10)   # y-axis tolerance
BASE_NODE_TOL_Z = (-1, 35)    # z-axis tolerance

# Directional tolerances for platform nodes (mm) relative to design values:
PLATFORM_NODE_TOL_X = (-20, 20)
PLATFORM_NODE_TOL_Y = (-20, 20)
PLATFORM_NODE_TOL_Z = (-20, 20)

# Tolerance for retracted lengths (mm) relative to design values:
RETRACTED_LENGTH_TOL = (-50, 50)

# Tolerance for conversion factors (percentage deviation relative to design):
CONVERSION_FACTOR_TOL = (-0.10, 0.10)  # ±10%

# Penalty weights for soft constraints:
PENALTY_WEIGHT_BASE = 10.0
PENALTY_WEIGHT_PLATFORM = 10.0
PENALTY_WEIGHT_RETRACTED_DEV = 10.0
PENALTY_WEIGHT_CONVERSION = 10.0
# Also penalize the overall range of retracted lengths if needed.
PENALTY_WEIGHT_RANGE = 10.0

##############################
# Data and Helper Functions  #
##############################

def compute_robot_geometry():
    # Design model parameters (mm and deg)
    initial_height = 565.0      
    base_radius = 410.0         
    plat_radius = 219.075       
    base_small_angle = 20.0     
    plat_small_angle = 8.6024446  
    plat_x_axis_offset = 154.3012223  

    deg_to_rad = math.pi / 180

    gamma2 = base_small_angle
    gamma0 = 210 - gamma2 / 2
    gamma1 = 120 - gamma2

    lambda1 = plat_small_angle
    lambda2 = 120 - lambda1
    lambda0 = plat_x_axis_offset

    base_nodes = np.zeros((6, 3))
    init_plat_nodes = np.zeros((6, 3))
    for node in range(6):
        first_angle_index = int(math.floor(node / 2))
        second_angle_index = int(math.floor((node + 1) / 2))
        angle_base = gamma0 + gamma1 * first_angle_index + gamma2 * second_angle_index
        angle_plat = lambda0 + lambda1 * first_angle_index + lambda2 * second_angle_index

        base_nodes[node, 0] = base_radius * math.cos(angle_base * deg_to_rad)
        base_nodes[node, 1] = base_radius * math.sin(angle_base * deg_to_rad)
        base_nodes[node, 2] = 0

        init_plat_nodes[node, 0] = plat_radius * math.cos(angle_plat * deg_to_rad)
        init_plat_nodes[node, 1] = plat_radius * math.sin(angle_plat * deg_to_rad)
        init_plat_nodes[node, 2] = 0

    start_pos = np.array([0, 0, initial_height])
    init_leg_lengths = np.linalg.norm(init_plat_nodes + start_pos - base_nodes, axis=1)

    return base_nodes, init_plat_nodes, init_leg_lengths

##############################
# STEP 1: Load Calibration Data
##############################

main_folder_path = '/home/jetson/Desktop/Jugglebot/experimenting/platform_calibration/'
data = pd.read_csv(main_folder_path + '108_calibration_poses.csv')

leg_data = data[['leg_length_1', 'leg_length_2', 'leg_length_3', 
                 'leg_length_4', 'leg_length_5', 'leg_length_6']].to_numpy()  # shape (108,6)
pose_positions = data[['position_x', 'position_y', 'position_z']].to_numpy()      # shape (108,3)
pose_quaternions = data[['orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']].to_numpy()  # shape (108,4)

##############################
# STEP 2: Define Optimization Variables
##############################

base_points_init, platform_points_init, retracted_lengths_init = compute_robot_geometry()

# Manually measured conversion mapping: values are in rev/mm.
mm_to_rev = np.array([13.21144, 14.0456, 13.8233, 14.124109, 14.16007, 14.03256]) * 1e-3
# Compute initial conversion factors (mm/rev) as the reciprocal.
conversion_factors_init = 1 / mm_to_rev

def pack_params(base, platform, retracted, conv):
    return np.hstack([base.flatten(), platform.flatten(), retracted, conv])

def unpack_params(params):
    base = params[0:18].reshape(6, 3)
    platform = params[18:36].reshape(6, 3)
    retracted = params[36:42]
    conv = params[42:48]
    return base, platform, retracted, conv

initial_params = pack_params(base_points_init, platform_points_init,
                             retracted_lengths_init, conversion_factors_init)

##############################
# STEP 3: Residual Function with Penalty Constraints
##############################

def residuals(params):
    base, platform, retracted, conv = unpack_params(params)
    res = []
    # Measurement residuals.
    for j in range(len(pose_positions)):
        R = quat2mat(pose_quaternions[j])
        t = pose_positions[j]
        for i in range(6):
            platform_point_global = R @ platform[i] + t
            L_model = np.linalg.norm(platform_point_global - base[i]) - retracted[i]
            L_meas = conv[i] * leg_data[j, i]
            res.append(L_model - L_meas)
    
    # Penalty: Base nodes must remain within directional limits.
    for i in range(6):
        for d, tol in enumerate([BASE_NODE_TOL_X, BASE_NODE_TOL_Y, BASE_NODE_TOL_Z]):
            diff = base[i, d] - base_points_init[i, d]
            if diff < tol[0]:
                res.append(PENALTY_WEIGHT_BASE * (tol[0] - diff))
            elif diff > tol[1]:
                res.append(PENALTY_WEIGHT_BASE * (diff - tol[1]))
    
    # Penalty: Platform nodes directional limits.
    for i in range(6):
        for d, tol in enumerate([PLATFORM_NODE_TOL_X, PLATFORM_NODE_TOL_Y, PLATFORM_NODE_TOL_Z]):
            diff = platform[i, d] - platform_points_init[i, d]
            if diff < tol[0]:
                res.append(PENALTY_WEIGHT_PLATFORM * (tol[0] - diff))
            elif diff > tol[1]:
                res.append(PENALTY_WEIGHT_PLATFORM * (diff - tol[1]))
    
    # Penalty: Retracted lengths must remain within limits.
    for i in range(6):
        diff = retracted[i] - retracted_lengths_init[i]
        if diff < RETRACTED_LENGTH_TOL[0]:
            res.append(PENALTY_WEIGHT_RETRACTED_DEV * (RETRACTED_LENGTH_TOL[0] - diff))
        elif diff > RETRACTED_LENGTH_TOL[1]:
            res.append(PENALTY_WEIGHT_RETRACTED_DEV * (diff - RETRACTED_LENGTH_TOL[1]))
    
    # Penalty: Conversion factors must remain within percentage limits.
    for i in range(6):
        perc_dev = (conv[i] / conversion_factors_init[i]) - 1
        if perc_dev < CONVERSION_FACTOR_TOL[0]:
            res.append(PENALTY_WEIGHT_CONVERSION * (CONVERSION_FACTOR_TOL[0] - perc_dev))
        elif perc_dev > CONVERSION_FACTOR_TOL[1]:
            res.append(PENALTY_WEIGHT_CONVERSION * (perc_dev - CONVERSION_FACTOR_TOL[1]))
    
    # Optionally, penalize overall range of retracted lengths.
    retracted_range = np.max(retracted) - np.min(retracted)
    allowed_range = RETRACTED_LENGTH_TOL[1] - RETRACTED_LENGTH_TOL[0]
    res.append(PENALTY_WEIGHT_RANGE * max(0, retracted_range - allowed_range))
    
    return np.array(res)

def objective_scalar(params):
    return np.sum(residuals(params)**2)

##############################
# STEP 4: Directional Bounds Function
##############################

def get_bounds():
    bounds = []
    # Base nodes: apply directional tolerances.
    for i in range(6):
        # x coordinate:
        init_val = base_points_init[i, 0]
        bounds.append((init_val + BASE_NODE_TOL_X[0], init_val + BASE_NODE_TOL_X[1]))
        # y coordinate:
        init_val = base_points_init[i, 1]
        bounds.append((init_val + BASE_NODE_TOL_Y[0], init_val + BASE_NODE_TOL_Y[1]))
        # z coordinate:
        init_val = base_points_init[i, 2]
        bounds.append((init_val + BASE_NODE_TOL_Z[0], init_val + BASE_NODE_TOL_Z[1]))
    # Platform nodes: directional tolerances.
    for i in range(6):
        init_val = platform_points_init[i, 0]
        bounds.append((init_val + PLATFORM_NODE_TOL_X[0], init_val + PLATFORM_NODE_TOL_X[1]))
        init_val = platform_points_init[i, 1]
        bounds.append((init_val + PLATFORM_NODE_TOL_Y[0], init_val + PLATFORM_NODE_TOL_Y[1]))
        init_val = platform_points_init[i, 2]
        bounds.append((init_val + PLATFORM_NODE_TOL_Z[0], init_val + PLATFORM_NODE_TOL_Z[1]))
    # Retracted lengths:
    for i in range(6):
        init_val = retracted_lengths_init[i]
        bounds.append((init_val + RETRACTED_LENGTH_TOL[0], init_val + RETRACTED_LENGTH_TOL[1]))
    # Conversion factors: allowed percentage variation.
    for i in range(6):
        init_val = conversion_factors_init[i]
        bounds.append((init_val * (1 + CONVERSION_FACTOR_TOL[0]), init_val * (1 + CONVERSION_FACTOR_TOL[1])))
    return bounds

##############################################
# Solver Classes
##############################################

class LSOptimizer:
    def __init__(self, method):
        self.method = method
        self.result = None
        self.solution = None

    def solve(self, initial_params):
        self.result = least_squares(residuals, initial_params, method=self.method, verbose=2)
        self.solution = unpack_params(self.result.x)
        return self.solution

    def print_solution(self):
        base, platform, retracted, conv = self.solution
        print(f"LSOptimizer ({self.method}) solution:")
        print("Base Attachment Points:\n", base)
        print("Platform Attachment Points:\n", platform)
        print("Fully Retracted Leg Lengths:\n", retracted)
        print("Conversion Factors (mm/rev):\n", conv)
        print("Final cost (sum of squares):", np.sum(residuals(self.result.x)**2))
        print()

class ScalarMinimizer:
    def __init__(self):
        self.result = None
        self.solution = None

    def solve(self, initial_params):
        self.result = minimize(objective_scalar, initial_params, method='trust-constr', options={'verbose': 2})
        self.solution = unpack_params(self.result.x)
        return self.solution

    def print_solution(self):
        base, platform, retracted, conv = self.solution
        print("ScalarMinimizer (trust-constr) solution:")
        print("Base Attachment Points:\n", base)
        print("Platform Attachment Points:\n", platform)
        print("Fully Retracted Leg Lengths:\n", retracted)
        print("Conversion Factors (mm/rev):\n", conv)
        print("Final cost (sum of squares):", objective_scalar(self.result.x))
        print()

class DiffEvolSolver:
    def __init__(self, quick_mode=False):
        self.quick_mode = quick_mode
        self.result = None
        self.solution = None
        self.bounds = get_bounds()

    def solve(self, initial_params):
        # Use lower iteration/population for quick testing.
        maxiter = 50 if self.quick_mode else 200
        popsize = 5 if self.quick_mode else 15
        self.result = differential_evolution(objective_scalar, self.bounds, maxiter=maxiter, popsize=popsize, disp=True)
        self.solution = unpack_params(self.result.x)
        return self.solution

    def print_solution(self):
        base, platform, retracted, conv = self.solution
        print("Differential Evolution solution:")
        print("Base Attachment Points:\n", base)
        print("Platform Attachment Points:\n", platform)
        print("Fully Retracted Leg Lengths:\n", retracted)
        print("Conversion Factors (mm/rev):\n", conv)
        print("Final cost (sum of squares):", objective_scalar(self.result.x))
        print()

class HybridSolver:
    """
    A hybrid solver that first uses differential evolution for a global search,
    then refines the result with least_squares.
    """
    def __init__(self, quick_mode=False):
        self.quick_mode = quick_mode
        self.de_result = None
        self.ls_result = None
        self.solution = None

    def solve(self, initial_params):
        bounds = get_bounds()
        # Global search using DE.
        maxiter = 50 if self.quick_mode else 200
        popsize = 5 if self.quick_mode else 15
        self.de_result = differential_evolution(objective_scalar, bounds, maxiter=maxiter, popsize=popsize, disp=True)
        # Refine with local least-squares.
        self.ls_result = least_squares(residuals, self.de_result.x, verbose=2)
        self.solution = unpack_params(self.ls_result.x)
        return self.solution

    def print_solution(self):
        base, platform, retracted, conv = self.solution
        print("Hybrid (DE + LS) solution:")
        print("Base Attachment Points:\n", base)
        print("Platform Attachment Points:\n", platform)
        print("Fully Retracted Leg Lengths:\n", retracted)
        print("Conversion Factors (mm/rev):\n", conv)
        print("Final cost (sum of squares):", np.sum(residuals(self.ls_result.x)**2))
        print()

##############################################
# Validation Functions (using 60-pose data)
##############################################

def evaluate_solution(params, leg_data_val, pose_positions_val, pose_quaternions_val):
    base, platform, retracted, conv = unpack_params(params)
    errors = []
    for j in range(len(pose_positions_val)):
        R = quat2mat(pose_quaternions_val[j])
        t = pose_positions_val[j]
        for i in range(6):
            platform_point_global = R @ platform[i] + t
            L_model = np.linalg.norm(platform_point_global - base[i]) - retracted[i]
            L_meas = conv[i] * leg_data_val[j, i]
            errors.append(L_model - L_meas)
    return errors

def compute_error_statistics(errors):
    errors = np.array(errors)
    mean_err = np.mean(errors)
    std_err = np.std(errors)
    max_err = np.max(errors)
    min_err = np.min(errors)
    rms_err = np.sqrt(np.mean(errors**2))
    return mean_err, std_err, max_err, min_err, rms_err

def validate_solution(label, params, validation_file):
    data_val = pd.read_csv(validation_file)
    leg_data_val = data_val[['leg_length_1', 'leg_length_2', 'leg_length_3',
                              'leg_length_4', 'leg_length_5', 'leg_length_6']].to_numpy()
    pose_positions_val = data_val[['position_x', 'position_y', 'position_z']].to_numpy()
    pose_quaternions_val = data_val[['orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']].to_numpy()
    
    errors = evaluate_solution(params, leg_data_val, pose_positions_val, pose_quaternions_val)
    mean_err, std_err, max_err, min_err, rms_err = compute_error_statistics(errors)
    print(f"Validation Results for {label}:")
    print(f"  Mean error: {mean_err:.3f} mm")
    print(f"  Std error: {std_err:.3f} mm")
    print(f"  Max error: {max_err:.3f} mm")
    print(f"  Min error: {min_err:.3f} mm")
    print(f"  RMS error: {rms_err:.3f} mm")
    print()

##############################################
# Solver Selection
##############################################

# Specify which solvers to run by listing their names.
# Available options: 'LS_TRF', 'LS_DOGBOX', 'MIN_TC', 'DE', 'HYBRID'
active_solver_names = ['LS_TRF', 'LS_DOGBOX', 'MIN_TC', 'DE', 'HYBRID']

solutions = {}

if 'LS_TRF' in active_solver_names:
    solver = LSOptimizer(method='trf')
    solutions['LS_TRF'] = solver.solve(initial_params)
    solver.print_solution()

if 'LS_DOGBOX' in active_solver_names:
    solver = LSOptimizer(method='dogbox')
    solutions['LS_DOGBOX'] = solver.solve(initial_params)
    solver.print_solution()

if 'MIN_TC' in active_solver_names:
    solver = ScalarMinimizer()
    solutions['MIN_TC'] = solver.solve(initial_params)
    solver.print_solution()

if 'DE' in active_solver_names:
    # For quick testing, set quick_mode=True; set to False for full convergence.
    solver = DiffEvolSolver(quick_mode=True)
    solutions['DE'] = solver.solve(initial_params)
    solver.print_solution()

if 'HYBRID' in active_solver_names:
    solver = HybridSolver(quick_mode=True)
    solutions['HYBRID'] = solver.solve(initial_params)
    solver.print_solution()

##############################################
# Visualization of Calibration Results
##############################################

plt.figure(figsize=(14, 6))
plt.subplot(1,2,1)
plt.title("Base Attachment Points")
plt.scatter(base_points_init[:,0], base_points_init[:,1], c='blue', label='Initial', marker='o', s=100)
colors = {'LS_TRF':'red', 'LS_DOGBOX':'green', 'MIN_TC':'magenta', 'DE':'orange', 'HYBRID':'purple'}
for key, sol in solutions.items():
    base_sol = sol[0]
    plt.scatter(base_sol[:,0], base_sol[:,1], c=colors[key], label=key, marker='x', s=100)
for i in range(6):
    plt.annotate(f"{i+1}", (base_points_init[i,0], base_points_init[i,1]), color='blue')
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.legend()
plt.axis('equal')

plt.subplot(1,2,2)
plt.title("Platform Attachment Points (Platform Frame)")
plt.scatter(platform_points_init[:,0], platform_points_init[:,1], c='blue', label='Initial', marker='o', s=100)
for key, sol in solutions.items():
    platform_sol = sol[1]
    plt.scatter(platform_sol[:,0], platform_sol[:,1], c=colors[key], label=key, marker='x', s=100)
for i in range(6):
    plt.annotate(f"{i+1}", (platform_points_init[i,0], platform_points_init[i,1]), color='blue')
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.legend()
plt.axis('equal')
plt.tight_layout()
plt.show()

plt.figure(figsize=(10,6))
index = np.arange(6)
width = 0.15
i = 0
for key, sol in solutions.items():
    retracted_sol = sol[2]
    plt.bar(index + i*width, retracted_sol, width, label=key)
    i += 1
plt.axhline(0, color='black', linewidth=0.5)
plt.title("Optimized Fully Retracted Leg Lengths")
plt.xlabel("Leg Number")
plt.ylabel("Retraction (mm)")
plt.xticks(index + width, [str(i+1) for i in range(6)])
plt.legend()
plt.tight_layout()
plt.show()

##############################################
# Validation Using the 60-Pose Dataset
##############################################

validation_file = main_folder_path + '60_calibration_poses.csv'
print("\nValidation using 60-pose dataset:")
validate_solution("Original Geometry", initial_params, validation_file)
validate_solution("Optimized Geometry (Hybrid)", HybridSolver(quick_mode=True).ls_result.x if 'HYBRID' in active_solver_names else None, validation_file)