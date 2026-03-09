# Kinematics

This page covers how platform poses are converted to leg lengths (inverse kinematics), how velocities and accelerations are mapped through the Jacobian, and how leg lengths can be converted back to poses (forward kinematics).

**Source files:**

- [geometry.py](https://github.com/PDJ/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/geometry.py) — platform dimensions
- [ik_solver.py](https://github.com/PDJ/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py) — all kinematic computations
- [conversions.py](https://github.com/PDJ/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/conversions.py) — mm/rev and force/torque conversions

## Platform Geometry

The Stewart platform has 6 legs connecting base nodes to platform nodes. The geometry is loaded from `hardware_config.yaml` by the `StewartGeometry` class.

```python
from jugglebot.motion.geometry import StewartGeometry
geom = StewartGeometry()
```

### Key Dimensions

| Property | Value | Description |
|---|---|---|
| `base_nodes` | 6×3 array | Base attachment points (mm), in the base frame |
| `plat_nodes` | 6×3 array | Platform attachment points (mm), in the platform body frame |
| `init_height_mm` | 574.3 | Home height: distance from base to platform centre (mm) |
| `leg_stroke_mm` | 280 | Total available leg extension range (mm) |
| `spool_radius_mm` | ~11.4 per leg | String spool radius, derived from `mm_to_rev` conversion |
| `ball_joint_offset_mm` | offset | Ball joint offset applied to initial leg lengths |
| `init_leg_lengths_mm` | 6-element | Leg lengths at the home pose (mm) |

### Leg Extension Convention

Leg extensions are measured from fully retracted:

- `0 mm` = fully retracted 
- `280 mm` = fully extended
- At home, each leg is at its `init_leg_lengths_mm` value (somewhere in the middle of the stroke)

## Position Inverse Kinematics

**Function:** `pose_to_leg_lengths(pos, rot, geom)` → 6 leg extensions in mm

Given a platform pose (position offset from home + rotation matrix), compute the length of each leg.

### How It Works

Each leg connects a fixed base node $\mathbf{b}_i$ to a moving platform node. The platform node's position in the base frame is:

$$\mathbf{p}_i = \mathbf{pos} + \begin{bmatrix} 0 \\ 0 \\ h_{\text{init}} \end{bmatrix} + \mathbf{R} \cdot \mathbf{a}_i$$

where $\mathbf{a}_i$ is the platform node in the body frame, $\mathbf{R}$ is the rotation matrix, and $h_{\text{init}}$ is the home height.

The leg vector is $\mathbf{l}_i = \mathbf{p}_i - \mathbf{b}_i$, and the leg length is $\|\mathbf{l}_i\|$.

The extension is:

$$\text{extension}_i = \|\mathbf{l}_i\| - L_{i,\text{init}} + L_{i,\text{init\_ext}}$$

where $L_{i,\text{init}}$ is the geometric leg length at home and $L_{i,\text{init\_ext}}$ is the home extension.

### Usage

```python
from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix
import numpy as np

pos = np.array([0.0, 0.0, 10.0])        # 10mm above home
rot = rotvec_to_rot_matrix(np.zeros(3))  # no tilt
extensions = pose_to_leg_lengths(pos, rot, geom)  # (6,) array in mm
```

!!! warning
    `pose_to_leg_lengths` does **not** check whether the extensions are within the stroke range. Use `check_leg_extensions()` from `workspace.py` or `check_reachability()` to verify.

## The Jacobian

**Function:** `compute_jacobian(pos, rot, geom)` → 6×6 matrix

The Jacobian $\mathbf{J}$ maps platform twist (linear + angular velocity) to leg extension rates:

$$\dot{\mathbf{q}} = \mathbf{J} \cdot \begin{bmatrix} v_x \\ v_y \\ v_z \\ \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}$$

where $\dot{\mathbf{q}}$ is a vector of 6 leg extension rates in mm/s.

### Construction

Each row $i$ of the Jacobian is constructed from the unit leg direction $\hat{\mathbf{l}}_i$ and the moment arm $\mathbf{r}_i \times \hat{\mathbf{l}}_i$:

$$\mathbf{J}_{i,:} = \begin{bmatrix} \hat{\mathbf{l}}_i^T & (\mathbf{r}_i \times \hat{\mathbf{l}}_i)^T \end{bmatrix}$$

where $\mathbf{r}_i = \mathbf{R} \cdot \mathbf{a}_i$ is the platform node position relative to the platform centre, expressed in the base frame.

### Mixed Units

The first three columns of $\mathbf{J}$ relate translational velocity (mm/s) to leg velocity (mm/s) — they are dimensionless direction cosines. The last three columns relate angular velocity (rad/s) to leg velocity (mm/s) — they have units of mm/rad.

This means the raw condition number of $\mathbf{J}$ is ~450 at home, which seems high but is normal for this unit convention. All condition-number thresholds in the system use **relative** values (multiples of the home condition number) rather than absolute values.

### Condition Number

The condition number $\kappa(\mathbf{J})$ indicates how sensitive leg motions are to Cartesian motions. A high condition number means small Cartesian motions require large (or uneven) leg motions — indicating proximity to a singularity.

At home: $\kappa \approx 450$. Across the reachable workspace: $\kappa \in [449, 644]$.

Thresholds used in the system:

| Threshold | Value | Used By |
|---|---|---|
| Soft limit | $1.5 \times \kappa_{\text{home}} \approx 675$ | `workspace.py` — speed ramp-down |
| Hard limit | $2.0 \times \kappa_{\text{home}} \approx 900$ | `workspace.py` — trajectory abort; `trajectory.py` — feasibility rejection |

## Velocity Inverse Kinematics

**Function:** `twist_to_leg_velocities(twist, pos, rot, geom)` → 6 leg velocities in mm/s

Simply computes $\dot{\mathbf{q}} = \mathbf{J} \cdot \mathbf{twist}$. The result is converted to motor velocities (rev/s) for the `vel_ff` field via `leg_velocities_to_motor_velocities()`.

## Acceleration Inverse Kinematics

**Function:** `accel_to_leg_accels(accel, twist, pos, rot, geom)` → 6 leg accelerations in mm/s²

Accounts for the Jacobian's time derivative:

$$\ddot{\mathbf{q}} = \mathbf{J} \cdot \ddot{\mathbf{x}} + \dot{\mathbf{J}} \cdot \dot{\mathbf{x}}$$

The $\dot{\mathbf{J}}$ term (bias acceleration) is computed numerically via finite differences in `compute_jacobian_dot()` with a timestep of $10^{-7}$ seconds.

This is used by the dynamics model to compute reflected motor inertia feedforward.

## Forward Kinematics

**Function:** `leg_lengths_to_pose(extensions_mm, geom)` → (pos, rot)

Given 6 leg extensions, find the platform pose. This is an iterative solution using Newton-Raphson:

1. Start from an initial guess (defaults to home pose)
2. Compute leg lengths at current guess via position IK
3. Compute the error between target and current leg lengths
4. Update the guess using the Jacobian: $\Delta\mathbf{x} = \mathbf{J}^{-1} \cdot \Delta\mathbf{q}$
5. Repeat until error < $10^{-10}$ mm or 50 iterations

FK is used primarily for validation (IK → FK round-trip tests), not in the real-time control loop.

## Rotation Utilities

The `ik_solver` module includes several rotation representation converters:

| Function | Conversion |
|---|---|
| `rotvec_to_rot_matrix(rotvec)` | Rotation vector (axis×angle) → 3×3 matrix, via Rodrigues formula |
| `rot_matrix_to_rotvec(R)` | 3×3 matrix → rotation vector |
| `quat_to_rot_matrix(w, x, y, z)` | Quaternion → 3×3 matrix (auto-normalizes) |
| `rot_matrix_to_quat(R)` | 3×3 matrix → quaternion `(w, x, y, z)`, via Shepperd's method |
| `skew(v)` | 3-vector → 3×3 skew-symmetric matrix |

Shepperd's method for `rot_matrix_to_quat` uses a 4-branch selection that avoids the numerical instability of the standard formula near 180° rotations.

## Unit Conversions

The `conversions.py` module provides bidirectional conversions between leg space (mm) and motor space (revolutions):

### Position

```python
from jugglebot.motion.conversions import extensions_mm_to_revs, revs_to_extensions_mm

motor_revs = extensions_mm_to_revs(extensions_mm, geom)  # mm -> rev
extensions = revs_to_extensions_mm(motor_revs, geom)      # rev -> mm
```

The conversion uses per-leg `mm_to_rev` factors from the hardware config (each leg's spool may differ slightly).

### Velocity

```python
from jugglebot.motion.conversions import leg_velocities_to_motor_velocities

motor_vel_rps = leg_velocities_to_motor_velocities(leg_vel_mm_s, geom)
```

Same linear scaling as position (velocity in mm/s × mm_to_rev factor = velocity in rev/s).

### Force / Torque

```python
from jugglebot.motion.conversions import leg_forces_to_motor_torques

motor_torques_Nm = leg_forces_to_motor_torques(leg_forces_N, geom)
```

Converts via the spool radius: $\tau = F \cdot r_{\text{spool}}$. The spool radius is derived from the `mm_to_rev` conversion factor: $r = 1 / (2\pi \cdot \text{mm_to_rev})$.

## Verification

The kinematics module was validated with 6 offline tests (all PASS):

| Test | Method | Result |
|---|---|---|
| Regression vs legacy IK | Compare against `sp_ik.py` | Exact match (0.00 mm error) |
| Numerical Jacobian | Analytical vs finite-difference, 30 poses | Max error 1.11×10⁻⁶ |
| Round-trip integration | Twist → leg velocities → RK4 integrate → compare | Max error 2.19×10⁻⁵ mm/s |
| Bias term (J-dot) | Analytical vs numerical J-dot | Max error 1.71×10⁻⁶ mm/s² |
| FK round-trip | IK → FK → compare, 20 poses | Exact match (0.00 mm, 0.00 rad) |
| Singularity map | Sweep workspace | 929/1944 poses reachable, cond 449–644 |

See [Validation Results](results.md) for the full test record.
