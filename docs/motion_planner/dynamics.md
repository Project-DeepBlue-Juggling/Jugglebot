# Dynamics

This page covers how the motion planner computes motor torques from the physics of the system — gravity, platform inertia, and reflected motor inertia. These torques are sent as the `torque_ff` feedforward field to the ODrive controllers.

**Source files:**

- [dynamics.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/dynamics.py) — gravity wrench, inertia wrench, feedforward torques
- [conversions.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/conversions.py) — leg force to motor torque conversion

## Why Feedforward Torques?

Without feedforward, the ODrive's PID has to discover the gravity load by observing position error — the platform sags, error accumulates, and the PID reacts. With feedforward, the motion planner tells each motor "you need to apply this much torque to counteract gravity" before any error develops.

At static holds on this platform, motor stiction (~0.075 Nm per leg) actually exceeds the gravity load per leg (~0.018 Nm), so gravity feedforward has minimal visible effect. But during fast dynamic motions (trajectory tracking, ball catching), the inertia forces become significant and feedforward substantially reduces tracking error.

The feedforward has three components, added together:

```
torque_ff = gravity_torque + platform_inertia_torque + reflected_motor_inertia_torque
```

## Dynamics Parameters

```python
from jugglebot.motion.dynamics import DynamicsParams

params = DynamicsParams.from_config()
```

| Parameter | Value | Source |
|---|---|---|
| `mass_kg` | 0.96 | Platform + payload mass |
| `com_offset_mm` | [-14.5, -67.0, 54.0] | Centre of mass offset from platform centre, in body frame |
| `gravity_mps2` | 9.806 | Gravitational acceleration |
| `inertia_tensor_kgmm2` | 3×3 matrix | Platform rotational inertia from Onshape CAD |
| `motor_rotor_inertia_kgm2` | 2.75×10⁻⁴ | D6374 motor rotor inertia |

## Component 1: Gravity Wrench

**Function:** `compute_gravity_wrench(rot, params)` → 6-element wrench [Fx, Fy, Fz, τx, τy, τz]

The gravity wrench represents the force and torque that gravity exerts on the platform, expressed in the base (world) frame.

### Force Component

The gravitational force is always straight down:

$$\mathbf{F}_{\text{gravity}} = \begin{bmatrix} 0 \\ 0 \\ -m \cdot g \end{bmatrix}$$

Units: Newtons.

### Torque Component

If the centre of mass is offset from the platform centre, gravity creates a torque about the platform centre:

$$\boldsymbol{\tau}_{\text{gravity}} = \mathbf{r}_{\text{com,world}} \times \mathbf{F}_{\text{gravity}}$$

where $\mathbf{r}_{\text{com,world}} = \mathbf{R} \cdot \mathbf{r}_{\text{com,body}}$ rotates the body-frame CoM offset into the world frame.

Units: N·mm (because the offset is in mm).

### Support Wrench

The legs need to provide the **opposite** of the gravity wrench to hold the platform in place:

$$\mathbf{W}_{\text{support}} = -\mathbf{W}_{\text{gravity}}$$

## Component 2: Platform Inertia Wrench

**Function:** `compute_inertia_wrench(rot, twist, accel, params)` → 6-element wrench

During acceleration, the platform's mass and rotational inertia must be overcome. This is the full Newton-Euler formulation.

### Translational Inertia

The force required to accelerate the platform mass, accounting for the CoM offset:

$$\mathbf{F}_{\text{inertia}} = m \cdot \mathbf{a}_{\text{com}}$$

where $\mathbf{a}_{\text{com}}$ includes both the linear acceleration of the platform centre and the centripetal/angular acceleration effects at the offset CoM position.

### Rotational Inertia

The torque required to angularly accelerate the platform:

$$\boldsymbol{\tau}_{\text{inertia}} = \mathbf{I}_{\text{world}} \cdot \boldsymbol{\alpha} + \boldsymbol{\omega} \times (\mathbf{I}_{\text{world}} \cdot \boldsymbol{\omega})$$

where $\mathbf{I}_{\text{world}} = \mathbf{R} \cdot \mathbf{I}_{\text{body}} \cdot \mathbf{R}^T$ is the inertia tensor rotated into the world frame, and $\boldsymbol{\omega} \times (\mathbf{I} \cdot \boldsymbol{\omega})$ is the gyroscopic coupling term.

### Unit Scaling

Internally, accelerations are in mm/s² and the inertia tensor is in kg·mm². The function handles the mixed unit conversions to produce forces in N and torques in N·mm.

## From Wrench to Leg Forces

**Function:** `gravity_to_leg_forces(pos, rot, geom, params)` → 6 leg forces (N)

The critical step: decomposing a 6D wrench into 6 scalar leg forces. This uses the Jacobian transpose relationship from the principle of virtual work:

$$\mathbf{J}^T \cdot \mathbf{f} = \mathbf{W}$$

Solving for leg forces:

$$\mathbf{f} = (\mathbf{J}^T)^{-1} \cdot \mathbf{W}_{\text{support}}$$

In code, this is computed as `np.linalg.solve(J.T, W_support)` — which is numerically more stable than explicitly computing the matrix inverse.

!!! note
    The formula is $\mathbf{f} = \mathbf{J}^{-T} \cdot \mathbf{W}$, **not** $\mathbf{f} = \mathbf{J}^T \cdot \mathbf{W}$. This is a common source of confusion. The Jacobian transpose maps leg forces to wrenches; we need the **inverse** to go the other direction.

## From Leg Forces to Motor Torques

**Function:** `leg_forces_to_motor_torques(forces_N, geom)` → 6 motor torques (Nm)

Each leg's string wraps around a spool of radius $r_{\text{spool}}$. The motor torque is simply:

$$\tau_{\text{motor}} = F_{\text{leg}} \cdot r_{\text{spool}}$$

The spool radius is derived from the `mm_to_rev` conversion factor in the hardware config:

$$r_{\text{spool}} = \frac{1}{2\pi \cdot \text{mm_to_rev}}$$

Each leg may have a slightly different spool radius due to manufacturing variation.

## Component 3: Reflected Motor Inertia

**Function:** `compute_reflected_inertia(geom, rotor_inertia_kgm2)` → 6 reflected inertias (kg)

When a leg accelerates, it must also accelerate the motor rotor through the spool. The reflected inertia per leg is:

$$J_{\text{reflected}} = \frac{J_{\text{rotor}}}{r_{\text{spool}}^2}$$

The reflected inertia torque per leg during acceleration is:

$$\tau_{\text{reflected}} = J_{\text{reflected}} \cdot \ddot{q}_{\text{motor}} \cdot r_{\text{spool}}$$

where $\ddot{q}_{\text{motor}}$ is the motor angular acceleration (rad/s²), computed from leg acceleration via the spool radius.

## Full Feedforward Computation

**Function:** `compute_full_feedforward_torques(pos, rot, twist, accel, geom, params)` → 6 motor torques (Nm)

This is the top-level function called by the control loop every cycle. It combines all three components:

```
1. Compute gravity wrench → W_gravity
2. Compute inertia wrench → W_inertia  (zero at static holds)
3. Total wrench: W_support = -(W_gravity + W_inertia)
4. Decompose to leg forces: f = solve(J.T, W_support)
5. Convert to motor torques: τ = f * r_spool
6. Add reflected motor inertia per leg
7. Return total τ_ff (6 values)
```

### Typical Values

| Scenario | Per-leg gravity torque | Per-leg inertia torque | Motor stiction |
|---|---|---|---|
| Static hold at active pose | ~0.018 Nm | 0 Nm | ~0.075 Nm |
| Moderate trajectory (50% speed) | ~0.018 Nm | ~0.005 Nm | ~0.075 Nm |
| Fast trajectory (100% speed) | ~0.018 Nm | ~0.02 Nm | ~0.075 Nm |
| Ball-catching speed | ~0.018 Nm | significant | ~0.075 Nm |

At current speeds, stiction dominates. The inertia feedforward becomes important at the high accelerations expected during ball catching.

### CAN Encoding Note

The `torque_ff` value is sent over CAN as an `int16` with 0.0001 Nm resolution (scale factor 10000). A typical per-leg gravity torque of 0.018 Nm encodes to ~180 counts, providing good resolution. The ODrive's 8 kHz PID absorbs any residual quantization error.

## API Reference

### DynamicsParams

```python
@dataclass
class DynamicsParams:
    mass_kg: float
    com_offset_mm: np.ndarray       # (3,) body frame
    gravity_mps2: float
    inertia_tensor_kgmm2: np.ndarray  # (3, 3), optional
    motor_rotor_inertia_kgm2: float   # optional, default 0

    @classmethod
    def from_config(cls) -> 'DynamicsParams'
```

### Functions

| Function | Input | Output |
|---|---|---|
| `compute_gravity_wrench(rot, params)` | Rotation matrix, params | (6,) wrench [N, N·mm] |
| `compute_inertia_wrench(rot, twist, accel, params)` | Rotation, twist, accel, params | (6,) wrench [N, N·mm] |
| `gravity_to_leg_forces(pos, rot, geom, params)` | Pose, geometry, params | (6,) forces [N] |
| `gravity_to_motor_torques(pos, rot, geom, params)` | Pose, geometry, params | (6,) torques [Nm] |
| `compute_full_feedforward_torques(pos, rot, twist, accel, geom, params)` | Full state, geom, params | (6,) torques [Nm] |
| `compute_reflected_inertia(geom, rotor_inertia)` | Geometry, rotor inertia | (6,) reflected inertia [kg] |

## Verification

The dynamics model was validated with 7 offline tests and multiple hardware tests:

| Test | Result |
|---|---|
| Gravity wrench at active pose (level) | Zero torque component (CoM torque only when tilted) |
| Gravity wrench at tilt | Torque direction and magnitude match geometry |
| Leg force decomposition round-trip | $\mathbf{J}^T \cdot \mathbf{f} = \mathbf{W}$ verified to machine precision |
| Motor torque sign convention | All positive at active pose (legs support upward) |
| Hardware gravity ff | 55.8% reduction in ODrive corrective current; 10.1% magnitude match |
| Hardware inertia ff (T5–T7) | Full FF reduces PID effort by 1.5–2.8% at 100% speed |

See [Validation Results](results.md) for details.
