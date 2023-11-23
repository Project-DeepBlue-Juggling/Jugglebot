# 2D Target Approach Simulator

## Overview
This Python script simulates the movement of an end-effector in a 2D space as it approaches a series of target points. The simulation visualizes the path taken by the end-effector using a dynamic animation, where the end-effector moves towards each target point for a specified duration or percentage of total time.

## Features
- **Two Modes of Operation:**
  - **Loop Mode:** The end-effector continuously loops through the target points.
  - **Finite Mode:** The end-effector moves towards each target for a specified percentage of the total duration.
- **Customizable Parameters:**
  - Maximum velocity and acceleration of the end-effector.
  - List of target points in 2D space.
  - Duration for which each target is approached (in finite mode).
  - Total duration of the animation (in finite mode).
- **Visualization:**
  - The path of the end-effector is visualized as a line that changes color based on the current target.
  - Target points are displayed as colored dots.

## Usage

### Prerequisites
- Python 3.x
- NumPy
- Matplotlib

### Running the Simulation
1. **Import the `TargetApproachSimulator` class from the script.**
2. **Instantiate the simulator** with initial parameters like initial position, maximum velocity, and acceleration.
3. **Call the `animate_movement` method** of the simulator instance with the list of target points and other necessary parameters based on the desired mode (loop or finite).

#### Example: Loop Mode
```python
end_effector = TargetApproachSimulator()
target_points = [[10, 10], [20, 20], [30, 10]]  # Define target points
end_effector.animate_movement(target_points, mode='loop')
```

#### Example: Finite Mode
```python
end_effector = TargetApproachSimulator()
target_points = [[10, 10], [20, 20], [30, 10]]  # Define target points
percentages = [0.3, 0.4, 0.3]  # Define time percentages for each target
total_duration = 10  # Total duration of the animation
end_effector.animate_movement(target_points, mode='finite', total_duration=total_duration, percentages=percentages)
```

## Customization
- Modify target points, velocities, accelerations, and durations as needed to simulate different scenarios.
- Adjust colors and other visualization parameters within the `animate_movement` method for different aesthetic preferences.
