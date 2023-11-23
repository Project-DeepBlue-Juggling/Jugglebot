import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.collections import LineCollection

class TargetApproachSimulator:
    def __init__(self, initial_position=[0.0, 0.0], max_velocity=10.0, acceleration=30.0):
        self._init_position = np.array(initial_position)
        self._init_velocity = np.array([0.0, 0.0])

        self.position = self._init_position
        self.velocity = self._init_velocity
        self.max_velocity = max_velocity
        self.acceleration = acceleration
    
    def update_position(self, target, time_delta):
        target = np.array(target, dtype=float)
        direction = target - self.position
        distance = np.linalg.norm(direction)

        if distance > 0:
            direction = direction / distance  # Normalize the direction vector

            # Update velocity in the direction of the target
            self.velocity += self.acceleration * direction * time_delta
            # Ensure the velocity does not exceed the max velocity
            if np.linalg.norm(self.velocity) > self.max_velocity:
                self.velocity = self.velocity / np.linalg.norm(self.velocity) * self.max_velocity

            # Update position
            self.position += self.velocity * time_delta
        else:
            print("Reached or very close to the target")

        # print(f"Position: [{self.position[0]:.2f}, {self.position[1]:.2f}], "
        #       f"Velocity: [{self.velocity[0]:.2f}, {self.velocity[1]:.2f}], "
        #       f"Distance to Target: {distance:.2f}, "
        #       f"Direction: [{direction[0]:.2f}, {direction[1]:.2f}]")
        return self.position
    
    def animate_movement(self, target_points, time_delta=0.1, mode='loop', percentages = None,
                        total_duration=None, anim_speed = 100,
                        save_anim=False, file_name=None, file_path=None):
        # Define a set of 10 distinct colors for the targets and paths
        colors = plt.cm.get_cmap('tab10', 10)

        # Normalize percentages (in case they weren't already)
        percentages = [p / sum(percentages) for p in percentages]
        durations = [total_duration * p for p in percentages]
        cumulative_time = np.cumsum(durations)

        # Calculate the durations for each target point
        if mode == 'finite' and total_duration and percentages:
            should_repeat = False

        elif mode == 'loop' and percentages:
            should_repeat = True

        else:
            print(f"Some variable is un-set (or not set correctly).\n"
                  f"mode = {mode}, "
                  f"percentages = {percentages}, "
                  f"total duration = {total_duration}. \n"
                  f"Quitting.")
            return

        # Set up the plot
        fig, ax = plt.subplots()
        # If using auto-sizing axis limits:
        axis_limit_multiplier = 1.5
        ax.set_xlim(min(min(target_points, key=lambda x: x[0])[0], -0.5) * axis_limit_multiplier, 
                    max(target_points, key=lambda x: x[0])[0] * axis_limit_multiplier)
        ax.set_ylim(min(min(target_points, key=lambda x: x[1])[1], -0.5) * axis_limit_multiplier, 
                    max(target_points, key=lambda x: x[1])[1] * axis_limit_multiplier)
        
        # If setting axis limits manually:
        # ax.set_xlim(-15, 35)
        # ax.set_ylim(-15, 35)
        formatted_percentages = ', '.join([f'{p:.2f}' for p in percentages])
        ax.set_title(f'init_pos = [{self._init_position[0]:.2f}, {self._init_position[1]:.2f}], '
                     f'init_vel = [{self._init_velocity[0]:.2f}, {self._init_velocity[1]:.2f}]\n'
                     f'max_vel = {self.max_velocity}, accel = {self.acceleration}\n'
                     f'percentages = [{formatted_percentages}]', fontsize=10)

        end_effector_dot, = ax.plot([], [], 'ro')  # Red dot for the end-effector

        # Plot each target point with its corresponding color
        for i, target in enumerate(target_points):
            ax.plot(*target, 'o', color=colors(i))

        # Initialize line collection for the trail
        line_segments = LineCollection([], linewidths=(2,))
        ax.add_collection(line_segments)

        # Path history and colors for the trail
        path_history = []
        segment_colors = []
        last_target_index = None  # Track the last target index

        # Initialization function: plot the background of each frame
        def init():
            line_segments.set_segments([])
            end_effector_dot.set_data([], [])
            return line_segments, end_effector_dot

        # Animation update function
        def animate(i):
            nonlocal last_target_index
            elapsed_time = i * time_delta
            
            # Determine the current target based on the elapsed time
            current_target_index = np.searchsorted(cumulative_time, elapsed_time, side='right') % len(target_points)
            current_target = target_points[current_target_index]

            self.update_position(current_target, time_delta)

            # Check if target has changed
            if current_target_index != last_target_index:
                # Start a new segment for the new target
                if path_history:
                    # Continue from the last position
                    path_history.append([path_history[-1][-1], self.position.tolist()])
                else:
                    # Starting the first segment
                    path_history.append([[0, 0], self.position.tolist()])
                segment_colors.append(colors(current_target_index))
                last_target_index = current_target_index

            else:
                # Add new position to the current segment
                path_history[-1].append(self.position.tolist())

            # Update line segments with the path history and colors
            line_segments.set_segments(path_history)
            line_segments.set_colors(segment_colors)

            end_effector_dot.set_data(self.position[0], self.position[1])
            return line_segments, end_effector_dot

        # Create the animation
        frames = int(total_duration / time_delta) if mode == 'finite' else int(cumulative_time[-1] / time_delta)
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames,
                                       interval=time_delta * anim_speed, blit=True, repeat=should_repeat)
        
        if save_anim and file_name and file_path:
            file_name_full = file_path + 'target_approach_' + file_name + '.gif'
            anim.save(filename=file_name_full,writer='ffmpeg', fps=1000/(time_delta * anim_speed))  # Change writer and fps if desired
        plt.show()
    
end_effector = TargetApproachSimulator(max_velocity=4.0, acceleration=10.0)

# Points used in the circle animation:
# target_points =[[-10.0, 0.0], 
#                 [-8.09, -5.88], 
#                 [10.0, 0.0], 
#                 [8.09, 5.88], 
#                 [3.09, 9.51], 
#                 [-3.09, 9.51], 
#                 [3.09, -9.51], 
#                 [-3.09, -9.51], 
#                 [-8.09, 5.88], 
#                 [8.09, -5.88]]

target_points = [[5, 10], [30, 7], [40, 0]]

percentages = [0.5, 1, 1.5]#, 1, 1, 1, 1, 1, 1, 1]  
total_duration = 11.5 # Total time for the animation in seconds
animation_speed = 100  # Arbitrary units. Smaller number is faster

save_animation = True
abs_file_path = 'C:\\Users\\harri\\Google Drive\\Projects\\Current\\Jugglebot\\Coding\\GitHub\\simulations\\bezieresque_control\\'
anim_name = 'actual_test'

end_effector.animate_movement(target_points, percentages=percentages, mode='finite', 
                              total_duration=total_duration, anim_speed=animation_speed,
                              save_anim=save_animation, file_name=anim_name, file_path=abs_file_path)
