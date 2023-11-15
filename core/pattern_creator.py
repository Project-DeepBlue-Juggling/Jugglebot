import time
import numpy as np

class PatternCreator:
    def flat_circle_path(height, radius, time_start, freq = 0.5):
        time_elapsed = time.time() - time_start

        # Determine the x and y coordinates for the circle
        x = radius * np.cos(2 * np.pi * freq * time_elapsed)
        y = radius * np.sin(2 * np.pi * freq * time_elapsed)
        z = height

        pose = [x, y, z, 0, 0, 0]

        return pose
    
    def conical_path(height_of_platform, cone_height, radius, time_start, freq=0.5):
        time_elapsed = time.time() - time_start

        cone_angle = np.arctan2(radius, cone_height)

        # x, y and z coordinates
        x = radius * np.cos(2 * np.pi * freq * time_elapsed)
        y = radius * np.sin(2 * np.pi * freq * time_elapsed)
        z = height_of_platform

        # Rotation angles
        phi = np.rad2deg(cone_angle * np.sin(2 * np.pi * freq * time_elapsed))
        theta = -np.rad2deg(cone_angle * np.cos(2 * np.pi * freq * time_elapsed))

        pose = [x, y, z, phi, theta, 0]

        return pose

