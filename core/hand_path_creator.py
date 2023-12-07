from stewart_platform import StewartPlatform
import math
import numpy as np
import matplotlib.pyplot as plt


class HandPathCreator:
    def __init__(self, state_in, state_out, hand_zlimits=[0.3, 0.1], num_frames=100):
        '''Creates a list of points that represent the hand trajectory over the next path segment.

        One path segment is the holding path, the other is the empty path.

        Each point consists of three terms: position, orientation and time.
        Position is a 3x1 vector of the spatial position of the hand at the given timestamp
        Orientation is a 3x1 vector of hand orientation (in base frame) at the given timestamp
        
        Inputs:
            - state_in      : State array for initial "event" ((pos), (vel), (time)) ({m}, {m/s}, {sec})
            - state_out     : State array for final "event" ((pos), (vel), (time)) ({m}, {m/s}, {sec})
            - hand_zlimits  : A tuple of [hand_full_zspan, hand_empty_zspan] {m}
            - num_frames    : How many frames to discretize the path segment into

        Returns:
            - path: A ((3, 1), (3, 1), (1, 1)) tuple of ((pos), (ori), (timestamp)) representing the pos and orientation 
                    of the hand at each timestamp
            
        '''
        # Store the Class inputs for easier reference in methods
        self._prev_event = state_in 
        self._next_event = state_out
        self._travel_limits = hand_zlimits
        self._num_frames = num_frames

        self._move_duration = (state_out['time'] - state_in['time'])[0]     # Duration of this next path {sec}

        # Terms for the bezier curve
        self.ppath = np.zeros((num_frames, 1))  # Proportion through the path at each frame

        self._control_points = np.zeros((4, 3))  # Array of the bezier control points
        self._coefficients = np.array(([1, 0, 0, 0],
                                       [-3, 3, 0, 0],
                                       [3, -6, 3, 0],
                                       [-1, 3, -3, 1]))  # Coefficients of the bezier matrix
        
        self._angles = np.zeros(4)  # Angles for move (inx, iny, outx, outy)


    def _calc_position_path(self):
        ''' 
        Finds the position path for the hand over this path segment
        Creates a four-point Bézier curve that ensures tangency with the incoming and outgoing pose positions (ie. smooth position). 
        Then scales the position along the bezier such that the hand velocity is matched with the incoming and outgoing pose vels.
        '''

        # Re-cast the governing events to save typing/space
        p_in = [self._prev_event['pos'][0], self._prev_event['vel'][0]]
        p_out = [self._next_event['pos'][0], self._next_event['vel'][0]]

        # Determine if the start of the path is a catch or a throw (z component of v_in will be -ve for a catch)
        catch_or_throw = p_in[1][2] / abs(p_in[1][2]) if p_in[1][2] != 0 else 1 # -1 for catch, 1 for throw
        catch_or_throw_0_1 = int((1 + catch_or_throw) / 2) # 0 for catch, 1 for throw

        # Pick limit for this move (0 is hold, 1 is empty)
        travel_lim_z = self._travel_limits[catch_or_throw_0_1]

        # Get the duration of this move
        duration = self._move_duration

        # Get number of frames for this move. "+1" is needed because we later remove one element from the path vector
        # so that there isn't any overlap between catching and throwing.
        num_frames = int(self._num_frames) + 1

        # Set up control points (CPs) (P0, P1, P2, P3)
        P0 = p_in[0]   # First CP is the "incoming" point
        P3 = p_out[0]  # Last CP is the "outgoing" point

        if p_in[1][2] == 0:
            # If initial velocity is 0 then the second CP is coincident with the first. In practice this will only happen at the start
            # and it removes the /0 in the "real" calculation of P1, below
            P1 = P0
        else:
            # If init vel is nonzero, find P1 so P0P1 is tangent to ball path and P1 is travel_lim_z m away from the "home" plane
            P1 = P0 + catch_or_throw * travel_lim_z * np.array((p_in[1][0] / p_in[1][2], p_in[1][1] / p_in[1][2], 1))

        if p_out[1][2] == 0:
            # Same as for p_in.
            P2 = P3
        else:
            P2 = P3 + catch_or_throw * travel_lim_z * np.array((p_out[1][0] / p_out[1][2], p_out[1][1] / p_out[1][2], 1))

        # Store the control points and cast them to the Class attribute for external referencing
        control_points = np.array((P0, P1, P2, P3))
        self._control_points = control_points

        # Get the coefficients for the bezier
        coefficients = self._coefficients

        ################################################################################################
        #                          Calculate the length of the Bézier curve                            #
        ################################################################################################
        path_length = 0  # Initialize the path length
        pt_prev = (0, 0, 0)  # Resolves the warning in the if statement later
        num_points = 50  # Number of points to use to calc the length of the bezier (ie. discretize with this many pts)

        for i in range(num_points + 1):
            t = i / num_points  # Scale t to be 0 < t <= 1
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

        a = 3 * (v_out + v_in - 2 * path_length / duration) / (duration ** 2)
        b = (6 * path_length - duration * (2 * v_out + 4 * v_in)) / (duration ** 2)
        c = v_in
        v = []
        ppath = []  # A vector of how far long the main bezier the hand is at each frame, for *this move*

        ################################################################################################
        #                         Creating the position and velocity curves                            #
        ################################################################################################
        for t in np.linspace(0, duration, num_frames):
            s = (1 / 3) * a * t ** 3 + (1 / 2) * b * t ** 2 + c * t
            ppath.append(s)
            v_temp = a * t ** 2 + b * t + c
            v.append(v_temp)

        ################################################################################################
        #                             Correct in case of < 0 velocities                                #
        ################################################################################################
        # If the velocity is ever below zero, change the (velocity) profile to be two linear sections with v = 0 between them
        if round(min(v), 3) < 0:
            ppath = []  # Clear the old _ppath as it will be populated with new values
            v = []  # Clear the old velocity vector so that it's ready for new values
            t1 = path_length / v_in if v_in != 0 else 0  # If v_in = 0 then we're moving from home (ie. P1 = P0)
            t2 = path_length / v_out
            th = duration - (t1 + t2)

            for t in np.linspace(0, duration, num_frames):
                if t < t1:
                    ppath.append(v_in * (t - t ** 2 / (2 * t1)))
                    v.append(v_in * (1 - t / t1))
                elif t < t1 + th:
                    ppath.append(ppath[-1]) if v_in != 0 else ppath.append(0)  # Same as for t1
                    v.append(0)
                else:
                    ppath.append(v_out * ((t ** 2 - duration ** 2) / (2 * t2) + ((t1 + th) / t2) * (duration - t))
                                 + path_length)
                    v.append((v_out / t2) * (t - (t1 + th)))

        ################################################################################################
        #                                     Finishing touches                                        #
        ################################################################################################
        # Scale 't' to be between 0 and 1 ('t' because that's what's often used in Bezier documentation)
        ppath = [val / path_length for val in ppath]

        # Remove the first element of _ppath so that there's no overlap between catching and throwing
        ppath = ppath[1:]

        # Update the Class attribute
        self.ppath = np.array((ppath)).reshape(len(ppath), 1)


if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111)#, projection='3d')

    path_state_dtype = np.dtype([
        ('pos', 'f8', (3,)),
        ('vel', 'f8', (3,)),
        ('time', 'f8')
    ])

    state_in = np.array([([-0.1, 0, 1],
                          [0.1, 0, 1],
                          0)], dtype=path_state_dtype) # State array for initial event ((pos), (vel), (time))
    
    state_out = np.array([([0.1, 0, 1],
                           [-0.1, 0, -1],
                           1)], dtype=path_state_dtype) # State array for initial event ((pos), (vel), (time))
    
    path = HandPathCreator(state_in, state_out)
    path._calc_position_path()

    ax.plot(path.ppath)
    plt.show()

