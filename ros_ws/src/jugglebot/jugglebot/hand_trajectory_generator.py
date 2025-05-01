"""
Created on Mon May 13 20:25:20 2024

Original author: jonbe
Modified by: Harrison Low

Note that all trajectories are generated in meters. The conversion to revolutions is done in the ROS2 node.
(Plots can be generated in either meters or revolutions by setting the unit parameter in the plot_results function)
"""

import matplotlib.pyplot as plt
import numpy as np

class HandTrajGenerator:
    HAND_SPOOL_EFFECTIVE_RADIUS = 5.210 # {mm} Measured experimentally. (stroke / [revs_to_move_stroke * 2 * pi])
    LINEAR_GAIN_FACTOR = 1.0 # To manually adjust the linear gain
    LINEAR_GAIN = (1000 / (np.pi * HAND_SPOOL_EFFECTIVE_RADIUS * 2)) * LINEAR_GAIN_FACTOR # {rev/m}

    def __init__(self, throw_height_m=0.2, throw_range_mm=0.0, inertia_ratio=0.747,
                 throw_vel_hold_pct=0.05, catch_vel_ratio=0.8, catch_vel_hold_pct=0.1,
                 sample_rate=500):
        
        # Motor characteristics
        self.motor_rotor_mass_g = 19.1
        self.motor_rotor_radius_m = 0.015

        # Masses (g). Not actually used anywhere, just kept as a database.
        self.mgn9h_carriage_mass_g = 26
        self.printed_hand_mass_g = 26.5
        self.green_ball_mass_g = 149.8
        self.infinity_ball_mass_g = 105.7
        self.gballz_ball_mass_g = 115.8
        self.norwik_ball_mass_g = 71.1
        self.jon_ball_mass_g = 130

        # Inertia
        self.total_inertia_ref_to_hand = 0.281  # kg
        self.eff_spool_r = 0.005134  # m

        # Inputs
        self.throw_height = throw_height_m  # Height of throw in meters
        self.plat_throw_range = throw_range_mm  # How far the ball travels along the x-y plane during the throw {mm}
        self.inertia_ratio = inertia_ratio  # Ratio of hand actuator inertia without ball over total inertia with ball in hand
        self.throw_vel_hold_pct = throw_vel_hold_pct  # % of total stroke used for velocity hold segment of throw
        self.catch_vel_ratio = catch_vel_ratio  # Relative Speed of hand compared to ball at catch
        self.catch_vel_hold_pct = catch_vel_hold_pct  # % of total stroke used for velocity hold segment of catch
        self.sample_rate = sample_rate  # Hz

        dist_from_bottom_stroke_to_throw_pos = 0.1936  # Dist from bottom of hand stroke to throw pos {m} (`x2`. TEMPORARY HARD CODE)
        dist_from_bottom_stroke_to_plat_COM = 0.129  # Dist from bottom of hand stroke to platform COM {m} (Found in Onshape)
        dist_from_ball_COM_to_bottom_of_linear_guide = 0.05995  # Dist from ball COM to bottom of linear guide {m} (Found in Onshape)
        self.dist_from_plat_COM_to_throw_pos = (dist_from_bottom_stroke_to_throw_pos + 
                                                dist_from_ball_COM_to_bottom_of_linear_guide - 
                                                dist_from_bottom_stroke_to_plat_COM)  # Dist from platform COM to throw pos {m}

        # Constants
        self.g_mPs2 = 9.806
        self.hand_mech_stroke_m = 0.358
        self.mech_stroke_margin_m = 0.02
        self.total_throw_stroke_m = self.hand_mech_stroke_m - 2 * self.mech_stroke_margin_m
        self.t_end_of_profile_hold_s = 0.1

        self.calculate_throw_parameters()
        self.calculate_catch_parameters()

    def calculate_throw_parameters(self):
        plat_throw_range_m = self.plat_throw_range / 1000  # Convert throw range to meters
        self.throw_angle_rad = np.arctan(plat_throw_range_m / (4 * self.throw_height)) # Angle of the throw from vertical {rad}
        self.ball_throw_range = plat_throw_range_m - 2 * self.dist_from_plat_COM_to_throw_pos * np.sin(self.throw_angle_rad) # Range of ball {m}
        
        print(f'ball throw range: {self.ball_throw_range:.3f} m, throw angle: {np.rad2deg(self.throw_angle_rad):.3f} deg')

        self.air_time_s = np.sqrt(8 * self.throw_height / self.g_mPs2)

        self.throw_vel_mPs = np.sqrt(self.g_mPs2 * pow(self.ball_throw_range, 2) / (8 * self.throw_height) + 2 * self.g_mPs2 * self.throw_height)
        print(f'release velocity: {self.throw_vel_mPs:.3f} m/s')

        self.throw_vel_hold_stroke_m = self.throw_vel_hold_pct * self.total_throw_stroke_m
        self.throw_accel_decel_stroke_m = self.total_throw_stroke_m - self.throw_vel_hold_stroke_m
        self.t_throw_accel_s = (2 / (self.inertia_ratio + 1)) * self.throw_accel_decel_stroke_m / self.throw_vel_mPs
        self.t_throw_vel_hold_s = self.throw_vel_hold_stroke_m / self.throw_vel_mPs
        self.t_throw_decel_s = self.t_throw_accel_s * self.inertia_ratio

        self.throw_accel_mPs2 = self.throw_vel_mPs / self.t_throw_accel_s
        # print(f'throw accel: {self.throw_accel_mPs2:.4f} m/s^2')
        self.throw_decel_mPs2 = -1 * self.throw_accel_mPs2 / self.inertia_ratio

        self.t1 = self.t_throw_accel_s
        self.t2 = self.t1 + self.t_throw_vel_hold_s
        self.t3 = self.t2 + self.t_throw_decel_s
        self.x1 = (1 / 2) * self.throw_accel_mPs2 * pow(self.t_throw_accel_s, 2)
        self.x2 = self.x1 + self.throw_vel_mPs * self.t_throw_vel_hold_s
        self.x3 = self.x2 + self.throw_vel_mPs * self.t_throw_decel_s + (1 / 2) * self.throw_decel_mPs2 * pow(self.t_throw_decel_s, 2)
        self.v1 = self.throw_vel_mPs
        self.a0 = self.throw_accel_mPs2
        self.a2 = self.throw_decel_mPs2

        # Save the duration that the throw trajectory takes
        self.throw_duration_s = self.t3

    def calculate_catch_parameters(self):
        self.v_hand_catch_mPs = -1 * self.catch_vel_ratio * self.throw_vel_mPs
        self.catch_inertia_ratio = 1 / self.inertia_ratio

        self.catch_vel_hold_stroke_m = self.catch_vel_hold_pct * self.total_throw_stroke_m
        self.catch_accel_decel_stroke_m = self.total_throw_stroke_m - self.catch_vel_hold_stroke_m
        self.t_catch_accel_s = -1 * (2 / (self.catch_inertia_ratio + 1)) * self.catch_accel_decel_stroke_m / self.v_hand_catch_mPs
        self.t_catch_vel_hold_s = -1 * self.catch_vel_hold_stroke_m / self.v_hand_catch_mPs
        self.t_catch_decel_s = self.t_catch_accel_s * self.catch_inertia_ratio

        self.catch_accel_mPs2 = self.v_hand_catch_mPs / self.t_catch_accel_s
        self.catch_decel_mPs2 = -1 * self.catch_accel_mPs2 / self.catch_inertia_ratio

        self.t5 = self.t2 + self.air_time_s - (1 / 2) * self.t_catch_vel_hold_s
        self.t4 = self.t5 - self.t_catch_accel_s
        self.t6 = self.t5 + self.t_catch_vel_hold_s
        self.t7 = self.t6 + self.t_catch_decel_s
        self.t8 = self.t7 + self.t_end_of_profile_hold_s
        self.x5 = self.x3 + (1 / 2) * self.catch_accel_mPs2 * pow(self.t_catch_accel_s, 2)
        self.x6 = self.x5 + self.v_hand_catch_mPs * self.t_catch_vel_hold_s
        self.v5 = self.v_hand_catch_mPs
        self.a4 = self.catch_accel_mPs2
        self.a6 = self.catch_decel_mPs2

        # Save the duration that the catch trajectory takes
        self.catch_duration_s = self.t7 - self.t4

    def generate_throw_time_series(self):
        # Generate the time series for the throw
        t = [0, self.t1, self.t2, self.t3]

        x = [0, self.x1, self.x2, self.x3]
        v = [0, self.v1, self.v1, 0]
        a = [self.a0, 0, self.a2, 0]

        t_now = 0
        delta_t = 1 / self.sample_rate
        t_throw = []
        x_throw = []
        v_throw = []
        a_throw = []

        i = 0
        while t_now < t[-1]:
            while t_now > t[i + 1]:
                i += 1
            t_throw.append(t_now)
            x_throw.append(x[i] + v[i] * (t_now - t[i]) + (1 / 2) * a[i] * pow(t_now - t[i], 2))
            v_throw.append(v[i] + a[i] * (t_now - t[i]))
            a_throw.append(a[i])
            t_now += delta_t

        # Ensure the velocity and acceleration are 0 at the end of the throw
        v_throw[-1] = 0
        a_throw[-1] = 0

        tor_throw = self.convert_acceleration_to_torque(a_throw)

        return t_throw, x_throw, v_throw, a_throw, tor_throw

    def generate_catch_time_series(self):
        # Generate the time series for the catch, with time starting from the start of the catch (t4)
        t = [self.t4, self.t5, self.t6, self.t7]

        # Subtract t4 from all time values, so that the time series starts from 0
        t = [t_i - self.t4 for t_i in t]

        x = [self.x3, self.x5, self.x6, 0]
        v = [0, self.v5, self.v5, 0]
        a = [self.a4, 0, self.a6, 0]

        t_now = 0
        delta_t = 1 / self.sample_rate
        t_catch = []
        x_catch = []
        v_catch = []
        a_catch = []

        i = 0
        while t_now < t[-1]:
            while t_now > t[i + 1]:
                i += 1
            t_catch.append(t_now)
            x_catch.append(x[i] + v[i] * (t_now - t[i]) + (1 / 2) * a[i] * pow(t_now - t[i], 2))
            v_catch.append(v[i] + a[i] * (t_now - t[i]))
            a_catch.append(a[i])
            t_now += delta_t

        # Ensure the velocity and acceleration are 0 at the end of the catch
        v_catch[-1] = 0
        a_catch[-1] = 0

        tor_catch = self.convert_acceleration_to_torque(a_catch)

        return t_catch, x_catch, v_catch, a_catch, tor_catch

    def generate_command_time_series(self):
        step_type = ['a', 'v', 'a', 'x', 'a', 'v', 'a', 'x', 'e']
        t = [0, self.t1, self.t2, self.t3, self.t4, self.t5, self.t6, self.t7, self.t8]
        x = [0, self.x1, self.x2, self.x3, self.x3, self.x5, self.x6, 0, 0]
        v = [0, self.v1, self.v1, 0, 0, self.v5, self.v5, 0, 0]
        a = [self.a0, 0, self.a2, 0, self.a4, 0, self.a6, 0, 0]

        t_now = 0.0
        delta_t = 1 / self.sample_rate
        t_cmd = []
        x_cmd = []
        v_cmd = []
        a_cmd = []

        i = 0
        while t_now < t[-1]:
            while t_now > t[i + 1]:
                i += 1
            if step_type[i] == 'a':
                t_cmd.append(t_now)
                x_cmd.append(x[i] + v[i] * (t_now - t[i]) + (1 / 2) * a[i] * pow(t_now - t[i], 2))
                v_cmd.append(v[i] + a[i] * (t_now - t[i]))
                a_cmd.append(a[i])
            elif step_type[i] == 'v':
                t_cmd.append(t_now)
                x_cmd.append(x[i] + v[i] * (t_now - t[i]))
                v_cmd.append(v[i])
                a_cmd.append(0)
            elif step_type[i] == 'x':
                t_cmd.append(t_now)
                x_cmd.append(x[i])
                v_cmd.append(0)
                a_cmd.append(0)
            elif step_type[i] == 'e':
                t_cmd.append(t_now)
                x_cmd.append(x[i])
                v_cmd.append(0)
                a_cmd.append(0)

            t_now += delta_t

        tor_cmd = self.convert_acceleration_to_torque(a_cmd)

        return t_cmd, x_cmd, v_cmd, a_cmd, tor_cmd

    def convert_acceleration_to_torque(self, accel_cmd=None):
        F = [b * self.total_inertia_ref_to_hand for b in accel_cmd]
        T = [c * self.eff_spool_r for c in F]
        return T

    def plot_results(self, t, x, v, a, title=None, unit='rev'):
        if unit == 'rev':
            # Scale the position, velocity and acceleration by multiplying by the linear gain
            x = [element * self.LINEAR_GAIN for element in x]
            v = [element * self.LINEAR_GAIN for element in v]
            a = [element * self.LINEAR_GAIN for element in a]

            # Set the y-axis labels
            y_label_pos = 'Position (rev)'
            y_label_vel = 'Velocity (rev/s)'
            y_label_acc = 'Acceleration (rev/s^2)'

        elif unit == 'm':
            # Set the y-axis labels
            y_label_pos = 'Position (m)'
            y_label_vel = 'Velocity (m/s)'
            y_label_acc = 'Acceleration (m/s^2)'
            
        elif unit == '-' or unit == None:
            # If the unit is '-', don't scale the values
            y_label_pos = 'Position'
            y_label_vel = 'Velocity'
            y_label_acc = 'Acceleration'

        # Else, if the unit isn't either 'rev', 'm' or '-'/'none', return an error
        else:
            print("Invalid unit. Please enter 'rev', 'm', '-' or None.")
            return

        plt.figure()
        # plt.clf()
        plt.subplot(411)
        plt.plot(t, x)
        # Label the axes
        plt.ylabel(y_label_pos)
        plt.subplot(412)
        plt.plot(t, v)
        plt.ylabel(y_label_vel)
        plt.subplot(413)
        plt.plot(t, a)
        plt.ylabel(y_label_acc)
        plt.subplot(414)
        plt.plot(t, self.convert_acceleration_to_torque(a))
        plt.ylabel('Torque (Nm)')
        plt.xlabel('Time (s)')

        if title:
            plt.suptitle(title)
        # plt.show()

    def set_flight_parameters(self, throw_height, throw_range=0.0, inertia_ratio=0.7,
                            throw_vel_hold_pct=0.05, catch_vel_ratio=0.8, catch_vel_hold_pct=0.1):
        '''Set the parameters for the throw and catch'''
        self.throw_height = throw_height
        self.plat_throw_range = throw_range
        self.inertia_ratio = inertia_ratio
        self.throw_vel_hold_pct = throw_vel_hold_pct
        self.catch_vel_ratio = catch_vel_ratio
        self.catch_vel_hold_pct = catch_vel_hold_pct

        self.calculate_throw_parameters()
        self.calculate_catch_parameters()

    def get_full_trajectory(self):
        '''Generates and returns the trajectory'''
        # Generate the command time series
        t, x, v, a, tor = self.generate_command_time_series()

        # Multiply the position and velocity by the linear gain to convert from m to rev
        x = [element * self.LINEAR_GAIN for element in x]
        v = [element * self.LINEAR_GAIN for element in v]

        return t, x, v, tor
    
    def get_throw_trajectory(self):
        '''Generates and returns the throw trajectory'''
        # Generate the throw time series
        t, x, v, a, tor = self.generate_throw_time_series()

        # Multiply the position and velocity by the linear gain to convert from m to rev
        x = [element * self.LINEAR_GAIN for element in x]
        v = [element * self.LINEAR_GAIN for element in v]

        return t, x, v, tor, self.air_time_s
    
    def get_catch_trajectory(self):
        '''Generates and returns the catch trajectory'''
        # Generate the catch time series
        t, x, v, a, tor = self.generate_catch_time_series()

        # Multiply the position and velocity by the linear gain to convert from m to rev
        x = [element * self.LINEAR_GAIN for element in x]
        v = [element * self.LINEAR_GAIN for element in v]

        return t, x, v, tor
    
    def get_catch_time_and_position(self):
        '''
        Returns the time and position of the catch
        Note: The time is relative to the start of the catch trajectory and the position is in meters
        '''
        return self.t5 - self.t4 , self.x5

    def get_throw_angle(self):
        '''Returns the angle of the throw from the vertical in radians'''
        return self.throw_angle_rad
    
    def get_throw_velocity(self):
        '''Returns the release velocity of the throw in m/s'''
        return self.throw_vel_mPs

    def get_smooth_move_trajectory(self, start_pos, target_pos, duration=2.0):
        '''
        Generates and returns a trajectory to move the hand from the given starting position to the target position
        over the duration specified. The trajectory is generated using a quintic polynomial for the position, as
        this ensures that the velocity and acceleration are continuous at the start and end points. The velocity
        and acceleration are set to 0 at the start and end points. 
        
        Note that this method is unit-agnostic, so the input positions should be in the same unit as the
        desired output trajectory.
        '''
        
        # Calculate the coefficients of the quintic polynomial
        a = start_pos
        b = 0
        c = 0
        d = (10 * (target_pos - start_pos)) / pow(duration, 3)
        e = (-15 * (target_pos - start_pos)) / pow(duration, 4)
        f = (6 * (target_pos - start_pos)) / pow(duration, 5)

        # Generate the time series
        t_now = 0.0
        delta_t = 1 / self.sample_rate
        t_traj = []
        x_traj = []
        v_traj = []
        a_traj = []

        while t_now < duration:
            x_traj.append(a + b * t_now + c * pow(t_now, 2) + d * pow(t_now, 3) + e * pow(t_now, 4) + f * pow(t_now, 5))
            v_traj.append(b + 2 * c * t_now + 3 * d * pow(t_now, 2) + 4 * e * pow(t_now, 3) + 5 * f * pow(t_now, 4))
            a_traj.append(2 * c + 6 * d * t_now + 12 * e * pow(t_now, 2) + 20 * f * pow(t_now, 3))
            t_traj.append(t_now)
            t_now += delta_t

        # Append the final position to the trajectory
        x_traj.append(target_pos)
        v_traj.append(0.0)
        a_traj.append(0.0)
        t_traj.append(duration)

        # Convert the acceleration to torque
        tor_traj = self.convert_acceleration_to_torque(a_traj)

        return t_traj, x_traj, v_traj, a_traj, tor_traj

if __name__ == '__main__':
    # Set the unit to generate the plots in (either 'rev' or 'm')
    plot_unit = 'rev'

    sim = HandTrajGenerator(throw_height_m=1.43, throw_range_mm=622.8)
    t, x, v, a, tor = sim.generate_throw_time_series()
    sim.plot_results(t, x, v, a, title='Throw', unit=plot_unit)
    print(f'Number of points in throw trajectory: {len(t)}')

    # t, x, v, a, tor = sim.generate_catch_time_series()
    # sim.plot_results(t, x, v, a, title='Catch', unit=plot_unit)

    # t, x, v, a, tor = sim.generate_command_time_series()
    # sim.plot_results(t, x, v, a, title='Throw and Catch', unit=plot_unit)

    # print(f'Number of points in catch trajectory: {len(t)}')
    # print(f'Ball air time = {sim.air_time_s:.3f} s')
    # print(f'Throw duration = {sim.throw_duration_s:.3f} s')
    # print(f'Catch duration = {sim.catch_duration_s:.3f} s')

    # # Calculate the minimum duration the hand will be holding for (assuming throw immediately after catch)
    # holding_time = sim.throw_duration_s + sim.catch_duration_s
    # print(f'Holding time = {holding_time:.3f} s')

    # # Calculate the minimum 'beat' time
    # min_beat_time = (sim.air_time_s + holding_time) / 2
    # print(f'Minimum beat time = {min_beat_time:.3f} s')

    # # Calculate the duration the hand will be empty for
    # empty_time = min_beat_time - holding_time
    # print(f'Empty time = {empty_time:.3f} s')

    """Generate a smooth move trajectory"""
    # t, x, v, a, tor = sim.get_smooth_move_trajectory(0.0, 10.0, duration=2.0)
    # sim.plot_results(t, x, v, a, title='Smooth Move', unit=None)

    plt.show()
