"""
Created on Mon May 13 20:25:20 2024

Original author: jonbe
Modified by: Harrison Low
"""

import matplotlib.pyplot as plt
import numpy as np

class HandTrajGenerator:
    def __init__(self, throw_height=0.6, throw_range=0.0, inertia_ratio=0.7,
                 throw_vel_hold_pct=0.05, catch_vel_ratio=0.8, catch_vel_hold_pct=0.1,
                 sample_rate=500):
        
        # Motor characteristics
        self.motor_rotor_mass_g = 19.1
        self.motor_rotor_radius_m = 0.015

        # Masses (g)
        self.mgn9h_carriage_mass_g = 26
        self.printed_hand_mass_g = 26.5
        self.green_ball_mass_g = 149.8
        self.infinity_ball_mass_g = 105.7
        self.gballz_ball_mass_g = 115.8
        self.jon_ball_mass_g = 130

        # Inertia
        self.total_inertia_ref_to_hand = 0.37  # kg
        self.eff_spool_r = 0.005134  # m

        # Inputs
        self.throw_height = throw_height  # Height of throw in meters
        self.plat_throw_range = throw_range  # How far the COM of the platform moves during the throw {mm}
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

        self.throw_duration_s = np.sqrt(8 * self.throw_height / self.g_mPs2)

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

        self.t5 = self.t2 + self.throw_duration_s - (1 / 2) * self.t_catch_vel_hold_s
        self.t4 = self.t5 - self.t_catch_accel_s
        self.t6 = self.t5 + self.t_catch_vel_hold_s
        self.t7 = self.t6 + self.t_catch_decel_s
        self.t8 = self.t7 + self.t_end_of_profile_hold_s
        self.x5 = self.x3 + (1 / 2) * self.catch_accel_mPs2 * pow(self.t_catch_accel_s, 2)
        self.x6 = self.x5 + self.v_hand_catch_mPs * self.t_catch_vel_hold_s
        self.v5 = self.v_hand_catch_mPs
        self.a4 = self.catch_accel_mPs2
        self.a6 = self.catch_decel_mPs2

    def generate_command_time_series(self):
        step_type = ['a', 'v', 'a', 'x', 'a', 'v', 'a', 'x', 'e']
        t = [0, self.t1, self.t2, self.t3, self.t4, self.t5, self.t6, self.t7, self.t8]
        x = [0, self.x1, self.x2, self.x3, self.x3, self.x5, self.x6, 0, 0]
        v = [0, self.v1, self.v1, 0, 0, self.v5, self.v5, 0, 0]
        a = [self.a0, 0, self.a2, 0, self.a4, 0, self.a6, 0, 0]

        t_now = 0
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

        self.t_cmd = t_cmd
        self.x_cmd = x_cmd
        self.v_cmd = v_cmd
        self.a_cmd = a_cmd

        self.tor_cmd = self.convert_acceleration_to_torque()

    def convert_acceleration_to_torque(self):
        F = [b * self.total_inertia_ref_to_hand for b in self.a_cmd]
        T = [c * self.eff_spool_r for c in F]
        return T

    def plot_results(self):
        plt.figure(1)
        plt.clf()
        plt.subplot(311)
        plt.plot(self.t_cmd, self.x_cmd)
        plt.subplot(312)
        plt.plot(self.t_cmd, self.v_cmd)
        plt.subplot(313)
        plt.plot(self.t_cmd, self.a_cmd)
        plt.show()

    def set_throw_parameters(self, throw_height, throw_range, inertia_ratio=0.7,
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

    def get_trajectory(self):
        '''Generates and returns the trajectory'''
        # Generate the command time series
        self.generate_command_time_series()

        # Convert the lists to numpy arrays before returning
        return np.array(self.t_cmd), np.array(self.x_cmd), np.array(self.v_cmd), np.array(self.tor_cmd)

    def get_throw_angle(self):
        '''Returns the angle of the throw from the vertical in radians'''
        return self.throw_angle_rad
    
    def get_throw_velocity(self):
        '''Returns the release velocity of the throw in m/s'''
        return self.throw_vel_mPs

if __name__ == '__main__':
    sim = HandTrajGenerator()
    sim.plot_results()
