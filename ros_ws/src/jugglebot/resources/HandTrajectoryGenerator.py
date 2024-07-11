# -*- coding: utf-8 -*-
"""
Created on Mon May 13 20:25:20 2024

@author: jonbe
"""
import matplotlib.pyplot as plt
import pandas as pd

#inputs
throw_duration_s = 0.5      # Airborne time for ball (assumes throw and catch at same height)
inertia_ratio = 0.7         # Ratio of hand actuator inertia without ball over total inertia with ball in hand
throw_vel_hold_pct = 0.05   # % of total stroke used for velocity hold segment of throw
catch_vel_ratio = 0.8       # Relative Speed of hand compared to ball at catch
catch_vel_hold_pct = 0.1    # % of total stroke used for velocity hold segment of catch

#constants
g_mPs2 = 9.806
hand_mech_stroke_m = .358
mech_stroke_margin_m = .02
total_throw_stroke_m = hand_mech_stroke_m - 2*mech_stroke_margin_m
#print(f'Total throw stroke: {total_throw_stroke_m:0.5} m')
t_end_of_profile_hold_s = 0.1

#ball velocity calculation
throw_vel_mPs = throw_duration_s * g_mPs2 / 2
print(f'release velocity: {throw_vel_mPs:0.3} m/s')
height_over_rel_m = (1/2) * (throw_vel_mPs * throw_duration_s) - (1/8) * g_mPs2 * pow(throw_duration_s,2)
print(f'height over release point: {height_over_rel_m:0.5} m')

#throw segment
throw_vel_hold_stroke_m = throw_vel_hold_pct * total_throw_stroke_m
#print(f'Throw vel hold stroke: {throw_vel_hold_stroke_m:0.5} m')
throw_accel_decel_stroke_m = total_throw_stroke_m - throw_vel_hold_stroke_m
#print(f'Throw accel and decel stroke: {throw_accel_decel_stroke_m:0.5} m')
t_throw_accel_s = (2 / (inertia_ratio + 1)) * throw_accel_decel_stroke_m / throw_vel_mPs
#print(f'throw accel duration: {t_throw_accel_s:0.4} s')
t_throw_vel_hold_s = throw_vel_hold_stroke_m / throw_vel_mPs
#print(f'throw vel hold duration: {t_throw_vel_hold_s:0.4} s')
t_throw_decel_s = t_throw_accel_s * inertia_ratio
#print(f'throw decel duration: {t_throw_decel_s:0.4} s')

throw_accel_mPs2 = throw_vel_mPs / t_throw_accel_s
print(f'throw accel: {throw_accel_mPs2:0.4} m/s^2')
throw_decel_mPs2 = -1 * throw_accel_mPs2 / inertia_ratio

t1 = t_throw_accel_s            #start of throw vel hold
t2 = t1 + t_throw_vel_hold_s    #release time
t3 = t2 + t_throw_decel_s       #stop at top of stroke
x1 = (1/2) * throw_accel_mPs2 * pow(t_throw_accel_s,2)
x2 = x1 + throw_vel_mPs * t_throw_vel_hold_s
x3 = x2 + throw_vel_mPs * t_throw_decel_s + (1/2) * throw_decel_mPs2 * pow(t_throw_decel_s,2)
x4 = x3
v1 = throw_vel_mPs
a0 = throw_accel_mPs2
a2 = throw_decel_mPs2

#catch segment
v_hand_catch_mPs = -1 * catch_vel_ratio * throw_vel_mPs
catch_inertia_ratio = 1 / inertia_ratio

catch_vel_hold_stroke_m = catch_vel_hold_pct * total_throw_stroke_m
#print(f'Catch vel hold stroke: {catch_vel_hold_stroke_m:0.5} m')
catch_accel_decel_stroke_m = total_throw_stroke_m - catch_vel_hold_stroke_m
#print(f'Catch accel and decel stroke: {catch_accel_decel_stroke_m:0.5} m')

t_catch_accel_s = -1 * (2 / (catch_inertia_ratio + 1)) * catch_accel_decel_stroke_m / v_hand_catch_mPs
#print(f'catch accel duration: {t_catch_accel_s:0.4} s')
t_catch_vel_hold_s = -1 * catch_vel_hold_stroke_m / v_hand_catch_mPs
#print(f'catch vel hold duration: {t_catch_vel_hold_s:0.4} s')
t_catch_decel_s = t_catch_accel_s * catch_inertia_ratio
#print(f'catch decel duration: {t_catch_decel_s:0.4} s')

catch_accel_mPs2 = v_hand_catch_mPs / t_catch_accel_s
#print(f'catch accel: {catch_accel_mPs2:0.4} m/s^2')
catch_decel_mPs2 = -1 * catch_accel_mPs2 / catch_inertia_ratio  
#print(f'catch decel: {catch_decel_mPs2:0.4} m/s^2')

t5 = t2 + throw_duration_s - (1/2)*t_catch_vel_hold_s  #start of catch constant velocity
t4 = t5 - t_catch_accel_s                              #start of catch motion
t6 = t5 + t_catch_vel_hold_s                           #start of catch decel
t7 = t6 + t_catch_decel_s                              #end of catch decel, start of hold
t8 = t7 + t_end_of_profile_hold_s
x5 = x4 + (1/2) * catch_accel_mPs2 * pow(t_catch_accel_s,2)
x6 = x5 + v_hand_catch_mPs * t_catch_vel_hold_s
v5 = v_hand_catch_mPs
a4 = catch_accel_mPs2
a6 = catch_decel_mPs2

#step_type = ['a','v','a','x','a','v','a']
step_type = ['a','v','a','x','a','v','a','x','e']
t = [0,  t1, t2, t3, t4, t5, t6, t7, t8]
x = [0,  x1, x2, x3, x3, x5, x6, 0,  0]
v = [0,  v1, v1, 0,  0,  v5, v5, 0,  0]
a = [a0, 0,  a2, 0,  a4, 0,  a6, 0,  0]

print(step_type)
print(t)
print(x)
print(v)
print(a)

#generate command time series
t_now = 0
sample_rate = 100 # Hz
delta_t = 1/sample_rate
t_cmd = []
x_cmd = []
v_cmd = []
a_cmd = []

i = 0
while t_now < t[-1]:
    while t_now > t[i+1]:
        i += 1
    if step_type[i] == 'a':
        t_cmd.append(t_now)
        x_cmd.append(x[i] + v[i]*(t_now - t[i]) + (1/2)*a[i]*pow(t_now-t[i],2))
        v_cmd.append(v[i] + a[i]*(t_now - t[i]))
        a_cmd.append(a[i])
    elif step_type[i] == 'v':
        t_cmd.append(t_now)
        x_cmd.append(x[i] + v[i]*(t_now - t[i]))
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

#Plot x,v,a
plt.figure(1);
plt.clf()
plt.subplot(311);
plt.plot(t_cmd,x_cmd)
plt.subplot(312);
plt.plot(t_cmd,v_cmd)
plt.subplot(313);
plt.plot(t_cmd,a_cmd)

#Convert acceleration to torque feedforward
total_inertia_ref_to_hand = .37; #kg
eff_spool_r = .00497; #m

F = [b*total_inertia_ref_to_hand for b in a_cmd];
T = [c*eff_spool_r for c in F];


#Export to csv
path = './'
steps_row = [len(t_cmd)]
data = [t_cmd,x_cmd,v_cmd,a_cmd,T,steps_row]
data = pd.DataFrame(data)
data.to_csv(path+f'{throw_duration_s}s_{sample_rate}Hz_throw.csv', index=False,float_format="%.6f")
