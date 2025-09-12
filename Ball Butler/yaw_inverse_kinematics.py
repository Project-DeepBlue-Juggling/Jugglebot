# Interactive UI using only matplotlib (no ipywidgets required).
# Three sliders (s, x, y) control the geometry; the figure updates the rotated line,
# the point, and shows the chosen rotation angle in radians and degrees.
#
# Controls:
# - Drag the sliders for s, x, y at the bottom.
# - The chosen solution is the minimal rotation from the initial orientation (theta=0, line y=s).
#
# Requires only matplotlib and numpy.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def wrap_pi(angle):
    """Wrap angle to (-pi, pi]."""
    a = (angle + np.pi) % (2*np.pi) - np.pi
    return a if a != -np.pi else np.pi

def solve_thetas(x, y, s):
    """
    Return (theta1, theta2, chosen_theta). If no solution, returns (nan, nan, nan).
    Using: -x*sin(theta) + y*cos(theta) = s
    Solutions: theta = atan2(-x, y) ± arccos(s / r), r = hypot(x, y).
    """
    r = np.hypot(x, y)
    if r < abs(s) or r == 0:
        return np.nan, np.nan, np.nan
    base = np.arctan2(-x, y)
    delta = np.arccos(np.clip(s / r, -1.0, 1.0))
    t1 = wrap_pi(base + delta)
    t2 = wrap_pi(base - delta)
    chosen = t1 if abs(t1) <= abs(t2) else t2
    return t1, t2, chosen

# Initial values
s0, x0, y0 = 1.0, 4.0, 6.0

# Figure layout
plt.close('all')
fig, ax = plt.subplots(figsize=(7.5, 7.5))
plt.subplots_adjust(left=0.08, right=0.98, bottom=0.24, top=0.92)

ax.set_aspect('equal', adjustable='box')
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.grid(True, linestyle=":")
title = ax.set_title("Rotate the line at offset s so it passes through P=(x,y)")

# Artists
(circle_line,) = ax.plot([], [], lw=2, label="circle")
(main_line,) = ax.plot([], [], lw=2, label="chosen line")
(alt_line,) = ax.plot([], [], lw=1, ls="--", alpha=0.5, label="alternate line")
(point_scatter,) = ax.plot([], [], "o", ms=7, label="P=(x,y)")
ax.plot(0, 0, "o", ms=4, alpha=0.7, label="origin")

# Text readouts
angle_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left")
info_text = ax.text(0.02, 0.92, "", transform=ax.transAxes, va="top", ha="left", color="#b00020")

# Slider axes
axcolor = "#f0f0f0"
ax_s = plt.axes([0.08, 0.14, 0.84, 0.03], facecolor=axcolor)
ax_x = plt.axes([0.08, 0.09, 0.84, 0.03], facecolor=axcolor)
ax_y = plt.axes([0.08, 0.04, 0.84, 0.03], facecolor=axcolor)

s_slider = Slider(ax_s, "s", -10.0, 10.0, valinit=s0, valstep=0.1)
x_slider = Slider(ax_x, "x", -20.0, 20.0, valinit=x0, valstep=0.1)
y_slider = Slider(ax_y, "y", -20.0, 20.0, valinit=y0, valstep=0.1)

def line_points(s, theta, span):
    """Return x,y arrays for a long line in direction theta, offset by s via its normal."""
    n = np.array([-np.sin(theta),  np.cos(theta)])  # unit normal
    d = np.array([ np.cos(theta),  np.sin(theta)])  # direction
    p0 = s * n
    t = np.linspace(-span, span, 2)
    pts = p0[:, None] + d[:, None] * t[None, :]
    return pts[0, :], pts[1, :]

def update(_=None):
    s = s_slider.val
    x = x_slider.val
    y = y_slider.val
    
    # Solve angles
    t1, t2, t = solve_thetas(x, y, s)
    
    # View and circle radii
    base_r = max(1.0, abs(s) * 1.25)
    rP = np.hypot(x, y)
    view_r = max(base_r, rP, 2.0) * 1.2
    circ_r = max(base_r, 1.0)
    
    # Draw circle
    th = np.linspace(0, 2*np.pi, 400)
    circle_line.set_data(circ_r * np.cos(th), circ_r * np.sin(th))
    
    # Point
    point_scatter.set_data([x], [y])
    
    # Axis limits
    ax.set_xlim(-view_r, view_r)
    ax.set_ylim(-view_r, view_r)
    
    # Lines + text
    if np.isnan(t):
        # No solution; show the initial line y = s faintly
        X = np.array([-view_r*1.1, view_r*1.1])
        Y = np.full_like(X, s)
        main_line.set_data(X, Y)
        main_line.set_alpha(0.3)
        alt_line.set_data([], [])
        
        angle_text.set_text("")
        info_text.set_text(f"No solution: |P| = {np.hypot(x,y):.3f} < |s| = {abs(s):.3f}.")
    else:
        info_text.set_text("")
        # Main (chosen) solution
        Xmain, Ymain = line_points(s, t, span=view_r*2.2)
        main_line.set_data(Xmain, Ymain)
        main_line.set_alpha(1.0)
        # Alternate solution (if different)
        if not np.isnan(t1) and not np.isnan(t2):
            alt = t2 if np.isclose(t, t1) else t1
            Xalt, Yalt = line_points(s, alt, span=view_r*2.2)
            alt_line.set_data(Xalt, Yalt)
        else:
            alt_line.set_data([], [])
        
        # Angle readout
        t_deg = np.degrees(t)
        t1_deg = np.degrees(t1) if not np.isnan(t1) else np.nan
        t2_deg = np.degrees(t2) if not np.isnan(t2) else np.nan
        angle_text.set_text(
            f"θ (chosen) = {t:.6f} rad = {t_deg:.3f}°\n"
            f"Other: θ₁={t1:.6f} rad ({t1_deg:.3f}°), θ₂={t2:.6f} rad ({t2_deg:.3f}°)"
        )
    
    fig.canvas.draw_idle()

# Initial draw and connect
update()
s_slider.on_changed(update)
x_slider.on_changed(update)
y_slider.on_changed(update)

plt.show()
