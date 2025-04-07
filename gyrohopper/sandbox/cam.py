import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Cam parameters
base_radius = 50  # Base radius of the cam (mm)
ramp_height = 20   # Height of the ramp (mm)
angle_start_ramp = 90  # Starting angle of the ramp (degrees)
angle_end_ramp = 180   # Ending angle of the ramp (degrees)

# Follower parameters
follower_radius = 10  # Radius of the follower (mm)

# Create the cam profile
def create_cam_profile(base_radius, ramp_height, angle_start_ramp, angle_end_ramp):
    angles = np.linspace(0, 360, 500)  # Full rotation in degrees
    radii = np.full_like(angles, base_radius)

    # Apply the ramp height to the specified angular region
    ramp_start_idx = np.searchsorted(angles, angle_start_ramp)
    ramp_end_idx = np.searchsorted(angles, angle_end_ramp)
    ramp_region = np.linspace(0, 1, ramp_end_idx - ramp_start_idx)
    radii[ramp_start_idx:ramp_end_idx] += ramp_region * ramp_height

    return angles, radii

# Generate the cam profile
angles, radii = create_cam_profile(base_radius, ramp_height, angle_start_ramp, angle_end_ramp)

# Convert polar coordinates to Cartesian for plotting
x = radii * np.cos(np.radians(angles))
y = radii * np.sin(np.radians(angles))

# Function to calculate follower position
def calculate_follower_position(angle, radii, angles):
    idx = np.searchsorted(angles, angle % 360)
    follower_x = radii[idx] * np.cos(np.radians(angle))
    follower_y = radii[idx] * np.sin(np.radians(angle))
    return follower_x, follower_y

# Initial follower position
follower_angle = 0
follower_x, follower_y = calculate_follower_position(follower_angle, radii, angles)

# Create the plot
fig, ax = plt.subplots(figsize=(8, 8))
plt.subplots_adjust(bottom=0.25)

# Plot cam profile
cam_line, = ax.plot(x, y, label="Cam Profile", color="blue")
ax.scatter([0], [0], color="red", label="Cam Center")  # Cam center

# Plot follower
follower_circle = plt.Circle((follower_x, follower_y), follower_radius, color="green", label="Follower", alpha=0.5)
ax.add_patch(follower_circle)

# Plot details
ax.set_title("Cam Profile with Follower", fontsize=14)
ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.axis("equal")
ax.grid(True)
ax.legend()

# Add slider for acceleration simulation
ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03])
slider = Slider(ax_slider, 'Angle', 0, 360, valinit=0)

# Update function for the slider
def update(val):
    global follower_circle
    angle = slider.val
    follower_x, follower_y = calculate_follower_position(angle, radii, angles)
    follower_circle.set_center((follower_x, follower_y))
    fig.canvas.draw_idle()

slider.on_changed(update)

# Display the plot
plt.show()
