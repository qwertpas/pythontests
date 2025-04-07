import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib.widgets import Slider

# Define the curvature calculation
def curvature(x, y):
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = np.abs(ddx * dy - ddy * dx) / ((dx**2 + dy**2)**1.5)
    return np.sum(curvature)

# Define control points
square_size = 2.0
control_points = np.array([
    [-square_size / 2, -square_size / 2],
    [ 0, -square_size/2],   # Symmetric point (can move up and down)
    [ square_size / 2, -square_size / 2],
    [ square_size / 2,  square_size / 2],
    [ 0,  square_size/2],  # Symmetric point (can move up and down)
    [-square_size / 2,  square_size / 2],
    [-square_size / 2, -square_size / 2],
])

# Optimization variables: y-coordinates of the last two points

from scipy.interpolate import splprep, splev

def objective(y):
    updated_points = control_points.copy()
    updated_points[1, 1] = -y[0]
    updated_points[4, 1] = y[0]

    # Fit a smooth curve through the points
    t = np.linspace(0, 1, len(updated_points))
    fine_t = np.linspace(0, 1, 500)

    tck, _ = splprep([updated_points[:, 0], updated_points[:, 1]], s=0, k=3, per=True)
    x, y = splev(fine_t, tck)
    shape_pts = np.column_stack((x, y))

    perimeter = np.sum(np.sqrt(np.sum(np.diff(shape_pts, axis=0)**2, axis=1)))  # Calculate the perimeter
    print(perimeter)
    target_perimeter = 10

    # Compute the total curvature
    return (target_perimeter - perimeter)**2

# Initial guess for the y-coordinates of the symmetric points
initial_guess = [2]

# Optimize
result = minimize(objective, initial_guess, bounds=[(0.5, 3.0)], options={'disp': True})
optimized_y = result.x[0]

print(f"y={optimized_y}, {objective([optimized_y])}")


# Update control points with the optimized y-coordinate
# control_points[4, 1] = optimized_y
# control_points[5, 1] = -optimized_y

# Generate the smooth curve
from scipy.interpolate import splprep, splev
tck, _ = splprep([control_points[:, 0], control_points[:, 1]], s=0, k=3)
fine_t = np.linspace(0, 1, 500)
x, y = splev(fine_t, tck)

# Plot the result
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
line, = ax.plot(x, y, label='Minimal Curvature Shape')
points = ax.scatter(control_points[:, 0], control_points[:, 1], color='red', label='Control Points')
ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
ax.axvline(0, color='gray', linestyle='--', linewidth=0.5)
ax.set_aspect('equal', adjustable='box')
ax.set_title('Shape with Minimal Curvature Passing Through Control Points')
ax.legend()

# Add slider for adjusting the height of the symmetric points
ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03])
slider = Slider(ax_slider, 'Height of Symmetric Points', 0.5, 3.0, valinit=optimized_y)

# Update function for the slider
def update(val):
    height = slider.val
    control_points[4, 1] = height
    control_points[1, 1] = -height
    tck, _ = splprep([control_points[:, 0], control_points[:, 1]], s=0, k=3, per=True)
    x, y = splev(fine_t, tck)
    line.set_data(x, y)
    points.set_offsets(control_points)
    fig.canvas.draw_idle()

update(val=optimized_y)
slider.on_changed(update)

plt.show()
