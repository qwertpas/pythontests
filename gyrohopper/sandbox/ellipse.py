
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Approximation of ellipse perimeter (Ramanujan's formula)
def ellipse_perimeter(a, b):
    h = ((a - b)**2) / ((a + b)**2)
    return np.pi * (a + b) * (1 + (3 * h) / (10 + np.sqrt(4 - 3 * h)))

# Function to calculate semi-major and semi-minor axes for a constant perimeter
def calculate_axes(perimeter, shape_factor):
    def error_function(a):
        b = a * shape_factor
        return ellipse_perimeter(a, b) - perimeter

    # Use numerical solving to find the correct semi-major axis 'a'
    from scipy.optimize import bisect
    a = bisect(error_function, 0.1, 100)  # Find 'a' such that the perimeter matches
    b = a * shape_factor
    return a, b

# Initial parameters
initial_shape_factor = 0.5  # Ratio b/a
perimeter = 20  # Constant perimeter

a, b = calculate_axes(perimeter, initial_shape_factor)

def plot_ellipse(ax, a, b):
    t = np.linspace(0, 2 * np.pi, 500)
    x = a * np.cos(t)
    y = b * np.sin(t)
    ellipse, = ax.plot(x, y, label=f"Ellipse (P = {perimeter:.2f})")
    return ellipse

# Setup plot
fig, ax = plt.subplots(figsize=(8, 8))
plt.subplots_adjust(bottom=0.25)

# Initial ellipse
ellipse = plot_ellipse(ax, a, b)
ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
ax.axvline(0, color='gray', linestyle='--', linewidth=0.5)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_aspect('equal')
ax.set_title("Ellipse with Constant Perimeter")
ax.legend()

# Add slider for shape adjustment
ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03])
slider = Slider(ax_slider, 'Shape Factor (b/a)', 0.1, 1.0, valinit=initial_shape_factor)

# Update function for the slider
def update(val):
    shape_factor = slider.val
    a, b = calculate_axes(perimeter, shape_factor)
    t = np.linspace(0, 2 * np.pi, 500)
    x = a * np.cos(t)
    y = b * np.sin(t)
    ellipse.set_data(x, y)
    ax.legend([ellipse], [f"Ellipse (P = {perimeter:.2f},{ellipse_perimeter(a, b):.2f}, b/a = {shape_factor:.2f})"])
    fig.canvas.draw_idle()

slider.on_changed(update)

# Display the plot
plt.show()


    # ax.legend([ellipse], [f"Ellipse (P = {perimeter:.2f},{ellipse_perimeter(a, b):.2f}, b/a = {shape_factor:.2f})"])
