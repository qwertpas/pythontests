from scipy.interpolate import CubicSpline
import numpy as np

# Sample data points
x = np.array([0, 1, 2, 3, 4])
y = np.array([1, 2, 1, 2, 1])

# Create a closed cubic spline
cs = CubicSpline(x, y, bc_type='periodic')

# Evaluate the spline at new points
x_new = np.linspace(0, 4, 100)
y_new = cs(x_new)

# Plot the original data and the spline
import matplotlib.pyplot as plt
plt.plot(x, y, 'o', label='data')
plt.plot(x_new, y_new, label='closed spline')
plt.legend()
plt.show()