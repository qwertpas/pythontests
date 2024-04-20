import numpy as np
import matplotlib.pyplot as plt

# Parameters
L1 = 1.0  # Length of link 1
L2 = 0.8  # Length of link 2
L3 = 0.6  # Length of link 3
wall_position = 2.0  # Position of the vertical wall
robot_position = np.array([0.0, 1.5, 0.0])  # Initial position of the robot end-effector
theta1 = np.pi / 4  # Initial angle of joint 1
theta2 = np.pi / 6  # Initial angle of joint 2
theta3 = np.pi / 3  # Initial angle of joint 3

# Simulation parameters
dt = 0.01  # Time step
total_time = 5.0  # Total simulation time
num_steps = int(total_time / dt)

# Simulation arrays
positions = np.zeros((num_steps, 3))  # Array to store end-effector positions
forces = np.zeros(num_steps)  # Array to store contact forces

# Simulation loop
for t in range(num_steps):
    # Forward kinematics
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2) + L3 * np.sin(theta1 + theta2 + theta3)
    
    positions[t] = [x, y, 0.0]  # Store end-effector position
    
    # Calculate distance to the wall
    distance_to_wall = wall_position - x
    
    # Calculate contact force
    if distance_to_wall <= 0:
        contact_force = -distance_to_wall * 10.0  # Simple linear force model
    else:
        contact_force = 0.0
    
    forces[t] = contact_force  # Store contact force
    
    # Inverse kinematics to calculate joint velocities
    J = np.array([[-L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2) - L3 * np.sin(theta1 + theta2 + theta3),
                   -L2 * np.sin(theta1 + theta2) - L3 * np.sin(theta1 + theta2 + theta3),
                   -L3 * np.sin(theta1 + theta2 + theta3)],
                  [L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3),
                   L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3),
                   L3 * np.cos(theta1 + theta2 + theta3)]])
    
    joint_velocities = np.linalg.pinv(J) @ np.array([0, -contact_force])  # Pseudo-inverse to solve for joint velocities
    
    # Update joint angles
    theta1 += joint_velocities[0] * dt
    theta2 += joint_velocities[1] * dt
    theta3 += joint_velocities[2] * dt

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(positions[:, 0], positions[:, 1], label='End-effector trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('3-Axis Robot Arm Simulation')
plt.legend()
plt.grid(True)

plt.figure(figsize=(10, 6))
plt.plot(np.arange(num_steps) * dt, forces, label='Contact force')
plt.xlabel('Time')
plt.ylabel('Contact Force')
plt.title('Contact Force Over Time')
plt.legend()
plt.grid(True)

plt.show()
