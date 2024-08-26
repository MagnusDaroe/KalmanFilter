import numpy as np
import matplotlib.pyplot as plt

# Load data from the files
actual_position = np.loadtxt('Sim_data/actual_position.txt')
estimated_position = np.loadtxt('Sim_data/estimated_position.txt')
bias = np.loadtxt('Sim_data/bias_values.txt')

# Assuming the position files also contain velocity data (X, Y, Z, Vx, Vy, Vz)
actual_x = actual_position[:, 0]
actual_y = actual_position[:, 1]
actual_z = actual_position[:, 2]
actual_vx = actual_position[:, 3]
actual_vy = actual_position[:, 4]
actual_vz = actual_position[:, 5]

estimated_x = estimated_position[:, 0]
estimated_y = estimated_position[:, 1]
estimated_z = estimated_position[:, 2]
estimated_vx = estimated_position[:, 3]
estimated_vy = estimated_position[:, 4]
estimated_vz = estimated_position[:, 5]

# Assuming the bias file contains Bias X, Bias Y, Bias Z
bias_x = bias[:, 0]
bias_y = bias[:, 1]
bias_z = bias[:, 2]

# Time vector (assuming each entry corresponds to a sequential time step)
time = np.arange(len(actual_x))

# Plot Position
plt.figure(figsize=(14, 10))

plt.subplot(3, 1, 1)
plt.plot(time, actual_x, color='darkorange', linestyle='-', label='Actual X')
plt.plot(time, estimated_x, color='royalblue', linestyle='--', label='Estimated X')
plt.plot(time, actual_y, color='darkorange', linestyle='-.', label='Actual Y')
plt.plot(time, estimated_y, color='royalblue', linestyle=':', label='Estimated Y')
plt.plot(time, actual_z, color='darkorange', linestyle='--', label='Actual Z')
plt.plot(time, estimated_z, color='royalblue', linestyle='-', label='Estimated Z')
plt.title('Position')
plt.xlabel('Time Step')
plt.ylabel('Position (m)')
plt.legend()

# Plot Velocity
plt.subplot(3, 1, 2)
plt.plot(time, actual_vx, color='firebrick', linestyle='-', label='Actual Vx')
plt.plot(time, estimated_vx, color='teal', linestyle='--', label='Estimated Vx')
plt.plot(time, actual_vy, color='firebrick', linestyle='-.', label='Actual Vy')
plt.plot(time, estimated_vy, color='teal', linestyle=':', label='Estimated Vy')
plt.plot(time, actual_vz, color='firebrick', linestyle='--', label='Actual Vz')
plt.plot(time, estimated_vz, color='teal', linestyle='-', label='Estimated Vz')
plt.title('Velocity')
plt.xlabel('Time Step')
plt.ylabel('Velocity (m/s)')
plt.legend()

# Plot Bias
plt.subplot(3, 1, 3)
plt.plot(time, bias_x, color='green', linestyle='-', label='Bias X')
plt.plot(time, bias_y, color='darkgreen', linestyle='--', label='Bias Y')
plt.plot(time, bias_z, color='forestgreen', linestyle=':', label='Bias Z')
plt.title('Bias')
plt.xlabel('Time Step')
plt.ylabel('Bias')
plt.legend()

# Adjust layout and show plot
plt.tight_layout()
plt.show()
