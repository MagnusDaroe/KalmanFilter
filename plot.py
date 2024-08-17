import numpy as np
import matplotlib.pyplot as plt

# Load the data from the file
data = np.loadtxt('data.txt')

# Extract columns from the data
actual_x = data[:, 0]
actual_y = data[:, 1]
actual_z = data[:, 2]
estimated_x = data[:, 3]
estimated_y = data[:, 4]
estimated_z = data[:, 5]
actual_vx = data[:, 6]
actual_vy = data[:, 7]
actual_vz = data[:, 8]
estimated_vx = data[:, 9]
estimated_vy = data[:, 10]
estimated_vz = data[:, 11]
bias_x = data[:, 12]
bias_y = data[:, 13]
bias_z = data[:, 14]

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
