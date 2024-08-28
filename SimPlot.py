import numpy as np
import matplotlib.pyplot as plt

# Load data from the files
actual_position = np.loadtxt(r'Sim_data\actual_position.txt')
estimated_position = np.loadtxt(r'Sim_data\estimated_position.txt')
bias_position = np.loadtxt(r'Sim_data\actual_position.txt')
actual_orientation = np.loadtxt(r'Sim_data\actual_orientation.txt')
estimated_orientation = np.loadtxt(r'Sim_data\estimated_orientation.txt')
bias_orientation = np.loadtxt(r'Sim_data\bias_orientation.txt')

# Extract data
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

bias_x = bias_position[:, 0]
bias_y = bias_position[:, 1]
bias_z = bias_position[:, 2]

actual_roll = actual_orientation[:, 0]
actual_pitch = actual_orientation[:, 1]
actual_yaw = actual_orientation[:, 2]

estimated_roll = estimated_orientation[:, 0]
estimated_pitch = estimated_orientation[:, 1]
estimated_yaw = estimated_orientation[:, 2]

# Extract bias orientation data (assuming it contains Roll, Pitch, Yaw biases)
bias_roll = bias_orientation[:, 0]
bias_pitch = bias_orientation[:, 1]
bias_yaw = bias_orientation[:, 2]

time = np.arange(len(actual_x))

# Create the first figure for Position, Velocity, and Bias
plt.figure(figsize=(14, 12))

# Plot Position
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

plt.tight_layout()
plt.show()

# Create the second figure for Orientation and combined Bias & Orientation
plt.figure(figsize=(14, 10))

# Plot Orientation
plt.subplot(2, 1, 1)
plt.plot(time, actual_roll, color='magenta', linestyle='-', label='Actual Roll')
plt.plot(time, estimated_roll, color='purple', linestyle='--', label='Estimated Roll')
plt.plot(time, actual_pitch, color='cyan', linestyle='-.', label='Actual Pitch')
plt.plot(time, estimated_pitch, color='blue', linestyle=':', label='Estimated Pitch')
plt.plot(time, actual_yaw, color='yellow', linestyle='--', label='Actual Yaw')
plt.plot(time, estimated_yaw, color='orange', linestyle='-', label='Estimated Yaw')
plt.title('Orientation')
plt.xlabel('Time Step')
plt.ylabel('Angle (radians)')
plt.legend()

# Plot Bias in Orientation
plt.subplot(2, 1, 2)
plt.plot(time, bias_roll, color='magenta', linestyle='-', label='Bias Roll')
plt.plot(time, bias_pitch, color='cyan', linestyle='--', label='Bias Pitch')
plt.plot(time, bias_yaw, color='yellow', linestyle=':', label='Bias Yaw')
plt.title('Bias in Orientation')
plt.xlabel('Time Step')
plt.ylabel('Bias (radians)')
plt.legend()

plt.tight_layout()
plt.show()
