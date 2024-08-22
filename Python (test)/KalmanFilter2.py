import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.1  # time step
total_time = 20  # total time for the simulation
num_steps = int(total_time / dt)

# True initial state [x position, y position, x velocity, y velocity]
true_state = np.array([0, 0, 1, 1])  # starting at (0,0) with velocity (1,1)

# State transition matrix (constant velocity model in 2D)
A = np.array([[1, 0, dt, 0], 
              [0, 1, 0, dt], 
              [0, 0, 1, 0], 
              [0, 0, 0, 1]])

# Process noise covariance matrix
Q = np.eye(4)

# Measurement matrices for GPS (position in 2D)
H_gps = np.array([[1, 0, 0, 0], 
                  [0, 1, 0, 0]])

# Measurement noise covariance matrices
R_gps = np.array([[2**2, 0], 
                  [0, 2**2]])  # GPS noise with std deviation of 5 meters

# Initial estimates
x_est = np.array([0, 0, 0, 0])  # initial state estimate
P_est = np.eye(4)  # initial covariance estimate

# Arrays to store results
true_states = []
gps_measurements = []
estimates = []

# Simulate the process
for step in range(num_steps):
    # True state update (no noise added for simplicity)
    true_state = A @ true_state

    # Generate noisy measurements
    gps_measurement = H_gps @ true_state + np.random.normal(0, 2, size=(2,))

    # Kalman Filter Prediction Step
    x_pred = A @ x_est
    P_pred = A @ P_est @ A.T + Q

    # Kalman Filter Update Step (GPS)
    y_gps = gps_measurement - H_gps @ x_pred  # Measurement residual
    S_gps = H_gps @ P_pred @ H_gps.T + R_gps  # Innovation covariance
    K_gps = P_pred @ H_gps.T @ np.linalg.inv(S_gps)  # Kalman gain
    x_est = x_pred + K_gps @ y_gps  # State update
    P_est = (np.eye(4) - K_gps @ H_gps) @ P_pred  # Covariance update

    # Store results
    true_states.append(true_state.copy())
    gps_measurements.append(gps_measurement)
    estimates.append(x_est.copy())

# Convert lists to arrays for plotting
true_states = np.array(true_states)
gps_measurements = np.array(gps_measurements)
estimates = np.array(estimates)

# Plot the results
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(true_states[:, 0], true_states[:, 1], label='True Position')
plt.plot(gps_measurements[:, 0], gps_measurements[:, 1], 'r.', label='GPS Measurements')
plt.plot(estimates[:, 0], estimates[:, 1], 'g-', label='Kalman Filter Estimate')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(true_states[:, 2], label='True X Velocity')
plt.plot(estimates[:, 2], 'g-', label='Kalman Filter X Velocity Estimate')
plt.plot(true_states[:, 3], label='True Y Velocity')
plt.plot(estimates[:, 3], 'b-', label='Kalman Filter Y Velocity Estimate')
plt.xlabel('Time Step')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
