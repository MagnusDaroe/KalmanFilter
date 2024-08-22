import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.1  # time step
total_time = 20  # total time for the simulation
num_steps = int(total_time / dt)

# True initial state [position, velocity]
true_state = np.array([0, 20])  # start at position 0 with velocity 20 m/s

# State transition matrix (constant velocity model)
A = np.array([[1, dt], [0, 1]])

# Process noise covariance matrix
Q = np.array([[1, 0], [0, 1]])

# Measurement matrices for GPS (position) and speedometer (velocity)
H_gps = np.array([[1, 0]])
H_speedometer = np.array([[0, 1]])

# Measurement noise covariance matrices
std_GPS = 100
std_speedometer = 2
R_gps = np.array([[std_GPS**2]])  # GPS noise with std deviation of 5 meters
R_speedometer = np.array([[std_speedometer**2]])  # Speedometer noise with std deviation of 2 m/s

# Initial estimates
x_est = np.array([0, 0])  # initial state estimate
P_est = np.eye(2)  # initial covariance estimate

# Arrays to store results
true_states = []
gps_measurements = []
speedometer_measurements = []
estimates = []

# Simulate the process
for step in range(num_steps):
    # True state update (no noise added for simplicity)
    true_state = A @ true_state

    # Generate noisy measurements
    gps_measurement = H_gps @ true_state + np.random.normal(0, std_GPS)
    speedometer_measurement = H_speedometer @ true_state + np.random.normal(0, std_speedometer)

    # Kalman Filter Prediction Step
    x_pred = A @ x_est
    P_pred = A @ P_est @ A.T + Q

    # Kalman Filter Update Step (GPS)
    ## Measurement residual
    y_gps = gps_measurement - H_gps @ x_pred 
    # Innovation covariance
    S_gps = H_gps @ P_pred @ H_gps.T + R_gps
    # Kalman gain 
    K_gps = P_pred @ H_gps.T @ np.linalg.inv(S_gps) 
     # State update
    x_est = x_pred + K_gps @ y_gps
    # Covariance update
    P_est = (np.eye(2) - K_gps @ H_gps) @ P_pred 

    # Kalman Filter Update Step (Speedometer)
    y_speedometer = speedometer_measurement - H_speedometer @ x_est # Measurement residual
    S_speedometer = H_speedometer @ P_est @ H_speedometer.T + R_speedometer
    K_speedometer = P_est @ H_speedometer.T @ np.linalg.inv(S_speedometer)
    x_est = x_est + K_speedometer @ y_speedometer
    P_est = (np.eye(2) - K_speedometer @ H_speedometer) @ P_est

    # Store results
    true_states.append(true_state.copy())
    gps_measurements.append(gps_measurement)
    speedometer_measurements.append(speedometer_measurement)
    estimates.append(x_est.copy())

# Convert lists to arrays for plotting
true_states = np.array(true_states)
gps_measurements = np.array(gps_measurements)
speedometer_measurements = np.array(speedometer_measurements)
estimates = np.array(estimates)

# Plot the results
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(true_states[:, 0], label='True Position')
plt.plot(gps_measurements, 'r.', label='GPS Measurements')
plt.plot(estimates[:, 0], 'g-', label='Kalman Filter Estimate')
plt.xlabel('Time Step')
plt.ylabel('Position (m)')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(true_states[:, 1], label='True Velocity')
plt.plot(speedometer_measurements, 'r.', label='Speedometer Measurements')
plt.plot(estimates[:, 1], 'g-', label='Kalman Filter Estimate')
plt.xlabel('Time Step')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()


#Function to 