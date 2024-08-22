import numpy as np
import matplotlib.pyplot as plt

step = 0.1

def predict(A, B, x_est, u,  P_est, Q):
    """
    A: State transition matrix
    B: Control matrix
    x_est: State estimate
    u: Control input
    P_est: Covariance estimate
    Q: Process noise covariance matrix
    """
    x_pred = A @ x_est + B @ u # State prediction
    P_pred = A @ P_est @ A.T + Q # Covariance prediction
    return x_pred, P_pred

def update(x_pred, P_pred, H, R, measurement):
    """
    x_pred: State prediction
    P_pred: Covariance prediction
    H: Measurement matrix
    R: Measurement noise covariance matrix
    measurement: Measurement vector

    Returns:
    x_est: State estimate
    P_est: Covariance estimate 
    """

    y = measurement - H @ x_pred # Measurement residual
    S = H @ P_pred @ H.T + R # Innovation covariance
    K = P_pred @ H.T @ np.linalg.inv(S) # Kalman gain
    x_est = x_pred + K @ y # State update
    P_est = (np.eye(len(x_pred)) - K @ H) @ P_pred # Covariance update
    return x_est, P_est


def simulate_kalman_filter(A, Q, H, R, x_est, P_est, num_steps, dt):


def graph_kalman_filter(true_states, gps_measurements, estimates):
    true_states = np.array(true_states)
    gps_measurements = np.array(gps_measurements)
    estimates = np.array(estimates)

    plt.figure(figsize=(10, 6))
    plt.plot(true_states[:, 0], true_states[:, 1], label='True Position', color='blue')
    plt.scatter(gps_measurements[:, 0], gps_measurements[:, 1], label='GPS Measurements', color='red')
    plt.plot(estimates[:, 0], estimates[:, 1], label='Kalman Filter Estimate', color='green')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Kalman Filter 2D Position Estimation')
    plt.legend()
    plt.grid(True)
    plt.show()


true_states, gps_measurements, estimates = simulate_kalman_filter(A=np.array([[1, 0, step, 0],
                                   [0, 1, 0, step],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                       Q=np.array([[0.1, 0, 0, 0],
                                   [0, 0.1, 0, 0],
                                   [0, 0, 0.1, 0],
                                   [0, 0, 0, 0.1]]),
                       H=np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0]]),
                       R=np.array([[2, 0],
                                   [0, 2]]),
                       x_est=np.array([0, 0, 1, 1]),
                       P_est=np.eye(4),
                       num_steps=100,
                       dt=0.1)

graph_kalman_filter(true_states, gps_measurements, estimates)