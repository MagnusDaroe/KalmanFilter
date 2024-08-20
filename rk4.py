import numpy as np
import matplotlib.pyplot as plt

dt = 0.1
samples = 1000

def f(state, accel):
    # Extract velocity from the state and acceleration from the input
    f = np.zeros((6, 1))
    f[0] = state[3]
    f[1] = state[4]
    f[2] = state[5]
    f[3] = accel[0]
    f[4] = accel[1]
    f[5] = accel[2]
    return f

def rk4(state, accel, dt):
    k1 = f(state, accel)
    k2 = f(state + dt/2*k1, accel)
    k3 = f(state + dt/2*k2, accel)
    k4 = f(state + dt*k3, accel)
    return state + dt/6*(k1 + 2*k2 + 2*k3 + k4)

def euler(state, accel, dt):
    return state + dt * f(state, accel)


def main():
    state = np.zeros((6, 1))
    accel = np.array([2.0, 2.0, 2.0])
    accel_add = np.array([0.1, 0.1, 0.1])

    rk4_state = []
    euler_state = []

    for i in range(samples+1):
        accel += accel_add
        # Update the state using RK4 and Euler methods
        state_rk4 = rk4(state, accel, dt)
        state_euler = euler(state, accel, dt)
        
        rk4_state.append(state_rk4.flatten())
        euler_state.append(state_euler.flatten())

        state = state_rk4  # update state with RK4 (or Euler if you prefer)

    # Convert lists to numpy arrays for easy slicing
    rk4_state = np.array(rk4_state)
    euler_state = np.array(euler_state)

    # Plot the results
    plt.figure(figsize=(14, 10))
    plt.subplot(3, 1, 1)
    plt.plot(rk4_state[:, 0], color='royalblue', linestyle='-', label='RK4 X')
    plt.plot(euler_state[:, 0], color='darkorange', linestyle='-', label='Euler X')
    plt.title('Position X')
    plt.xlabel('Time Step')
    plt.ylabel('Position (m)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(rk4_state[:, 1], color='royalblue', linestyle='-', label='RK4 Y')
    plt.plot(euler_state[:, 1], color='darkorange', linestyle='-', label='Euler Y')
    plt.title('Position Y')
    plt.xlabel('Time Step')
    plt.ylabel('Position (m)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(rk4_state[:, 2], color='royalblue', linestyle='-', label='RK4 Z')
    plt.plot(euler_state[:, 2], color='darkorange', linestyle='-', label='Euler Z')
    plt.title('Position Z')
    plt.xlabel('Time Step')
    plt.ylabel('Position (m)')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
