// cmake .. -G "MinGW Makefiles"
// mingw32-make
//./KalmanFilter

#include "KalmanFilter.h"
#include "Simulate.h"
#include <iostream>
#include <cmath>

// Create an instance of KalmanFilter
KalmanFilter kf(Vector::Zero(12), Vector::Zero(6));

// Create an instance of Simulate
Simulate sim;

// Initialize actual acceleration and measurement acceleration. The actual acceleration is applied in a global frame
Vector actual_acceleration = Vector::Ones(3) * 0.05;
Vector meas_acceleration(3);

// Initialize orientation state. Orientation acceleration is in a global frame
Vector actual_w_acceleration = Vector::Ones(3) * 0.01;
Vector meas_w_velocity = Vector::Zero(3);
Vector meas_orientation = Vector::Zero(3);

// Initialize rotation matrix
Matrix rotation_matrix = Matrix::Identity(3, 3);

// Initialize GPS and barometer measurements
Vector gps = Vector::Zero(2);
Vector baro = Vector::Zero(1);

// Actual state
Vector actual_state_position = Vector::Zero(6);
Vector actual_state_orientation = Vector::Zero(6);

// Estimated state
Vector estimated_position = Vector::Zero(6);
Vector estimated_bias = Vector::Zero(3);

// Sampling rate of sensors
float gps_sample_rate = 2.0;
float barometer_sample_rate = 10;
float magnometer_sample_rate = 10;
float accel_sample_rate = 70;
float gyro_sample_rate = 70;

// Noise
Matrix gps_noise_covariance = 0.1 * Matrix::Identity(2, 2);
Matrix baro_noise_covariance = 0.1 * Matrix::Identity(1, 1);
Matrix accel_noise_covariance = 0.078 * Matrix::Identity(3, 3);
Matrix gyro_noise_covariance = 0.01 * Matrix::Identity(3, 3);
Matrix mag_noise_covariance = 0.01 * Matrix::Identity(3, 3);

// sample amount and time step
double dt = 1.0 / 70;
int sample_time_sec = 10;
int sample_amount = static_cast<int>(sample_time_sec / dt);

// Function to save actual position and velocity. List with paths
std::vector<std::string> file_paths = {
    "../Sim_data/actual_position.txt",
    "../Sim_data/actual_orientation.txt",
    "../Sim_data/estimated_position.txt",
    "../Sim_data/estimated_orientation.txt",
    "../Sim_data/bias_postion.txt",
    "../Sim_data/bias_orientation.txt"};

void predict(int timestep)
{
    // Predict step
    if (sim.make_measurement(accel_sample_rate, dt, timestep))
    {
        // Simulate acceleration measurement
        meas_acceleration = actual_acceleration;
        meas_acceleration = kf.add_noise(meas_acceleration, accel_noise_covariance);

        // Make prediction
        kf.predict_position(dt, meas_acceleration, rotation_matrix);
    }

    if (sim.make_measurement(gyro_sample_rate, dt, timestep))
    {
        // Simulate angular acceleration measurement
        meas_w_velocity = actual_state_orientation.tail(3);
        meas_w_velocity = kf.add_noise(meas_w_velocity, gyro_noise_covariance);

        // Make prediction
        kf.predict_orientation(dt, meas_w_velocity);
    }
}

void update(int timestep)
{
    // Update step

    // Position update
    if (sim.make_measurement(gps_sample_rate, dt, timestep))
    {
        // Simulate GPS measurement
        gps = actual_state_position.head(2);
        gps = kf.add_noise(gps, gps_noise_covariance);

        kf.update_position("GPS", gps);
    }

    if (sim.make_measurement(barometer_sample_rate, dt, timestep))
    {
        // Simulate barometer measurement
        baro << actual_state_position(2);
        baro = kf.add_noise(baro, 0.1 * Matrix::Identity(1, 1));
        kf.update_position("Barometer", baro);
    }

    // Orientation update
    if (sim.make_measurement(magnometer_sample_rate, dt, timestep))
    {
        // Simulate magnometer measurement
        meas_orientation = actual_state_orientation.head(3);

        // Print size of meas_orientation and mag_noise_covariance
        meas_orientation = kf.add_noise(meas_orientation, mag_noise_covariance);

        kf.update_orientation("Magnometer", kf.angleToQuat(meas_orientation), mag_noise_covariance);
    }
}

int main()
{

    // Initialize KalmanFilter object
    Vector state_position = kf.get_position_state();
    Vector state_orientation = kf.get_orientation_state();
    Vector state_orientation_euler = kf.get_orientation_state_euler();

    // Clear the files before writing
    sim.clear_files(file_paths);

    // Print initial position state
    std::cout << "Starting simulation.... will simulate for " << sample_time_sec << " seconds." << std::endl;

    // Generate trapezoidal profile
    Matrix accelProfile = sim.trapezoidalProfile(sample_amount, 0.01);

    // Print initial position covariance
    for (int i = 0; i < sample_amount; ++i)
    {
        // Extract actual acceleration and angular acceleration
        actual_acceleration = accelProfile.col(i);
        //actual_w_acceleration = accelProfile.col(i);
        actual_w_acceleration = Vector::Ones(3) * 0.01;

        // Simulate motion propagation based on acceleration
        actual_state_position = sim.simulate_motion(actual_state_position, actual_acceleration, dt);
        actual_state_orientation = sim.simulate_motion(actual_state_orientation, actual_w_acceleration, dt);

        // Save actual position and velocity for plotting
        sim.save_info_to_file(file_paths[0], actual_state_position);
        sim.save_info_to_file(file_paths[1], actual_state_orientation);

        // Predict step
        predict(i);

        // Update step
        update(i);

        // Get estimated position & orientation state
        state_position = kf.get_position_state();
        state_orientation_euler = kf.get_orientation_state_euler();

        // Save estimated position for plotting. Unpack the state vector
        estimated_position = state_position.head(6);
        estimated_bias = state_position.tail(3);

        sim.save_info_to_file(file_paths[2], estimated_position);
        sim.save_info_to_file(file_paths[4], estimated_bias);

        // Save estimated orientation and bias for plotting
        estimated_bias = state_orientation.tail(3);
        sim.save_info_to_file(file_paths[3], state_orientation_euler);
        sim.save_info_to_file(file_paths[5], estimated_bias);
    }

    std::cout << "Simulation complete. Data saved to files." << std::endl;

    return 0;
}
