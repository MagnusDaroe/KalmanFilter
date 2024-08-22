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

// Initialize KalmanFilter object
Vector state_position = kf.get_position_state();
Vector state_orientation = kf.get_orientation_state();

// Initialize actual acceleration and measurement acceleration. The actual acceleration is applied in a global frame
Vector actual_acceleration = Vector::Ones(3) * 0.05;
Vector meas_acceleration(3);

// Initialize orientation state. Orientation acceleration is in a global frame
Vector actual_w_acceleration = Vector::Ones(3) * 0.01;
Vector meas_w_acceleration = Vector::Zero(3);

// Initialize rotation matrix
Matrix rotation_matrix = Matrix::Identity(3, 3);

// Initialize GPS and barometer measurements
Vector gps = Vector::Zero(2);
Vector baro = Vector::Zero(1);

//Actual state
Vector actual_state = Vector::Zero(6);

//Estimated state
Vector estimated_position = Vector::Zero(6);
Vector estimated_bias = Vector::Zero(3);

// Sampling rate of sensors
float gps_sample_rate = 2.0;
float barometer_sample_rate = 10;

double dt = 0.1;
Matrix gps_noise_covariance = 0.1 * Matrix::Identity(2, 2);

// sample amount
int sample_amount = 1000;

// Function to save actual position and velocity. List with paths
std::vector<std::string> file_paths = {
        "../data/actual_position.txt",
        "../data/actual_orientation.txt",
        "../data/estimated_position.txt",
        "../data/estimated_orientation.txt",
        "../data/bias_values.txt"
    };

int main()
{
    // Clear the files before writing
    sim.clear_files(file_paths);

    // Print initial position state
    std::cout << "Starting simulation" << std::endl;
    
    //Generate trapezoidal profile
    Matrix accelProfile = sim.trapezoidalProfile(sample_amount, 0.1);

    // Print initial position covariance
    for (int i = 0; i < sample_amount; ++i)
    {
        //Extract actual acceleration
        actual_acceleration = accelProfile.col(i);

        // Simulate motion propegation based on acceleration
        actual_state = sim.simulate_motion(actual_state, actual_acceleration, dt);

        // Save actual position and velocity for plotting
        sim.save_info_to_file(file_paths[0], actual_state);

        // Simulate acceleration measurement
        meas_acceleration = actual_acceleration;
        meas_acceleration = kf.add_noise(meas_acceleration, Matrix::Identity(3, 3)*0.0001f);

        // Predict position
        kf.predict_position(dt, meas_acceleration, rotation_matrix);

        // Update position based on sensor measurements
        if (sim.make_measurement(gps_sample_rate, dt, i))
        {
            // Simulate GPS measurement
            gps = actual_state.head(2);
            gps = kf.add_noise(gps, gps_noise_covariance);

            kf.update_position("GPS", gps);
        }

        if (sim.make_measurement(barometer_sample_rate, dt, i))
        {
            // Simulate barometer measurement
            baro << actual_state(2);
            baro = kf.add_noise(baro, 0.1 * Matrix::Identity(1, 1));
            kf.update_position("Barometer", baro);
        }

        // Get estimated position state
        state_position = kf.get_position_state();

        // Save estimated position for plotting. Unpack the state vector
        estimated_position = state_position.head(6);
        estimated_bias = state_position.tail(3);

        sim.save_info_to_file(file_paths[2], estimated_position);
        sim.save_info_to_file(file_paths[4], estimated_bias);
    }

    return 0;
}
