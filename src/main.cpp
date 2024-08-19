#include "KalmanFilter.h"
#include <iostream>
#include <fstream>

// Create an instance of KalmanFilter
KalmanFilter kf(Vector::Zero(12), Vector::Zero(6));

// Initialize KalmanFilter object
Vector state_position = kf.get_position_state();

// Initialize actual acceleration and measurement acceleration
Vector actual_acceleration = Vector::Ones(3) * 0.1;
Vector meas_acceleration(3);

// Initialize rotation matrix
Matrix rotation_matrix = Matrix::Identity(3, 3);

// Initialize GPS and barometer measurements
Vector gps = Vector::Zero(2);
Vector baro = Vector::Zero(1);

// Initialize position and velocity
Eigen::Vector3f position = Eigen::Vector3f::Zero();
Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
Eigen::Vector3f old_velocity = Eigen::Vector3f::Zero();

// Sampling rate of sensors
float gps_sample_rate = 1.0;
float barometer_sample_rate = 10;

double dt = 0.1;
Matrix gps_noise_covariance = 0.1 * Matrix::Identity(2, 2);

// Initialize vectors to store actual and estimated position and velocity
std::vector<double> actual_x, actual_y, actual_z, estimated_x, estimated_y, estimated_z;
std::vector<double> actual_vx, actual_vy, actual_vz, estimated_vx, estimated_vy, estimated_vz;
std::vector<double> bias_x, bias_y, bias_z;

// sample amount
int sample_amount = 1000;

bool make_measurement(const float sample_rate,const float dt, const int i)
{
    float condition = 1/(sample_rate * dt);

    if (1/(sample_rate * dt) < 1){
        return true;
    }
    else if (i % static_cast<int>(condition) == 0)
    {
        return true;
    }
    
    return false;
}

int main()
{
    // Print initial position state
    std::cout << "Initial position state: " << state_position.transpose() << std::endl;

    // Print initial position covariance
    for (int i = 0; i < sample_amount; ++i)
    {
        // Simulate motion
        velocity += actual_acceleration * dt;
        position += (velocity+old_velocity) * dt/2;

        // Save actual position and velocity for plotting
        actual_x.push_back(position(0));
        actual_y.push_back(position(1));
        actual_z.push_back(position(2));
        actual_vx.push_back(velocity(0));
        actual_vy.push_back(velocity(1));
        actual_vz.push_back(velocity(2));

        // Simulate acceleration measurement
        meas_acceleration = actual_acceleration;
        meas_acceleration = kf.add_noise(meas_acceleration, Matrix::Identity(3, 3)*0.0001f);

        kf.predict_position(dt, meas_acceleration, rotation_matrix);

        if (make_measurement(gps_sample_rate, dt, i))
        {
            // Simulate GPS measurement
            gps = position.head(2);
            gps = kf.add_noise(gps, gps_noise_covariance);

            kf.update_position("GPS", gps);
        }

        if (make_measurement(barometer_sample_rate, dt, i))
        {
            // Simulate barometer measurement
            baro << position(2);
            baro = kf.add_noise(baro, 0.1 * Matrix::Identity(1, 1));
            kf.update_position("Barometer", baro);
        }

        state_position = kf.get_position_state();

        // Save estimated position for plotting
        estimated_x.push_back(state_position(0));
        estimated_y.push_back(state_position(1));
        estimated_z.push_back(state_position(2));
        estimated_vx.push_back(state_position(3));
        estimated_vy.push_back(state_position(4));
        estimated_vz.push_back(state_position(5));
        bias_x.push_back(state_position(9));
        bias_y.push_back(state_position(10));
        bias_z.push_back(state_position(11));

        //std::cout << "Updated covariance: " << kf.get_position_covariance() << std::endl;

        old_velocity = velocity;
    }

    // save in file for plotting
    std::ofstream file("../data.txt");
    for (int i = 0; i < actual_x.size(); ++i)
    {
        file << actual_x[i] << " " << actual_y[i] << " " << actual_z[i] << " " << estimated_x[i] << " " << estimated_y[i] << " " << estimated_z[i] << " " << actual_vx[i] << " " << actual_vy[i] << " " << actual_vz[i] << " " << estimated_vx[i] << " " << estimated_vy[i] << " " << estimated_vz[i] << " " << bias_x[i] << " " << bias_y[i] << " " << bias_z[i] << std::endl;
    }

    return 0;
}
