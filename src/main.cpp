// cmake .. -G "MinGW Makefiles"
// mingw32-make
//./KalmanFilter

#include "KalmanFilter.h"
#include <iostream>
#include <fstream>
#include <cmath>

// Create an instance of KalmanFilter
KalmanFilter kf(Vector::Zero(12), Vector::Zero(6));

// Initialize KalmanFilter object
Vector state_position = kf.get_position_state();
Vector state_orientation = kf.get_orientation_state();

// Initialize actual acceleration and measurement acceleration. The actual acceleration is applied in a global frame
Vector actual_acceleration = Vector::Ones(3) * 0.1;
Vector meas_acceleration(3);

// Initialize orientation state. Orientation acceleration is in a global frame
Vector actual_w_acceleration = Vector::Ones(3) * 0.01;
Vector meas_w_acceleration = Vector::Zero(3);

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

// sample amount
int sample_amount = 1000;

// Generate motion profile
Matrix trapezoidalProfile(int numSamples, double maxAccel) {
    Matrix accelProfile(3, numSamples);
    float accel = 0;

    // Determine the number of samples in the ramp-up and ramp-down phases
    int rampSamples = numSamples / 3;
    int zeroAccelSamples = numSamples - 2 * rampSamples;

    // Ramp-up phase
    for (int i = 0; i < rampSamples; ++i) {
        //accel = maxAccel * (static_cast<double>(i) / rampSamples);
        accel = maxAccel;

        accelProfile(0,i) = accel;
        accelProfile(1,i) = accel;
        accelProfile(2,i) = accel;
    }

    // Zero acceleration phase
    for (int i = rampSamples; i < rampSamples + zeroAccelSamples; ++i) {
        accelProfile(0,i) = 0;
        accelProfile(1,i) = 0;
        accelProfile(2,i) = 0;
    }

    // Ramp-down phase
    for (int i = rampSamples + zeroAccelSamples; i < numSamples; ++i) {
        //accel = maxAccel * (1.0 - static_cast<double>(i - rampSamples) / rampSamples);
        accel = -maxAccel;
        
        accelProfile(0,i) = accel;
        accelProfile(1,i) = accel;
        accelProfile(2,i) = accel;
    }

    return accelProfile;
}

// Function to check if a measurement should be made
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

// Function to save actual position and velocity
void save_actual_position()
{
    // Save actual position and velocity for plotting in actual_values.txt
    std::ofstream file("../actual_position.txt", std::ios::app);
    if (file.is_open())
    {
        file << position(0) << " " << position(1) << " " << position(2) << " " << velocity(0) << " " << velocity(1) << " " << velocity(2) << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

void save_estimated_position(){
    // Save estimated position and velocity for plotting in estimated_values.txt
    std::ofstream file("../estimated_position.txt", std::ios::app);
    
    // Check if the file is open before writing
    if (file.is_open())
    {
        file << state_position(0) << " " << state_position(1) << " " << state_position(2) << " " << state_position(3) << " " << state_position(4) << " " << state_position(5) << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing." << std::endl;   
    }
}

void save_actual_orientation()
{
    // Save actual orientation for plotting in actual_orientation.txt
    std::ofstream file("../actual_orientation.txt", std::ios::app);
    if (file.is_open())
    {
        //file << x(0) << " " << x(1) << " " << x(2) << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

void save_estimated_orientation()
{
    // Save estimated orientation for plotting in estimated_orientation.txt
    std::ofstream file("../estimated_orientation.txt", std::ios::app);
    if (file.is_open())
    {
        file << state_orientation(0) << " " << state_orientation(1) << " " << state_orientation(2) << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

void save_bias_values()
{
    // Open the file in append mode to avoid overwriting
    std::ofstream file("../bias_values.txt", std::ios::app);

    // Check if the file is open before writing
    if (file.is_open())
    {
        file << state_position(6) << " " << state_position(7) << " " << state_position(8) << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

void clear_files()
{
    // Clear the files before writing
    std::ofstream file1("../actual_position.txt");
    std::ofstream file2("../estimated_position.txt");
    std::ofstream file3("../actual_orientation.txt");
    std::ofstream file4("../estimated_orientation.txt");
    std::ofstream file5("../bias_values.txt");

    // Check if the files are open before writing
    if (file1.is_open() && file2.is_open() && file3.is_open() && file4.is_open() && file5.is_open())
    {
        file1 << "";
        file2 << "";
        file3 << "";
        file4 << "";
        file5 << "";
    }
    else
    {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}


int main()
{
    // Clear the files before writing
    clear_files();

    // Print initial position state
    std::cout << "Starting simulation" << std::endl;
    
    //Generate trapezoidal profile
    Matrix accelProfile = trapezoidalProfile(sample_amount, 0.1);

    // Print initial position covariance
    for (int i = 0; i < sample_amount; ++i)
    {
        //Extract actual acceleration
        actual_acceleration = accelProfile.col(i);

        // Simulate motion
        velocity += actual_acceleration * dt;
        position += (velocity+old_velocity) * dt/2;

        // Save actual position and velocity for plotting
        save_actual_position();

        // Simulate acceleration measurement
        meas_acceleration = actual_acceleration;
        meas_acceleration = kf.add_noise(meas_acceleration, Matrix::Identity(3, 3)*0.0001f);

        // Predict position
        kf.predict_position(dt, meas_acceleration, rotation_matrix);

        // Update position based on sensor measurements
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

        // Get estimated position state
        state_position = kf.get_position_state();

        // Save estimated position for plotting
        save_estimated_position();
        save_bias_values();
    
        // Save actual and estimated position and velocity
        old_velocity = velocity;
    }

    return 0;
}
