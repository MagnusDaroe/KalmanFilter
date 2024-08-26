#include "Simulate.h"
#include <random>
#include <cmath>
#include <iostream>
#include <cassert>
#include <fstream>

// Constructor
Simulate::Simulate()
{
    

}

// Descretization: RK4 method

// Extract velocity from state and save velocity and acceleration in a 6x1 vector
Vector Simulate::motion_func(Vector state, Vector acceleration)
{
    // Extract velocity from state
    Vector vel = state.tail(3);

    // dy/dt vector. Save velocity and acceleration in a 6x1 vector
    Vector dydt = Vector::Zero(6);
    dydt << vel, acceleration;

    return dydt;
}

// Simulate motion using RK4 method, input state (pos,vel) and acceleration, output new state
Vector Simulate::simulate_motion(Vector state, Vector3 acceleration, float dt)
{
    Vector k1 = Simulate::motion_func(state, acceleration);
    Vector k2 = Simulate::motion_func(state + k1 * dt / 2, acceleration);
    Vector k3 = Simulate::motion_func(state + k2 * dt / 2, acceleration);
    Vector k4 = Simulate::motion_func(state + k3 * dt, acceleration);

    return state + (k1 + 2 * k2 + 2 * k3 + k4) * dt / 6;
}

// Motion profile generation

// Generate a trapezoidal acceleration profile
Matrix Simulate::trapezoidalProfile(int numSamples, double maxAccel) {
    
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


// Measurement related functions

// Function to check if a measurement should be made
bool Simulate::make_measurement(const float sample_rate,const float dt, const int i)
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


// File save related functions

void Simulate::save_info_to_file(const std::string& file_path, const Vector& data) {
    std::ofstream file(file_path, std::ios::app);
    
    if (file.is_open()) {
        for (int i = 0; i < data.size(); ++i) {
            file << data[i];
            if (i < data.size() - 1) {
                file << " "; 
            }
        }
        file << std::endl;
    } else {
        std::cerr << "Unable to open file: " << file_path << std::endl;
    }
}

// Implementation of clear_files
void Simulate::clear_files(const std::vector<std::string>& files) {
    for (const auto& file_path : files) {
        std::ofstream file(file_path);
        
        if (file.is_open()) {
            file << "";
        } else {
            std::cerr << "Unable to open file: " << file_path << "Creating the file..."<<std::endl;

            std::ofstream new_file(file_path);
            if (new_file.is_open()) {
                new_file << "";
            } else {
                std::cerr << "Unable to create file: " << file_path << std::endl;
            }

        }
    }
}