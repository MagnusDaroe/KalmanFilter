#ifndef SIMULATE_H
#define SIMULATE_H

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <iostream>

// Define aliases for Eigen types
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;
using Vector3 = Eigen::Vector3f;
using Vector4 = Eigen::Vector4f;

class Simulate
{
private:
    
    // Descretization
    Vector motion_func(Vector state, Vector acceleration);
   

public:
    // Constructor
    Simulate();

    // Simulate motion
    Vector simulate_motion(Vector state, Vector3 acceleration, float dt);

    //Simulate trapezoidal motion profile. Input: number of samples and max acceleration. Output: 3xnumSamples matrix with acceleration profile
    Matrix trapezoidalProfile(int numSamples, double maxAccel);

    // Measurement related functions
    bool make_measurement(const float sample_rate,const float dt, const int i);

    // File save related functions. Input a list of paths to files to clear
    void save_info_to_file(const std::string& file_path, const Vector& data);
    void clear_files(const std::vector<std::string>& files);

};

#endif // Simulate_H
