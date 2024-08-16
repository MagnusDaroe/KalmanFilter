#include "KalmanFilter.h"
#include <iostream>

int main()
{
    // Define initial states for position and orientation
    Eigen::Vector3f init_pos_state(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f init_orient_state(0.0f, 0.0f, 0.0f);

    // Create an instance of KalmanFilter
    KalmanFilter kf(init_pos_state, init_orient_state);

    // Define a quaternion (4 elements) and scalar qw
    Eigen::Vector4f quaternion(0.0f, 0.0f, 0.0f, 1.0f); // Example quaternion
    float qw = 1.0f; // Example scalar

    // Call the quatToAngle function
    Eigen::Vector3f euler_angles = kf.quatToAngle(quaternion, qw);

    // Output the results
    std::cout << "Euler angles: " << std::endl;
    std::cout << "X: " << euler_angles(0) << std::endl;
    std::cout << "Y: " << euler_angles(1) << std::endl;
    std::cout << "Z: " << euler_angles(2) << std::endl;

    return 0;
}
