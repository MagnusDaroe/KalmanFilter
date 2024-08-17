#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <iostream>

#define PI 3.141592653589

// Define aliases for Eigen types
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;
using Vector3 = Eigen::Vector3f;
using Vector4 = Eigen::Vector4f;

class KalmanFilter
{
private:
    // State variables
    Vector state_position;
    Vector state_orientation;
    Vector state_quaternion;
    Vector state_gibbs;

    // Error covariance matrices
    Matrix err_corr_position;
    Matrix err_corr_orientation;

    // Process noise matrices
    Matrix process_noise_position;
    Matrix process_noise_orientation;

    // Sensor noise matrices
    Matrix sensor_noise_position;
    Matrix sensor_noise_orientation;

    // Transition matrices
    Matrix F_position;
    Matrix B_position;
    Matrix F_orientation;
    Matrix B_orientation;

    Matrix G;
    Matrix Q;

    // Operations
    Vector quatToAngle(Vector q, float qw) const;
    Matrix crossMatrix(Vector inputvector) const;
    Vector shuster_multiply(Vector Y, Vector X) const;

    // Transition matrices for position and orientation definitions
    Matrix Get_F_position(float dt, const Matrix& rotation_matrix) const;
    Matrix Get_B_position(float dt) const;
    Matrix Get_F_orientation(float dt) const;
    Matrix Get_B_orientation(float dt) const;

    // Sensor transition matrices definitions
    Matrix get_GPS_transition() const;
    Matrix get_Barometer_transition() const;

    // Noise covariance matrices definitions
    Matrix Get_Noise_corr(const std::string &sensor) const;
   
public:
    // Constructor
    KalmanFilter(const Vector &init_pos_state, const Vector &init_orient_state);

    // Position and orientation prediction and update functions
    void predict_position(float dt, const Vector &a_meas, const Matrix &rotation_matrix);
    void update_position(const std::string &abs_sensor, const Vector &y);

    void predict_orientation(float dt, const Vector &w_meas);
    void update_orientation(const std::string &abs_sensor, const Vector &y, const Vector Noise);

    // Simulation functions
    Vector add_noise(const Vector &meas, const Matrix &noise_covariance) const;

    // Getters
    Vector get_position_state() const;
    Vector get_orientation_state() const;
    Vector get_orientation_state_euler() const;
    Matrix get_position_covariance() const;
};

#endif // KALMANFILTER_H
