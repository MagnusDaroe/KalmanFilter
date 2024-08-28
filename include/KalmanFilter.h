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
    Vector state_position = Vector::Zero(12);
    Vector state_orientation = Vector::Zero(6);
    Vector state_quaternion = Vector::Zero(4);
    Vector state_gibbs = Vector3::Zero();

    // Error covariance matrices
    Matrix err_corr_position;
    Matrix err_corr_orientation;

    // Process noise matrices
    Matrix process_noise_position;
    Matrix process_noise_orientation;

    // Sensor related matrices
    Matrix sensor_noise_position;
    Matrix sensor_noise_orientation;
    Matrix sensor_transition_position;
    Matrix sensor_transition_orientation;

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
    void Get_Sensor_matrices(const std::string &abs_sensor);
    Matrix get_GPS_transition() const;
    Matrix get_Barometer_transition() const;
   
    // Sensor information
    float gps_variance = 1.0f;
    float baro_gps_variance = 1.0f;

public:
    // Constructorw
    KalmanFilter(const Vector &init_pos_state, const Vector &init_orient_state);

    // Position and orientation prediction and update functions
    void predict_position(float dt, const Vector &a_meas, const Matrix &rotation_matrix);
    void update_position(const std::string &abs_sensor, const Vector &y);

    void predict_orientation(float dt, const Vector &w_meas);
    void update_orientation(const std::string &abs_sensor, const Vector &y, const Matrix Noise);

    //Convert angle to quaternion
    Vector angleToQuat(Vector euler) const;

    // Simulation functions
    Vector add_noise(const Vector &meas, const Matrix &noise_covariance) const;

    // Getters
    Vector get_position_state() const;
    Vector get_orientation_state() const;
    Vector get_orientation_state_euler() const;
    Matrix get_position_covariance() const;
};

#endif // KALMANFILTER_H
