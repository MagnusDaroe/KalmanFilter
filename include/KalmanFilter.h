#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <iostream>

#define PI 3.141592653589

using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;
using Vector3 = Eigen::Vector3f;
using Vector4 = Eigen::Vector4f;

class KalmanFilter
{
private:
    Vector state_position;
    Vector state_orientation;

    Matrix err_corr_position;
    Matrix err_corr_orientation;

    Matrix process_noise_position;
    Matrix process_noise_orientation;

    Matrix sensor_noise_position;
    Matrix sensor_noise_orientation;

    Matrix F_position;
    Matrix B_position;
    Matrix F_orientation;
    Matrix B_orientation;

    Vector quatToAngle(Vector q, float qw) const;
    Matrix crossMatrix(Vector inputvector) const;

    Matrix Get_F_position(float dt, const Matrix& rotation_matrix) const;
    Matrix Get_B_position(float dt) const;
    Matrix Get_F_orientation(float dt) const;
    Matrix Get_B_orientation(float dt) const;

    Matrix get_GPS_transition() const;
    Matrix get_Barometer_transition() const;

    Vector shuster_multiply(Vector Y, Vector X) const;

public:
    KalmanFilter(const Vector &init_pos_state, const Vector &init_orient_state);

    void predict_position(float dt, const Vector &meas_acceleration, const Matrix &rotation_matrix);
    void update_position(const std::string &sensor, const Vector &y);

    void predict_orientation(float dt, const Vector &meas_angular_velocity);
    void update_orientation(const std::string &abs_sensor, const Vector &y);

    Vector add_noise(const Vector &meas, const Matrix &noise_covariance) const;

    Vector quatToAngle(Vector q, float qw) const;

    Vector get_position_state() const;
    Vector get_orientation_state() const;
    Matrix get_position_covariance() const;
};

#endif // KALMANFILTER_H
