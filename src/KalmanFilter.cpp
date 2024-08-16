#include "KalmanFilter.h"
#include <random>
#include <cmath>
#include <iostream>
#include <cassert>

KalmanFilter::KalmanFilter(const Vector &init_pos_state, const Vector &init_orient_state)
    : state_position(init_pos_state), state_orientation(init_orient_state),
      err_corr_position(Matrix::Identity(12, 12)),
      err_corr_orientation(Matrix::Identity(6, 6)),
      process_noise_position(Matrix::Identity(12, 12)),
      process_noise_orientation(Matrix::Identity(6, 6))
{
    F_position = Matrix::Identity(12, 12);
    F_orientation = Matrix::Identity(6, 6);
    B_position = Matrix::Zero(12, 3);
    B_orientation = Matrix::Zero(6, 3);
}

Vector KalmanFilter::quatToAngle(Vector q, float qw) const
{
    double q_y = (q(1) * qw) - (q(0) * q(2));
    float ang_y = 0;

    if (q_y > 0.5)
    {
        ang_y = M_PI / 2.0f + asinf((2 * q_y) - 1);
    }
    else if (q_y < -0.5)
    {
        ang_y = asinf(2 * q_y + 1) - M_PI / 2.0f;
    }
    else
    {
        ang_y = asinf(2 * q_y);
    }

    Vector3 euler = {atan2f(2.0f * (q(1) * q(2) + q(0) * qw), 1.0f - 2.0f * (pow(q(0), 2.0f) + pow(q(1), 2.0f))),
        ang_y,
        atan2f(2.0f * (q(0) * q(1) + q(2) * qw), 1.0f - 2.0f * (pow(q(1), 2.0f) + pow(q(2), 2.0f)))};

    return euler;
}


Matrix KalmanFilter::crossMatrix(Vector inputvector) const
{
    Matrix crossMatrix{ 
        {0, -inputvector(2), inputvector(1)},
        {inputvector(2), 0, -inputvector(0)},
        {-inputvector(1), inputvector(0), 0}
    };
    return crossMatrix;
}




Vector KalmanFilter::shuster_multiply(Vector Y, Vector X) const
{
    Vector4 output{ 
        0, 0, 0, 0 
    };

    output(0) = Y(3) * X(0) + X(3) * Y(0) - (Y(1) * X(2) - Y(2) * X(1));
    output(1) = Y(3) * X(1) + X(3) * Y(1) - (Y(2) * X(0) - Y(0) * X(2));
    output(2) = Y(3) * X(2) + X(3) * Y(2) - (Y(0) * X(1) - Y(1) * X(0));
    output(3) = Y(3) * X(3) - (Y(0) * X(0) + Y(1) * X(1) + Y(2) * X(2));

    return output;
}



Matrix KalmanFilter::Get_F_position(float dt, const Matrix &rotation_matrix) const
{
    Matrix temp_F_position = Matrix::Identity(12, 12);
    temp_F_position.block<3, 3>(0, 3) = dt * Matrix::Identity(3, 3);
    temp_F_position.block<3, 3>(0, 6) = 0.5f * dt * dt * Matrix::Identity(3, 3);
    temp_F_position.block<3, 3>(3, 9) = rotation_matrix * -Matrix::Identity(3, 3);
    temp_F_position.block<3, 3>(3, 6) = 0.5f * Matrix::Identity(3, 3);
    return temp_F_position;
}

Matrix KalmanFilter::Get_B_position(float dt) const
{
    Matrix temp_B_position = Matrix::Zero(12, 3);
    temp_B_position.block<3, 3>(0, 0) = 0.25f * dt * dt * Matrix::Identity(3, 3);
    temp_B_position.block<3, 3>(3, 0) = 0.5f * dt * Matrix::Identity(3, 3);
    temp_B_position.block<3, 3>(6, 0) = Matrix::Identity(3, 3);
    return temp_B_position;
}

Matrix KalmanFilter::Get_F_orientation(float dt) const
{
    Matrix temp_F_orientation = Matrix::Identity(6, 6);
    temp_F_orientation.block<3, 3>(0, 3) = dt * Matrix::Identity(3, 3);
    return temp_F_orientation;
}

Matrix KalmanFilter::Get_B_orientation(float dt) const
{
    Matrix temp_B_orientation = Matrix::Zero(6, 3);
    temp_B_orientation.block<3, 3>(0, 0) = 0.5f * dt * dt * Matrix::Identity(3, 3);
    return temp_B_orientation;
}

Matrix KalmanFilter::get_GPS_transition() const
{
    Matrix temp_transition = Matrix::Zero(2, 12);
    temp_transition.block<2, 2>(0, 0) = Matrix::Identity(2, 2);
    return temp_transition;
}

Matrix KalmanFilter::get_Barometer_transition() const
{
    Matrix temp_transition = Matrix::Zero(1, 12);
    temp_transition(0, 2) = 1.0f;
    return temp_transition;
}

void KalmanFilter::predict_position(float dt, const Vector &meas_acceleration, const Matrix &rotation_matrix)
{
    F_position = Get_F_position(dt, rotation_matrix);
    B_position = Get_B_position(dt);

    state_position = F_position * state_position + B_position * meas_acceleration;
    err_corr_position = F_position * err_corr_position * F_position.transpose() + process_noise_position;
}



void KalmanFilter::update_position(const std::string &sensor, const Vector &y)
{
    Matrix sensor_transition;

    if (sensor == "GPS")
    {
        sensor_transition = get_GPS_transition();
        sensor_noise_position = 0.1f * Matrix::Identity(2, 2);
    }
    else if (sensor == "Barometer")
    {
        sensor_transition = get_Barometer_transition();
        sensor_noise_position = 0.1f * Matrix::Identity(1, 1);
    }
    else
    {
        throw std::invalid_argument("Unknown sensor type");
    }

    Vector Meas_residual = y - sensor_transition * state_position;
    Matrix Innovation_covariance = sensor_transition * err_corr_position * sensor_transition.transpose() + sensor_noise_position;
    Matrix Kalman_gain = err_corr_position * sensor_transition.transpose() * Innovation_covariance.inverse();

    std::cout << "Kalman gain: " << Kalman_gain << std::endl;

    state_position += Kalman_gain * Meas_residual;
    err_corr_position -= Kalman_gain * sensor_transition * err_corr_position;
}



void KalmanFilter::predict_orientation(float dt, const Vector &meas_angular_velocity)
{
    // Implementation needed
}

Vector KalmanFilter::add_noise(const Vector &meas, const Matrix &noise_covariance) const
{
    assert(noise_covariance.rows() == noise_covariance.cols() && noise_covariance.rows() == meas.size());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> dist(0.0f, 1.0f);

    Vector uncorrelated_noise(meas.size());
    for (int i = 0; i < meas.size(); ++i)
    {
        uncorrelated_noise(i) = dist(gen);
    }

    Matrix L = noise_covariance.llt().matrixL();
    Vector correlated_noise = L * uncorrelated_noise;

    return meas + correlated_noise;
}

Vector KalmanFilter::get_position_state() const
{
    return state_position;
}

Vector KalmanFilter::get_orientation_state() const
{
    return state_orientation;
}

Matrix KalmanFilter::get_position_covariance() const
{
    return err_corr_position;
}
