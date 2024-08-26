#include "KalmanFilter.h"
#include <random>
#include <cmath>
#include <iostream>
#include <cassert>

// Constructor
KalmanFilter::KalmanFilter(const Vector &init_pos_state, const Vector &init_orient_state)
    : state_position(init_pos_state), state_orientation(init_orient_state),
      err_corr_position(Matrix::Identity(12, 12)),
      err_corr_orientation(Matrix::Identity(6, 6)),
      process_noise_position(Matrix::Identity(12, 12) * 0.1f),
      process_noise_orientation(Matrix::Identity(6, 6))
{
    F_position = Matrix::Identity(12, 12);
    F_orientation = Matrix::Identity(6, 6);
    B_position = Matrix::Zero(12, 3);
    B_orientation = Matrix::Zero(6, 3);

    G = Matrix{
        {-1,0,0,0,0,0},
        {0,-1,0,0,0,0},
        {0,0,-1,0,0,0},
        {0,0,0,1,0,0},
        {0,0,0,0,1,0},
        {0,0,0,0,0,1},
    };

    // Process noise for orientation
    Q = Matrix::Identity(6, 6)*0.1f;


}

// Operations
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

 Vector KalmanFilter::angleToQuat(Vector euler) const{
    // Convert euler angles to quaternion. The quat is in the form [x,y,z,w]
    float cy = cos(euler(2) * 0.5);
    float sy = sin(euler(2) * 0.5);
    float cp = cos(euler(1) * 0.5);
    float sp = sin(euler(1) * 0.5);
    float cr = cos(euler(0) * 0.5);
    float sr = sin(euler(0) * 0.5);

    Vector4 q{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };

    return q;
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


// Position

    // Private functions
Matrix KalmanFilter::Get_F_position(float dt, const Matrix &rotation_matrix) const
{
    Matrix temp_F_position = Matrix::Identity(12, 12);
    temp_F_position.block<3, 3>(0, 3) = dt * Matrix::Identity(3, 3);
    temp_F_position.block<3, 3>(0, 6) = 0.5f * dt * dt * Matrix::Identity(3, 3);
    temp_F_position.block<3, 3>(0, 9) = rotation_matrix * -Matrix::Identity(3, 3) * dt;
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

void KalmanFilter::Get_Sensor_matrices(const std::string &abs_sensor){
     if (abs_sensor == "GPS")
    {
        sensor_transition_position = get_GPS_transition();
        sensor_noise_position = gps_variance * Matrix::Identity(2, 2);
    }
    else if (abs_sensor == "Barometer")
    {
        sensor_transition_position = get_Barometer_transition();
        sensor_noise_position = baro_gps_variance * Matrix::Identity(1, 1);
    }
    else
    {
        throw std::invalid_argument("Unknown sensor type");
    }

}

    // Public functions
void KalmanFilter::predict_position(float dt, const Vector &a_meas, const Matrix &rotation_matrix)
{
    // Get the state transition matrices for the state vector and the acceleration vector
    F_position = Get_F_position(dt, rotation_matrix);
    B_position = Get_B_position(dt);

    // Predicts the current state onto the next step using the state transition matrix for the state vector. Note, half of the predicted acceleration is used and half the measurment
    state_position = F_position * state_position + B_position * a_meas;
    err_corr_position = F_position * err_corr_position * F_position.transpose() + process_noise_position;
}

void KalmanFilter::update_position(const std::string &abs_sensor, const Vector &y)
{
    // Get sensor matrices for update
    Get_Sensor_matrices(abs_sensor);

    // Calculates the residual, the innovation covariance and the Kalman gain
    Vector Meas_residual = y - sensor_transition_position * state_position;
    Matrix Innovation_covariance = sensor_transition_position * err_corr_position * sensor_transition_position.transpose() + sensor_noise_position;
    Matrix Kalman_gain = err_corr_position * sensor_transition_position.transpose() * Innovation_covariance.inverse();

    // Updates the state vector and the error state covariance matrix
    state_position += Kalman_gain * Meas_residual;
    err_corr_position -= Kalman_gain * sensor_transition_position * err_corr_position;
}


// Orientation

    // Private functions
Matrix KalmanFilter::Get_B_orientation(float dt) const
{
    Matrix temp_B_orientation = Matrix::Zero(6, 3);
    temp_B_orientation.block<3, 3>(0, 0) = 0.5f * dt * dt * Matrix::Identity(3, 3);
    return temp_B_orientation;
}

    // Public functions
void KalmanFilter::predict_orientation(float dt, const Vector &w_meas)
{
    // Store predicted bias of the angular velocity
    Vector3 bias_w{{state_orientation(3)}, {state_orientation(4)}, {state_orientation(5)}};

    // Finds the best estimate angular velocity, which is the measured angular velocity minus the estimated bias
    Vector3 w_est = w_meas - bias_w;

    // Finds the cross matrix for the estimated angular velocity
    Matrix W_cross = -crossMatrix(w_est);

    // Finds the L2 norm of the angular velocity vector, which is used to solve the taylor series around (t+deltat)
    float w_meas_lenght = w_meas.norm();

    // Factor inside cosine and sines from the taylor series solution
    float scalar = (w_meas_lenght*dt)/2;
    float scalar_sine = (float) sin(scalar);
    Vector3 w_scalar = (w_est/w_meas_lenght) * scalar_sine;

    // Finds the prediction matrix for the state quaternion
    Matrix F_orientation_quat{
        {(float) cos(scalar),w_scalar(2),-w_scalar(1),w_scalar(0)},
        {-w_scalar(2),(float) cos(scalar),w_scalar(0),w_scalar(1)},
        {w_scalar(1),-w_scalar(0),(float) cos(scalar),w_scalar(2)},
        {-w_scalar(0),-w_scalar(1),-w_scalar(2),(float) cos(scalar)},
    };
   
    // Finds the prediction matrix for the error state covariance
     Matrix F_err_corr{
        {W_cross(0,0),W_cross(0,1),W_cross(0,2),-1,0,0},
        {W_cross(1,0),W_cross(1,1),W_cross(1,2),0,-1,0},
        {W_cross(2,0),W_cross(2,1),W_cross(2,2),0,0,-1},
        {0,0,0,0,0,0},
        {0,0,0,0,0,0},
        {0,0,0,0,0,0},
    };


    // Predicts the current state onto the next step using the prediction matrix for the state quaternion
    state_quaternion = F_orientation_quat * state_quaternion;

    // Finds the derivative of the state covariance, which is used to approximate the predicted covariance
    Matrix err_corr_orientation_dot = dt * (F_err_corr * err_corr_orientation * F_err_corr.transpose() + G * Q * G.transpose());
    err_corr_orientation = err_corr_orientation + err_corr_orientation_dot;
}

void KalmanFilter::update_orientation(const std::string &abs_sensor, const Vector &y, const Matrix Sensor_Noise_corr)
{
    // Picks out the part of the covariance matrix which is with respect to the error parameters "gibbs" ie: [P_gibbs]
    Matrix P_gibbs = err_corr_orientation.block<3, 3>(0, 0);
   
    // Picks outthe first three rows, describing the covariance between "a" and "b" ie: [Pa Pc]
    Matrix PAC = err_corr_orientation.block<3, 6>(0, 0);
    
    // Picks out the part of the error state which coinsides with the gibs parameters
    state_gibbs = state_orientation.segment(0, 3);
    
    // Calculates the inverse of the predicted quaternion
    Vector4 state_quaternion_inv{
        {-state_quaternion(0)},
        {-state_quaternion(1)},
        {-state_quaternion(2)},
        {state_quaternion(3)},
    };


    // Finds the error quaternion describing the difference between the measured and predicted value
    Vector4 error_quat = shuster_multiply(y, state_quaternion_inv);
 
    // Maps the error quaternion to gibbs parameters
    Vector3 error_gibbs{
        {(float) 2.0*error_quat(0)/error_quat(3)},
        {(float) 2.0*error_quat(1)/error_quat(3)},
        {(float) 2.0*error_quat(2)/error_quat(3)},
    };
    
    // Picks out the first three columns of the covariance matrix ie: [Pa Pc^(T)]^(T)
    // Note that PAC is NOT is equal Pact due to numerical approximations, so the covariance matrix is NOT symetric
    Matrix Pact = err_corr_orientation.block<6, 3>(0, 0);

    // Calculates the Kalman gain: PAC^(T)*[PA + R]^(-1)
    Matrix Kalman_gain = Pact*(P_gibbs + Sensor_Noise_corr).inverse();

    // Calculates the change for the error state vector and updates the error state vector
   
    state_orientation = state_orientation + Kalman_gain*error_gibbs;

    // Maps the the gibbs parameters of the error state vector to quaternion state
    Vector4 delta_q{
        {state_orientation(0)},
        {state_orientation(1)},
        {state_orientation(2)},
        {2},    
    };

    // Calculates the updated state quaternion
    state_quaternion = shuster_multiply(delta_q, state_quaternion);

    // Normalises the updated state quaternion and updates the state quaternion
    state_quaternion = state_quaternion/state_quaternion.norm();

    // Resets the gibbs parameters in the error state
    state_orientation(0) = 0;
    state_orientation(1) = 0;
    state_orientation(2) = 0;

    // Updates the error state covariance matrix
    err_corr_orientation = err_corr_orientation - Kalman_gain * PAC;
}

// Sensor transitions - Accelerometer, Gyroscope, Magnetometer, GPS, Barometer
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



// Simulation
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


// Getters
Vector KalmanFilter::get_position_state() const
{
    return state_position;
}

Vector KalmanFilter::get_orientation_state() const
{
    return state_orientation;
}

Vector KalmanFilter::get_orientation_state_euler() const{
    
    //Extract imaginary parts of quaternion
    Vector q = state_quaternion.head(3);
    // Extract real part of quaternion
    float qw = state_quaternion(3);

    Vector3 euler = quatToAngle(q, qw) * (180/PI);

    return euler;
}

Matrix KalmanFilter::get_position_covariance() const
{
    return err_corr_position;
}
