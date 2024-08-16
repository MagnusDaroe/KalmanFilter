//g++ -std=c++11 -IC:\MagnusProjekter\KalmanFilter\libs\eigen -IC:\MagnusProjekter\KalmanFilter\include PoseEstimation.cpp -o main.exe

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>  

//Include own header file
#include <MEKF.h>
#include <KF.h>

using Matrix = Eigen::MatrixXf; 
using Vector = Eigen::VectorXf;

class KalmanFilter
{
private:
    // State vector: [position (m), velocity (m/s), acceleration (m/s^2), bias]
    Vector state_position;
    // State vector: [orientation (Gibbs vector), bias]
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

    // Operations

    // Find euler angles (radians) from quaternion
    Vector quatToAngle(Vector q, float qw){
        double q_y = (q(1) * qw) - (q(0) * q(2));
        float ang_y = 0;

        if (q_y > 0.5){
            ang_y = PI/2.0 + asinf((2 * q_y)-1); 
        } else if (q_y < -0.5){
            ang_y = asinf(2 * q_y + 1) - PI/2.0;
        } else {
            ang_y = asinf(2 * q_y);
        }

        //  Vector of euler angels [yaw, pitch, roll]^T
        Vector euler{
        {atan2f(2.0 * (q(1) * q(2) + q(0) * qw), 1.0 - 2.0 * (pow(q(0), 2.0) + pow(q(1), 2.0)))},
        {ang_y},
        {atan2f(2.0 * (q(0) * q(1) + q(2) * qw), 1.0 - 2.0 * (pow(q(1), 2.0) + pow(q(2), 2.0)))}};

        return euler;
    }

    Matrix crossMatrix(Vector inputvector){
        Matrix crossMatrix{
            {0, -inputvector(2), inputvector(1)},
            {inputvector(2), 0, -inputvector(0)},
            {-inputvector(1), inputvector(0), 0},
        };
    return crossMatrix;
    }

    // Function to calculate the shuster multiplication of two quaternions
    Vector MEKF::shuster_multiply(Vector Y, Vector X)
    {
    Vector output{
        {0},
        {0},
        {0},
        {0},
    };

    output(0) = Y(3)*X(0) + X(3)*Y(0) - (Y(1)*X(2) - Y(2)*X(1));
    output(1) = Y(3)*X(1) + X(3)*Y(1) - (Y(2)*X(0) - Y(0)*X(2));
    output(2) = Y(3)*X(2) + X(3)*Y(2) - (Y(0)*X(1) - Y(1)*X(0));
    output(3) = Y(3)*X(3) - (Y(0)*X(0) + Y(1)*X(1) + Y(2)*X(2));

    return output;
    }

    // State transition matrices
    Matrix Get_F_position(float dt, const Matrix& rotation_matrix)
    {
        Matrix temp_F_position = Matrix::Identity(12, 12);
        temp_F_position.block<3,3>(0, 3) = dt * Matrix::Identity(3, 3);
        temp_F_position.block<3,3>(0, 6) = 0.5 * dt * dt * Matrix::Identity(3, 3);
        temp_F_position.block<3,3>(3, 9) = rotation_matrix * -Matrix::Identity(3, 3);
        temp_F_position.block<3,3>(3, 6) = 0.5 * Matrix::Identity(3, 3);
        return temp_F_position;
    }

    Matrix Get_B_position(float dt)
    {
        Matrix temp_B_position = Matrix::Zero(12, 3);
        temp_B_position.block<3,3>(0, 0) = 0.25 * dt * dt * Matrix::Identity(3, 3);
        temp_B_position.block<3,3>(3, 0) = 0.5 * dt * Matrix::Identity(3, 3);
        temp_B_position.block<3,3>(6, 0) = Matrix::Identity(3, 3);
        return temp_B_position;
    }

    Matrix Get_F_orientation(float dt)
    {
        Matrix temp_F_orientation = Matrix::Identity(6, 6);
        temp_F_orientation.block<3,3>(0, 3) = dt * Matrix::Identity(3, 3);
        return temp_F_orientation;
    }

    Matrix Get_B_orientation(float dt)
    {
        Matrix temp_B_orientation = Matrix::Zero(6, 3);
        temp_B_orientation.block<3,3>(0, 0) = 0.5 * dt * dt * Matrix::Identity(3, 3);
        return temp_B_orientation;
    }

    // Sensor transition matrices

    Matrix get_GPS_transition()
    {
        Matrix temp_transition = Matrix::Zero(2, 12);
        temp_transition.block<2,2>(0,0) = Matrix::Identity(2,2);
        return temp_transition;
    }

    Matrix get_Barometer_transition()
    {
        Matrix temp_transition = Matrix::Zero(1, 12);
        temp_transition(0, 2) = 1.0;
        return temp_transition;
    }



public:
    KalmanFilter(const Vector &init_pos_state, const Vector &init_orient_state)
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

    // Position estimation
    void predict_position(float dt, const Vector &meas_acceleration, const Matrix &rotation_matrix)
    {
        F_position = Get_F_position(dt, rotation_matrix);
        B_position = Get_B_position(dt);

        state_position = F_position * state_position + B_position * meas_acceleration;
        err_corr_position = F_position * err_corr_position * F_position.transpose() + process_noise_position;
    }

    void update_position(const std::string &sensor, const Vector &y)
    {
        Matrix sensor_transition;

        if (sensor == "GPS")
        {
            sensor_transition = get_GPS_transition();
            sensor_noise_position = 0.1 * Matrix::Identity(2, 2);
        }
        else if (sensor == "Barometer")
        {
            sensor_transition = get_Barometer_transition();
            sensor_noise_position = 0.1 * Matrix::Identity(1, 1);
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

    // Orientation estimation
    void predict_orientation(float dt, const Vector &meas_angular_velocity)
    {
       
    }
 

    Vector add_noise(const Vector &meas, const Matrix &noise_covariance)
    {
        assert(noise_covariance.rows() == noise_covariance.cols() && noise_covariance.rows() == meas.size());

        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<float> dist(0.0, 1.0);

        Vector uncorrelated_noise(meas.size());
        for (int i = 0; i < meas.size(); ++i)
        {
            uncorrelated_noise(i) = dist(gen);
        }

        Matrix L = noise_covariance.llt().matrixL();
        Vector correlated_noise = L * uncorrelated_noise;

        return meas + correlated_noise;
    }

    Vector get_position_state() const { return state_position; }
    Vector get_orientation_state() const { return state_orientation; }
    Matrix get_position_covariance() const { return err_corr_position; }
};

int main()
{
    KalmanFilter kf(Vector::Zero(12), Vector::Zero(6));

    Vector state_position = kf.get_position_state();
    std::cout << "Initial position state: " << state_position.transpose() << std::endl;

    Vector actual_acceleration(3);
    actual_acceleration << 0.1, 0.1, 0.1;
    Vector meas_acceleration(3);

    Matrix rotation_matrix = Matrix::Identity(3, 3);
    Vector gps = Vector::Zero(2);
    Vector baro = Vector::Zero(1);

    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

    // Sampling rate of sensors
    float gps_sample_rate = 1.0;
    float barometer_sample_rate = 0.1;

    double dt = 0.1;
    Matrix gps_noise_covariance = 0.1 * Matrix::Identity(2, 2);

    std::vector<double> actual_x, actual_y,actual_z, estimated_x, estimated_y, estimated_z;
    std::vector<double> actual_vx, actual_vy, actual_vz, estimated_vx, estimated_vy, estimated_vz;
    std::vector<double> bias_x, bias_y, bias_z;

    for (int i = 0; i < 100; ++i)
    {
        
        // Simulate motion
        velocity += actual_acceleration * dt;
        position += velocity * dt;

        // Save actual position and velocity for plotting
        actual_x.push_back(position(0));
        actual_y.push_back(position(1));
        actual_z.push_back(position(2));
        actual_vx.push_back(velocity(0));
        actual_vy.push_back(velocity(1));
        actual_vz.push_back(velocity(2));


        // Simulate acceleration measurement
        meas_acceleration = actual_acceleration;
        meas_acceleration = kf.add_noise(meas_acceleration, Matrix::Identity(3, 3));

        kf.predict_position(dt, meas_acceleration, rotation_matrix);

        if (i % static_cast<int>(1.0 / gps_sample_rate) == 0){
                // Simulate GPS measurement
                gps = position.head(2);
                gps = kf.add_noise(gps, gps_noise_covariance);

                kf.update_position("GPS", gps);
        }

        if (i % static_cast<int>(1.0 / barometer_sample_rate) == 0){
                Vector barometer_meas(1);
                barometer_meas << position(2);
                barometer_meas = kf.add_noise(barometer_meas, 0.1 * Matrix::Identity(1, 1));
                kf.update_position("Barometer", barometer_meas);
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

        std::cout << "Updated covariance: " << kf.get_position_covariance() << std::endl;

    }

        // save in file for plotting
        std::ofstream file("data.txt");
        for (int i = 0; i < actual_x.size(); ++i)
        {
        file << actual_x[i] << " " << actual_y[i] << " " << actual_z[i] << " " << estimated_x[i] << " " << estimated_y[i] << " " << estimated_z[i] << " " << actual_vx[i] << " " << actual_vy[i] << " " << actual_vz[i] << " " << estimated_vx[i] << " " << estimated_vy[i] << " " << estimated_vz[i] << " " << bias_x[i] << " " << bias_y[i] << " " << bias_z[i] << std::endl;
        }


    return 0;
}
