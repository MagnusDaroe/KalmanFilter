#include "Eigen/Dense"
using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;
#define PI 3.14159265

class MEKF
{
private:
    // Initializiation of all variables used in the MEKF
    Matrix G{
        {-1, 0, 0, 0, 0, 0},
        {0, -1, 0, 0, 0, 0},
        {0, 0, -1, 0, 0, 0},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1},
    };
    Matrix i6{
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1},
    };
    Matrix P{
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
    };
    Matrix Q{
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
    };
    Matrix R{
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
    };
    Vector X_k{
        //[q, b]
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
    };
    Vector X_k4{
        //[q, b]
        {0},
        {0},
        {0},
        {1},
    };
    Vector w_kprev{
        //[q, b]
        {0},
        {0},
        {0},
    };
    Vector w_hat_k{
        //[q, b]
        {0},
        {0},
        {0},
    };

    Vector shuster_multiply(Vector Y, Vector X);

public:
    void predict_Orientation(float delta_T, double measX, double measY, double measZ);
    // predict next state

    void update_Orientation(Vector Y, Vector noise);
    // calculate the update step

    Vector get_estimate();
    // returns the estimate in euler angles

    void setup(Vector gyro_noise, Vector randwalk_noise)
    {
        Matrix noise_mat{
            {(float)randwalk_noise(0) * (float)randwalk_noise(0), 0, 0, 0, 0, 0},
            {0, (float)randwalk_noise(1) * (float)randwalk_noise(1), 0, 0, 0, 0},
            {0, 0, (float)randwalk_noise(2) * (float)randwalk_noise(2), 0, 0, 0},
            {0, 0, 0, (float)gyro_noise(0) * (float)gyro_noise(0), 0, 0},
            {0, 0, 0, 0, (float)gyro_noise(1) * (float)gyro_noise(1), 0},
            {0, 0, 0, 0, 0, (float)gyro_noise(2) * (float)gyro_noise(2)}};
        Q = Q + noise_mat;
    };
};
