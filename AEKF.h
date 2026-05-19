#ifndef AEKF_H
#define AEKF_H

#include <Eigen/Dense>

class AdaptiveEKF {
public:
    AdaptiveEKF();

    // Initialize the filter
    void init(float dt);

    // Predict step using Gyroscope data (gx, gy, gz in rad/s)
    void predict(float gx, float gy, float gz);

    // Update step using Accelerometer data (ax, ay, az)
    void update(float ax, float ay, float az);

    // Get current quaternions
    void get_quaternions(float* q_out);

private:
    float dt_; // Time step

    // State vector: [qw, qx, qy, qz]^T
    Eigen::Vector4f x_;

    // State covariance matrix (4x4)
    Eigen::Matrix4f P_;

    // Process noise covariance (4x4)
    Eigen::Matrix4f Q_;

    // Measurement noise covariance (3x3)
    Eigen::Matrix3f R_;

    // Gravity reference vector
    Eigen::Vector3f g_ref_;
};

#endif // AEKF_H
