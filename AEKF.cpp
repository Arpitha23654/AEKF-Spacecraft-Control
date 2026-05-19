#include "AEKF.h"
#include <cmath>

using namespace Eigen;

AdaptiveEKF::AdaptiveEKF() {}

void AdaptiveEKF::init(float dt) {
    dt_ = dt;
    x_ << 1.0f, 0.0f, 0.0f, 0.0f; // Initial quaternion (no rotation)
    P_ = Matrix4f::Identity() * 0.1f;
    Q_ = Matrix4f::Identity() * 0.001f;
    R_ = Matrix3f::Identity() * 0.1f;
    g_ref_ << 0.0f, 0.0f, 1.0f; // Gravity points down Z in reference frame
}

void AdaptiveEKF::predict(float gx, float gy, float gz) {
    // Gyro integration (kinematics)
    Matrix4f Omega;
    Omega << 0.0f, -gx, -gy, -gz,
              gx,  0.0f,  gz, -gy,
              gy, -gz,  0.0f,  gx,
              gz,  gy, -gx,  0.0f;

    Matrix4f F = Matrix4f::Identity() + 0.5f * dt_ * Omega;

    // State prediction
    x_ = F * x_;
    x_.normalize(); // Ensure quaternion remains valid

    // Covariance prediction
    P_ = F * P_ * F.transpose() + Q_;
}

void AdaptiveEKF::update(float ax, float ay, float az) {
    // Normalize accelerometer reading
    Vector3f z_meas(ax, ay, az);
    if (z_meas.norm() > 0.001f) {
        z_meas.normalize();
    }

    // Predicted gravity from current quaternions
    float qw = x_(0), qx = x_(1), qy = x_(2), qz = x_(3);
    Vector3f z_pred;
    z_pred(0) = 2.0f * (qx * qz - qw * qy);
    z_pred(1) = 2.0f * (qw * qx + qy * qz);
    z_pred(2) = qw * qw - qx * qx - qy * qy + qz * qz;

    // Innovation
    Vector3f y = z_meas - z_pred;

    // Jacobian of Measurement Model (H)
    Matrix<float, 3, 4> H;
    H << -2.0f*qy,  2.0f*qz, -2.0f*qw,  2.0f*qx,
          2.0f*qx,  2.0f*qw,  2.0f*qz,  2.0f*qy,
          2.0f*qw, -2.0f*qx, -2.0f*qy,  2.0f*qz;

    // Innovation covariance
    Matrix3f S = H * P_ * H.transpose() + R_;

    // Kalman Gain
    Matrix<float, 4, 3> K = P_ * H.transpose() * S.inverse();

    // State update
    x_ = x_ + K * y;
    x_.normalize();

    // Covariance update
    P_ = (Matrix4f::Identity() - K * H) * P_;
}

void AdaptiveEKF::get_quaternions(float* q_out) {
    q_out[0] = x_(0);
    q_out[1] = x_(1);
    q_out[2] = x_(2);
    q_out[3] = x_(3);
}
