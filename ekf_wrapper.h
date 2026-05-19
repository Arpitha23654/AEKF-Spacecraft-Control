#ifndef EKF_WRAPPER_H
#define EKF_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the filter with a given time step (dt)
void EKF_Init(float dt);

// Feed Gyroscope data (rad/s)
void EKF_Predict(float gx, float gy, float gz);

// Feed Accelerometer data (raw or m/s^2)
void EKF_Update(float ax, float ay, float az);

// Get the current calculated quaternions
void EKF_GetQuaternions(float* q_out);

#ifdef __cplusplus
}
#endif

#endif // EKF_WRAPPER_H
