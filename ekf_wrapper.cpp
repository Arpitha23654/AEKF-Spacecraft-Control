#include "ekf_wrapper.h"
#include "AEKF.h"

// Global instance of the C++ AEKF class
AdaptiveEKF filter;

extern "C" {

void EKF_Init(float dt) {
    filter.init(dt);
}

void EKF_Predict(float gx, float gy, float gz) {
    filter.predict(gx, gy, gz);
}

void EKF_Update(float ax, float ay, float az) {
    filter.update(ax, ay, az);
}

void EKF_GetQuaternions(float* q_out) {
    filter.get_quaternions(q_out);
}

}
