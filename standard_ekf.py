import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def calculate_jacobian_F(gyro_vector, dt):
    wx, wy, wz = gyro_vector
    Omega = np.array([
        [  0, -wx, -wy, -wz],
        [ wx,   0,  wz, -wy],
        [ wy, -wz,   0,  wx],
        [ wz,  wy, -wx,   0]
    ])
    return np.eye(4) + 0.5 * Omega * dt

# --- 1. SETUP ---
time_steps = 200
dt = 0.1

# The initial state (Quaternion: qw, qx, qy, qz)
x = np.array([1.0, 0.0, 0.0, 0.0]) 

# P: Initial Uncertainty (We start very uncertain)
P = np.eye(4) * 1.0 

# Q: Physics Engine Noise (We trust our math mostly)
Q = np.eye(4) * 0.001 

# R: Hardware Sensor Noise (We know our sensor is cheap/noisy)
R = np.eye(4) * 0.5 

# H: Measurement Matrix (Direct 1-to-1 mapping)
H = np.eye(4)

# Data arrays for plotting
true_yaw = []
ekf_yaw = []

# --- 2. THE EKF LOOP ---
for i in range(time_steps):
    
    # Fake true physics (Constant Yaw spin at 10 deg/sec)
    true_omega = np.array([0, 0, np.radians(10)])
    
    # Fake noisy hardware gyro reading (Adding nasty static)
    noisy_omega = true_omega + np.random.normal(0, 0.05, 3)
    
    # ---------------------------------------------
    # STAGE 1: PREDICT (Blindfolded)
    # ---------------------------------------------
    F = calculate_jacobian_F(noisy_omega, dt)
    
    # Eq 1. Predict State
    x_predict = F @ x 
    
    # Eq 2. Predict Covariance
    P_predict = F @ P @ F.T + Q
    
    # ---------------------------------------------
    # STAGE 2: UPDATE (Opening eyes to the sensor)
    # ---------------------------------------------
    
    # Let's say our Star Tracker gives us a noisy absolute angle reading
    fake_sensor_quaternion = x_predict + np.random.normal(0, 0.1, 4)
    
    # Eq 3. Kalman Gain (The Trust 'Brain')
    # Note: np.linalg.inv() calculates the inverse of the matrix
    S = H @ P_predict @ H.T + R
    K = P_predict @ H.T @ np.linalg.inv(S)
    
    # Eq 4. State Update (Fuse prediction with sensor)
    x = x_predict + K @ (fake_sensor_quaternion - (H @ x_predict))
    
    # CRITICAL: Quaternions must equal 1! Normalize it.
    x = x / np.linalg.norm(x)
    
    # Eq 5. Covariance Update (Shrink the uncertainty)
    P = (np.eye(4) - K @ H) @ P_predict
    
    # --- Logging for the Graph ---
    # Convert quaternions back to Euler angles for graphing
    rot_tk = Rotation.from_quat([x[1], x[2], x[3], x[0]]) # SciPy takes x,y,z,w
    ekf_yaw.append(rot_tk.as_euler('xyz', degrees=True)[2])
    
    true_yaw.append( (i * 10 * dt) % 360 ) # The true math

# --- 3. GRAPH IT ---
plt.figure(figsize=(10,5))
plt.plot(true_yaw, label="True Satellite Yaw (Perfect)", color="green", linewidth=3)
plt.plot(ekf_yaw, label="EKF Output (Filtered)", color="blue", linestyle="dashed")
plt.title("Standard Extended Kalman Filter in Action")
plt.xlabel("Time Step (0.1s)")
plt.ylabel("Yaw Angle (Degrees)")
plt.legend()
plt.show()
