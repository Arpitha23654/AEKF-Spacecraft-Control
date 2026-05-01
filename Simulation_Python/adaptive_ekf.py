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

x = np.array([1.0, 0.0, 0.0, 0.0]) 
P = np.eye(4) * 1.0 
Q = np.eye(4) * 0.001 
R = np.eye(4) * 0.5 
H = np.eye(4)

true_yaw = []
ekf_yaw = []
innovation_history = [] # <--- Added this

# --- 2. THE EKF LOOP ---
for i in range(time_steps):
    
    true_omega = np.array([0, 0, np.radians(10)])
    noisy_omega = true_omega + np.random.normal(0, 0.05, 3)
    
    F = calculate_jacobian_F(noisy_omega, dt)
    
    # Eq 1 & 2. PREDICT
    x_predict = F @ x 
    P_predict = F @ P @ F.T + Q
    
    # --- STAGE 2: UPDATE ---
    # Normal space environment
    if i < 100 or i > 150:
        fake_sensor_quaternion = x_predict + np.random.normal(0, 0.1, 4)
    # SOLAR FLARE!
    else:
        fake_sensor_quaternion = x_predict + np.random.normal(0, 5.0, 4)
        
    # Calculate the Innovation (Notice the 4 spaces of indentation!)
    innovation_vector = fake_sensor_quaternion - (H @ x_predict)
    innovation_magnitude = np.linalg.norm(innovation_vector)
    innovation_history.append(innovation_magnitude)
     # --- THE ADAPTIVE UPGRADE (AEKF) ---
    # If the innovation spikes, dynamically increase the R matrix to stop trusting the sensor!
    if innovation_magnitude > 1.0:
        R_dynamic = np.eye(4) * (0.5 + (innovation_magnitude * 10000))
    else:
        R_dynamic = np.eye(4) * 0.5

    # Eq 3, 4, 5. UPDATE
    S = H @ P_predict @ H.T + R_dynamic 
    K = P_predict @ H.T @ np.linalg.inv(S)
    
    x = x_predict + K @ innovation_vector # (Using the innovation_vector we just made!)
    x = x / np.linalg.norm(x)
    
    P = (np.eye(4) - K @ H) @ P_predict
    
    # --- Logging ---
    rot_tk = Rotation.from_quat([x[1], x[2], x[3], x[0]]) 
    ekf_yaw.append(rot_tk.as_euler('xyz', degrees=True)[2])
    true_yaw.append( (i * 10 * dt) % 360 ) 

# --- 3. GRAPH IT ---
plt.figure(figsize=(10,5))
plt.plot(true_yaw, label="True Satellite Yaw", color="green", linewidth=3)
plt.plot(ekf_yaw, label="EKF Output", color="blue", linestyle="dashed")
plt.title("Standard EKF Failure During Solar Flare")
plt.legend()

# --- 4. NEW INNOVATION GRAPH ---
plt.figure(figsize=(10,3))
plt.plot(innovation_history, color="red")
plt.title("Innovation Magnitude (The 'Danger' Signal)")
plt.xlabel("Time Step (0.1s)")
plt.ylabel("Innovation Size")
plt.grid(True)

plt.show()
