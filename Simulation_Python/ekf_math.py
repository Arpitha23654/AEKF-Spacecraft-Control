import numpy as np

def calculate_jacobian_F(gyro_vector, dt):
    """
    Calculates the discrete-time Jacobian Matrix (F) for quaternion kinematics.
    gyro_vector: [wx, wy, wz] in radians per second
    dt: time step in seconds
    """
    wx, wy, wz = gyro_vector
    
    # 1. This is the continuous-time Omega matrix (The Calculus derivative)
    # Notice the mathematical pattern of the negatives!
    Omega = np.array([
        [  0, -wx, -wy, -wz],
        [ wx,   0,  wz, -wy],
        [ wy, -wz,   0,  wx],
        [ wz,  wy, -wx,   0]
    ])
    
    # 2. We convert it to discrete-time for computer software
    # F = Identity_Matrix + (0.5 * Omega * dt)
    identity_matrix = np.eye(4)
    F = identity_matrix + 0.5 * Omega * dt
    
    return F

# --- Let's test it! ---
gyro_reading = [0.1, 0.05, -0.2] # Fake rad/sec reading
time_step = 0.1 # 10 Hz

F_matrix = calculate_jacobian_F(gyro_reading, time_step)
print("The Jacobian Matrix (F) for this millisecond:")
print(np.round(F_matrix, 4))
