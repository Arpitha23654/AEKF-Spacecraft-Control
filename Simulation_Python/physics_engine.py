import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. Initial State
# The spacecraft starts perfectly flat (Roll=0, Pitch=0, Yaw=0)
current_rotation = R.from_euler('xyz', [0, 0, 0], degrees=True)

# 2. Define the Gyroscope Reading (Angular Velocity)
# Spinning at 10 degrees per second on the Z-axis (Yaw)
omega_deg_per_sec = np.array([0, 0, 10]) 
# Math is always done in Radians!
omega_rad_per_sec = np.radians(omega_deg_per_sec) 

# 3. Time variables
dt = 0.1  # We calculate the math every 0.1 seconds (10 Hz loop)
total_time = 9.0  # Run the simulation for 9 seconds

print(f"Starting Quaternion: {current_rotation.as_quat()}")

# 4. The Physics Integration Loop
time = 0.0
while time < total_time:
    # Calculate the small rotation that occurred during 'dt'
    delta_rotation = R.from_rotvec(omega_rad_per_sec * dt)
    
    # Append the small rotation to the current rotation
    current_rotation = delta_rotation * current_rotation
    
    time += dt

# 5. Result
final_euler = current_rotation.as_euler('xyz', degrees=True)
print(f"Final Quaternion: {current_rotation.as_quat()}")
print(f"Final Euler Angles: {final_euler}")
