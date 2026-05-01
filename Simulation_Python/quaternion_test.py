import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. Define a rotation in standard Euler Angles (Roll, Pitch, Yaw) in degrees
# Let's say our satellite rolled 45 degrees, pitched 30 degrees, and yawed 0 degrees.
euler_angles = [45, 30, 0] 

# 2. Convert these Euler Angles to a Quaternion using SciPy
r = R.from_euler('xyz', euler_angles, degrees=True)
quaternion = r.as_quat()

print("Euler Angles (Roll, Pitch, Yaw):", euler_angles)
print("Spacecraft Quaternion [qx, qy, qz, qw]:", quaternion)

# 3. Prove it works! Let's rotate a physical vector.
# Imagine a solar panel pointing straight out on the X-axis [1, 0, 0]
solar_panel_vector = [1, 0, 0]

# Where is the solar panel pointing AFTER the satellite rotates?

print("Original Solar Panel Vector:", solar_panel_vector)
print("New Solar Panel Vector after rotation:", new_pointing_vector)
