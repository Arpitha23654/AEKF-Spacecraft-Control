import numpy as np
import matplotlib.pyplot as plt

# 1. Setup Time Array for 10 seconds (100 samples at 10Hz)
time = np.linspace(0, 10, 100)

# 2. The TRUE Physics (What is actually happening in space)
# Let's say the satellite is spinning at exactly 5 rad/sec on the Z-axis
true_gyro_z = np.full(100, 5.0) 

# 3. The DIRTY Sensor Data (What the hardware actually reads)
# A. Hardware Noise (random fluctuations)
# We use a standard deviation of 0.8 to simulate a cheap sensor
hardware_noise = np.random.normal(loc=0.0, scale=0.8, size=100)

# B. Thermal Bias Drift
# The sensor has a constant error of +1.5 due to getting hot
thermal_bias = 1.5 

# Add them together to get the final fake reading
noisy_gyro_z = true_gyro_z + hardware_noise + thermal_bias

# 4. Plot the difference
plt.figure(figsize=(10, 5))
plt.plot(time, true_gyro_z, label='True Physics (Clean)', color='green', linewidth=3)
plt.plot(time, noisy_gyro_z, label='Dirty Sensor Reading', color='red', linestyle='dashed')
plt.title("Why We Need The Extended Kalman Filter")
plt.xlabel("Time (Seconds)")
plt.ylabel("Angular Velocity (Rad/Sec)")
plt.legend()
plt.grid(True)
plt.show()
