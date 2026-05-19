import serial
from vpython import *

# === 1. SETUP COM PORT ===
COM_PORT = 'COM11'  # Keep it as COM11!
BAUD_RATE = 115200

# === 2. BUILD THE 3D SATELLITE ===
scene.title = "AEKF Real-Time Satellite Visualization"
scene.width = 1000
scene.height = 700
scene.background = color.black
scene.range = 6  # This automatically zooms the camera out!

body = box(length=4, width=2, height=1, color=color.cyan)
panel1 = box(pos=vector(0, 0, 1.5), length=4, width=0.1, height=2, color=color.blue)
panel2 = box(pos=vector(0, 0, -1.5), length=4, width=0.1, height=2, color=color.blue)
sat_model = compound([body, panel1, panel2])

# === 3. CONNECT TO STM32 ===
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"✅ Connected to STM32 on {COM_PORT}!")
except Exception as e:
    print(f"❌ Failed to connect! Is PuTTY closed?")
    exit()

# === 4. REAL-TIME MATH LOOP ===
while True:
    rate(50)  # Required to make the graphics update
    try:
        line = ser.readline().decode('utf-8').strip()
        
        # We check if we got a valid line, and PRINT it so we can see it!
        if line.startswith("Q: ["):
            print(line) 
            
            line = line.replace("Q: [", "").replace("]", "")
            parts = line.split(",")
            
            if len(parts) == 4:
                qw = float(parts[0])
                qx = float(parts[1])
                qy = float(parts[2])
                qz = float(parts[3])
                
                # Convert Quaternions to 3D Space
                f_x = 1.0 - 2.0*qy**2 - 2.0*qz**2
                f_y = 2.0*qx*qy - 2.0*qz*qw
                f_z = 2.0*qx*qz + 2.0*qy*qw
                
                u_x = 2.0*qx*qy + 2.0*qz*qw
                u_y = 1.0 - 2.0*qx**2 - 2.0*qz**2
                u_z = 2.0*qy*qz - 2.0*qx*qw
                
                # Apply Rotation
                sat_model.axis = vector(f_x, f_y, f_z)
                sat_model.up = vector(u_x, u_y, u_z)
    except Exception as e:
        pass  
