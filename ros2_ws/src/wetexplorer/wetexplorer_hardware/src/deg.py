from scipy.spatial.transform import Rotation as R
import numpy as np

# Define Euler angles in degrees (roll, pitch, yaw)
euler_deg = [0, 00, -0]

# Create rotation from Euler (XYZ convention: roll-pitch-yaw)
rotation = R.from_euler('xyz', euler_deg, degrees=True)

# Convert to quaternion (x, y, z, w)
q = rotation.as_quat()
print(f"x={q[0]}, y={q[1]}, z={q[2]}, w={q[3]}")
