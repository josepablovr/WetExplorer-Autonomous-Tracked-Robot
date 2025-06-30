import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import pandas as pd

# Step 1: Define the 4x4 transformation matrices
poses_matrix = [
    np.array([[ 0.98490061,  0.17265559,  0.01268251,  1.69267927],
              [ 0.17204112, -0.96795973, -0.18290933, -0.75572741],
              [-0.01930416,  0.18232942, -0.98304798,  5.25695786],
              [ 0.        ,  0.        ,  0.        ,  1.        ]]),

    np.array([[ 6.68293282e-01,  7.43897298e-01,  9.48476147e-04,  1.72002604e+00],
              [ 7.33379047e-01, -6.58629298e-01, -1.68412057e-01, -7.82602163e-01],
              [-1.24656580e-01,  1.13244239e-01, -9.85716227e-01,  5.24795319e+00],
              [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]),

    np.array([[ 0.87052978, -0.49150169,  0.02457625,  1.64473936],
              [-0.48396866, -0.86409924, -0.13822749, -0.80604939],
              [ 0.08917536,  0.10843702, -0.99009553,  5.26492004],
              [ 0.        ,  0.        ,  0.        ,  1.        ]]),

    np.array([[ 0.94745142, -0.31946046, -0.01675778,  1.75733754],
              [-0.31765995, -0.93333828, -0.16724772, -0.78372793],
              [ 0.03778835,  0.16378237, -0.98577248,  5.25553234],
              [ 0.        ,  0.        ,  0.        ,  1.        ]]),

    np.array([[-3.86164962e-02, -9.99253074e-01,  1.43516970e-03,  1.74376146e+00],
              [-9.84848933e-01,  3.78167720e-02, -1.69240867e-01, -7.78534802e-01],
              [ 1.69060183e-01, -7.94891464e-03, -9.85573675e-01,  5.24192991e+00],
              [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])]


# Step 2: Extract full 6D poses [x, y, z, roll, pitch, yaw]
poses_6d = []
for mat in poses_matrix:
    t = mat[:3, 3]
    rpy = R.from_matrix(mat[:3, :3]).as_euler('xyz', degrees=False)
    poses_6d.append(np.concatenate([t, rpy]))
poses_6d = np.array(poses_6d)

# Step 3: Define your thresholds
max_translation_tol = 0.05  # meters
max_orientation_tol_deg = 5.0  # degrees
max_orientation_tol_rad = np.deg2rad(max_orientation_tol_deg)

# Step 4: Compute orientation scale
orientation_scale = max_translation_tol / max_orientation_tol_rad

# Step 5: Choose whether to include yaw
include_yaw = False  # set to False to ignore yaw

# Step 6: Prepare feature vectors for DBSCAN
if include_yaw:
    features = poses_6d.copy()
    features[:, 3:6] *= orientation_scale  # scale roll, pitch, yaw
else:
    features = np.hstack([
        poses_6d[:, :3],         # x, y, z
        poses_6d[:, 3:5] * orientation_scale  # roll, pitch
    ])

# Step 7: Compute DBSCAN eps
eps = np.sqrt(
    max_translation_tol**2 +
    (orientation_scale * max_orientation_tol_rad)**2
)

# Step 8: Run DBSCAN
db = DBSCAN(eps=eps, min_samples=2).fit(features)
labels = db.labels_

# Step 9: Create a DataFrame for results
col_names = ["X", "Y", "Z", "Roll (rad)", "Pitch (rad)", "Yaw (rad)"]
df = pd.DataFrame(poses_6d, columns=col_names)
df["Cluster Label"] = labels

# Step 10: Plot translation points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
colors = plt.cm.get_cmap("tab10", len(set(labels)))
for idx, label in enumerate(labels):
    color = "k" if label == -1 else colors(label)
    ax.scatter(poses_6d[idx, 0], poses_6d[idx, 1], poses_6d[idx, 2], color=color, s=80)
ax.set_title("Translation Points with Cluster Labels")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
plt.tight_layout()

# Step 11: Print DataFrame
print(df)

# Step 12: Print mean and std for each cluster
for label in sorted(set(labels)):
    if label == -1:
        continue
    cluster_data = df[df["Cluster Label"] == label]
    mean_vals = cluster_data.mean(numeric_only=True)
    std_vals = cluster_data.std(numeric_only=True)
    print(f"\n📦 Cluster {label}")
    print("Mean:")
    print(mean_vals)
    print("Std Dev:")
    print(std_vals)

plt.show()
