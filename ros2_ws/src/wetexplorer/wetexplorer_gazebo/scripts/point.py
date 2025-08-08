import torch
import open3d as o3d

target = "exc2"
# Step 1: Load the .ply file using Open3D
ply_file_path = target+".ply"
point_cloud = o3d.io.read_point_cloud(ply_file_path)

# Step 2: Convert point cloud to a numpy array and then to a PyTorch tensor
# Extract points from the point cloud, which should be an Nx3 array
points = torch.tensor(point_cloud.points).float()  # Convert to float tensor
print(points[0])
points *=10
print(points[0])
# Step 3: Save the tensor as a .pth file
pth_file_path = target+".pth"
torch.save(points, pth_file_path)

print(f"Point cloud saved to {pth_file_path}")
# Step 5: Convert the scaled tensor back to an Open3D point cloud
scaled_point_cloud = o3d.geometry.PointCloud()
scaled_point_cloud.points = o3d.utility.Vector3dVector(points.numpy())  # Convert tensor to numpy array for Open3D


