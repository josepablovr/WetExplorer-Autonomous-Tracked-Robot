import open3d as o3d
import numpy as np
import os

# Paths to your files
scene_pcd_path = r"real1.ply"  # Update with your actual file name and extension
cad_mesh_path = r"simv3.ply"

# Load the real scene point cloud
scene_pcd = o3d.io.read_point_cloud(scene_pcd_path)
cad_pcd = o3d.io.read_point_cloud(cad_mesh_path)
print(f"Loaded scene point cloud with {np.asarray(scene_pcd.points).shape[0]} points.")

# Load the CAD model (as a mesh)
cad_mesh = o3d.io.read_triangle_mesh(cad_mesh_path)
cad_mesh.compute_vertex_normals()

# **Scale the CAD mesh from millimeters to meters**
scale_factor = 1  # 1 mm = 0.001 meters
cad_mesh.scale(scale_factor, center=cad_mesh.get_center())

# Convert the CAD mesh to a point cloud by sampling points on the surface
##cad_pcd = cad_mesh.sample_points_uniformly(number_of_points=6000)  # KEY FACTOR

# Preprocess the point clouds
voxel_size = 0.0045  # Adjust based on your data scale

print("Downsampling point clouds...")
scene_pcd_down = scene_pcd.voxel_down_sample(voxel_size)
cad_pcd_down = cad_pcd.voxel_down_sample(voxel_size)

# Estimate normals (required for some algorithms)
print("Estimating normals...")
radius_normal = voxel_size * 2
scene_pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
cad_pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

# Compute FPFH features for global registration
print("Computing FPFH features...")
radius_feature = voxel_size * 5  # (5 default)
scene_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    scene_pcd_down,
    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
)
cad_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    cad_pcd_down,
    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
)

# Parameters for the loop
distance_threshold = voxel_size * 1.5
max_attempts = 10  # Limite massimo di tentativi per evitare loop infiniti
attempt = 0
desired_fitness = 0.23
result_global = None
best_result = None
best_fitness = -1.0  # Inizializza con un valore minimo

print("Starting global registration loop...")
while attempt < max_attempts:
    print(f"Attempt {attempt + 1} of {max_attempts}")
    current_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        cad_pcd_down, scene_pcd_down, cad_fpfh, scene_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(8000000, 0.999)
    )
    
    print(f"Global registration result for attempt {attempt + 1}:")
    print(current_result)
    print(f"Fitness: {current_result.fitness:.4f}")
    
    # Aggiorna il miglior risultato se il fitness attuale è superiore
    if current_result.fitness > best_fitness:
        best_fitness = current_result.fitness
        best_result = current_result
    
    if current_result.fitness >= desired_fitness:
        print(f"Desired fitness {desired_fitness} achieved.")
        result_global = current_result
        break
    else:
        print(f"Desired fitness {desired_fitness} not achieved. Retrying...")
        attempt += 1

if result_global is None:
    if best_result is not None:
        print(f"Desired fitness of {desired_fitness} not achieved after {max_attempts} attempts.")
        print(f"Using the best obtained fitness of {best_fitness:.4f} for ICP.")
        result_global = best_result
    else:
        print(f"No valid registration result obtained after {max_attempts} attempts.")
        exit(1)  # Termina lo script con un errore
else:
    print(f"Using the registration result with fitness {result_global.fitness:.4f}.")

# Refine registration using ICP
print("Refining registration using ICP...")
distance_threshold_icp = voxel_size * 0.5
result_icp = o3d.pipelines.registration.registration_icp(
    cad_pcd_down, scene_pcd_down, distance_threshold_icp,
    result_global.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
)

print("ICP registration result:")
print(result_icp)
print(f"Fitness after ICP: {result_icp.fitness:.4f}")

# Apply the transformation to the original CAD point cloud
print("Applying transformation to the original CAD point cloud...")
cad_pcd.transform(result_icp.transformation)

# Visualize the aligned point clouds with a smaller window
print("Visualizing the aligned point clouds with a smaller window...")

# Create a visualizer with a smaller window size
vis = o3d.visualization.Visualizer()
vis.create_window(width=800, height=600)  # Set window size to 800x600
vis.add_geometry(cad_pcd.paint_uniform_color([1, 0, 0]))  # CAD model in red
vis.add_geometry(scene_pcd.paint_uniform_color([0, 1, 0]))  # Scene point cloud in green

# Optional: Adjust the view
vis.get_render_option().background_color = np.array([0.05, 0.05, 0.05])  # Dark background
vis.get_render_option().point_size = 2.0  # Increase point size for better visibility

vis.run()
vis.destroy_window()

# Optionally, merge the point clouds
print("Merging point clouds...")
merged_pcd = scene_pcd + cad_pcd
merged_pcd = merged_pcd.voxel_down_sample(voxel_size=voxel_size)

# Save the merged point cloud
merged_pcd_path = r"/home/au-robotics/MircoProjects/PointCloudReg/lab2/merged.ply"
o3d.io.write_point_cloud(merged_pcd_path, merged_pcd)
print(f"Merged point cloud saved to {merged_pcd_path}")