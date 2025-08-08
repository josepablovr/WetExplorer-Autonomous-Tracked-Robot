import open3d as o3d
import numpy as np
import os



src_path = "/ros2_ws/src/wetexplorer/wetexplorer_vision_predator/wetexplorer_vision_predator/OverlapPredator/assets/model.ply"
tgt_path = "/ros2_ws/src/wetexplorer/wetexplorer_vision_predator/wetexplorer_vision_predator/OverlapPredator/assets/output.ply"



src_pcd = o3d.io.read_point_cloud(src_path)
tgt_pcd = o3d.io.read_point_cloud(tgt_path)
src_pcd = src_pcd.voxel_down_sample(0.0045)
tgt_pcd = tgt_pcd.voxel_down_sample(0.0045)




# Preprocess the point clouds
voxel_size = 0.0045  # Adjust based on your data scale

print("Downsampling point clouds...")

# Estimate normals (required for some algorithms)
print("Estimating normals...")
radius_normal = voxel_size * 2
tgt_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
src_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
threshold = 0.02
trans_init = np.asarray([[1,0, 0, 0.0],
                         [0, 1, 0, 0.0],
                         [0, 0, 1, 0.0], 
                         [0.0, 0.0, 0.0, 1.0]])

# Compute FPFH features for global registration
print("Computing FPFH features...")
radius_feature = voxel_size * 5  # (5 default)
scene_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    tgt_pcd,
    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
)
cad_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    src_pcd,
    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
)
reg_p2p = o3d.pipelines.registration.registration_icp(
    src_pcd, tgt_pcd, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
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
        src_pcd, tgt_pcd, cad_fpfh, scene_fpfh, True,
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
    src_pcd, tgt_pcd, distance_threshold_icp,
    result_global.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
)

print("ICP registration result:")
print(result_icp)
print(f"Fitness after ICP: {result_icp.fitness:.4f}")

# Apply the transformation to the original CAD point cloud
print("Applying transformation to the original CAD point cloud...")
src_pcd.transform(result_icp.transformation)

# Visualize the aligned point clouds with a smaller window
print("Visualizing the aligned point clouds with a smaller window...")

# Create a visualizer with a smaller window size
vis = o3d.visualization.Visualizer()
vis.create_window(width=800, height=600)  # Set window size to 800x600
vis.add_geometry(src_pcd.paint_uniform_color([1, 0, 0]))  # CAD model in red
vis.add_geometry(tgt_pcd.paint_uniform_color([0, 1, 0]))  # Scene point cloud in green

# Optional: Adjust the view
vis.get_render_option().background_color = np.array([0.05, 0.05, 0.05])  # Dark background
vis.get_render_option().point_size = 2.0  # Increase point size for better visibility

vis.run()
vis.destroy_window()
