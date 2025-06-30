"""
Scripts for pairwise registration demo

Author: Shengyu Huang
Last modified: 22.02.2021
"""
import os, torch, time, shutil, json,glob,sys,copy, argparse
import numpy as np
#import rclpy
from easydict import EasyDict as edict
from torch.utils.data import Dataset
from torch import optim, nn
import open3d as o3d

#cwd = os.getcwd()
cwd = "/ros2_ws/src/wetexplorer/wetexplorer_vision_predator/wetexplorer_vision_predator/OverlapPredator"
sys.path.append(cwd)
from datasets.indoor import IndoorDataset
from datasets.dataloader import get_dataloader
from models.architectures import KPFCNN
from lib.utils import load_obj, setup_seed,natural_key, load_config
from lib.benchmark_utils import ransac_pose_estimation, to_o3d_pcd, get_blue, get_yellow, to_tensor
from lib.trainer import Trainer
from lib.loss import MetricLoss
import shutil
setup_seed(0)
from scipy.spatial.transform import Rotation as R

class ThreeDMatchDemo(Dataset):
    """
    Load subsampled coordinates, relative rotation and translation
    Output(torch.Tensor):
        src_pcd:        [N,3]
        tgt_pcd:        [M,3]
        rot:            [3,3]
        trans:          [3,1]
    """
    def __init__(self,config, src_path, tgt_path):
        super(ThreeDMatchDemo,self).__init__()
        self.config = config
        self.src_path = src_path
        self.tgt_path = tgt_path


        self.src_pcd = None       
        self.tgt_pcd = None
        self.src_feats = None
        self.tgt_feats = None
   
   

    def __len__(self):
        return 1


    def __getitem__(self,item): 
        # get pointcloud
   
        
        # src_pcd = torch.load(self.src_path).to(torch.float32)
        # src_pcd = src_pcd.numpy()        
        # tgt_pcd = torch.load(self.tgt_path).to(torch.float32)
        # tgt_pcd = tgt_pcd.numpy()   

        # print("SOURCE")
        
        #print(f"Source Number of Points: {len(src_pcd)}")      
        #print(f"Target Number of Points: {len(tgt_pcd)}")
        
        src_pcd = o3d.io.read_point_cloud(self.src_path)
        tgt_pcd = o3d.io.read_point_cloud(self.tgt_path)
        src_pcd = src_pcd.voxel_down_sample(0.0045)
        tgt_pcd = tgt_pcd.voxel_down_sample(0.0045)
        src_pcd = np.array(src_pcd.points).astype(np.float32)
        tgt_pcd = np.array(tgt_pcd.points).astype(np.float32)
        print(f"Source Number of Points: {len(src_pcd)}")      
        print(f"Target Number of Points: {len(tgt_pcd)}")

        src_feats=np.ones_like(src_pcd[:,:1]).astype(np.float32)
        tgt_feats=np.ones_like(tgt_pcd[:,:1]).astype(np.float32)

        # fake the ground truth information
        rot = np.eye(3).astype(np.float32)
        trans = np.ones((3,1)).astype(np.float32)
        correspondences = torch.ones(1,2).long()

        return src_pcd,tgt_pcd,src_feats,tgt_feats,rot,trans, correspondences, src_pcd, tgt_pcd, torch.ones(1)

def lighter(color, percent):
    '''assumes color is rgb between (0, 0, 0) and (1,1,1)'''
    color = np.array(color)
    white = np.array([1, 1, 1])
    vector = white-color
    return color + vector * percent


def draw_registration_result(src_raw, tgt_raw, src_overlap, tgt_overlap, src_saliency, tgt_saliency, tsfm):
    ########################################
    # 1. input point cloud
 
    src_pcd_before = to_o3d_pcd(src_raw)
    tgt_pcd_before = to_o3d_pcd(tgt_raw)
    src_pcd_before.paint_uniform_color(get_yellow())
    tgt_pcd_before.paint_uniform_color(get_blue())
    src_pcd_before.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))
    tgt_pcd_before.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))

    ########################################
    # 2. overlap colors
    rot, trans = to_tensor(tsfm[:3,:3]), to_tensor(tsfm[:3,3][:,None])
    src_overlap = src_overlap[:,None].repeat(1,3).numpy()
    tgt_overlap = tgt_overlap[:,None].repeat(1,3).numpy()
    src_overlap_color = lighter(get_yellow(), 1 - src_overlap)
    tgt_overlap_color = lighter(get_blue(), 1 - tgt_overlap)
    src_pcd_overlap = copy.deepcopy(src_pcd_before)
    src_pcd_overlap.transform(tsfm)
    tgt_pcd_overlap = copy.deepcopy(tgt_pcd_before)
    src_pcd_overlap.colors = o3d.utility.Vector3dVector(src_overlap_color)
    tgt_pcd_overlap.colors = o3d.utility.Vector3dVector(tgt_overlap_color)

    ########################################
    # 3. draw registrations
    src_pcd_after = copy.deepcopy(src_pcd_before)
    src_pcd_after.transform(tsfm)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name='Input', width=960, height=540, left=0, top=0)
    vis1.add_geometry(src_pcd_before)
    vis1.add_geometry(tgt_pcd_before)
    vis1.add_geometry(coordinate_frame)

    vis2 = o3d.visualization.Visualizer()
    vis2.create_window(window_name='Inferred overlap region', width=960, height=540, left=0, top=600)
    vis2.add_geometry(src_pcd_overlap)
    vis2.add_geometry(tgt_pcd_overlap)
    vis2.add_geometry(coordinate_frame)
    vis3 = o3d.visualization.Visualizer()
    vis3.create_window(window_name ='Our registration', width=960, height=540, left=960, top=0)
    vis3.add_geometry(src_pcd_after)
    vis3.add_geometry(tgt_pcd_before)
    
    vis3.add_geometry(coordinate_frame)
    
    while True:
        vis1.update_geometry(src_pcd_before)
        vis3.update_geometry(tgt_pcd_before)
        if not vis1.poll_events():
            break
        vis1.update_renderer()

        vis2.update_geometry(src_pcd_overlap)
        vis2.update_geometry(tgt_pcd_overlap)
        if not vis2.poll_events():
            break
        vis2.update_renderer()

        vis3.update_geometry(src_pcd_after)
        vis3.update_geometry(tgt_pcd_before)
        if not vis3.poll_events():
            break
        vis3.update_renderer()

    vis1.destroy_window()
    vis2.destroy_window()
    vis3.destroy_window()    


def estimate(config, demo_loader):
    config.model.eval()
    c_loader_iter = iter(demo_loader)
    with torch.no_grad():
        
        inputs = next(c_loader_iter)
        ##################################
        # load inputs to device.
        for k, v in inputs.items():  
            if type(v) == list:
                inputs[k] = [item.to(config.device) for item in v]
            else:
                inputs[k] = v.to(config.device)

        ###############################################
        # forward pass
       
        feats, scores_overlap, scores_saliency = config.model(inputs)  #[N1, C1], [N2, C2]
      
    
        pcd = inputs['points'][0]
        len_src = inputs['stack_lengths'][0][0]
        c_rot, c_trans = inputs['rot'], inputs['trans']
        correspondence = inputs['correspondences']
        
        src_pcd, tgt_pcd = pcd[:len_src], pcd[len_src:]
        src_raw = copy.deepcopy(src_pcd)
        tgt_raw = copy.deepcopy(tgt_pcd)


        
        
        src_feats, tgt_feats = feats[:len_src].detach().cpu(), feats[len_src:].detach().cpu()
        src_overlap, src_saliency = scores_overlap[:len_src].detach().cpu(), scores_saliency[:len_src].detach().cpu()
        tgt_overlap, tgt_saliency = scores_overlap[len_src:].detach().cpu(), scores_saliency[len_src:].detach().cpu()

        ########################################
        # do probabilistic sampling guided by the score
        src_scores = src_overlap * src_saliency
        tgt_scores = tgt_overlap * tgt_saliency

        if(src_pcd.size(0) > config.n_points):
            idx = np.arange(src_pcd.size(0))
            probs = (src_scores / src_scores.sum()).numpy().flatten()
            idx = np.random.choice(idx, size= config.n_points, replace=False, p=probs)
            src_pcd, src_feats = src_pcd[idx], src_feats[idx]
        if(tgt_pcd.size(0) > config.n_points):
            idx = np.arange(tgt_pcd.size(0))
            probs = (tgt_scores / tgt_scores.sum()).numpy().flatten()
            idx = np.random.choice(idx, size= config.n_points, replace=False, p=probs)
            tgt_pcd, tgt_feats = tgt_pcd[idx], tgt_feats[idx]
        

        ########################################
        # run ransac and draw registration
        tsfm = ransac_pose_estimation(src_pcd, tgt_pcd, src_feats, tgt_feats, mutual=False)
        print("TSFM: ", tsfm)
        transformation = tsfm.copy()
        #transformation[:3, 3] /= 10
       
      

        roll = np.radians(0)  # Rotation around X-axis
        pitch = np.radians(0)  # Rotation around Y-axis
        yaw = np.radians(0)  # Rotation around Z-axis

                # Create rotation matrices for each axis
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Combine the rotations (order: R_z * R_y * R_x)
        R_xyz = R_z @ R_y @ R_x
        
        # Create a 4x4 homogeneous rotation matrix
        rotation_matrix = np.eye(4)
        rotation_matrix[:3, :3] = R_xyz  # Set the 3x3 rotation part
        transformation = rotation_matrix @ transformation  # Apply rotation
        
        

        src_icp = copy.deepcopy(src_pcd)
        tgt_icp = copy.deepcopy(tgt_pcd)
        src_icp = to_o3d_pcd(src_icp)
        tgt_icp = to_o3d_pcd(tgt_icp)

        radius_normal = 0.0045 * 2
        src_icp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        tgt_icp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        distance_threshold_icp = 0.0045 * 0.5
        result_icp = o3d.pipelines.registration.registration_icp(
            src_icp, tgt_icp, distance_threshold_icp,
            tsfm,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )
        tsfm = result_icp.transformation

        print("ICP: ", tsfm)

        validate_object_transform(tsfm, offset_world = 70, threshold = 30)

        draw_registration_result(src_raw, tgt_raw, src_overlap, tgt_overlap, src_saliency, tgt_saliency, tsfm)
        return transformation


def validate_object_transform(tsfm, offset_world, threshold):
    """
    Validate the object transformation by checking its rotation in the world frame.

    Args:
        tsfm (numpy.ndarray): 4x4 transformation matrix of the object in the origin frame.
        offset_world (float): Offset applied to the world frame rotation (in degrees).
        threshold (float): Maximum allowed deviation for rotations (in degrees).

    Returns:
        bool: True if all conditions are met, False otherwise.
    """
    # Extract the rotation matrix (top-left 3x3 part of tsfm)
    rotation_object = tsfm[:3, :3].copy()

    # Define the rotations to form the world frame
    rotation_x_neg90 = R.from_euler('x', 0, degrees=True).as_matrix()
    rotation_z_neg90 = R.from_euler('z', 0, degrees=True).as_matrix()
    rotation_offset = R.from_euler('x', offset_world, degrees=True).as_matrix()

    # Combine rotations for the world frame
    rotation_world_base = np.dot(rotation_z_neg90, rotation_x_neg90)
    rotation_world = np.dot(rotation_offset, rotation_world_base)

    # Transform the object's rotation into the world frame
    R_world_inv = rotation_world.T  # For a rotation matrix, the inverse is the transpose
    R_object_in_world = np.dot(R_world_inv, rotation_object)

    # Convert the object's rotation in the world frame to Euler angles
    r_object_in_world = R.from_matrix(R_object_in_world)
    euler_object_in_world = r_object_in_world.as_euler('xyz', degrees=True)

    # Extract individual rotations in the world frame
    rotation_x_world, rotation_y_world, rotation_z_world = euler_object_in_world

    # Check conditions
    is_z_within_limit_world = abs(rotation_z_world) < threshold
    is_x_within_limit_world = abs(rotation_x_world) < threshold

    # Print results (optional for debugging)
    print("Object in World Frame Rotations:")
    print(f"X: {rotation_x_world}°, Y: {rotation_y_world}°, Z: {rotation_z_world}°")
    print(f"Z rotation within limit (< {threshold}°): {is_z_within_limit_world}")
    print(f"X rotation within limit (< {threshold}°): {is_x_within_limit_world}")

    # Return True if all conditions are met, otherwise False
    return is_z_within_limit_world and is_x_within_limit_world

# Define a utility to convert a rotation matrix to a transformation matrix
def create_transformation(rotation_matrix, translation=np.zeros(3)):
    transformation = np.eye(4)
    transformation[:3, :3] = rotation_matrix
    transformation[:3, 3] = translation
    return transformation


def Predate_Pose():
   
    config_path = os.path.join(cwd, 'configs/test/indoor.yaml')
    config = load_config(config_path)
    config = edict(config)
    if config.gpu_mode:
        config.device = torch.device('cuda')
    else:
        config.device = torch.device('cpu')
    
    # model initialization
    config.architecture = [
        'simple',
        'resnetb',
    ]
    for i in range(config.num_layers-1):
        config.architecture.append('resnetb_strided')
        config.architecture.append('resnetb')
        config.architecture.append('resnetb')
    for i in range(config.num_layers-2):
        config.architecture.append('nearest_upsample')
        config.architecture.append('unary')
    config.architecture.append('nearest_upsample')
    config.architecture.append('last_unary')
    config.model = KPFCNN(config).to(config.device)
    
    # create dataset and dataloader
    neighborhood_limits = np.array([38, 36, 36, 38])
    #info_train = load_obj(config.train_info)
    #train_set = IndoorDataset(info_train,config,data_augmentation=True)
    
    demo_set = ThreeDMatchDemo(config, config.src_pcd, config.tgt_pcd)

    # _, neighborhood_limits = get_dataloader(dataset=train_set,
    #                                     batch_size=config.batch_size,
    #                                     shuffle=True,
    #                                     num_workers=config.num_workers,
    #                                     )
    demo_loader, _ = get_dataloader(dataset=demo_set,
                                        batch_size=config.batch_size,
                                        shuffle=False,
                                        num_workers=1,
                                        neighborhood_limits=neighborhood_limits)

    # load pretrained weights
    assert config.pretrain != None
    state = torch.load(config.pretrain,weights_only=False)
    config.model.load_state_dict(state['state_dict'])

    # do pose estimation
    transformation = estimate(config, demo_loader)
    return transformation







if __name__ == '__main__':
    Predate_Pose()