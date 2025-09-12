#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Predator registration refactor:
- Proper dataset contract for OverlapPredator's collate_fn (10-tuple)
- Pure-PyTorch guided sampling (dtype-safe, AMP-safe)
- AMP via torch.amp.autocast('cuda', ...)
- Robust device, logging, and path checks
"""

import os
import sys
import copy
import logging
from pathlib import Path
from typing import Optional, Tuple
import time
import numpy as np
import torch
from torch.utils.data import Dataset
import open3d as o3d
from easydict import EasyDict as edict
from scipy.spatial.transform import Rotation as R

# -----------------------------------------------------------------------------
# Logging
# -----------------------------------------------------------------------------
log = logging.getLogger("predator_refactor")
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

# -----------------------------------------------------------------------------
# OverlapPredator repo location (adjust if needed)
# -----------------------------------------------------------------------------
PREDATOR_ROOT = "/ros2_ws/src/wetexplorer/wetexplorer_vision_predator/wetexplorer_vision_predator/OverlapPredator"
if PREDATOR_ROOT not in sys.path:
    sys.path.append(PREDATOR_ROOT)

# OverlapPredator imports
from datasets.dataloader import get_dataloader
from models.architectures import KPFCNN
from lib.utils import load_obj, setup_seed, load_config
from lib.benchmark_utils import (
    ransac_pose_estimation, to_o3d_pcd, get_blue, get_yellow, to_tensor
)
from scripts.cal_overlap import get_overlap_ratio

# -----------------------------------------------------------------------------
# Globals / defaults
# -----------------------------------------------------------------------------
setup_seed(0)
DEFAULT_VOXEL_SIZE = 0.025  # m (used for downsampling CAD/real clouds)
DEBUG = False

# -----------------------------------------------------------------------------
# Dataset that satisfies get_dataloader's collate_fn (10-tuple per sample)
# -----------------------------------------------------------------------------
class ThreeDMatchDemo(Dataset):
    """
    Returns ONE item as a 10-tuple required by collate_fn_descriptor:
      (src_pcd, tgt_pcd, src_feats, tgt_feats, rot, trans,
       matching_inds, src_pcd_raw, tgt_pcd_raw, sample)

    - src_* and tgt_* arrays are np.float32
    - matching_inds/sample can be torch tensors (collate handles them)
    """

    def __init__(self, config, src_path: str, tgt_path: str, voxel: float = DEFAULT_VOXEL_SIZE):
        super().__init__()
        self.config = config  # required by OverlapPredator dataloader
        self.src_path = Path(src_path) if src_path else None
        self.tgt_path = Path(tgt_path) if tgt_path else None
        self.voxel = float(voxel)

        # Filled by _get_CAD_model() and _update_Real_model()
        self.src_pcd = None
        self.tgt_pcd = None
        self.src_feats = None
        self.tgt_feats = None
        self.src_raw = None
        self.tgt_raw = None
       

        

    def __len__(self):
        return 1

    # --- one-time CAD (source) load ---
    def _get_CAD_model(self):
        if not self.src_path or not self.src_path.is_file():
            raise FileNotFoundError(f"CAD/Source PCD not found: {self.src_path}")
        src = o3d.io.read_point_cloud(str(self.src_path))
        if len(src.points) == 0:
            raise ValueError("CAD/Source cloud is empty.")
        src_ds = src.voxel_down_sample(self.voxel)
        self.src_pcd = np.asarray(src_ds.points, dtype=np.float32)
        self.src_raw = np.asarray(src.points, dtype=np.float32)
        self.src_feats = np.ones((self.src_pcd.shape[0], 1), dtype=np.float32)
        if DEBUG:
            log.info("Source points (ds=%g): %d", self.voxel, self.src_pcd.shape[0])

    # --- per-call real (target) update ---
    def _update_Real_model(self, tgt_array: Optional[np.ndarray] = None):
        if tgt_array is not None:
            # Expecting (N, 3) float array
            assert tgt_array.ndim == 2 and tgt_array.shape[1] == 3, "tgt_array must be (N,3)"
            self.tgt_raw = tgt_array.astype(np.float32)
            # downsample with Open3D for consistency
            tgt_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.tgt_raw))
            tgt_ds = tgt_o3d.voxel_down_sample(self.voxel)
            self.tgt_pcd = np.asarray(tgt_ds.points, dtype=np.float32)
        else:
            if not self.tgt_path or not self.tgt_path.is_file():
                raise FileNotFoundError(f"Real/Target PCD not found: {self.tgt_path}")
            tgt = o3d.io.read_point_cloud(str(self.tgt_path))
            if len(tgt.points) == 0:
                raise ValueError("Real/Target cloud is empty.")
            tgt_ds = tgt.voxel_down_sample(self.voxel)
            self.tgt_pcd = np.asarray(tgt_ds.points, dtype=np.float32)
            self.tgt_raw = np.asarray(tgt.points, dtype=np.float32)

        self.tgt_feats = np.ones((self.tgt_pcd.shape[0], 1), dtype=np.float32)
        log.info("Target points (ds=%g): %d", self.voxel, self.tgt_pcd.shape[0])

    def __getitem__(self, idx):
        assert self.src_pcd is not None, "Call _get_CAD_model() first."
        assert self.tgt_pcd is not None, "Call _update_Real_model() before fetching a sample."

        rot = np.eye(3, dtype=np.float32)
        trans = np.zeros((3, 1), dtype=np.float32)
        matching_inds = torch.ones(1, 2).long()  # dummy
        sample = torch.ones(1)                   # dummy

        return (
            self.src_pcd,          # src_pcd (Ns,3) float32
            self.tgt_pcd,          # tgt_pcd (Nt,3) float32
            self.src_feats,        # src_feats (Ns,1)
            self.tgt_feats,        # tgt_feats (Nt,1)
            rot,                   # rot (3,3)
            trans,                 # trans (3,1)
            matching_inds,         # matching indices (K,2) long
            self.src_raw,          # src_raw (Ns0,3)
            self.tgt_raw,          # tgt_raw (Nt0,3)
            sample                 # sample tensor
        )

# -----------------------------------------------------------------------------
# Core estimation
# -----------------------------------------------------------------------------
def _to_device(batch: dict, device: torch.device) -> dict:
    out = {}
    for k, v in batch.items():
        if isinstance(v, list):
            out[k] = [vi.to(device, non_blocking=True) for vi in v]
        elif torch.is_tensor(v):
            out[k] = v.to(device, non_blocking=True)
        else:
            out[k] = v
    return out

def _guided_sample(pcd: torch.Tensor,
                   fts: torch.Tensor,
                   ov: torch.Tensor,
                   sa: torch.Tensor,
                   n_pts: int) -> Tuple[torch.Tensor, torch.Tensor]:
    """Pure-PyTorch probabilistic sampling; safe dtypes; returns (pcd_sel, fts_sel)."""
    if pcd.size(0) <= n_pts:
        return pcd, fts
    scores = (ov * sa).to(torch.float32).view(-1)
    ssum = scores.sum()
    if (not torch.isfinite(ssum)) or ssum.item() <= 0:
        idx = torch.randperm(pcd.size(0))[:n_pts]
    else:
        probs = (scores / ssum).clamp_min(1e-12)
        probs = probs / probs.sum()
        k = min(n_pts, pcd.size(0))
        idx = torch.multinomial(probs, k, replacement=False)
    return pcd[idx], fts[idx]

def project_to_so3(Rm: np.ndarray) -> np.ndarray:
    """Project a nearly-rotation matrix to SO(3) via SVD."""
    U, _, Vt = np.linalg.svd(Rm)
    R_proj = U @ Vt
    if np.linalg.det(R_proj) < 0:
        U[:, -1] *= -1
        R_proj = U @ Vt
    return R_proj

def estimate(config: edict, demo_loader) -> Tuple[np.ndarray, float]:
    """
    Run model → guided sampling → RANSAC → overlap ratio.
    Returns (4x4 transform, overlap_ratio).
    """
    device = config.device
    model = config.model.eval()

    it = iter(demo_loader)
    batch = next(it)
    batch = _to_device(batch, device)

    # Inference (AMP preferred on CUDA)
    use_amp = (device.type == "cuda")
    amp_dtype = torch.bfloat16 if (use_amp and torch.cuda.is_bf16_supported()) else torch.float16

    with torch.inference_mode():
        if hasattr(torch, "amp"):
            ctx = torch.amp.autocast("cuda", enabled=use_amp, dtype=amp_dtype)
        else:  # fallback (older PyTorch)
            ctx = torch.cuda.amp.autocast(enabled=use_amp)


        # --- timing: sync -> start -> forward -> sync -> stop
        if use_amp:
            torch.cuda.synchronize()
        t0 = time.perf_counter()

        with ctx:
            feats, scores_overlap, scores_saliency = model(batch)

        if use_amp:
            torch.cuda.synchronize()
        t1 = time.perf_counter()
        infer_ms = (t1 - t0) * 1000.0
        if DEBUG:
            print(f"[Predator] Inference time: {infer_ms:.2f} ms (AMP={use_amp}, dtype={amp_dtype})")



    t0 = time.perf_counter()

    # Unpack points
    pcd_cat = batch["points"][0]           # (N,3) tensor
    lens = batch["stack_lengths"][0]       # (2,) [Ns, Nt]
    len_src = int(lens[0].item())
    src_pcd = pcd_cat[:len_src]
    tgt_pcd = pcd_cat[len_src:]

    # Move features/scores to CPU float32 for downstream ops
    feats = feats.to(torch.float32)  # stay on device
    scores_overlap = scores_overlap.detach().cpu().to(torch.float32)
    scores_saliency = scores_saliency.detach().cpu().to(torch.float32)

    src_feats = feats[:len_src]
    tgt_feats = feats[len_src:]
    src_overlap = scores_overlap[:len_src]
    tgt_overlap = scores_overlap[len_src:]
    src_saliency = scores_saliency[:len_src]
    tgt_saliency = scores_saliency[len_src:]

    # Guided sampling
    n_points = int(getattr(config, "n_points", 5000))
    src_pcd_s, src_feats_s = _guided_sample(src_pcd, src_feats, src_overlap, src_saliency, n_points)
    tgt_pcd_s, tgt_feats_s = _guided_sample(tgt_pcd, tgt_feats, tgt_overlap, tgt_saliency, n_points)

    # RANSAC pose
    T = ransac_pose_estimation(src_pcd_s, tgt_pcd_s, src_feats_s, tgt_feats_s, mutual=False)
    T = T.copy()

    # Overlap ratio (Open3D)
    src_raw = src_pcd.detach().cpu()
    tgt_raw = tgt_pcd.detach().cpu()
    src_o3d = to_o3d_pcd(src_raw)
    tgt_o3d = to_o3d_pcd(tgt_raw)
    src_o3d.transform(T)
    overlap_ratio = get_overlap_ratio(src_o3d, tgt_o3d)

    if DEBUG:
        log.info("RANSAC done. Overlap ratio: %.4f", overlap_ratio)


    icp_radius = 3*DEFAULT_VOXEL_SIZE
    icp_max_iter = 10
    # ICP refinement (point-to-plane)
    src_icp = to_o3d_pcd(src_pcd_s.cpu())
    tgt_icp = to_o3d_pcd(tgt_pcd_s.cpu())

    rad_norm = DEFAULT_VOXEL_SIZE * 2.0
    src_icp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=rad_norm, max_nn=30))
    tgt_icp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=rad_norm, max_nn=30))

    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=icp_max_iter)
    result_icp = o3d.pipelines.registration.registration_icp(
        src_icp, tgt_icp, icp_radius, T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria
    )
    T_icp = result_icp.transformation.astype(np.float32)
    # Project rotation to SO(3) to avoid drift
    T_icp[:3, :3] = project_to_so3(T_icp[:3, :3])

    
    delta_t = T_icp[:3, 3].astype(np.float32)- T[:3, 3].astype(np.float32)
    disp = float(np.linalg.norm(delta_t))
    if DEBUG:
        print(f"[ICP] Δt = [{delta_t[0]:.4f}, {delta_t[1]:.4f}, {delta_t[2]:.4f}] m | |Δt| = {disp:.4f} m")


     # Overlap ratio (Open3D)
    src_raw = src_pcd.detach().cpu()
    tgt_raw = tgt_pcd.detach().cpu()
    src_o3d = to_o3d_pcd(src_raw)
    tgt_o3d = to_o3d_pcd(tgt_raw)
    src_o3d.transform(T_icp)
    overlap_ratio = get_overlap_ratio(src_o3d, tgt_o3d)
    if DEBUG:
        log.info("ICP done. Overlap ratio: %.4f", overlap_ratio)

    t1 = time.perf_counter()
    infer_ms = (t1 - t0) * 1000.0
    if DEBUG:
        print(f"[Predator] Post_Processing time: {infer_ms:.2f} ms")
        draw_registration_result(src_raw, tgt_raw, src_overlap, tgt_overlap, src_saliency, tgt_saliency, T,T_icp)
    return T_icp, float(overlap_ratio)


def lighter(color, percent):
    '''assumes color is rgb between (0, 0, 0) and (1,1,1)'''
    color = np.array(color)
    white = np.array([1, 1, 1])
    vector = white-color
    return color + vector * percent
def draw_registration_result(src_raw, tgt_raw, src_overlap, tgt_overlap, src_saliency, tgt_saliency, tsfm, tsfm_icp):
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

    src_pcd_after_icp = copy.deepcopy(src_pcd_before)
    src_pcd_after_icp.transform(tsfm_icp)
    
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name='RANSAC', width=960, height=540, left=0, top=0)
    vis1.add_geometry(src_pcd_after)
    vis1.add_geometry(tgt_pcd_before)
    vis1.add_geometry(coordinate_frame)

    vis2 = o3d.visualization.Visualizer()
    vis2.create_window(window_name='ICP Refinement', width=960, height=540, left=960, top=0)
    vis2.add_geometry(src_pcd_after_icp)
    vis2.add_geometry(tgt_pcd_before)
    vis2.add_geometry(coordinate_frame)

 
    
    while True:
        vis1.update_geometry(src_pcd_after)       
        if not vis1.poll_events():
            break
        vis1.update_renderer()

        vis2.update_geometry(src_pcd_after_icp)
        vis2.update_geometry(tgt_pcd_before)
        if not vis2.poll_events():
            break
        vis2.update_renderer()



    vis1.destroy_window()
    vis2.destroy_window()

# -----------------------------------------------------------------------------
# Predator facade
# -----------------------------------------------------------------------------
class Predator:
    def __init__(self, workspace,
                 config_path: str = os.path.join(PREDATOR_ROOT, "configs/test/indoor.yaml"),
                 voxel_size: float = DEFAULT_VOXEL_SIZE):
        self.config_path = config_path
        self.voxel_size = float(voxel_size)

        self.config: Optional[edict] = None
        self.neighborhood_limits = None
        self.demo_set: Optional[ThreeDMatchDemo] = None
        self.demo_loader = None
        self.initialized = False
        self.workspace = workspace

    def initialize(self) -> bool:
        cfg = edict(load_config(self.config_path))
        cfg['src_pcd']  = self.workspace + cfg['src_pcd']
        cfg['tgt_pcd']  = self.workspace + cfg['tgt_pcd']

        # Update misc.pretrain (flattened key)
        cfg['pretrain'] = self.workspace + cfg['pretrain']


        # Device
        use_cuda = bool(cfg.get("gpu_mode", False)) and torch.cuda.is_available()
        cfg.device = torch.device("cuda" if use_cuda else "cpu")
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True

        # Architecture
        cfg.architecture = ['simple', 'resnetb']
        for _ in range(cfg.num_layers - 1):
            cfg.architecture += ['resnetb_strided', 'resnetb', 'resnetb']
        for _ in range(cfg.num_layers - 2):
            cfg.architecture += ['nearest_upsample', 'unary']
        cfg.architecture += ['nearest_upsample', 'last_unary']

        # Model
        model = KPFCNN(cfg).to(cfg.device).eval()
        ckpt_path = Path(cfg.pretrain)
        if not ckpt_path.is_file():
            raise FileNotFoundError(f"Pretrained weights not found: {ckpt_path}")
        state = torch.load(str(ckpt_path), map_location=cfg.device)
        model.load_state_dict(state['state_dict'])
        cfg.model = model
        if DEBUG:
            log.info("Model loaded: %s", ckpt_path.name)

        # Neighborhood limits (from train set)
        info_train = load_obj(cfg.train_info)
 
        
        self.neighborhood_limits = np.array([38, 36, 36, 38])
      

        # Demo dataset + CAD load
        self.demo_set = ThreeDMatchDemo(cfg, cfg.src_pcd, cfg.tgt_pcd, voxel=self.voxel_size)
        self.demo_set._get_CAD_model()

        self.config = cfg
        self.initialized = True
        return True

    def _build_loader(self):
        assert self.demo_set is not None and self.neighborhood_limits is not None
        self.demo_loader, _ = get_dataloader(
            dataset=self.demo_set,
            batch_size=self.config.batch_size,
            shuffle=False,
            num_workers=1,
            neighborhood_limits=self.neighborhood_limits
        )

    def load_CAD_model(self):
        if not self.initialized:
            self.initialize()
        self.demo_set._get_CAD_model()

    def update_PointCloud(self, tgt_array: Optional[np.ndarray] = None):
        if not self.initialized:
            self.initialize()
        self.demo_set._update_Real_model(tgt_array)
        self._build_loader()


    def run_Estimation(self, tgt_array: Optional[np.ndarray] = None) -> Tuple[np.ndarray, float]:
        """
        Update target cloud (if provided) and run a single registration.
        Returns (4x4 transform, overlap_ratio).
        """
        if not self.initialized:
            self.initialize()
        self.update_PointCloud(tgt_array)
        if DEBUG:
            log.info("Running Predator estimation…")
        T, ratio = estimate(self.config, self.demo_loader)
        if DEBUG:
            log.info("Predator finished. Overlap ratio=%.4f", ratio)
        return T.copy(), ratio

    # (optional) keep your validation helper
    @staticmethod
    def validate_object_transform(tsfm: np.ndarray, offset_world: float = 70.0, threshold: float = 30.0) -> bool:
        rotation_object = tsfm[:3, :3].copy()
        rotation_world = R.from_euler('x', offset_world, degrees=True).as_matrix() @ R.from_euler('z', 0, degrees=True).as_matrix() @ R.from_euler('x', 0, degrees=True).as_matrix()
        R_object_in_world = rotation_world.T @ rotation_object
        rx, ry, rz = R.from_matrix(R_object_in_world).as_euler('xyz', degrees=True)
        ok = abs(rz) < threshold and abs(rx) < threshold
        if DEBUG:
            log.info("Object in world: Rx=%.2f°, Ry=%.2f°, Rz=%.2f° | ok=%s", rx, ry, rz, ok)
        return ok

# -----------------------------------------------------------------------------
# Script entry (example usage)
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    pred = Predator()
    # Example 1: use file paths from YAML (cfg.src_pcd / cfg.tgt_pcd)
    T, ratio = pred.run_Estimation()  # no array passed → loads target from cfg.tgt_pcd
    np.set_printoptions(precision=6, suppress=True)
    #print("Transform:\n", T)
    #print("Overlap ratio:", ratio)

    # Example 2: if you have a numpy (N,3) target in memory:
    # new_tgt = np.random.rand(10000, 3).astype(np.float32)
    # T2, ratio2 = pred.run_Estimation(new_tgt)
