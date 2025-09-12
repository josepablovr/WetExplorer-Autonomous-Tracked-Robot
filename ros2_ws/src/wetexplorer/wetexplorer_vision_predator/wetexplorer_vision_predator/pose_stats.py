import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R

class Transform_Statistics:
    def __init__(self,
                 min_overlap=0.1):
        """
        Initialize with clustering thresholds and whether to include yaw.
        """
        self.min_overlap=0.1

    def rpy_to_matrix(self, roll, pitch, yaw):
            # roll (x), pitch (y), yaw (z)
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)

            R = np.array([
                [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                [-sp,   cp*sr,            cp*cr           ]
            ])
            return R
    

    def rotation_mean_chordal_weighted(self, R_list, weights):
            w = np.asarray(weights, dtype=float)
            w /= w.sum()
            A = sum(w[i] * R_list[i] for i in range(len(R_list)))  # 3x3
            U, _, Vt = np.linalg.svd(A)
            Rm = U @ Vt
            # ensure det=+1
            if np.linalg.det(Rm) < 0:
                U[:, -1] *= -1
                Rm = U @ Vt
            return Rm

    def average_se3_weighted(self, transform_matrices, overlap_ratios):
        w = np.asarray(overlap_ratios, dtype=float)
        w /= w.sum()

        R_list = [T[:3, :3] for T in transform_matrices]
        t_arr  = np.stack([T[:3, 3] for T in transform_matrices], axis=0)

        R_avg = self.rotation_mean_chordal_weighted(R_list, w)
        t_avg = (w[:, None] * t_arr).sum(axis=0)

        T_avg = np.eye(4)
        T_avg[:3, :3] = R_avg
        T_avg[:3, 3]  = t_avg
        return T_avg


    def average_pose(self, transform_matrices, overlap_ratios):
        scores = np.array(overlap_ratios)
        norm_scores = (scores - scores.min()) / (scores.max() - scores.min())
        normalized = norm_scores**3 # emphasize higher ones
        T_avg = self.average_se3_weighted(transform_matrices, normalized)
        return T_avg
   
