import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R

class TransformClusterer:
    def __init__(self,
                 max_translation_tol=0.05,
                 max_orientation_tol_deg=5.0,
                 include_yaw=False):
        """
        Initialize with clustering thresholds and whether to include yaw.
        """
        self.max_translation_tol     = max_translation_tol
        self.max_orientation_tol_rad = np.deg2rad(max_orientation_tol_deg)
        self.include_yaw             = include_yaw
        # compute orientation scale
        self.orientation_scale = max_translation_tol / self.max_orientation_tol_rad

    def _matrices_to_poses(self, mats):
        """
        Convert list of 4×4 matrices to an (N×D) pose array [x,y,z,roll,pitch,(yaw)].
        """
        poses = []
        for mat in mats:
            t   = mat[:3, 3]
            rpy = R.from_matrix(mat[:3, :3]).as_euler('xyz', degrees=False)
            if self.include_yaw:
                poses.append(np.concatenate([t, rpy]))
            else:
                poses.append(np.concatenate([t, rpy[:2]]))
        return np.array(poses)

    def cluster_and_mean(self, mats):
        """
        Cluster the inverse of input 4×4 transforms with DBSCAN (min_samples=2),
        compute average in inverse space, then invert to return mean transform.
        Returns None if no valid cluster found.
        """
        # 0) Invert each transform
        mats_inv = [np.linalg.inv(m) for m in mats]

        # 1) Build pose vectors for clustering from inverted mats
        poses = self._matrices_to_poses(mats_inv)
        if self.include_yaw:
            features = poses.copy()
            features[:, 3:6] *= self.orientation_scale
        else:
            features = np.hstack([
                poses[:, :3],
                poses[:, 3:5] * self.orientation_scale
            ])

        # 2) Compute eps
        eps = np.sqrt(
            self.max_translation_tol**2 +
            (self.orientation_scale * self.max_orientation_tol_rad)**2
        )

        # 3) DBSCAN
        labels = DBSCAN(eps=eps, min_samples=2).fit_predict(features)
        valid_labels, counts = np.unique(labels[labels >= 0], return_counts=True)
        if len(valid_labels) == 0:
            return None

    
        # --- NEW: print size of biggest cluster ---------------------------------
        biggest_cluster_size = counts.max()
        print(f"[DBSCAN] biggest cluster contains {biggest_cluster_size} points")

        # 4) Pick largest cluster
        best_label = valid_labels[np.argmax(counts)]
        idxs       = np.where(labels == best_label)[0]

        # 5) Mean translation (in inverse space)
        translations = poses[idxs, :3]
        mean_t       = translations.mean(axis=0)

        # 6) Compute average rotation in inverse space via inverse-mean-inverse
        rot_mats = [mats_inv[i][:3, :3] for i in idxs]
        rot_inv  = [R.T for R in rot_mats]
        R_inv_mean = np.mean(rot_inv, axis=0)
        U, _, Vt = np.linalg.svd(R_inv_mean)
        R_inv_mean_ortho = U @ Vt
        R_mean_inv = np.linalg.inv(R_inv_mean_ortho)

        # 7) Assemble mean inverse transform
        mean_inv = np.eye(4)
        mean_inv[:3, :3] = R_mean_inv
        mean_inv[:3, 3]  = mean_t

        # 8) Invert average transform to get mean in original space
        mean_mat = np.linalg.inv(mean_inv)
        return mean_mat
