import os
import cv2

import numpy as np


if __name__ == '__main__':
    data_root = "/home/red0orange/Data/TUM/rgbd_dataset_freiburg1_teddy"
    data_txt_path = os.path.join(data_root, "rgb.txt")
    # output_data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/TestData/DvoidSLAMOuput-Teddy/"
    output_data_root = "/home/red0orange/Data/gcc_server/edgecloud/output_for_construct_map"

    # data
    data_infos = np.loadtxt(data_txt_path, dtype=np.object).tolist()

    # output data
    time_data = np.load(os.path.join(output_data_root, "tum_rgbd_dataset_freiburg1_teddy_stride02_tstamps.npy"))
    pose_data = np.load(os.path.join(output_data_root, "tum_rgbd_dataset_freiburg1_teddy_stride02_poses.npy"))
    kf_index_data = np.load(os.path.join(output_data_root, "tum_rgbd_dataset_freiburg1_teddy_stride02_indexes.npy"))
    kf_index_data = {i: j for i, j in zip(range(kf_index_data.size), kf_index_data.tolist())}
    inv_kf_index_data = {j: i for i, j in kf_index_data.items()}
    cov_data = np.load(os.path.join(output_data_root, "tum_rgbd_dataset_freiburg1_teddy_stride02_table_visibility.npy"))
    map_point_idx_xyz_uv_data = np.load(os.path.join(output_data_root, "tum_rgbd_dataset_freiburg1_teddy_stride02_idx_xyz_uv.npy"))
    map_point_indexes = np.arange(map_point_idx_xyz_uv_data.shape[0])

    # for
    pass