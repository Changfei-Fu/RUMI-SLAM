#!/usr/bin/python3
import os
import copy
import logging
import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

matplotlib.use("Agg")

from evo.core import metrics
from evo.core import geometry
from evo.tools import file_interface
from evo.tools import plot
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D

import rospy
import rospkg

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose


def poses_to_trajectory(poses):
    stamps = []
    xyz = []
    quat = []

    for pose in poses:
        stamps.append(pose.header.stamp.to_sec())
        pose = pose.pose
        xyz.append([pose.position.x, pose.position.y, pose.position.z])
        quat.append(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        pass

    stamps = np.array(stamps)
    xyz = np.array(xyz)
    quat = np.array(quat)
    quat = np.roll(quat, 1, axis=1)  # shift 1 column -> w in front column
    return PoseTrajectory3D(xyz, quat, stamps)


def read_tum_trajectory_file(file_path):
    raw_mat = file_interface.csv_read_matrix(file_path, delim=" ", comment_str="#")
    error_msg = (
        "TUM trajectory files must have 8 entries per row "
        "and no trailing delimiter at the end of the rows (space)"
    )
    if not raw_mat or (len(raw_mat) > 0 and len(raw_mat[0]) != 8):
        raise file_interface.FileInterfaceException(error_msg)
    try:
        mat = np.array(raw_mat).astype(float)
    except ValueError:
        raise file_interface.FileInterfaceException(error_msg)
    stamps = mat[:, 0]  # n x 1
    mat = mat[np.argsort(stamps), :]

    stamps = mat[:, 0]  # n x 1
    xyz = mat[:, 1:4]  # n x 3

    if os.path.basename(os.path.dirname(file_path)) == "living_room_traj0_frei_png":
        xyz[:, 0] = -xyz[:, 0]
        pass
    if os.path.basename(os.path.dirname(file_path)) == "living_room_traj2_frei_png":
        xyz[:, 1] = -xyz[:, 1]
        pass
    if os.path.basename(os.path.dirname(file_path)) == "living_room_traj3_frei_png":
        xyz[:, 1] = -xyz[:, 1]
        pass
    elif os.path.basename(os.path.dirname(file_path)) == "traj0_frei_png":
        xyz[:, 1] = -xyz[:, 1]
        pass
    elif os.path.basename(os.path.dirname(file_path)) == "traj1_frei_png":
        xyz[:, 1] = -xyz[:, 1]
        pass
    elif os.path.basename(os.path.dirname(file_path)) == "traj2_frei_png":
        xyz[:, 1] = -xyz[:, 1]
        pass
    elif os.path.basename(os.path.dirname(file_path)) == "traj3_frei_png":
        xyz[:, 1] = -xyz[:, 1]
        pass

    quat = mat[:, 4:]  # n x 4
    quat = np.roll(quat, 1, axis=1)  # shift 1 column -> w in front column
    if not hasattr(file_path, "read"):  # if not file handle
        print("Loaded {} stamps and poses from: {}".format(len(stamps), file_path))
    return PoseTrajectory3D(xyz, quat, stamps)


def get_align_length(gt_traj, est_traj):
    max_diff = 0.01
    aligned_traj_ref, traj_est = sync.associate_trajectories(
        gt_traj, est_traj, max_diff
    )
    length = float(geometry.arc_len(aligned_traj_ref.positions_xyz))
    return length


if __name__ == "__main__":
    dataset_root = "/home/red0orange/projects/Cloud-Edge-SLAM/src/cloud_edge_slam/groundtruth"
    dataset_names = [
        # TUM
        "slam-tum/rgbd_dataset_freiburg1_floor",
        "slam-tum/rgbd_dataset_freiburg1_room",
        "slam-tum/rgbd_dataset_freiburg1_teddy",
        "slam-tum/rgbd_dataset_freiburg2_desk",
        "slam-tum/rgbd_dataset_freiburg2_pioneer_360",
        "slam-tum/rgbd_dataset_freiburg2_pioneer_slam",
        "slam-tum/rgbd_dataset_freiburg2_pioneer_slam2",
        "slam-tum/rgbd_dataset_freiburg2_pioneer_slam3",
        "slam-tum/rgbd_dataset_freiburg3_teddy",
        # Euroc
        "slam-euroc/MH04",
        "slam-euroc/MH05",
        "slam-euroc/V102",
        "slam-euroc/V103",
        "slam-euroc/V201",
        "slam-euroc/V202",
        "slam-euroc/V203",
        # ICL
        "slam-icl/living_room_traj0_frei_png",
        "slam-icl/living_room_traj1_frei_png",
        "slam-icl/living_room_traj2_frei_png",
        "slam-icl/living_room_traj3_frei_png",
        "slam-icl/traj0_frei_png",
        "slam-icl/traj1_frei_png",
        "slam-icl/traj2_frei_png",
        "slam-icl/traj3_frei_png",
    ]
    groundtruth_file_path_dict = {
        os.path.basename(i): os.path.join(dataset_root, i, "groundtruth.txt")
        for i in dataset_names
    }

    # dataset_name = "rgbd_dataset_freiburg1_floor"
    dataset_name = "rgbd_dataset_freiburg2_pioneer_360"
    # dataset_name = "living_room_traj1_frei_png"
    results_dir = "/home/red0orange/projects/Cloud-Edge-SLAM/results"
    result_paths = [os.path.join(results_dir, i) for i in os.listdir(results_dir)]
    times_indexes = np.argsort([os.path.getmtime(i) for i in result_paths])
    evo_result_path = result_paths[times_indexes[-1]]
    print(evo_result_path)

    whole_traj_path = os.path.join(evo_result_path, "whole_map.txt")
    cloud_traj_path = os.path.join(evo_result_path, "before_cloud.txt")
    front_path = os.path.join(evo_result_path, "before_edge_front.txt")
    back_path = os.path.join(evo_result_path, "before_edge_back.txt")
    front_common_area_path = os.path.join(evo_result_path, "edge_front_cloud_match.txt")
    back_common_area_path = os.path.join(evo_result_path, "edge_back_cloud_match.txt")

    traj_ref = read_tum_trajectory_file(groundtruth_file_path_dict[dataset_name])
    traj_whole = read_tum_trajectory_file(whole_traj_path)
    traj_front = read_tum_trajectory_file(front_path)
    traj_back = read_tum_trajectory_file(back_path)
    traj_cloud = read_tum_trajectory_file(cloud_traj_path)
    traj_front_common = read_tum_trajectory_file(front_common_area_path)
    traj_back_common = read_tum_trajectory_file(back_common_area_path)

    whole_length = get_align_length(traj_ref, traj_whole)
    front_length = get_align_length(traj_ref, traj_front)
    back_length = get_align_length(traj_ref, traj_back)
    cloud_length = get_align_length(traj_ref, traj_cloud)
    front_common_length = get_align_length(traj_ref, traj_front_common)
    back_common_length = get_align_length(traj_ref, traj_back_common)

    print("whole_length: ", whole_length)
    print("ori_cloud_length: ", cloud_length)
    print("ori_front_length: ", front_length)
    print("ori_back_length: ", back_length)
    print("front_length: ", front_common_length)
    print("back_length: ", back_common_length)