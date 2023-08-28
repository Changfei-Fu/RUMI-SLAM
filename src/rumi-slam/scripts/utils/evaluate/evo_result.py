import os
import copy

import logging
import numpy as np

import matplotlib.pyplot as plt
from evo.core import metrics
from evo.core import geometry
from evo.tools import file_interface
from evo.tools import plot
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D

logger = logging.getLogger(__name__)


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
        logger.debug(
            "Loaded {} stamps and poses from: {}".format(len(stamps), file_path)
        )
    return PoseTrajectory3D(xyz, quat, stamps)


if __name__ == "__main__":
    # ref_file = "/home/red0orange/Data/TUM/rgbd_dataset_freiburg3_structure_texture_near/CloudMergeResults/groundtruth_30s.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/CloudEdge/rgbd_dataset_freiburg2_pioneer_360/groundtruth_0-24s.txt"

    ref_file = "/media/red0orange/Elements SE/gt/gt_two.tum"
    # TUM
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg1_floor/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg1_room/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg1_teddy/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg2_360_kidnap/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg2_pioneer_360/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg2_pioneer_slam/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/am-tum/rgbd_dataset_freiburg2_pioneer_slam2/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg2_pioneer_slam3/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg2_desk/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg3_teddy/groundtruth.txt"

    # euroc
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-euroc/MH04/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg1_room/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-euroc/V102/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-euroc/V103/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-euroc/V201/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-euroc/V202/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-euroc/V203/groundtruth.txt"

    # icl
    # ref_file = (
    #     "/home/ruanjh/Workspace/datasets/slam-icl/living_room_traj3_frei_png/groundtruth.txt"
    # )
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-icl/traj2_frei_png/groundtruth.txt"
    # ref_file = (
    #     "/home/ruanjh/Workspace/datasets/slam-icl/living_room_traj0_frei_png/groundtruth.txt"
    # )
    # ref_file = (
    #     "/home/ruanjh/Workspace/datasets/slam-icl/living_room_traj1_frei_png/groundtruth.txt"
    # )
    # ref_file = (
    #     "/home/ruanjh/Workspace/datasets/slam-icl/living_room_traj2_frei_png/groundtruth.txt"
    # )
    # ref_file = (
    #     "/home/ruanjh/Workspace/datasets/slam-icl/living_room_traj3_frei_png/groundtruth.txt"
    # )
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-icl/traj1_frei_png/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-icl/traj2_frei_png/groundtruth.txt"
    # ref_file = "/home/ruanjh/Workspace/datasets/slam-icl/traj3_frei_png/groundtruth.txt"

    est_files_map = {
        # "before_cloud": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/before_cloud.txt",
        # "before_edge_front": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/before_edge_front.txt",
        # "before_edge_back": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/before_edge_back.txt",
        # # # "first_merge": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/first_merge.txt",
        # "second_merge": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/second_merge.txt",
        "whole": "/media/red0orange/Project/CloudEdgeSLAM/results/Full#Project#10-14-16_40_05/whole_map.txt",
        # "cloud": "/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg1_room/groundtruth.txt",
    }
    # est_files_map = {
    #     # "before_cloud": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/before_cloud.txt",
    #     # "before_edge_front": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/before_edge_front.txt",
    #     # "before_edge_back": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/before_edge_back.txt",
    #     # # "first_merge": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/first_merge.txt",
    #     # "second_merge": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/second_merge.txt",
    #     "whole": "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/whole_map.txt",
    # }

    # traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_ref = read_tum_trajectory_file(ref_file)  # 使用自己的具有对齐功能的
    # traj_by_label = {"reference": traj_ref}
    traj_by_label = {}
    for est_name, est_file_path in est_files_map.items():
        traj_est = read_tum_trajectory_file(est_file_path)
        if "cloud" in est_name:
            print("{}: ".format(est_name) + str(traj_est))
            pass

        max_diff = 0.01
        aligned_traj_ref, traj_est = sync.associate_trajectories(
            traj_ref, traj_est, max_diff
        )

        traj_est_aligned = copy.deepcopy(traj_est)
        traj_est_aligned.align(
            aligned_traj_ref, correct_scale=True, correct_only_scale=False
        )

        tum_ate_equivalent = metrics.APE(metrics.PoseRelation.translation_part)
        tum_ate_equivalent.process_data((aligned_traj_ref, traj_est_aligned))
        print(
            "{}'s ate: ".format(est_name),
            tum_ate_equivalent.get_statistic(metrics.StatisticsType.rmse),
        )

        ref_length = float(geometry.arc_len(traj_ref.positions_xyz))
        est_length = float(geometry.arc_len(traj_est_aligned.positions_xyz))
        print("traj length rate: {}".format(est_length / ref_length))

        ref_duration = float(traj_ref.timestamps[-1] - traj_ref.timestamps[0])
        est_duration = float(
            traj_est_aligned.timestamps[-1] - traj_est_aligned.timestamps[0]
        )
        print("traj duration rate: {}".format(est_duration / ref_duration))

        traj_by_label[est_name] = traj_est_aligned
        traj_by_label["align_ref"] = aligned_traj_ref

    fig = plt.figure()
    plot.trajectories(fig, traj_by_label, plot.PlotMode.xyz)
    plt.show()
    pass
