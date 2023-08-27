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
import cv_bridge

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cloud_edge_slam.srv import Evo, EvoResponse


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


class Visualiser:
    def __init__(self) -> None:
        self.fig = plt.figure()
        pass

    def get_traj_img(self, traj_by_label):
        self.fig.clf()
        plot.trajectories(self.fig, traj_by_label, plot.PlotMode.xyz)
        # convert it to an OpenCV image/numpy array

        canvas = FigureCanvas(self.fig)
        canvas.draw()
        img = np.array(self.fig.canvas.get_renderer()._renderer)
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

        # img = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep="")
        # img = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img


if __name__ == "__main__":
    rospy.init_node("evo_node")

    topic_name = "/cloud_edge_evo_temp"
    rospack = rospkg.RosPack()
    dataset_root = os.path.join(rospack.get_path('cloud_edge_slam'), "groundtruth")

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

    br = cv_bridge.CvBridge()

    vis = Visualiser()

    def evo(req):
        if req.use_ref:
            dataset_name = req.dataset_name
            poses = req.poses
            ori_duration = req.ori_duration

            gt_file_path = groundtruth_file_path_dict[dataset_name]
            traj_ref = read_tum_trajectory_file(gt_file_path)  # 使用自己的具有对齐功能的
            traj_by_label = {"reference": traj_ref}
            traj_est = poses_to_trajectory(poses=poses)

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
            ate = tum_ate_equivalent.get_statistic(metrics.StatisticsType.rmse)
            print("{}'s ate: ".format(dataset_name), ate)

            ref_length = float(geometry.arc_len(traj_ref.positions_xyz))
            est_length = float(geometry.arc_len(traj_est_aligned.positions_xyz))
            print("traj length rate: {}".format(est_length / ref_length))

            ref_duration = float(traj_ref.timestamps[-1] - traj_ref.timestamps[0])
            est_duration = float(
                traj_est_aligned.timestamps[-1] - traj_est_aligned.timestamps[0]
            )
            # rate = est_duration / ref_duration
            rate = est_duration / ori_duration
            print("traj duration rate: {}".format(rate))

            traj_by_label["align_reference"] = aligned_traj_ref
            traj_by_label["align_estimation"] = traj_est_aligned
            img = vis.get_traj_img(traj_by_label)
            msg = br.cv2_to_imgmsg(img)

            response = EvoResponse()
            response.ate = ate
            response.rate = rate
            response.ref_traj_duration = ref_duration
            response.est_traj_duration = est_duration
            response.traj_img = msg
        else:
            dataset_name = req.dataset_name
            poses = req.poses
            traj_est = poses_to_trajectory(poses=poses)
            traj_by_label = {"estimation": traj_est}
            img = vis.get_traj_img(traj_by_label)
            msg = br.cv2_to_imgmsg(img)

            est_duration = float(
                traj_est.timestamps[-1] - traj_est.timestamps[0]
            )

            response = EvoResponse()
            # response.ate = ate
            # response.rate = rate
            # response.ref_traj_duration = ref_duration
            response.est_traj_duration = est_duration
            response.traj_img = msg
            pass

        return response

    s = rospy.Service(topic_name, Evo, evo)
    rospy.spin()
