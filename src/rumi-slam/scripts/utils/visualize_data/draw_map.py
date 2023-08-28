import os
import copy
import numpy as np
import matplotlib.pyplot as plt

import rospy
import tf.transformations as T
import rosbag

import pypangolin as pango
from OpenGL.GL import *

from evo.core import sync
from evo.tools import file_interface
from evo.tools import plot
from evo.core.geometry import umeyama_alignment
from evo.core import lie_algebra as lie
from evo.core.trajectory import PoseTrajectory3D


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
    quat = mat[:, 4:]  # n x 4
    quat = np.roll(quat, 1, axis=1)  # shift 1 column -> w in front column
    if not hasattr(file_path, "read"):  # if not file handle
        print("Loaded {} stamps and poses from: {}".format(len(stamps), file_path))
    return PoseTrajectory3D(xyz, quat, stamps)


class OfflineMapDrawer(object):
    def __init__(
        self, bag_path, map_topic="/test_cloud_map", gt_file_path=None
    ) -> None:

        bag = rosbag.Bag(bag_path, "r")
        for topic, msg, t in bag.read_messages(topics=[map_topic]):
            self.map = msg
            break  # only read first map
        assert self.map is not None

        self.point_size = 2.0
        self.line_width = 3.0
        self.keyframe_size = 0.04
        self.mappoint_color = (0, 0, 0)
        self.keyframe_color = (0, 0, 1)
        self.gt_color = (0, 1, 0)

        # init keyframes
        self.keyframes = self.map.key_frames
        self.mappoints = self.map.map_points
        self.timestamp_list = [keyframe.mTimeStamp for keyframe in self.map.key_frames]

        Tcw_list = []
        Twc_list = []
        for keyframe in self.keyframes:
            Tcw = T.quaternion_matrix(
                quaternion=[getattr(keyframe.pose_cw.orientation, i) for i in "xyzw"]
            )
            Tcw[:3, -1] = [getattr(keyframe.pose_cw.position, i) for i in "xyz"]
            Twc = T.inverse_matrix(Tcw)
            Tcw_list.append(Tcw)
            Twc_list.append(Twc)

        self.trans_list = [Twc[:3, -1] for Twc in Twc_list]
        self.quat_list = [T.quaternion_from_matrix(Twc) for Twc in Twc_list]
        # init mappoints
        self.point_list = [
            [getattr(i.point, attr) for attr in "xyz"] for i in self.mappoints
        ]

        # align
        self.gt_file_path = gt_file_path
        if gt_file_path is not None:
            self.gt_points = read_tum_trajectory_file(gt_file_path).positions_xyz
            self.align_to_gt()

        # begin draw
        self.win = pango.CreateWindowAndBind("Offline Map Viewer", 1424, 768)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        self.pm = pango.ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000)
        # self.mv = pango.ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
        # self.mv = pango.ModelViewLookAt(0, 2.0, 2.6, 0, -0.3, 0.9, 0.0, -1.0, 0.0)
        self.mv = pango.ModelViewLookAt(
            0.47839712,
            0.15000692,
            1.73222642,
            -2.51,
            0.803,
            0.1965,
            0.0,
            0.0,
            1.0,
        )
        self.s_cam = pango.OpenGlRenderState(self.pm, self.mv)

        handler = pango.Handler3D(self.s_cam)
        self.d_cam = (
            pango.CreateDisplay()
            .SetBounds(
                pango.Attach(0),
                pango.Attach(1),
                pango.Attach.Pix(175),
                pango.Attach(1),
                -1024.0 / 768.0,
            )
            .SetHandler(handler)
        )

        pango.CreatePanel("ui").SetBounds(
            pango.Attach(0), pango.Attach(1), pango.Attach(0), pango.Attach.Pix(175)
        )
        var_ui = pango.Var("ui")
        var_ui.a_Button = False

        while not pango.ShouldQuit():
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            self.d_cam.Activate(self.s_cam)
            glClearColor(1.0, 1.0, 1.0, 1.0)

            self.draw_keyframes()
            self.draw_mappoints()
            if self.gt_file_path is not None:
                self.draw_gt_trajectory()

            pango.FinishFrame()
        pass

    def align_to_gt(self):
        gt_pose_trajectory = read_tum_trajectory_file(self.gt_file_path)

        timestamps = np.array(copy.deepcopy(self.timestamp_list))
        trans = np.array(copy.deepcopy(self.trans_list))
        quats = np.array(copy.deepcopy(self.quat_list))
        quats = np.roll(quats, 1, axis=1)
        predict_pose_trajectory = PoseTrajectory3D(trans, quats, timestamps)

        max_diff = 0.01
        (
            aligned_gt_pose_trajectory,
            aligned_predict_pose_trajectory,
        ) = sync.associate_trajectories(
            gt_pose_trajectory, copy.deepcopy(predict_pose_trajectory), max_diff
        )

        r_a, t_a, s = umeyama_alignment(
            aligned_predict_pose_trajectory.positions_xyz.T,
            aligned_gt_pose_trajectory.positions_xyz.T,
            True,
        )

        se3_t = lie.se3(r_a, t_a)
        predict_pose_trajectory.scale(s)
        predict_pose_trajectory.transform(se3_t)

        # fig = plt.figure()
        # plot.trajectories(
        #     fig,
        #     {"tes1": predict_pose_trajectory, "test2": gt_pose_trajectory},
        #     plot.PlotMode.xyz,
        # )
        # plt.show()

        self.trans_list = predict_pose_trajectory.positions_xyz.tolist()
        self.quat_list = np.roll(
            predict_pose_trajectory.orientations_quat_wxyz, -1, axis=1
        ).tolist()

        point_arr = np.array(self.point_list)
        point_arr *= s
        homo_point_arr = np.concatenate(
            [point_arr, np.ones(point_arr.shape[0])[:, None]], axis=1
        )
        align_point_arr = (se3_t @ homo_point_arr.transpose()).transpose()[:, :3]
        self.point_list = align_point_arr.tolist()
        pass

    def draw_keyframes(self):
        T_world_camera = self.s_cam.GetModelViewMatrix().Inverse()
        print(T_world_camera.Matrix())

        trans_list = self.trans_list
        quat_list = self.quat_list

        w = self.keyframe_size
        h = w * 0.75
        z = w * 0.6

        for i in range(len(trans_list)):
            trans = trans_list[i]
            quat = quat_list[i]
            Twc = T.quaternion_matrix(quat)
            Twc[:3, -1] = trans

            glPushMatrix()
            glMultMatrixf(np.transpose(Twc))
            # test_T = np.identity(4)
            # test_T[1, -1] += i
            # glMultMatrixf(test_T)

            glLineWidth(self.line_width)
            glColor3f(*self.keyframe_color)
            glBegin(GL_LINES)

            glVertex3f(0, 0, 0)
            glVertex3f(w, h, z)
            glVertex3f(0, 0, 0)
            glVertex3f(w, -h, z)
            glVertex3f(0, 0, 0)
            glVertex3f(-w, -h, z)
            glVertex3f(0, 0, 0)
            glVertex3f(-w, h, z)

            glVertex3f(w, h, z)
            glVertex3f(w, -h, z)

            glVertex3f(-w, h, z)
            glVertex3f(-w, -h, z)

            glVertex3f(-w, h, z)
            glVertex3f(w, h, z)

            glVertex3f(-w, -h, z)
            glVertex3f(w, -h, z)

            glEnd()
            glPopMatrix()
        pass

    def draw_mappoints(self):
        points = self.point_list

        glPointSize(self.point_size)
        glBegin(GL_POINTS)
        glColor3f(*self.mappoint_color)

        for i in range(len(points)):
            point = points[i]
            glVertex3f(*point)

        glEnd()
        pass

    def draw_gt_trajectory(self):
        for i in range(len(self.gt_points) - 1):
            glColor3f(*self.gt_color)
            glBegin(GL_LINES)
            point1 = self.gt_points[i]
            point2 = self.gt_points[i + 1]
            glVertex3f(*point1)
            glVertex3f(*point2)
            glEnd()
        pass


if __name__ == "__main__":
    # bag_path = "/home/red0orange/下载/cloud.bag"
    # bag_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure_2/edge_front.bag"
    # bag_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure_2/edge_back.bag"
    # bag_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure_2/cloud.bag"
    bag_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure_2/cloud_no_downsample.bag"
    # bag_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure_2/merge.bag"
    # bag_path = "/home/red0orange/下载/cloud_500-1000.bag"
    # bag_path = "/home/red0orange/文档/tmp-2/merge.bag"
    # bag_path = "/home/red0orange/文档/tmp-2/edge-front.bag"
    # bag_path = "/home/red0orange/文档/tmp-2/edge-back.bag"
    # bag_path = "/home/red0orange/文档/tmp-2/cloud.bag"
    # bag_path = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/TestData/All_ORB_Test_Offline_Map/texture/offline-merge-result-voxel.bag"
    # bag_path = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/TestData/0804-drovid-output/rgbd_dataset_freiburg3_structure_texture_near_merge_none_test_new_2.bag"

    # gt_file_path = None
    # gt_file_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/CloudMergeResults/groundtruth_30s.txt"
    # gt_file_path = "/home/red0orange/文档/tmp-2/groundtruth.txt"
    gt_file_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure/groundtruth0-30s.txt"
    OfflineMapDrawer(
        bag_path=bag_path, map_topic="/test_cloud_map", gt_file_path=gt_file_path
    )
    # OfflineMapDrawer(bag_path=bag_path, map_topic="/test_cloud_map")
    pass
