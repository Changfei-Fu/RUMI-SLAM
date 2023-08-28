import os
import numpy as np

import rospy
import rosbag

from cloud_edge_slam.msg import CloudMap
from cloud_edge_slam.msg import KeyFrame
from cloud_edge_slam.msg import MapPoint
from cloud_edge_slam.msg import Descriptor
from cloud_edge_slam.msg import KeyPoint
from cloud_edge_slam.msg import Observation
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


if __name__ == '__main__':
    rospy.init_node("test_export_dvoid_output")

    # data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/TestData/DvoidSLAMOuput-Teddy/"
    # data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/TestData/DvoidSLAMOutput-structure_texture_near"
    data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/TestData/0616DroidOutput-2"
    dataset_name = "tum_rgbd_dataset_freiburg3_structure_texture_near_stride01"
    # dataset_name = "tum_rgbd_dataset_freiburg1_teddy_stride02"

    time_data = np.load(os.path.join(data_root, dataset_name + "_tstamps.npy"))
    pose_data = np.load(os.path.join(data_root, dataset_name + "_poses.npy"))
    kf_index_data = np.load(os.path.join(data_root, dataset_name + "_indexes.npy"))
    kf_index_data = {i: j for i, j in zip(range(kf_index_data.size), kf_index_data.tolist())}
    inv_kf_index_data = {j: i for i, j in kf_index_data.items()}

    kf_map_point_uv = np.load(os.path.join(data_root, dataset_name + "_cam_mpt_uv.npy"))
    map_point_idx_xyz_uv_data = np.load(os.path.join(data_root, dataset_name + "_idx_xyz_duv.npy"))
    # map_point_idx_xyz_uv_data = np.load(os.path.join(data_root, dataset_name + "_mpt_downsampled.npy"))
    map_point_indexes = np.arange(map_point_idx_xyz_uv_data.shape[0])

    # data preprocess
    # TODO 主要是将mappoint的两个信息矩阵中的keyframe index转换成列表的index使用
    map_point_idx_xyz_uv_data[:, 0] = [inv_kf_index_data[i] for i in map_point_idx_xyz_uv_data[:, 0]]
    # kf_map_point_uv[:, 0] = [inv_kf_index_data[i] for i in kf_map_point_uv[:, 0]]
    # uv坐标处理为int
    kf_map_point_uv[:, 2] = np.round(kf_map_point_uv[:, 2])
    kf_map_point_uv[:, 3] = np.round(kf_map_point_uv[:, 3])

    test_set = set()
    for keypoint in kf_map_point_uv:
        test_set.add("{}_{}_{}".format(keypoint[0], keypoint[2], keypoint[3]))
    print("ratio: {} / {}".format(len(test_set), len(kf_map_point_uv)))

    pub_cloud_map = CloudMap()
    pub_cloud_map.edge_front_map_mnid = 0
    pub_cloud_map.edge_back_map_mnid = 0
    pub_cloud_map.header = Header()

    ros_keyframes = []
    for keyframe_i in range(time_data.shape[0]):
        # time stamp
        time_stamp = time_data[keyframe_i]
        ros_time_stamp = time_stamp

        # pose
        pose = pose_data[keyframe_i]
        ros_pose = Pose(position=Point(x=pose[0], y=pose[1], z=pose[2]), orientation=Quaternion(x=pose[3], y=pose[4], z=pose[5], w=pose[6]))

        # cur no descriptor

        # keypoint
        ros_keypoints = []
        # old
        map_point_infos = map_point_idx_xyz_uv_data[map_point_idx_xyz_uv_data[:, 0] == keyframe_i]
        map_point_index = map_point_indexes[map_point_idx_xyz_uv_data[:, 0] == keyframe_i]
        for keypoint_i in range(map_point_infos.shape[0]):
            ros_keypoint = KeyPoint()
            ros_keypoint.x = map_point_infos[keypoint_i][-2]
            ros_keypoint.y = map_point_infos[keypoint_i][-1]
            ros_keypoints.append(ros_keypoint)
        # new
        # map_point_infos = kf_map_point_uv[kf_map_point_uv[:, 0] == keyframe_i]
        # map_point_index = map_point_infos[:, 1].astype(np.int16)
        # for keypoint_i in range(map_point_infos.shape[0]):
        #     ros_keypoint = KeyPoint()
        #     ros_keypoint.x = map_point_infos[keypoint_i][2]
        #     ros_keypoint.y = map_point_infos[keypoint_i][3]
        #     ros_keypoints.append(ros_keypoint)

        ros_keyframe = KeyFrame()
        ros_keyframe.mTimeStamp = ros_time_stamp
        ros_keyframe.mnId = keyframe_i   # TODO 最好这部分也传回来
        ros_keyframe.pose_cw = ros_pose
        ros_keyframe.descriptors = []  # TODO descriptors
        ros_keyframe.key_points = ros_keypoints
        ros_keyframe.mvp_map_points_index = map_point_index.tolist()

        ros_keyframes.append(ros_keyframe)
        pass

    ros_mappoints = []
    for map_point_i in range(map_point_idx_xyz_uv_data.shape[0]):
        map_data = map_point_idx_xyz_uv_data[map_point_i]

        # point
        point = map_data[1:4]
        ros_point = Point(x=point[0], y=point[1], z=point[2])

        # observations
        ros_observations = []
        refer_keyframe_id = -1  # TODO 只用新数据就没有得到这个

        keyframe_id = int(map_data[0])
        refer_keyframe_id = keyframe_id
        keyframe = ros_keyframes[keyframe_id]
        refer_keypoint_index = keyframe.mvp_map_points_index.index(map_point_i)
        ros_observation = Observation()
        ros_observation.keyframe_id = keyframe_id
        ros_observation.refer_keypoint_index = refer_keypoint_index
        ros_observations.append(ros_observation)

        # cov_mappoint_infos = kf_map_point_uv[kf_map_point_uv[:, 1] == map_point_i]
        # for info in cov_mappoint_infos:   # TODO 目前共视图缺乏2D点匹配uv，无法添加其他keyframe的Observation
        #     ros_observation = Observation()
        #     keyframe_id = int(info[0])
        #     refer_keyframe_id = keyframe_id  # TODO change
        #     keyframe = ros_keyframes[keyframe_id]
        #     refer_keypoint_index = keyframe.mvp_map_points_index.index(map_point_i)
        #     ros_observation.keyframe_id = keyframe_id
        #     ros_observation.refer_keypoint_index = refer_keypoint_index
        #     ros_observations.append(ros_observation)

        ros_mappoint = MapPoint()
        ros_mappoint.mnId = map_point_i
        ros_mappoint.point = ros_point
        ros_mappoint.num_obs = len(ros_observations)
        ros_mappoint.observations = ros_observations
        ros_mappoint.ref_keyframe_id = refer_keyframe_id

        ros_mappoints.append(ros_mappoint)
        pass

    pub_cloud_map.key_frames = ros_keyframes
    pub_cloud_map.map_points = ros_mappoints

    publisher = rospy.Publisher("/test_cloud_map", CloudMap, queue_size=1)

    r = rospy.Rate(0.2)
    while True:
        publisher.publish(pub_cloud_map)
        print("publish success!")
        r.sleep()

    rospy.spin()
    pass