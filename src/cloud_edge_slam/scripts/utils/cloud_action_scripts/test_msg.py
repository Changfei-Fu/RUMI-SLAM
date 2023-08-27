import os
import numpy as np
import time

# import rospy
# import rosbag

from msg_action.msg import CloudMap
from msg_action.msg import KeyFrame
from msg_action.msg import MapPoint
from msg_action.msg import Descriptor
from msg_action.msg import KeyPoint
from msg_action.msg import Observation
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


def return_msg_map():
    time_1 = time.time()
    data_root = "/home/guangcheng/Workspace/edgecloud/output_all_frames"
    dataset_name = "tum_rgbd_dataset_freiburg2_pioneer_360_long_stride02"
    dataset_name = "tum_rgbd_dataset_freiburg3_structure_texture_near_all"
    dataset_name = "tum_rgbd_dataset_freiburg3_structure_texture_near_merge"


    # data_root = "/home/guangcheng/Workspace/edgecloud/output_for_construct_map"
    # dataset_name = "icl_living_room_traj3_frei_png_stride02"
    # dataset_name = "tum_rgbd_dataset_freiburg1_teddy_stride02"
    # dataset_name = "tum_rgbd_dataset_freiburg3_structure_texture_near_stride01"
    
    sum_time = 0
    sum_time_s = time.time()

    time_data = np.load(os.path.join(data_root, dataset_name + "_tstamps.npy"))
    pose_data = np.load(os.path.join(data_root, dataset_name + "_poses.npy"))
    kf_index_data = np.load(os.path.join(data_root, dataset_name + "_indexes.npy"))
    kf_index_data = {
        i: j for i, j in zip(range(kf_index_data.size), kf_index_data.tolist())
    }
    inv_kf_index_data = {j: i for i, j in kf_index_data.items()}

    map_point_xyz_data = np.load(
        os.path.join(data_root, dataset_name + "_mpt_downsampled.npy")
    )
    kf_map_point_uv = np.load(
        os.path.join(data_root, dataset_name + "_cam_mpt_uv_downsampled.npy")
    )

    preprocess_time = 0
    preprocess_time_s = time.time()
    # # 预处理
    # kf_map_point_uv[:, 0] = list(
    #     map(
    #         lambda i: inv_kf_index_data[i],
    #         kf_map_point_uv[:, 0].astype(np.int16).tolist(),
    #     )
    # )

    # # 部分map point没有被看到，这里预处理掉
    # map_point_index_map = {}
    # new_map_point_xyz_data = []
    # for map_point_i in range(map_point_xyz_data.shape[0]):
    #     cov_mappoint_infos = kf_map_point_uv[kf_map_point_uv[:, 1] == map_point_i]
    #     if cov_mappoint_infos.shape[0] == 0:
    #         print("unobserved points found")
    #         continue
    #     new_map_point_xyz_data.append(map_point_xyz_data[map_point_i].tolist())
    #     map_point_index_map[map_point_i] = len(new_map_point_xyz_data) - 1
    # map_point_xyz_data = np.array(new_map_point_xyz_data)
    # kf_map_point_uv[:, 1] = list(
    #     map(
    #         lambda i: map_point_index_map[i],
    #         kf_map_point_uv[:, 1].astype(np.int16).tolist(),
    #     )
    # )
    # preprocess_time_e = time.time()
    # preprocess_time = preprocess_time_e - preprocess_time_s
    print("preprocess time: {}".format(preprocess_time))

    pub_cloud_map = CloudMap()
    pub_cloud_map.edge_front_map_mnid = 0
    pub_cloud_map.edge_back_map_mnid = 0
    pub_cloud_map.header = Header()

    keyframe_time = 0
    keyframe_time_s = time.time()

    ros_keyframes = []
    max_cov_mappoint_size = int((kf_map_point_uv.shape[0] / time_data.shape[0]) * 5)
    cov_mappoint_infos_list = [
        ([None] * max_cov_mappoint_size) for i in range(map_point_xyz_data.shape[0])
    ]
    cov_mappoint_infos_list_index = [0] * map_point_xyz_data.shape[0]
    for keyframe_i in range(time_data.shape[0]):
        # time stamp
        time_stamp = time_data[keyframe_i]
        ros_time_stamp = time_stamp

        # pose
        pose = pose_data[keyframe_i]
        ros_pose = Pose(
            position=Point(x=pose[0], y=pose[1], z=pose[2]),
            orientation=Quaternion(x=pose[3], y=pose[4], z=pose[5], w=pose[6]),
        )

        # cur no descriptor

        # keypoint
        ros_keypoints = []
        kf_observations = kf_map_point_uv[kf_map_point_uv[:, 0] == keyframe_i]
        map_point_index = kf_observations[:, 1].astype(np.int16)
        for keypoint_i in range(kf_observations.shape[0]):
            ros_keypoint = KeyPoint()
            ros_keypoint.x = kf_observations[keypoint_i][2]
            ros_keypoint.y = kf_observations[keypoint_i][3]
            tmp_map_point_index = int(kf_observations[keypoint_i][1])
            try:
                cov_mappoint_infos_list[tmp_map_point_index][
                    cov_mappoint_infos_list_index[tmp_map_point_index]
                ] = kf_observations[keypoint_i]
                cov_mappoint_infos_list_index[tmp_map_point_index] += 1
            except:
                raise BaseException("max_cov_mappoint_size 不够大")
            ros_keypoints.append(ros_keypoint)

        ros_keyframe = KeyFrame()
        ros_keyframe.mTimeStamp = float(ros_time_stamp)
        ros_keyframe.mnId = keyframe_i  # TODO 最好这部分也传回来
        ros_keyframe.pose_cw = ros_pose
        ros_keyframe.descriptors = []  # TODO descriptors
        ros_keyframe.key_points = ros_keypoints
        ros_keyframe.mvp_map_points_index = map_point_index.tolist()

        ros_keyframes.append(ros_keyframe)
        pass

    keyframe_time_e = time.time()
    keyframe_time = keyframe_time_e - keyframe_time_s
    print("keyframe time: {}".format(keyframe_time))

    map_point_time = 0
    map_point_time_s = time.time()
    ros_mappoints = []
    for map_point_i in range(map_point_xyz_data.shape[0]):
        # point
        map_data = map_point_xyz_data[map_point_i]
        ros_point = Point(x=map_data[0], y=map_data[1], z=map_data[2])

        # observations
        ros_observations = []
        refer_keyframe_id = -1  # TODO 只用新数据就没有得到这个

        # cov_mappoint_infos = kf_map_point_uv[kf_map_point_uv[:, 1] == map_point_i]
        cov_mappoint_infos = cov_mappoint_infos_list[map_point_i]
        for (
            info
        ) in cov_mappoint_infos:  # TODO 目前共视图缺乏2D点匹配uv，无法添加其他keyframe的Observation
            if info is None:
                break
            ros_observation = Observation()
            keyframe_id = int(info[0])
            refer_keyframe_id = keyframe_id  # TODO change
            keyframe = ros_keyframes[keyframe_id]
            refer_keypoint_index = keyframe.mvp_map_points_index.index(map_point_i)
            ros_observation.keyframe_id = keyframe_id
            ros_observation.refer_keypoint_index = refer_keypoint_index
            ros_observations.append(ros_observation)

        # 如果没有被任何一帧看到，跳过该mappoint，但是不能在这里跳过，因为会打乱keyframe的map point index
        # 所以只能去读取的地方continue
        # if refer_keyframe_id == -1:
        #     continue

        ros_mappoint = MapPoint()
        ros_mappoint.mnId = map_point_i
        ros_mappoint.point = ros_point
        ros_mappoint.num_obs = len(ros_observations)
        ros_mappoint.observations = ros_observations
        ros_mappoint.ref_keyframe_id = refer_keyframe_id

        ros_mappoints.append(ros_mappoint)
        pass

    map_point_time_e = time.time()
    map_point_time = map_point_time_e - map_point_time_s
    print("map point time: {}".format(map_point_time))

    pub_cloud_map.key_frames = ros_keyframes
    pub_cloud_map.map_points = ros_mappoints

    sum_time_e = time.time()
    sum_time = sum_time_e - sum_time_s
    print("sum time: {}".format(sum_time))

    return pub_cloud_map