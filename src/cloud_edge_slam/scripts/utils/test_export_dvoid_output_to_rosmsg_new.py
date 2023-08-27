import os
import time
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


if __name__ == "__main__":
    rospy.init_node("test_export_dvoid_output")

    # data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/TestData/DvoidSLAMOuput-Teddy/"
    # data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/TestData/DvoidSLAMOutput-structure_texture_near"
    # data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/TestData/0616DroidOutput-2"
    # dataset_name = "tum_rgbd_dataset_freiburg3_structure_texture_near_stride01"
    # data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/TestData/0715DroidOutput"
    # dataset_name = "tum_rgbd_dataset_freiburg3_structure_texture_near_merge"

    data_root = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/TestData/0804-drovid-output/rgbd_dataset_freiburg3_structure_texture_near_merge"
    data_downsample_method = None
    # data_downsample_method = "feature"
    # data_downsample_method = "voxel"
    # data_downsample_method = "feature_voxel"
    # data_downsample_method = "voxel_feature"

    sum_time = 0
    sum_time_s = time.time()

    time_data = np.load(os.path.join(data_root, "tstamps.npy"))
    pose_data = np.load(os.path.join(data_root, "poses.npy"))
    kf_index_data = np.load(os.path.join(data_root, "indexes.npy"))
    kf_index_data = {
        i: j for i, j in zip(range(kf_index_data.size), kf_index_data.tolist())
    }
    inv_kf_index_data = {j: i for i, j in kf_index_data.items()}

    # map_point_xyz_data = np.load(os.path.join(data_root, dataset_name + "_mpt.npy"))
    # kf_map_point_uv = np.load(os.path.join(data_root, dataset_name + "_cam_mpt_uv.npy"))

    # map_point_xyz_data = np.load(os.path.join(data_root, dataset_name + "_mpt-YAL.npy"))
    # kf_map_point_uv = np.load(
    #     os.path.join(data_root, dataset_name + "_cam_mpt_uv-YAL.npy")
    # )
    map_point_xyz_data_path = os.path.join(
        data_root,
        "mpt.npy"
        if data_downsample_method is None
        else "mpt_{}.npy".format(data_downsample_method),
    )
    map_point_xyz_data = np.load(map_point_xyz_data_path)
    kf_map_point_data_path = os.path.join(
        data_root,
        "cam_mpt_uv.npy"
        if data_downsample_method is None
        else "cam_mpt_uv_{}.npy".format(data_downsample_method),
    )
    kf_map_point_uv = np.load(kf_map_point_data_path)

    # map_point_xyz_data = np.load(
    #     os.path.join(data_root, dataset_name + "_mpt_downsampled-YAL.npy")
    # )
    # kf_map_point_uv = np.load(
    #     os.path.join(data_root, dataset_name + "_cam_mpt_uv_downsampled-YAL.npy")
    # )

    # tmp_kf_map_point_uv = np.load(
    #     os.path.join(data_root, dataset_name + "_idx_xyz_uv-YAL.npy")
    # )
    # tmp_kf_map_point_uv_2 = np.load(
    #     os.path.join(data_root, dataset_name + "_idx_xyz_duv.npy")
    # )

    # print(tmp_kf_map_point_uv.shape)
    # print(len(set(tmp_kf_map_point_uv[:, 1])))
    # print(len(set(tmp_kf_map_point_uv[:, 2])))
    # print(len(set(tmp_kf_map_point_uv[:, 3])))

    # print(tmp_kf_map_point_uv_2.shape)
    # print(len(set(tmp_kf_map_point_uv_2[:, 1])))
    # print(len(set(tmp_kf_map_point_uv_2[:, 2])))
    # print(len(set(tmp_kf_map_point_uv_2[:, 3])))

    preprocess_time = 0
    preprocess_time_s = time.time()

    preprocess_index_time = 0
    # 预处理
    kf_map_point_uv[:, 0] = list(
        map(
            lambda i: inv_kf_index_data[i],
            kf_map_point_uv[:, 0].astype(np.int32).tolist(),
        )
    )

    # # 部分map point没有被看到，这里预处理掉
    # map_point_index_map = {}
    # new_map_point_xyz_data = []
    # for map_point_i in range(map_point_xyz_data.shape[0]):
    #     preprocess_index_time_s = time.time()
    #     cov_mappoint_infos = kf_map_point_uv[kf_map_point_uv[:, 1] == map_point_i]
    #     preprocess_index_time_e = time.time()
    #     preprocess_index_time += preprocess_index_time_e - preprocess_index_time_s
    #     if cov_mappoint_infos.shape[0] == 0:
    #         continue
    #     new_map_point_xyz_data.append(map_point_xyz_data[map_point_i].tolist())
    #     map_point_index_map[map_point_i] = len(new_map_point_xyz_data) - 1
    # map_point_xyz_data = np.array(new_map_point_xyz_data)
    # kf_map_point_uv[:, 1] = list(
    #     map(
    #         lambda i: map_point_index_map[i],
    #         kf_map_point_uv[:, 1].astype(np.int32).tolist(),
    #     )
    # )
    # preprocess_time_e = time.time()
    # preprocess_time = preprocess_time_e - preprocess_time_s
    # print("preprocess time: {}".format(preprocess_time))
    # print("preprocess index time: {}".format(preprocess_index_time))

    pub_cloud_map = CloudMap()
    pub_cloud_map.edge_front_map_mnid = 0
    pub_cloud_map.edge_back_map_mnid = 0
    pub_cloud_map.header = Header()

    keyframe_time = 0
    keyframe_time_s = time.time()

    ros_keyframes = []
    cov_mappoint_infos_list = np.full(
        fill_value=np.nan,
        dtype=np.float16,
        shape=(
            map_point_xyz_data.shape[0],
            time_data.shape[0],
            kf_map_point_uv.shape[1],
        ),
    )
    cov_mappoint_infos_list_index = np.full(
        fill_value=0, dtype=np.int32, shape=(map_point_xyz_data.shape[0])
    )
    # cov_mappoint_infos_list = [
    #     ([None] * max_cov_mappoint_size) for i in range(map_point_xyz_data.shape[0])
    # ]
    # cov_mappoint_infos_list_index = [0] * map_point_xyz_data.shape[0]
    for keyframe_i in range(time_data.shape[0]):
        print("Process KeyFrame: {} / {}".format(keyframe_i, time_data.shape[0]))
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
        map_point_index = kf_observations[:, 1].astype(np.int32)
        for keypoint_i in range(kf_observations.shape[0]):
            ros_keypoint = KeyPoint()
            ros_keypoint.x = kf_observations[keypoint_i][2]
            ros_keypoint.y = kf_observations[keypoint_i][3]
            tmp_map_point_index = int(kf_observations[keypoint_i][1])
            # if 1:
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
        print(
            "Process MapPoint: {} / {}".format(map_point_i, map_point_xyz_data.shape[0])
        )
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
            # if info is None:
            if np.isnan(info[0]):
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

    publisher = rospy.Publisher("/test_cloud_map", CloudMap, queue_size=1)

    r = rospy.Rate(0.2)
    while True:
        publisher.publish(pub_cloud_map)
        print("publish success!")
        r.sleep()

    rospy.spin()
    pass
