import os
import time
import psutil
import cv2

import cv_bridge
import cloud_edge_slam.msg
import rospy
import sensor_msgs.msg
import rosbag

import numpy as np


def get_process_pid(process_name):
    process = os.popen("ps -A | grep %s" % process_name)
    process_info = process.read()
    result = []
    for i in process_info.split(" "):
        if i != "":
            result.append(i)
    pid = int(result[0])
    return pid


if __name__ == "__main__":
    process_name = "stat_normal_bandwidth"
    pid = os.getpid()

    dataset_infos = [
        # TUM
        ["slam-tum/rgbd_dataset_freiburg1_floor", 6], 
        ["slam-tum/rgbd_dataset_freiburg1_room", 12], 
        ["slam-tum/rgbd_dataset_freiburg1_teddy", 10], 
        ["slam-tum/rgbd_dataset_freiburg2_desk", 4], 
        ["slam-tum/rgbd_dataset_freiburg2_pioneer_360", 10], 
        ["slam-tum/rgbd_dataset_freiburg2_pioneer_slam", 8], 
        # ["slam-tum/rgbd_dataset_freiburg2_pioneer_slam2", 10], 
        # ["slam-tum/rgbd_dataset_freiburg2_pioneer_slam3", 10], 
        ["slam-tum/rgbd_dataset_freiburg3_teddy", 6], 
        # # Euroc
        # ["slam-euroc/MH04", 10], 
        # ["slam-euroc/MH05", 10], 
        # ["slam-euroc/V102", 10], 
        # ["slam-euroc/V103", 10], 
        # ["slam-euroc/V201", 10], 
        # ["slam-euroc/V202", 10], 
        # ["slam-euroc/V203", 10], 
        # # ICL
        # ["slam-icl/living_room_traj0_frei_png", 0.01], 
        # ["slam-icl/living_room_traj1_frei_png", 10], 
        # ["slam-icl/living_room_traj2_frei_png", 10], 
        # ["slam-icl/living_room_traj3_frei_png", 10], 
        # ["slam-icl/traj0_frei_png", 10], 
        # ["slam-icl/traj1_frei_png", 10], 
        # ["slam-icl/traj2_frei_png", 10], 
        # ["slam-icl/traj3_frei_png", 10], 
    ]

    data_dir = "/media/red0orange/Data/数据集"
    sampling_dir = "/media/red0orange/Project/CloudEdgeSLAM/tum_result"
    gt_dir = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/groundtruth"
    data_dict = {
        os.path.basename(i[0]) : [os.path.join(data_dir, i[0]), os.path.join(sampling_dir, os.path.basename(i[0]), "result_dynamic_" + str(i[1]).zfill(2) + ".txt"), 
                os.path.join(gt_dir, i[0], "rgb.txt"), os.path.join(sampling_dir, str(os.path.basename(i[0])) + ".bag")] for i in dataset_infos
    }

    cur_dataset_name = "rgbd_dataset_freiburg2_pioneer_360"

    cur_mode = "nodownsample"
    # cur_mode = "downsample"

    image_dir = data_dict[cur_dataset_name][0]
    input_downsample_txt_path = data_dict[cur_dataset_name][1]
    input_gt_txt_path = data_dict[cur_dataset_name][2]
    output_bag_path = data_dict[cur_dataset_name][3]

    downsample_image_paths = np.loadtxt(input_downsample_txt_path, dtype=np.object)
    downsample_image_paths = [os.path.join(image_dir, "rgb", i + ".png") for i in downsample_image_paths]
    gt_timestamps = np.loadtxt(input_gt_txt_path, dtype=np.object)[:, 0]
    gt_timestamps = gt_timestamps.astype(np.float64)
    gt_image_paths = np.loadtxt(input_gt_txt_path, dtype=np.object)[:, 1]
    gt_image_paths = [os.path.join(image_dir, i) for i in gt_image_paths]

    duration = gt_timestamps[-1] - gt_timestamps[0]
    if cur_mode == "nodownsample":
        image_paths = gt_image_paths
    elif cur_mode == "downsample":
        image_paths = downsample_image_paths
    else:
        raise Exception()

    output_bag = rosbag.Bag(output_bag_path, "w")
    process_memory = []
    br = cv_bridge.CvBridge()
    for image_path in image_paths:
        image = cv2.imread(image_path)
        msg = br.cv2_to_imgmsg(image)
        msg.encoding = "rgb8"
        output_bag.write("/test_images", msg)

        memory = psutil.Process(pid).memory_info().rss / 1024 / 1024 / 1024
        process_memory.append(memory)
        pass

    output_bag.close()
    bag_size = int(os.stat(output_bag_path).st_size / (1024 * 1024))

    memory = psutil.Process(pid).memory_info().rss / 1024 / 1024 / 1024
    process_memory.append(memory)

    print("=================================")
    print("Max memory: {}".format(max(process_memory)))
    print("Speed: {}".format(bag_size / duration))
    print("Bag size: {}".format(bag_size))
    print("Traj duration: {}".format(duration))
    print("=================================")
    pass