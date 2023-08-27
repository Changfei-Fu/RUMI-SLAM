import os
import cv2
import numpy
import numpy as np
import rospy
import rosbag
import cv_bridge

if __name__ == "__main__":
    # bag_path = "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/test_cloud_images.bag"
    # bag = rosbag.Bag(bag_path)
    # br = cv_bridge.CvBridge()
    # for topic, msg, t in bag.read_messages():
    #     ori_image = br.imgmsg_to_cv2(msg)
    #     cv2.imshow("aaa", ori_image)
    #     cv2.waitKey(1)
    # bag.close()

    # bag info
    bag_path = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/TestData/0616-0629DroidOutput/EdgeBack.bag"
    cloud_map_topic_name = "/test_cloud_map"
    # origin data txt path
    origin_txt_path = "/home/red0orange/Data/TUM/rgbd_dataset_freiburg3_structure_texture_near/rgb.txt"
    # output txt path
    output_txt_path = "/home/red0orange/下载/rgb_cloudedge.txt"

    bag_keyframe_timestamps = []
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages(topics=[cloud_map_topic_name]):
        for keyframe in msg.key_frames:
            bag_keyframe_timestamps.append(keyframe.mTimeStamp)
    bag_keyframe_timestamps = np.array(bag_keyframe_timestamps)
    bag.close()

    origin_data = np.loadtxt(origin_txt_path, dtype=np.object).tolist()
    output_data = []
    for i in range(len(origin_data)):
        per_frame = origin_data[i]
        print(np.min(np.abs(bag_keyframe_timestamps - float(per_frame[0]))))
        if np.min(np.abs(bag_keyframe_timestamps - float(per_frame[0]))) < 0.001:
            output_data.append(origin_data[i])
    print(output_data)

    output_data = np.array(output_data, dtype=np.object)
    np.savetxt(output_txt_path, output_data, fmt="%s %s")
    pass
