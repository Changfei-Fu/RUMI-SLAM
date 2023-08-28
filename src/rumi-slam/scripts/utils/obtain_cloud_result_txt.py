import os
import cv2
import numpy as np
from turtle import goto
import rospy
import rosbag
import actionlib

import cv_bridge
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image
from cloud_edge_slam.msg import (
    Sequence,
    CloudMap,
    CloudSlamAction,
    CloudSlamGoal,
    CloudSlamResult,
)


if __name__ == "__main__":
    rospy.init_node("obtain_cloud_result")

    input_txt_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure_2/16-20s_1s_sample.txt"
    data_path = os.path.dirname(input_txt_path)
    ori_data = np.loadtxt(input_txt_path, dtype=np.object)

    output_map_bag_path = "/home/red0orange/下载/cloud_no_downsample.bag"
    cloud_topic_name = "/cloud_slam_temp"
    # cloud_topic_name = "/cloud_slam"
    map_topic_name = "/test_cloud_map"

    image_width = 640
    image_height = 480
    # image_height = 360
    # fx, fy, cx, cy = 457.5, 457.0, 319.7578, 173.1597  # Real
    # fx, fy, cx, cy = 520.908620, 521.007327, 325.141442, 249.701764  # TUM2
    fx, fy, cx, cy = 535.4, 539.2, 320.1, 247.6  # TUM3

    output_bag = rosbag.Bag(output_map_bag_path, "w")

    br = cv_bridge.CvBridge()
    client = actionlib.SimpleActionClient(cloud_topic_name, CloudSlamAction)
    client.wait_for_server()

    goal = CloudSlamGoal()

    seq = Sequence()
    camera_info = CameraInfo()
    camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_info.distortion_model = "plumb_bob"
    camera_info.width = image_width
    camera_info.height = image_height
    camera_info.binning_x = 0
    camera_info.binning_y = 0
    seq.camera = camera_info
    i = 0
    for time_stamp, image_path in ori_data:
        print("Adding {} image".format(i))
        i += 1

        img = cv2.imread(os.path.join(data_path, image_path))
        assert img.shape[0] != 0
        msg = br.cv2_to_imgmsg(img)
        msg.header.stamp = rospy.Time.from_sec(float(time_stamp))
        msg.encoding = "rgb8"

        seq.images.append(msg)
        seq.timestamps.append(msg.header.stamp.to_sec())
        pass

    goal.sequence = seq
    print("Begin Send Goal")
    client.send_goal(goal)
    print("End Send Goal")
    client.wait_for_result()
    result = client.get_result()

    map_msg = result.map
    output_bag.write(map_topic_name, map_msg)

    output_bag.close()
    pass
