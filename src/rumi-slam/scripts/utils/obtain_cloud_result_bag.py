import os
import rospy
import rosbag
import actionlib

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from cloud_edge_slam.msg import (
    Sequence,
    CloudMap,
    CloudSlamAction,
    CloudSlamGoal,
    CloudSlamResult,
)


if __name__ == "__main__":
    rospy.init_node("obtain_cloud_result")

    input_image_bag_path = "/media/red0orange/Project/short-2022-10-12-19-58-22.bag"
    output_map_bag_path = (
        "/media/red0orange/Project/short-2022-10-12-19-58-22_droid_map.bag"
    )
    cloud_topic_name = "/cloud_slam_temp"
    # image_topic_name = "/camera/rgb/image_color"
    image_topic_name = "/camera/color/image_raw"
    # image_topic_name = "/test_cloud_images"
    map_topic_name = "/test_cloud_map"

    image_width = 640
    image_height = 480
    # fx, fy, cx, cy = (
    #     605.9178466796875,
    #     604.9317016601562,
    #     321.8630676269531,
    #     248.41847229003906,
    # )  # Real Temp
    fx, fy, cx, cy = (
        611.892944,
        611.850098,
        327.753174,
        242.320953,
    )  # Real Temp
    # fx, fy, cx, cy = 520.908620, 521.007327, 325.141442, 249.701764  # TUM2
    # fx, fy, cx, cy = 535.4, 539.2, 320.1, 247.6  # TUM3

    input_bag = rosbag.Bag(input_image_bag_path, "r")
    output_bag = rosbag.Bag(output_map_bag_path, "w")

    client = actionlib.SimpleActionClient(cloud_topic_name, CloudSlamAction)
    client.wait_for_server()

    goal = CloudSlamGoal()

    seq = Sequence()
    camera_info = CameraInfo()
    camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_info.distortion_model = "plumb_bob"
    camera_info.width = 640
    camera_info.height = 480
    camera_info.binning_x = 0
    camera_info.binning_y = 0
    seq.camera = camera_info
    i = 0
    for topic, msg, t in input_bag.read_messages(topics=[image_topic_name]):
        print("Adding {} image".format(i))
        i += 1
        if i % 10 == 0:
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

    input_bag.close()
    output_bag.close()
    pass
