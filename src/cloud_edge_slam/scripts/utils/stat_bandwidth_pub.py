import os
import cv2
import rospy

import cv_bridge
import sensor_msgs.msg
from cloud_edge_slam.msg import Sequence
import rosbag

import numpy as np


if __name__ == "__main__":
    rospy.init_node("stat_bandwidth_pub")

    input_bag_path = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/results/Full#living_room_traj0_frei_png#10-11-20_27_59/cloud_1.bag"
    bag_size = int(os.stat(input_bag_path).st_size / (1024 * 1024))
    pub_topic_name = "stat_bandwidth_topic"

    input_bag = rosbag.Bag(input_bag_path, "r")
    pub = rospy.Publisher(pub_topic_name, Sequence, queue_size=1)

    seq_msg = Sequence()
    for topic, msg, t in input_bag.read_messages():
        seq_msg.images.append(msg)
        pass

    r = rospy.Rate(hz=0.05)
    while True:
        seq_msg.Header.stamp = rospy.Time.now()
        seq_msg.Header.seq = bag_size
        pub.publish(seq_msg)
        r.sleep()

    input_bag.close()
    pass
