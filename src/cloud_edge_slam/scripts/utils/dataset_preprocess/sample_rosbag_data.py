import os
import cv2

import cv_bridge
import sensor_msgs.msg
import rosbag

import numpy as np


if __name__ == "__main__":
    input_bag_path = "/media/red0orange/Data/whole.bag"
    output_bag_path = "/media/red0orange/Data/whole_sample.bag"

    input_bag = rosbag.Bag(input_bag_path, "r")
    output_bag = rosbag.Bag(output_bag_path, "w")

    br = cv_bridge.CvBridge()
    topic_names = [
        "/camera/color/image_raw",
    ]
    i = 0
    for topic, msg, t in input_bag.read_messages(topics=topic_names):
        if topic == "/camera/color/image_raw":
            i += 1
            if i % 3 == 0:
                output_bag.write("/camera/color/image_raw", msg)
            pass
        else:
            output_bag.write(topic, msg)
        pass

    input_bag.close()
    output_bag.close()
    pass
