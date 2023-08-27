import os
import cv2

import cv_bridge
import sensor_msgs.msg
import rosbag

import numpy as np


if __name__ == "__main__":
    # input_bag_path = "/media/red0orange/Data/数据集/CloudEdge/2022-08-09-16-51-18.bag"
    # output_bag_path = "/media/red0orange/Data/数据集/CloudEdge/2022-08-09-16-51-18_resize.bag"

    # input_bag_path = "/media/red0orange/Data/数据集/CloudEdge/2022-08-09-16-49-04.bag"
    # output_bag_path = "/media/red0orange/Data/数据集/CloudEdge/2022-08-09-16-49-04_resize.bag"

    # input_bag_path = "/media/red0orange/Data/数据集/CloudEdge/2022-08-09-16-56-00.bag"
    # output_bag_path = "/media/red0orange/Data/数据集/CloudEdge/2022-08-09-16-56-00_resize.bag"

    input_bag_path = "/media/red0orange/Elements SE/dataset/2022-10-09-17-30-37.bag"
    output_bag_path = (
        "/media/red0orange/Elements SE/dataset/2022-10-09-17-30-37-resize.bag"
    )

    input_image_size = [1280, 720]  # for check
    output_image_size = [640, 360]

    input_bag = rosbag.Bag(input_bag_path, "r")
    output_bag = rosbag.Bag(output_bag_path, "w")

    br = cv_bridge.CvBridge()
    topic_names = [
        "/camera/color/image_raw",
        "/camera/color/camera_info",
        "/vrpn_client_node/jackal2/pose",
    ]
    index = 0
    for topic, msg, t in input_bag.read_messages(topics=topic_names):
        print("Cur Process: {}".format(index))
        index += 1
        if topic == "/camera/color/image_raw":
            ori_image = br.imgmsg_to_cv2(msg)
            assert (
                ori_image.shape[0] == input_image_size[1]
                and ori_image.shape[1] == input_image_size[0]
            )
            resize_image = cv2.resize(ori_image, tuple(output_image_size))
            assert (
                resize_image.shape[0] == output_image_size[1]
                and resize_image.shape[1] == output_image_size[0]
            )
            resize_msg = br.cv2_to_imgmsg(resize_image)
            resize_msg.header = msg.header
            resize_msg.encoding = "rgb8"
            output_bag.write("/camera/rgb/image_color", resize_msg)
            pass
        else:
            output_bag.write(topic, msg)
        pass

    input_bag.close()
    output_bag.close()
    pass
