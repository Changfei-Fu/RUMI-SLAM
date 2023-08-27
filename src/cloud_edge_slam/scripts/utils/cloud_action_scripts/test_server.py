import numpy as np
import cv2  # import before cvbridge
import time

# ros
import rospy
import actionlib
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ros_image
from sensor_msgs.msg import CameraInfo

# custom
from cloud_edge_slam.msg import CloudSlamAction, CloudSlamResult


def func(server, goal):
    rospy.loginfo(len(goal.sequence.images))
    for i in range(len(goal.sequence.images)):
        img = bridge.imgmsg_to_cv2(goal.sequence.images[i], "bgr8")
        # cv2.imwrite("img_{0}.png".format(i), img)
        print(img.shape)

    time_s = time.time()
    # @note Todo
    result = CloudSlamResult()
    print("Time for constructing cloud map", time.time() - time_s)

    print(len(result.map.key_frames))
    print(len(result.map.map_points))
    print(goal.sequence.camera.K)
    print(type(goal.sequence.camera.K))
    print(goal.sequence.camera.K[0])

    server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("cloud_aaa")

    bridge = CvBridge()

    server = actionlib.SimpleActionServer(
        "/cloud_slam", CloudSlamAction, lambda x: func(server, x), False
    )
    server.start()
    rospy.loginfo("Slam on cloud starts running.")

    rospy.spin()
