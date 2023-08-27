# source /media/edward/Data/Workspace/LAB/edgecloud/test_ws/devel/setup.bash
# export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/media/edward/Data/Workspace/LAB/edgecloud/test_ws
from numpy import dtype
import numpy as np
import cv2
# ros
import rospy
import actionlib
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ros_image
from sensor_msgs.msg import CameraInfo
# custom
from msg_action.msg import Sequence
from msg_action.msg import CloudSlamAction, CloudSlamGoal, CloudSlamResult


if __name__ == "__main__":
    rospy.init_node("edge")
    client = actionlib.SimpleActionClient("cloudslam", CloudSlamAction)
    # Sending goals before the action server comes up would be useless.
    # This line waits until we are connected to the action server.
    client.wait_for_server()

    # image sequence
    msg_sequence = Sequence()
    
    msg_sequence.timestamps.append(1.)
    msg_sequence.timestamps.append(2.)

    bridge = CvBridge()
    # msg_images.image.append([10,10])
    msg_sequence.images.append(bridge.cv2_to_imgmsg(np.ones([50,50], dtype=np.uint8), 'mono8'))
    # msg_images.image.append(bridge.cv2_to_imgmsg(np.ones([50,50], dtype=np.uint8)*2, 'mono8'))
    # msg_images.image.append(bridge.cv2_to_imgmsg(np.ones([50,50], dtype=np.uint8)*2, 'mono8'))
    
    # cam info
    msg_sequence.intrinsics.K = [404.005825, 0.000000, 335.580380,
        0.000000, 404.368809, 250.727020,
        0.000000, 0.000000, 1.000000]

    # goal
    goal = CloudSlamGoal()
    goal.sequence = msg_sequence
    
    # send goal
    client.send_goal(goal)

    # wait result
    client.wait_for_result()
    print(client.get_result().points)
