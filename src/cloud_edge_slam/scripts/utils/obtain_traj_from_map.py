import rospy
import rosbag

import tf
import numpy as np
import tf.transformations as T


if __name__ == "__main__":
    map_bag_path = "/media/red0orange/Project/short-2022-10-12-19-58-22_droid_map.bag"
    output_traj_txt_path = (
        "/media/red0orange/Project/short-2022-10-12-19-58-22_droid_map_traj.txt"
    )

    map = None
    bag = rosbag.Bag(map_bag_path, "r")
    for topic, msg, t in bag.read_messages():
        map = msg
        break
    if map is None:
        raise BaseException("Error")

    f = open(output_traj_txt_path, "w")
    timestamp_list = []
    Twc_list = []
    Tcw_list = []
    for keyframe in map.key_frames:
        Tcw = T.quaternion_matrix(
            quaternion=[getattr(keyframe.pose_cw.orientation, i) for i in "xyzw"]
        )
        Tcw[:3, -1] = [getattr(keyframe.pose_cw.position, i) for i in "xyz"]
        Twc = T.inverse_matrix(Tcw)
        Tcw_list.append(Tcw)
        Twc_list.append(Twc)
        timestamp_list.append(keyframe.mTimeStamp)

        trans = Twc[:3, -1]
        quat = T.quaternion_from_matrix(Twc)
        quat = quat[[1, 2, 3, 0]]
        f.write(
            "{:.6f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f}".format(
                keyframe.mTimeStamp, *trans, *quat
            )
            + "\n"
        )

    f.close()
    pass
