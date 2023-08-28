import os
import numpy
import numpy as np
import rospy
import rosbag

if __name__ == "__main__":
    # bag info
    bag_path = (
        "/home/red0orange/Data/TUM/rgbd_dataset_freiburg2_pioneer_slam2_cut_10-40s.bag"
    )
    cloud_map_topic_name = "/camera/rgb/image_color"
    origin_txt_path = (
        "/home/red0orange/Data/TUM/rgbd_dataset_freiburg2_pioneer_slam2/rgb.txt"
    )
    output_txt_path = "/home/red0orange/Data/TUM/rgbd_dataset_freiburg2_pioneer_slam2/rgbd_dataset_freiburg2_pioneer_slam2_10-40s.txt"

    bag_keyframe_timestamps = []
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages(topics=[cloud_map_topic_name]):
        bag_keyframe_timestamps.append(msg.header.stamp.to_time())
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
