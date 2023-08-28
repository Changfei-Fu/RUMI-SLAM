import numpy as np


a = np.array([1, 2, 3, 4])
# print(a[(a == [1, 2]) | (a == 2)])
print(a == [1, 2])


import os
import shutil

dir = "/media/red0orange/Data/数据集/slam-euroc"
create_dir = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/groundtruth/slam-euroc"
for dir_name in os.listdir(dir):
    # os.mkdir(os.path.join(create_dir, dir_name))
    shutil.copy(os.path.join(dir, dir_name, "rgb.txt"), os.path.join(create_dir, dir_name, "rgb.txt"))
