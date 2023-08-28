import numpy as np
import cv2
from numpy import *

np.set_printoptions(suppress=True)
from matplotlib import pyplot as plt
from scipy.signal import convolve2d as conv2
import random
from scipy.spatial import distance
import itertools
import time
from tqdm import tqdm
from utils import concat
import multiprocessing


def path_return(image_trade, file_name):
    # file_name = format(file_name, ".6f")
    # file_path = image_trade + str(file_name) + ".png"
    file_path = image_trade + str(file_name)

    return str(file_path)


def detector_defind(edgeThreshold=34, nFeatures=100000000, nLevels=7, patchSize=16):
    detector = cv2.ORB_create()
    # print("ORB detector initiated!")

    return detector


def new_points_des(xyz_duv, index_num, dirname, image_list, orb, per_vis):
    time_0_s = time.time()
    image_path = path_return(dirname, image_list[index_num])
    # print(image_path)

    time_1_s = time.time()
    image = cv2.imread(image_path)
    time_1 = time.time() - time_1_s

    time_2_s = time.time()
    # curr_index_list = np.asarray([x for x in xyz_duv if int(x[0]) == index_num]) # modified
    curr_index_list = xyz_duv[xyz_duv[:, 0].astype(np.int32) == index_num]
    pt_list = curr_index_list[:, -3:-1]
    time_2 = time.time() - time_2_s
    pt = [cv2.KeyPoint(x=uv[0], y=uv[1], size=16) for uv in pt_list]

    time_3_s = time.time()
    kpnew, desnew = orb.compute(image, pt)
    time_3 = time.time() - time_3_s

    if vis:
        im_with_keypoints = cv2.drawKeypoints(
            image,
            kpnew,
            np.array([]),
            (255, 0, 0),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        plt.imshow(im_with_keypoints), plt.show()

    time_0 = time.time() - time_0_s
    # print(time_0, time_1/time_0, time_2/time_0, time_3/time_0)
    return kpnew, desnew, image, curr_index_list, pt_list


def create_graph(visibililty_graph):
    new_visibility = []
    last_index = [int(visibililty_graph[0][0])]
    for i in visibililty_graph:
        if i[2] < 0.1:
            continue
        if i[0] == last_index[0]:
            last_index.append(int(i[1]))
        else:
            new_visibility.append(last_index)
            last_index = [int(i[0]), int(i[1])]
    new_visibility.append(last_index)

    return new_visibility


def query_index(tp, uvquery_pt_list, uv_querylist):
    # uv_list = [uv for uv in tp]
    # uv_dist_index = distance.cdist(uv_list, uvquery_pt_list)
    # print(tp.shape, uvquery_pt_list.shape)
    uv_dist_index = distance.cdist(tp, uvquery_pt_list)  # modified
    min = np.argmin(uv_dist_index, axis=1)
    pt_uv = uv_querylist[min, 1]

    return min, pt_uv.reshape((-1, 1))


def func(per_group, new_point_xyz_uv, dir_name, images_list, new_visibility):
    print("task #", per_group)
    # print(per_group)
    merge_list = []
    index_list = []
    des_index_list = []
    bf = cv2.BFMatcher()
    orb_detector = detector_defind()
    length = new_visibility[per_group]
    index_graph_a = length[0]
    index_a = keyframe_index[index_graph_a]
    # print("index_query:", index_a)
    none_list = []

    keyPoints1_new, des1_new, img1, querylist, query_pt_list = new_points_des(
        new_point_xyz_uv, index_a, dir_name, images_list, orb_detector, length
    )

    time_0 = 0.0
    time_1 = 0.0
    time_2 = 0.0
    time_3 = 0.0
    time_4 = 0.0
    time_5 = 0.0

    for index_train in range(1, len(length)):
        time_0_s = time.time()  # <<<<<

        time_1_s = time.time()  # <<<<<
        index_graph_b = length[index_train]
        index_b = keyframe_index[index_graph_b]
        # print("index_train:", index_b)
        if index_b < index_a:
            continue
        keyPoints2_new, des2_new, img2, trainlist, train_pt_list = new_points_des(
            new_point_xyz_uv, index_b, dir_name, images_list, orb_detector, length
        )
        time_1 += time.time() - time_1_s  # <<<<<

        time_2_s = time.time()  # <<<<<
        matches = bf.knnMatch(des1_new, des2_new, k=2)
        # Apply ratio test
        good = np.asarray([[m] for m, n in matches if m.distance < 0.75 * n.distance])
        tp1 = np.float32([keyPoints1_new[m[0].queryIdx].pt for m in good])
        tp2 = np.float32([keyPoints2_new[m[0].trainIdx].pt for m in good])
        M, mask = cv2.findHomography(tp1, tp2, cv2.RANSAC, 5)
        time_2 += time.time() - time_2_s  # <<<<<

        time_3_s = time.time()  # <<<<<
        mask = mask.ravel() != 0
        tp1, tp2 = tp1[mask], tp2[mask]
        # time2 = time.time()
        good_1 = good[mask]
        desc_tp1 = [des1_new[m[0].queryIdx] for m in good_1]
        desc_tp2 = [des2_new[m[0].trainIdx] for m in good_1]
        time_3 += time.time() - time_3_s  # <<<<<

        # desc_tp_ = np.vstack((desc_tp1, desc_tp2)).reshape((-1,))
        desc_tp_ = np.vstack((desc_tp1, desc_tp2))
        time_4_s = time.time()  # <<<<<
        index_uv1, query_index_uv1 = query_index(tp1, query_pt_list, querylist)
        index_uv2, query_index_uv2 = query_index(tp2, train_pt_list, trainlist)
        time_4 += time.time() - time_4_s  # <<<<<
        desc_list_uv = np.vstack((query_index_uv1, query_index_uv2))
        index_unique = np.asarray(
            [np.argwhere(desc_list_uv == iads)[0] for iads in np.unique(desc_list_uv)]
        )
        index_uv_unique = desc_list_uv[index_unique].reshape((-1))

        time_5_s = time.time()  # <<<<<
        desc_tp_unique = desc_tp_[index_unique].reshape((-1, 32))
        value_list_new = np.hstack((query_index_uv1, query_index_uv2)).astype(int)

        index_list.extend(index_uv_unique)
        des_index_list.extend(desc_tp_unique)
        merge_list.extend(value_list_new)
        time_5 += time.time() - time_5_s  # <<<<<

        time_0 += time.time() - time_0_s  # <<<<<

    index_list = asarray(index_list)
    des_index_list = asarray(des_index_list)
    index_unique_all = [
        np.argwhere(index_list == iads)[0] for iads in np.unique(index_list)
    ]
    index_list_unique = index_list[index_unique_all]
    des_index_list_unique = des_index_list[index_unique_all].reshape((-1, 32))
    # desc_list = [ [int(index_list_unique[i]), des_index_list_unique[i]] for i in range(len(index_list_unique)) ]  # modified
    # desc_list = np.column_stack([ index_list_unique.reshape([-1, 1]), des_index_list_unique ])  # modified

    # time4 = time.time()
    merge_list = concat(merge_list)
    merge_list = [mer for mer in merge_list if len(mer) > 3]

    for mer in merge_list:
        # curr_mer_des = np.asarray([y[1:-1] for x in mer for y in desc_list if int(y[0]) == int(x)])  # modified
        curr_mer_des = des_index_list_unique[
            [np.where(index_list_unique == mer_i)[0][0] for mer_i in mer]
        ]  # modified
        matches1 = bf.knnMatch(curr_mer_des, curr_mer_des, k=2)
        distance_all = [y.distance for x, y in matches1]
        v1 = np.argmax(distance_all)
        del mer[v1]
        none_list.extend(mer)
        # print(none_list)
    # time5 = time.time()
    # print("merge_time:", time5 - time4)
    # pbar.update(1)

    return none_list, [time_0, time_1, time_2, time_3, time_4, time_5]


import glob
import os

# import pdb; pdb.set_trace()

datadir = "/home/guangcheng/Workspace/edgecloud/output_all_frames/"
data_name = "tum_rgbd_dataset_freiburg2_pioneer_360_long_stride02_"

All_Images_XYZ_uv = np.load(datadir + data_name + "idx_xyz_duv.npy")
timestamp_np = np.load(datadir + data_name + "tstamps.npy")
keyframe_index = np.load(datadir + data_name + "indexes.npy")
visibililty_graph = np.load(datadir + data_name + "graph.npy")
new_point_xyz_uv = np.load(datadir + data_name + "cam_mpt_uv.npy")
image_path = "/data/SLAMdatasets/tum/rgbd_dataset_freiburg2_pioneer_360/rgb/"
images_list = [x.split("/")[-1] for x in glob.glob(os.path.join(image_path, "*.png"))]
ext = os.path.splitext(images_list[0])[1]
images_list = sorted(images_list, key=lambda x: float(x.split(ext)[0]))


count = 0
vis = False
pool = multiprocessing.Pool(processes=40)
time_start = time.time()
new_visibility = create_graph(visibililty_graph)

# pbar = tqdm(total=len(new_visibility))
list_none = []
for per_group in range(len(new_visibility) >> 2):

    none_list_per = pool.apply_async(
        func,
        (
            per_group,
            new_point_xyz_uv,
            image_path,
            images_list,
            new_visibility,
        ),
    )
    list_none.append(none_list_per)

pool.close()
pool.join()

list_result = []
list_time = []
for none_ in list_none:
    result, times = none_.get()
    list_time.append(times)
    list_result.extend(result)

list_result = list(set(list_result))
All_Images_XYZ_uv_new = np.delete(All_Images_XYZ_uv, list_result, axis=0)

map_point_index_map = {}
new_map_point_xyz_data = []
for map_point_i in range(All_Images_XYZ_uv.shape[0]):
    if map_point_i in list_result:
        continue
    new_map_point_xyz_data.append(All_Images_XYZ_uv[map_point_i].tolist())
    map_point_index_map[map_point_i] = len(new_map_point_xyz_data) - 1
map_point_xyz_data = np.array(new_map_point_xyz_data)

have_use_indexes = list(set(map_point_index_map.keys()))

no_use_indexes = []
for i in range(new_point_xyz_uv.shape[0]):
    if new_point_xyz_uv[i, 1] not in have_use_indexes:
        no_use_indexes.append(i)
    pass
new_point_xyz_uv = np.delete(new_point_xyz_uv, no_use_indexes, axis=0)
new_point_xyz_uv[:, 1] = list(
    map(
        lambda i: map_point_index_map[i],
        new_point_xyz_uv[:, 1].astype(np.int16).tolist(),
    )
)

# All_Images_XYZ_uv_new = np.delete(All_Images_XYZ_uv_new, [4], axis=1)
print("Original:", All_Images_XYZ_uv.shape, "Final:", All_Images_XYZ_uv_new.shape)


# pbar.close()
np.save(
    "/home/guangcheng/Workspace/Test_merge/{}_idx_xyz_uv_downsampled.npy".format(
        data_name
    ),
    All_Images_XYZ_uv_new,
)

time_end = time.time()
print("Total time used:", time_end - time_start)
ts = np.array(list_time).sum(axis=0)
print(ts)
print(ts / ts[0])
