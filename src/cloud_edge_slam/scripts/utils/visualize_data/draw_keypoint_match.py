import os
import numpy as np
import cv2


if __name__ == "__main__":
    save_path = "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/results.png"
    img1 = cv2.imread("/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/img1.png")
    img2 = cv2.imread("/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/img2.png")
    file = cv2.FileStorage(
        "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/keypoint_match.yml", cv2.FILE_STORAGE_READ
    )

    img1_keypoint_data = (
        file.getNode("img1_keypoint").mat().squeeze().astype(np.float16)
    )
    img1_keypoint = [cv2.KeyPoint(x=i[0], y=i[1], size=1) for i in img1_keypoint_data]
    img2_keypoint_data = (
        file.getNode("img2_keypoint").mat().squeeze().astype(np.float16)
    )
    img2_keypoint = [cv2.KeyPoint(x=i[0], y=i[1], size=1) for i in img2_keypoint_data]
    keypoint_match_data = file.getNode("keypoint_match_indexes").mat().squeeze()
    keypoint_match = [
        cv2.DMatch(_imgIdx=0, _queryIdx=i[0], _trainIdx=i[1], _distance=0)
        for i in keypoint_match_data
    ]

    draw_img1 = img1.copy()
    blank_img = (
        np.ones([draw_img1.shape[0], draw_img1.shape[1] // 3, 3], dtype=np.uint8) * 255
    )
    draw_img1 = np.concatenate([draw_img1, blank_img], axis=1)

    out_img = np.array([])
    result_img = cv2.drawMatches(
        draw_img1,
        img1_keypoint,
        img2,
        img2_keypoint,
        keypoint_match,
        matchesThickness=1,
        outImg=out_img,
    )
    cv2.imshow("result_image", result_img)
    cv2.imwrite(save_path, result_img)
    cv2.waitKey(0)

    file.release()
    pass
