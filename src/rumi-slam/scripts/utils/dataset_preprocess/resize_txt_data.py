import os
import cv2

import numpy as np
from red0orange.file import get_image_files


if __name__ == "__main__":
    input_dir = "/media/red0orange/Data/数据集/slam-sustech/cam0/data"
    output_dir = "/media/red0orange/Data/数据集/slam-sustech/cam0/resize_data"

    input_image_size = [1280, 720]  # for check
    output_image_size = [640, 360]

    input_image_paths = get_image_files(input_dir)
    output_image_paths = [
        os.path.join(output_dir, os.path.basename(i)) for i in input_image_paths
    ]

    for i, (input_image_path, output_image_path) in enumerate(zip(input_image_paths, output_image_paths)):
        ori_image = cv2.imread(input_image_path)
        assert (
            ori_image.shape[0] == input_image_size[1]
            and ori_image.shape[1] == input_image_size[0]
        )
        resize_image = cv2.resize(ori_image, tuple(output_image_size))
        assert (
            resize_image.shape[0] == output_image_size[1]
            and resize_image.shape[1] == output_image_size[0]
        )
        cv2.imwrite(output_image_path, resize_image)
        pass

    pass
