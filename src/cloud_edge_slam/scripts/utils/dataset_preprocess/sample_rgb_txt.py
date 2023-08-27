import os
import numpy as np

from red0orange.file import get_files


if __name__ == "__main__":
    # rgb_input_txt_path = "/media/red0orange/Data/数据集/slam-sustech/cam0/resize_rgb_begin.txt"
    # rgb_output_txt_path = (
    #     "/media/red0orange/Data/数据集/slam-sustech/cam0/resize_rgb_begin_sample.txt"
    # )

    rgb_input_txt_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure_2/15-21s_2s.txt"
    rgb_output_txt_path = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg3_structure_texture_near/draw_figure_2/15-21s_2s_sample.txt"

    ori_data = np.loadtxt(rgb_input_txt_path, dtype=np.object)
    index = np.zeros(ori_data.shape[0], dtype=np.bool)

    sample_interval = 3
    for i in range(0, len(index)):
        if i % sample_interval == 0:
            index[i] = 1

    # sample_interval = 20
    # for i in range(0, int(0.3 * len(index))):
    #     if i % sample_interval == 0:
    #         index[i] = 1

    # sample_interval = 50
    # for i in range(int(0.3 * len(index)), len(index)):
    #     if i % sample_interval == 0:
    #         index[i] = 1

    ori_data = ori_data[index]
    np.savetxt(
        os.path.join(rgb_output_txt_path),
        ori_data,
        fmt="%s %s",
    )
    pass
