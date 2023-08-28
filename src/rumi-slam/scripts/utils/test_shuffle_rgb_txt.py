import os
import numpy as np


if __name__ == "__main__":
    input_rgb_txt_path = "/home/red0orange/Data/TUM/rgbd_dataset_freiburg3_structure_texture_near/rgb_0-30s.txt"
    output_rgb_txt_path = "/home/red0orange/Data/TUM/rgbd_dataset_freiburg3_structure_texture_near/rgb_shuffle_0-30s.txt"

    data = np.loadtxt(input_rgb_txt_path, dtype=np.object).tolist()
    data_len = len(data)

    first_half_data = data[: (data_len // 2)]
    second_half_data = data[(data_len // 2) :]

    second_half_data = second_half_data[::-1]

    first_half_data = np.array(first_half_data)
    second_half_data = np.array(second_half_data)

    np.savetxt(
        output_rgb_txt_path,
        np.concatenate([first_half_data, second_half_data], axis=0),
        fmt="%s",
    )
    pass
