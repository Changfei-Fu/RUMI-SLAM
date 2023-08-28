import os
import numpy as np

from red0orange.file import get_files


if __name__ == "__main__":
    base_path = "/media/red0orange/Data/数据集/slam-icl"

    file_paths = get_files(base_path, extensions=[".txt"], recurse=True)
    rgb_txt_file_paths = [
        i for i in file_paths if os.path.basename(i).split(".", maxsplit=1)[0] == "rgb"
    ]

    for rgb_txt_file_path in rgb_txt_file_paths:
        ori_data = np.loadtxt(rgb_txt_file_path, dtype=np.object)
        index = 0
        for line_data in ori_data:
            line_data[1] = "rgb/" + str(index) + ".png"
            index += 1
        np.savetxt(
            os.path.join(os.path.dirname(rgb_txt_file_path), "rgb_preprocess.txt"),
            ori_data,
            fmt="%s %s",
        )
        pass

    # for rgb_csv_file_path in rgb_csv_file_paths:
    #     data_list = []
    #     with open(rgb_csv_file_path, "r") as f:
    #         data = csv.reader(f, delimiter=",")
    #         for i in data:
    #             if not i[0].startswith("#"):
    #                 i[0] = i[0][:-9] + "." + i[0][-9:]
    #                 i[1] = os.path.join("mav0/cam0/data", i[1])
    #                 data_list.append(i)
    #     rgb_save_path = rgb_csv_file_path.rsplit(".", maxsplit=1)[0] + ".txt"
    #     with open(rgb_save_path, "w") as f:
    #         f.writelines([" ".join(i) + "\n" for i in data_list])

    pass
