import os
import csv

from red0orange.file import get_files


if __name__ == "__main__":
    base_path = "/media/red0orange/Data/数据集/slam-euroc"

    file_paths = get_files(base_path, extensions=[".csv"], recurse=True)
    rgb_csv_file_paths = [
        i for i in file_paths if os.path.basename(i).split(".", maxsplit=1)[0] == "rgb"
    ]

    for rgb_csv_file_path in rgb_csv_file_paths:
        data_list = []
        with open(rgb_csv_file_path, "r") as f:
            data = csv.reader(f, delimiter=",")
            for i in data:
                if not i[0].startswith("#"):
                    i[0] = i[0][:-9] + "." + i[0][-9:]
                    i[1] = os.path.join("mav0/cam0/data", i[1])
                    data_list.append(i)
        rgb_save_path = rgb_csv_file_path.rsplit(".", maxsplit=1)[0] + ".txt"
        with open(rgb_save_path, "w") as f:
            f.writelines([" ".join(i) + "\n" for i in data_list])

    pass
