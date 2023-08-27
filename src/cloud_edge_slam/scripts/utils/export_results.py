import os
import pandas as pd
import numpy as np

root_dir = '/home/red0orange/projects/Cloud-Edge-SLAM/new_results_2'  # 修改为存放结果文件夹的路径

# 创建一个空的Pandas DataFrame用于存放所有结果数据
results_df = pd.DataFrame()

for subdir, dirs, files in os.walk(root_dir):
    for file in files:
        if file == 'result.csv':
            filepath = os.path.join(subdir, file)
            with open(filepath) as f:
                data_dict = {}
                for line in f:
                    key, value = line.strip().split(',')
                    try:
                        value = float(value)
                    except ValueError:
                        pass  # 不是float类型的值保持原样
                    data_dict[key] = value
                # 将该次实验的结果数据转换成一个Pandas DataFrame行，并添加到总表格中
                row_df = pd.DataFrame.from_dict(data_dict, orient='index').T
                row_df['subdir'] = subdir  # 记录对应的子文件夹名称
                results_df = pd.concat([results_df, row_df], ignore_index=True)

# 将总表格保存为CSV文件
results_df.to_csv('experiment_results_3.csv', index=False)