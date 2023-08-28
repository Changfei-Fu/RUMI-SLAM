#!/bin/bash

# 循环执行程序 100 次
for i in {1..30}; do
  echo "执行程序，第 $i 次"
  roslaunch cloud_edge_slam main.launch

  # 检测程序的输出，当输出中包含指定文本时，杀死程序
  while read -r line; do
    if [[ $line == *"Export KeyFrame Trajectory"* ]]; then
      echo "发现指定文本，杀死程序..."
      sleep 2
      kill $!
      break
    fi
  done < <(roslaunch cloud_edge_slam main.launch | tee /dev/tty)
  sleep 5
done
