#!/home/red0orange/Projects/TidyBot/reorientbot-full/.anaconda3/bin/python
import os

from cv2 import mean
import rospy
import time
import psutil


def get_process_pid(process_name):
    process = os.popen("ps -A | grep %s" % process_name)
    process_info = process.read()
    result = []
    for i in process_info.split(" "):
        if i != "":
            result.append(i)
    pid = int(result[0])
    return pid


if __name__ == "__main__":
    process_name = "cloud_edge"
    process_memory = []
    start_time = time.time()
    end_time = None

    pid = None
    while True:
        time.sleep(0.1)
        print("Waiting process start!")
        try:
            pid = get_process_pid(process_name)
            break
        except BaseException:
            pass

    print("Waiting process end!")

    while True:
        time.sleep(0.1)
        try:
            memory = psutil.Process(pid).memory_info().rss / 1024 / 1024 / 1024
            process_memory.append(memory)
            print("当前进程的内存使用：%.4f GB" % (memory))
        except BaseException:
            end_time = time.time()
            break
    print("=================================")
    duration = end_time - start_time
    print("Duration: {}".format(duration))
    print("Avg memory: {}".format(sum(process_memory) / len(process_memory)))
    print("Max memory: {}".format(max(process_memory)))
    print("=================================")
    pass
