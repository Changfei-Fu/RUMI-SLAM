#!/usr/bin/env python
import os
import rospy
import time
import psutil

from std_msgs.msg import Float32


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
    rospy.init_node("pub_memory")

    topic_name = "/cloud_edge_memory_temp"
    cloud_edge_memory_pub = rospy.Publisher(topic_name, Float32, queue_size=3)

    process_name = "cloud_edge"
    pub_hz = 2

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

    r = rospy.Rate(hz=pub_hz)
    while True:
        r.sleep()
        try:
            memory = psutil.Process(pid).memory_info().rss / 1024 / 1024 / 1024
            cloud_edge_memory_pub.publish(Float32(data=memory))
            print("当前进程的内存使用：%.4f GB" % (memory))
        except BaseException:
            print("PID doesn't exist!")
            pass
    rospy.spin()
    pass
