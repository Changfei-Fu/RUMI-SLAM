# Cloud-Edge-SLAM
## 1. Install
### Cloud SLAM
In your server machine, follow the steps below to install Cloud Slam.

1. Install [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM) first, then add `self.video.tstamp[ix] = self.video.tstamp[ix+1]` in function `rm_keyframe` in factor_graph.py.
2. The codes of cloud slam are developed with this specific Droid-SLAM version. You may need to use this version to avoid potential bugs resulting from updates of Droid-SLAM.
```
cd /path/to/Droid-SLAM
git reset --hard 92027b31c06f42e4471706bbb46e7eb3cb3a5dd0 # 
```
3. Modified the paths at line #27 and #28 and #79 in `src/edgecloud/scripts/cloud_slam.py`  so that Droid-SLAM can be found and launched by the scripts of cloud slam.
4. Clone our project (`git clone https://github.com/Robotics-and-Computer-Vision-Lab/Cloud-Edge-SLAM.git`) and switch to branch of cloud slam `git checkout cloud`.
5. Compile the message and action files
```
catkin_make
```

### Edge SLAM
In your client machine, follow the steps below to install Edge Slam.

1. **ROS** installation: 
	- Follow the official guide to install ([ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)). Pay attention to your ubuntu version and select the correct ROS version.
	- **Depend ros packages** installation: Run `rosdep install --from-paths src -y` in **the root path of workspace**, which will install required ros packages automatically (such as `ros-noetic-cv-bridge`).
2. **OpenCV 3.4.8** installation:
	- Download [Download Link](https://github.com/opencv/opencv/archive/3.4.8.zip) and [Contrib Download Link](https://github.com/opencv/opencv_contrib/archive/refs/tags/3.4.8.zip).
	- **Build** opencv+opencv_contrib and **install** it into system using `cmake`. The installation steps can refer to this [blog](https://blog.csdn.net/Flag_ing/article/details/109508374).
3. **Eigen 3.4.0** installation: 
	- Download [Download Link](https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip).
	- Build eigen using `cmake`, run `mkdir build && cd build && cmake ..` and `sudo make install`.
4. **Pangolin** installation:
	- Clone `git clone https://github.com/stevenlovegrove/Pangolin.git` and change version `git checkout v0.6`.
	- Build pangolin using `cmake`, run `mkdir build && cd build && cmake ..` and `sudo make install`
5. **Clone our project** (`git clone https://github.com/Robotics-and-Computer-Vision-Lab/Cloud-Edge-SLAM.git`) and switch to branch of edge slam `git checkout master`.
5. **Inside thirdparty** (g2o, Sophus, DBoW2) libraries installation: 
	- All the inside thirdparty libraries are in `src/cloud_edge_slam/Thirdparty` dir.
	- Install DBoW2: `cd DBoW2 && mkdir build && cd build`, `cmake .. && make`. **Don't need to install it!**
	- Install g2o: `cd g2o && mkdir build && cd build`, `cmake .. && make`. **Don't need to install it!**
	- Install Sophus: `cd Sophus && mkdir build && cd build`, `cmake .. && make`. **Don't need to install it!**
6. **Unzip vocabulary**: `cd src/cloud_edge_slam/Vocabulary && tar -xf ORBvoc.txt.tar.gz`
7. **Build**: `catkin_make -DCMAKE_BUILD_TYPE=Release -j4`, the system will be very slow if do not build using `release`.
8. **Configure**:
Before finally running the system, we need to configure some parameters in the launch file. 
The following code is the minimum standard launch file which includes all the required parameters. The  `main.launch` in `src/cloud_edge_slam/launch` is the default launch file.
```yaml
<launch>
    <remap from="/camera/image_raw" to="/camera/rgb/image_color"/>
    <!--     launch ORB SLAM3  -->
    <node name="cloud_edge_slam" pkg="cloud_edge_slam" type="cloud_edge_slam_node" required="true" > 
        <param name="vocabulary_path" value="$(find cloud_edge_slam)/Vocabulary/ORBvoc.txt" />

        <!-- ！！！setting_path 参数：指定ORB SLAM3的配置文件 ！！！ -->
        <!-- ！！！data_type 参数：指定数据集的类型，txt或bag ！！！ -->
        <!-- ！！！data_path 参数：指定数据集的路径，即txt文件或bag文件路径 ！！！ -->

        <!-- TUM Dataset -->
        <param name="setting_path" value="$(find cloud_edge_slam)/config/TUM1.yaml" />
        <param name="data_type" value="txt" />
        <param name="data_path" value="/home/ruanjh/Workspace/datasets/slam-tum/rgbd_dataset_freiburg3_long_office_household/rgb.txt" />

        <!-- 指定结果的保存路径，将保存运行时间、消耗内存的一些信息 -->
        <param name="result_path" value="/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/src/cloud_edge_slam/results" />

        <!-- ！！！ 参数：Cloud SLAM Action名称，一定要和Cloud SLAM端保持一致 ！！！ -->
        <param name="cloud_topic_name" value="/cloud_slam" />

        <!-- ！！！参数：Merge时是否忽略Inliers的判断，若为true，无论算出来是什么情况都合并 ！！！ -->
        <param name="merge_anyway" value="false" />
        <!-- ！！！参数：（需要直接全部使用offline map bag时使用）是否Online运行，若为true，将等待一个Offline的包含所有map的msg后运行 ！！！ -->
        <param name="cloud_online" value="true" />
        <!-- ！！！参数：（一般不使用，需要手动rosbag play调试时使用）是否Online运行，若为true，将必须等待Cloud SLAM的Action连接成功后才初始化 ！！！ -->
        <param name="real_online" value="true" />
        <!-- ！！！ 参数：（一般保持打开）是否开启Cloud Merging线程 ！！！-->
        <param name="cloud_merge" value="true" />
        <!-- ！！！ 参数：是否保存上传给Cloud SLAM的msg bag ！！！-->
        <param name="save_cloud_bag" value="false" />

        <!-- ！！！ 参数：（调试时尽量先开启这个选项，测试完成后再进行不等待的实时运行）是否等待Cloud SLAM的结果并Merge完成后再继续运行，若为true，将必须等待Cloud SLAM的Action连接成功后才初始化 ！！！-->
        <param name="wait_cloud_result" value="true" />
        <!-- ！！！ 参数：每帧运行后睡眠的时间 ！！！-->
        <param name="main_loop_sleep_ms" value="30" />

        <!-- ！！！ 参数：Edge Front Map选取的最小keyframe数量要求 ！！！-->
        <param name="sampler_edge_front_kf_num" value="40" />
        <!-- ！！！ 参数：Edge Back Map选取的最小keyframe数量要求 ！！！-->
        <param name="sampler_edge_back_kf_num" value="40" />
        <!-- ！！！ 参数：Edge Front Map选取的最短时间要求 ！！！-->
        <param name="sampler_edge_front_min_time" value="3.0" />
        <!-- ！！！ 参数：Edge Back Map选取的最短时间要求 ！！！-->
        <param name="sampler_edge_back_min_time" value="3.0" />

        <!-- ！！！ 参数：光流法Downsample的PD和th参数设置 ！！！-->
        <param name="sampler_pd_kp" value="0.8" />
        <param name="sampler_pd_kd" value="0.08" />
        <param name="sampler_pd_th" value="12" />
        
        <!-- 参数：是否开启keyframe culling，若为ture，将影响Merge时关联的KF数量，效果不好 -->
        <param name="kf_culling" value="false" />
    </node>

    <!-- memory pub -->
    <!-- <node name="memory_pub" pkg="cloud_edge_slam" type="pub_memory.py" required="true" />  -->
    <!-- <node name="evo_server" pkg="cloud_edge_slam" type="evo_node.py" required="true" />  -->
</launch>
```

## 2. Connection between Cloud SLAM and Edge SLAM
1. Make sure server (the machine which runs Cloud SLAM) and clent (the machine which runs Edge SLAM) are in same LAN.
2. Obtain the `ip` of two machine using `ifconfig`.
3. Append following content to the `.bashrc` of **client machine**. Don't forget to fill the obtained ip in it.
```bash
export ROS_HOSTNAME=<your client ip>
export ROS_IP=<your client ip>
export ROS_MASTER_URI=http://<your server ip>:11311
```
4. Append following content to the `.bashrc` of **server machine**. Don't forget to fill the obtained ip in it.
```bash
export ROS_HOSTNAME=<your server ip>
export ROS_UP=<your server ip>
export ROS_MASTER_URI=http://<your server ip>:11311
```
5. Don't forget to restart your shell or source `.bashrc`.
Details can refer to [blog](https://blog.csdn.net/tianb03/article/details/110679579).

## 3. Run
Let's take a demo of a [TUM dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download) as an example of how to run our CloudEdgeSLAM.

1. Download a sequence of TUM dataset: `wget https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_nostructure_notexture_far.tgz -O ~/rgbd_dataset_freiburg3_nostructure_notexture_far.tgz`. Unzip it: `cd ~ && tar zxvf rgbd_dataset_freiburg3_nostructure_notexture_far.tgz`.
2. Modify the launch file (`src/cloud_edge_slam/launch/main.launch`) according to our dataset path: 
```yaml
<launch>
    <remap from="/camera/image_raw" to="/camera/rgb/image_color"/>
    <!--     launch ORB SLAM3  -->
    <node name="cloud_edge_slam" pkg="cloud_edge_slam" type="cloud_edge_slam_node" required="true" > 
        <param name="vocabulary_path" value="$(find cloud_edge_slam)/Vocabulary/ORBvoc.txt" />

        <!-- TUM Dataset -->
        <param name="setting_path" value="$(find cloud_edge_slam)/config/TUM3.yaml" />
        <param name="data_type" value="txt" />
        <param name="data_path" value="<your home path>/rgbd_dataset_freiburg3_nostructure_notexture_far/rgb.txt" />

        <param name="result_path" value="<your project path>/src/cloud_edge_slam/results" />

        <param name="cloud_topic_name" value="/cloud_slam" />

        <param name="merge_anyway" value="false" />
        <param name="cloud_online" value="true" />
        <param name="real_online" value="true" />
        <param name="cloud_merge" value="true" />
        <param name="save_cloud_bag" value="false" />

        <param name="wait_cloud_result" value="true" />
        <param name="main_loop_sleep_ms" value="30" />

        <param name="sampler_edge_front_kf_num" value="40" />
        <param name="sampler_edge_back_kf_num" value="40" />
        <param name="sampler_edge_front_min_time" value="3.0" />
        <param name="sampler_edge_back_min_time" value="3.0" />

        <param name="sampler_pd_kp" value="0.8" />
        <param name="sampler_pd_kd" value="0.08" />
        <param name="sampler_pd_th" value="12" />
        
        <param name="kf_culling" value="false" />
    </node>

    <!-- memory pub -->
    <!-- <node name="memory_pub" pkg="cloud_edge_slam" type="pub_memory.py" required="true" />  -->
    <!-- <node name="evo_server" pkg="cloud_edge_slam" type="evo_node.py" required="true" />  -->
</launch>
```
3. Launch Cloud SLAM in your server machine.
```bash
conda activate droidenv5 # switch to env for droidslam
python src/edgecloud/scripts/cloud_slam.py
```

4. Launch Edge SLAM in your client machine.
```bash
source devel/setup.bash
roslaunch cloud_edge_slam main.launch
```

## Others
### About Downsample Method
The default downsample method is the optical flow sampling, which means that you need to adjust the `PD` and `th` parameters for each different dataset.
If you want to debug other part of system, you can change the code to use asymmetric sampling for convenience. You need to uncomment **90-97** lines in `src/cloud_edge_slam/lib_src/KFDSample.cc` and comment **99-172** line of it.

### filesystem bug
**Error**: `fatal error: filesystem: No such file or directory`

**Solution**: Because the `<filesystem>` library need `g++-8`, you need to `sudo apt-get install g++-8` and `export CXX="g++-8" CC="gcc-8"` before `catkin_make`.

### Sophus CMake Bug
Error: `make_integer_sequence in namespace 'std' does not name a template type`
Solve: `set(CMAKE_CXX_STANDARD 11)` to `set(CMAKE_CXX_STANDARD 14)` in CMakeLists.txt
