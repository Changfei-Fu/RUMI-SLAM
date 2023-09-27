# RUMI-SLAM
## Run
Let's take a demo of a [TUM dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download) as an example of how to run our RUMI-SLAM.

1. Download a sequence of TUM dataset: `wget https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_nostructure_notexture_far.tgz -O ~/rgbd_dataset_freiburg3_nostructure_notexture_far.tgz`. Unzip it: `cd ~ && tar zxvf rgbd_dataset_freiburg3_nostructure_notexture_far.tgz`.
2. Modify the launch file (`src/rumi_slam/launch/main.launch`) according to our dataset path: 
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
3. Launch back-end SLAM in your server machine.
```bash
conda activate droidenv5 # switch to env for droidslam
python src/edgecloud/scripts/cloud_slam.py
```

4. Launch RUMI-SLAM in your client machine.
```bash
source devel/setup.bash
roslaunch rumi-slam main.launch
```

## Others
### About Downsample Method
The default downsample method is the optical flow sampling, which means that you need to adjust the `PD` and `th` parameters for each different dataset.
If you want to debug other part of system, you can change the code to use asymmetric sampling for convenience. You need to uncomment **90-97** lines in `src/rumi-slam/lib_src/KFDSample.cc` and comment **99-172** line of it.

### filesystem bug
**Error**: `fatal error: filesystem: No such file or directory`

**Solution**: Because the `<filesystem>` library need `g++-8`, you need to `sudo apt-get install g++-8` and `export CXX="g++-8" CC="gcc-8"` before `catkin_make`.

### Sophus CMake Bug
Error: `make_integer_sequence in namespace 'std' does not name a template type`
Solve: `set(CMAKE_CXX_STANDARD 11)` to `set(CMAKE_CXX_STANDARD 14)` in CMakeLists.txt
