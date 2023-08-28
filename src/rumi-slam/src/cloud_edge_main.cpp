/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "KeyFrame.h"
#include "sensor_msgs/image_encodings.h"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <sys/stat.h> 　
#include <sys/types.h> 　
#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <actionlib/client/simple_action_client.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <unistd.h>
#include <vector>

#include "globals.h"
#include "System.h"
#include "Map.h"
#include "Atlas.h"
#include "Converter.h"
#include "ORBVocabulary.h"
#include "CloudImageSampler.h"

#include "cloud_edge_slam/Evo.h"
#include "cloud_edge_slam/CloudSlamAction.h"
#include "cloud_edge_slam/CloudSlamActionFeedback.h"
#include "cloud_edge_slam/CloudSlamActionGoal.h"
#include "cloud_edge_slam/CloudSlamGoal.h"
#include "cloud_edge_slam/CloudSlamResult.h"
#include "cloud_edge_slam/Sequence.h"
#include "cloud_edge_slam/CloudMap.h"
#include "cloud_edge_slam/KeyFrame.h"
#include "cloud_edge_slam/KeyPoint.h"
#include "cloud_edge_slam/MapPoint.h"
#include "cloud_edge_slam/Descriptor.h"
#include "cloud_edge_slam/Observation.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "rosbag/query.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#define getRosParam(nh, param_name, output)                                     \
    if (!nh.getParam(param_name, output)) {                                     \
        ROS_ERROR_STREAM("!ERROR! cannot get necessary param: " << param_name); \
        exit(100);                                                              \
    }

namespace bfs = boost::filesystem;
using namespace std;

typedef actionlib::SimpleActionClient<cloud_edge_slam::CloudSlamAction> CloudClient;
// typedef actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> CloudClient;

class Grabber {
public:
    Grabber(ORB_SLAM3::System *pSLAM) :
        mpSLAM(pSLAM) {
    }
    // **********************************************
    // @note init
    void SetParameters(bool bWaitCloudResult, float nMainLoopSleep, string savePath, bool bSaveCloudBag);
    void SetNodeHandle(ros::NodeHandle *pNodeHandle);
    void SetOrbMapPublisher(ros::Publisher *pPublisher);
    void SetCloudImagesActionClient(CloudClient *pCloudImagesActionClient);
    // **********************************************

    // **********************************************
    // @note Main Tracking
    void RunBag(const string &bag_path);
    void RunTxt(const string &txt_path);
    void GrabImage(const sensor_msgs::ImageConstPtr &msg);
    // **********************************************

    // **********************************************
    // @note Main Cloud Interact
    // Pub Cloud Images
    void TrackImage(const cv::Mat &img, const double &timestamp, const float &imageScale);
    // Get Cloud Map
    void ActionFinishCb(const actionlib::SimpleClientGoalState &state, const cloud_edge_slam::CloudSlamResultConstPtr &result);
    // Get Memory
    void MemoryCb(const std_msgs::Float32ConstPtr &msg);
    // **********************************************

    // **********************************************
    // @note For Debug
    // Subscriber Callback: Pub Specify ORB Map
    void PubORBMapCb(const std_msgs::Int16ConstPtr &msg);
    // Subscriber Callback: Save Specify ORB Map
    void SaveORBMapCb(const std_msgs::Int16ConstPtr &msg);
    // Subscriber Callback: Read Cloud Map
    void GrabCloudMapCb(const cloud_edge_slam::CloudMapConstPtr &msg);
    // **********************************************

    // **********************************************
    // @note utils
    static cloud_edge_slam::CloudMap ORBMapToROSMap(ORB_SLAM3::Map *pMap);
    ORB_SLAM3::Map *ROSMapToORBMap(cloud_edge_slam::CloudMapConstPtr pMap);
    static void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);
    static void WriteCloudMapBag(cloud_edge_slam::CloudMap map, const std::string &save_path);
    static void WriteCloudImagesBag(std::vector<ORB_SLAM3::CloudImage> &vImages, const std::string &save_path);
    static void WriteSeqBag(const cloud_edge_slam::Sequence &seq, const std::string &save_path);
    // **********************************************

public:
    cloud_edge_slam::CloudSlamGoal goal;

    ORB_SLAM3::System *mpSLAM;

    ros::NodeHandle *mpNodeHandler;
    ros::Publisher *mpOrbMapPub;
    ros::Subscriber *mpMemorySub;
    CloudClient *mpCloudImagesActionClient = NULL;

    std::vector<float> mvMemory;
    std::map<double, float> mvBagSize;
    std::map<double, float> mvNoSamplingBagSize;

    bool mbWaitCloudResult;
    std::string mSaveDir;
    float mnMainLoopSleep;
    bool mbSaveCloudBag;
    double mDataDuration;
    double mStartTimestamp;
};

std::vector<std::string> SplitPath(std::string path) {
    std::vector<std::string> tokens;
    std::string token;
    std::stringstream ss(path);
    while (getline(ss, token, '/')) {
        tokens.push_back(token);
    }
    return tokens;
}

int id = 0;

int main(int argc, char **argv) {
    // ros::init(argc, argv, "Mono");
    ros::init(argc, argv, "Mono-temp");
    ros::start();

    ros::NodeHandle nh("~");

    // Debug: Fix param
    // string vocabularyPath = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/Vocabulary/ORBvoc.txt";
    // string settingPath = "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/config/TUM2.yaml";
    // string dataType = "bag";
    // string dataPath = "/media/red0orange/Data/数据集/CloudEdge/rgbd_dataset_freiburg2_pioneer_360_cut_0-24s.bag";
    // bool bRealOnline = true;
    // bool bCloudOnline = true;
    // bool bSetImgGray = false;

    string cloudTopicName;
    getRosParam(nh, "cloud_topic_name", cloudTopicName);
    string vocabularyPath;
    string settingPath;
    getRosParam(nh, "vocabulary_path", vocabularyPath);
    getRosParam(nh, "setting_path", settingPath);
    string dataType;
    string dataPath;
    string resultPath;
    getRosParam(nh, "data_type", dataType);
    getRosParam(nh, "data_path", dataPath);
    getRosParam(nh, "result_path", resultPath);
    bool bCloudMerge;
    bool bSaveCloudBag;
    bool bSaveEdgeTraj;
    bool bRealOnline;
    bool bMergeAnyway;
    bool bCloudOnline;
    bool bWaitCloudResult;
    float mainLoopSleep;
    bool bSetImgGray;
    bool bKFCulling;
    getRosParam(nh, "cloud_merge", bCloudMerge);
    getRosParam(nh, "save_cloud_bag", bSaveCloudBag);
    getRosParam(nh, "real_online", bRealOnline);
    getRosParam(nh, "merge_anyway", bMergeAnyway);
    getRosParam(nh, "cloud_online", bCloudOnline);
    getRosParam(nh, "wait_cloud_result", bWaitCloudResult);
    getRosParam(nh, "main_loop_sleep_ms", mainLoopSleep);
    getRosParam(nh, "kf_culling", bKFCulling);
    int nSamplerEdgeFrontKFNum;
    int nSamplerEdgeBackKFNum;
    float nSamplerEdgeFrontMinTime;
    float nSamplerEdgeBackMinTime;
    float samplerPDKp;
    float samplerPDKd;
    float samplerPDth;
    getRosParam(nh, "sampler_edge_front_kf_num", nSamplerEdgeFrontKFNum);
    getRosParam(nh, "sampler_edge_back_kf_num", nSamplerEdgeBackKFNum);
    getRosParam(nh, "sampler_edge_front_min_time", nSamplerEdgeFrontMinTime);
    getRosParam(nh, "sampler_edge_back_min_time", nSamplerEdgeBackMinTime);
    getRosParam(nh, "sampler_pd_kp", samplerPDKp);
    getRosParam(nh, "sampler_pd_kd", samplerPDKd);
    getRosParam(nh, "sampler_pd_th", samplerPDth);

    std::vector<string> splitPaths = SplitPath(dataPath);
    string datasetName = splitPaths[splitPaths.size() - 2];

    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%m-%d-%H_%M_%S");
    std::string str_time = ss.str();

    boost::filesystem::path parent_dir(resultPath);
    boost::filesystem::path sub_dir(datasetName + "#" + str_time);
    boost::filesystem::path finish_sub_dir("Full#" + datasetName + "#" + str_time);
    full_path_ = parent_dir / sub_dir;
    boost::filesystem::path full_finish_path_ = parent_dir / finish_sub_dir;
    string full_path = full_path_.string();
    string full_finish_path = full_finish_path_.string();
    int isCreate = mkdir(full_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    cerr << "create path: " << full_path << endl;
    if (!isCreate)
        cerr << "create path: " << full_path << endl;
    else
        cerr << "create path failed! error code: " << isCreate << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabularyPath, settingPath, ORB_SLAM3::System::MONOCULAR, true, bCloudMerge, bCloudOnline, bMergeAnyway, bKFCulling, nSamplerEdgeFrontKFNum, nSamplerEdgeBackKFNum, nSamplerEdgeFrontMinTime, nSamplerEdgeBackMinTime, samplerPDKp, samplerPDKd, samplerPDth);

    Grabber igb(&SLAM);

    igb.SetParameters(bWaitCloudResult, mainLoopSleep, full_path, bSaveCloudBag);

    igb.SetNodeHandle(&nh);

    ros::Publisher orbMapPub = nh.advertise<cloud_edge_slam::CloudMap>("/test_cloud_map", 1);
    igb.SetOrbMapPublisher(&orbMapPub);

    ros::ServiceClient evoClient = nh.serviceClient<cloud_edge_slam::Evo>("/cloud_edge_evo_temp");

    ros::Subscriber memorySub = nh.subscribe("/cloud_edge_memory_temp", 1, &Grabber::MemoryCb, &igb);

    // For Pub Cloud Images
    // CloudClient cloudImagesActionClient("/test_pub_cloud_images", true);
    CloudClient cloudImagesActionClient(cloudTopicName, true);
    if (bRealOnline) {
        ROS_INFO_STREAM("Waiting For Server Begin!");
        cloudImagesActionClient.waitForServer();
        ROS_INFO_STREAM("Waiting For Server End!");
        igb.SetCloudImagesActionClient(&cloudImagesActionClient);
    }

    // For Get Test Publish Map
    ros::Subscriber save_orb_map_sub = nh.subscribe("/test_save_orb_map", 1, &Grabber::SaveORBMapCb, &igb);

    // // For Get Test Publish Map
    // ros::Subscriber pub_orb_map_sub = nodeHandler.subscribe("/test_pub_orb_map", 1, &Grabber::PubORBMap, &igb);

    // // For rosbag online play
    // ros::Subscriber image_sub = nodeHandler.subscribe("/camera/rgb/image_color", 1, &Grabber::GrabImage, &igb);

    ros::Subscriber cloud_map_sub = nh.subscribe("/test_cloud_map", 1, &Grabber::GrabCloudMapCb, &igb);

    std::chrono::steady_clock::time_point timeStart = std::chrono::steady_clock::now();
    if (bCloudOnline) {
        if (dataType == "txt") {
            igb.RunTxt(dataPath);
        } else if (dataType == "bag") {
            igb.RunBag(dataPath);
        } else {
            ROS_ERROR_STREAM("Error Data Type!");
            exit(1000);
        }
    }
    std::chrono::steady_clock::time_point timeEnd = std::chrono::steady_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEnd - timeStart).count();
    duration /= 1e3;

    // begin export
    ORB_SLAM3::CloudSaveKeyFrameTrajectoryTUM(SLAM.GetAtlas()->GetDurationLongestMap(), (full_path_ / "whole_map.txt").string());

    std::vector<double> vTimestammp;
    std::vector<Eigen::Vector3f> vPosition;
    std::vector<Eigen::Quaternionf> vQuaternion;
    ORB_SLAM3::Map *pLongestMap = SLAM.GetAtlas()->GetDurationLongestMap();
    std::vector<ORB_SLAM3::KeyFrame *> vpKFs = pLongestMap->GetSortedKeyFrames();
    ORB_SLAM3::CloudExportKeyFrameTrajectoryTUM(pLongestMap, vTimestammp, vPosition, vQuaternion);
    cloud_edge_slam::Evo evo_srv;
    evo_srv.request.dataset_name = datasetName;
    evo_srv.request.ori_duration = igb.mDataDuration;
    evo_srv.request.use_ref = true;
    evo_srv.request.poses.clear();
    for (int i = 0; i < vTimestammp.size(); i++) {
        double timestamp = vTimestammp[i];
        Eigen::Vector3f position = vPosition[i];
        Eigen::Quaternionf quaternion = vQuaternion[i];
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(timestamp);
        pose.pose.position.x = position(0);
        pose.pose.position.y = position(1);
        pose.pose.position.z = position(2);
        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
        evo_srv.request.poses.push_back(pose);
    }
    if (evoClient.call(evo_srv)) {
        cerr << "evo client success" << endl;

        ofstream outFile;
        outFile.open((full_path_ / "result.csv").string(), ios::out); // 打开模式可省略
        outFile << "ate" << ',' << evo_srv.response.ate << endl;
        outFile << "rate" << ',' << evo_srv.response.rate << endl;
        outFile << "duration" << ',' << duration << endl;
        outFile << "front_cloud_match_num" << ',' << nFrontCloudMPMatchNum << endl;
        outFile << "back_cloud_match_num" << ',' << nBackCloudMPMatchNum << endl;
        outFile << "front_kf_num" << ',' << nSamplerEdgeFrontKFNum << endl;
        outFile << "back_kf_num" << ',' << nSamplerEdgeBackKFNum << endl;
        outFile << std::fixed << std::setprecision(1) << "lost_timestamp" << ',' << nLostTimeStamp-igb.mStartTimestamp << endl;
        outFile << std::fixed << std::setprecision(1) << "new_map_timestamp" << ',' << nNewMapTimeStamp-igb.mStartTimestamp << endl;
        outFile << "lost_time" << ',' << nLostTime << endl;
        // outFile << "max_memory" << ',' << *max_element(igb.mvMemory.begin(), igb.mvMemory.end()) << endl;

        float sumBagSize = 0;
        for (auto &iter : igb.mvBagSize) {
            if (iter.first < vpKFs[vpKFs.size() - 1]->mTimeStamp && iter.first > vpKFs[0]->mTimeStamp) {
                sumBagSize += iter.second;
            }
        }
        float sumNoSamplingBagSize = 0;
        for (auto &iter : igb.mvNoSamplingBagSize) {
            if (iter.first < vpKFs[vpKFs.size() - 1]->mTimeStamp && iter.first > vpKFs[0]->mTimeStamp) {
                sumNoSamplingBagSize += iter.second;
            }
        }
        outFile << "sum_bag_size" << ',' << sumBagSize << endl;
        outFile << "sum_nosampling_bag_size" << ',' << sumNoSamplingBagSize << endl;
        outFile << "ref_traj_duration" << ',' << evo_srv.response.ref_traj_duration << endl;
        outFile << "est_traj_duration" << ',' << evo_srv.response.est_traj_duration << endl;
        outFile << "speed" << ',' << sumBagSize / evo_srv.response.ref_traj_duration << endl;
        outFile << "nosampling_speed" << ',' << sumNoSamplingBagSize / evo_srv.response.ref_traj_duration << endl;
        outFile.close();

        cv::imwrite((full_path_ / "traj.png").string(), cv_bridge::toCvCopy(evo_srv.response.traj_img)->image);
    }

    // rename dir for tag
    boost::filesystem::rename(full_path_, full_finish_path_);

    // @note 直接暂停
    exit(1000);

    // Debug: 等待CloudMap
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

inline Sophus::SE3f toSophusPose(const geometry_msgs::Pose rosPose) {
    Eigen::Quaternion<float> quaterniond(rosPose.orientation.w, rosPose.orientation.x, rosPose.orientation.y, rosPose.orientation.z);
    Eigen::Vector3f pos(rosPose.position.x, rosPose.position.y, rosPose.position.z);
    Sophus::SE3f pose(quaterniond, pos);
    return pose;
}

inline sensor_msgs::CameraInfo getCameraInfo(int imageWidth, int imageHeight, Eigen::Matrix3f K) { // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    // vector<double> D{0.000094, -0.011701, 0.000383, -0.000507, 0.000000};
    boost::array<double, 9> K_array = {
        K(0, 0), K(0, 1), K(0, 2),
        K(1, 0), K(1, 1), K(1, 2),
        K(2, 0), K(2, 1), K(2, 2)};

    // boost::array<double, 12> P = {
    //     402.124725, 0.000000, 335.482488, 0.000000,
    //     0.000000, 403.765045, 250.954855, 0.000000,
    //     0.000000, 0.000000, 1.000000, 0.000000};
    // boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cam.width = imageWidth;
    cam.height = imageHeight;
    cam.distortion_model = "plumb_bob";
    cam.K = K_array;
    // cam.D = D;
    // cam.P = P;
    // cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;
    cam.header.frame_id = "camera"; //frame_id为camera，也就是相机名字
    cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    return cam;
}

void Grabber::SetCloudImagesActionClient(CloudClient *pCloudImagesActionClient) {
    mpCloudImagesActionClient = pCloudImagesActionClient;
}

void Grabber::SetParameters(bool bWaitCloudResult, float nMainLoopSleep, string savePath, bool bSaveCloudBag) {
    mbWaitCloudResult = bWaitCloudResult;
    mnMainLoopSleep = nMainLoopSleep;
    mSaveDir = savePath;
    mbSaveCloudBag = bSaveCloudBag;
}

void Grabber::SetNodeHandle(ros::NodeHandle *pNodeHandle) {
    mpNodeHandler = pNodeHandle;
}

void Grabber::SetOrbMapPublisher(ros::Publisher *pPublisher) {
    mpOrbMapPub = pPublisher;
}

void Grabber::TrackImage(const cv::Mat &img, const double &timestamp, const float &imageScale) {
    if (imageScale != 1.f) {
        int width = img.cols * imageScale;
        int height = img.rows * imageScale;
        cv::resize(img, img, cv::Size(width, height));
    }

    static int cnt = 0;
    ROS_INFO_STREAM_DELAYED_THROTTLE(1.0, "Deal One Image: " << cnt);
    cnt++;

    // @note input image
    mpSLAM->TrackMonocular(img, timestamp);

    // return;

    // 主动式检测Cloud Image
    std::vector<ORB_SLAM3::CloudImage> vCurrentProcessCloudImages;
    std::vector<ORB_SLAM3::CloudImage> vCurrentProcessCloudNoSamplingImages;
    int edgeFrontMapId, edgeBackMapId;
    mpSLAM->GetCloudProcessImages(vCurrentProcessCloudImages, vCurrentProcessCloudNoSamplingImages, edgeFrontMapId, edgeBackMapId);
    if (!vCurrentProcessCloudImages.empty()) {
        // @note print
        cout << "Main: Pub Cloud Images !" << endl;
        // debug: write cloud images
        static int index = 0;
        index++;
        if (mbSaveCloudBag) {
            string curCloudBagPath = (bfs::path(mSaveDir) / "cloud_").string() + to_string(index) + ".bag";
            WriteCloudImagesBag(vCurrentProcessCloudImages, curCloudBagPath);
            string curCloudNoSamplingBagPath = (bfs::path(mSaveDir) / "cloud_nosampling_").string() + to_string(index) + ".bag";
            WriteCloudImagesBag(vCurrentProcessCloudNoSamplingImages, curCloudNoSamplingBagPath);

            double curTimestamp = vCurrentProcessCloudImages[vCurrentProcessCloudImages.size() - 1].timestamp;
            auto bagSize = float(bfs::file_size(curCloudBagPath) / (1024 * 1024));
            mvBagSize[curTimestamp] = bagSize;
            auto noSamplingBagSize = float(bfs::file_size(curCloudNoSamplingBagPath) / (1024 * 1024));
            mvNoSamplingBagSize[curTimestamp] = noSamplingBagSize;
        }

        // @note Publish Cloud Images Action
        if (this->mpCloudImagesActionClient) {
            // @note print
            ROS_INFO_STREAM("Action Client");
            sensor_msgs::CameraInfo cameraInfo = getCameraInfo(mpSLAM->GetSetting()->newImSize().width, mpSLAM->GetSetting()->newImSize().height, mpSLAM->GetCamera()->toK_());

            cloud_edge_slam::Sequence imageSeqMsg;
            imageSeqMsg.Header.stamp = ros::Time::now();
            imageSeqMsg.camera = cameraInfo;
            imageSeqMsg.edge_front_map_mnid = edgeFrontMapId;
            imageSeqMsg.edge_back_map_mnid = edgeBackMapId;
            for (auto &image : vCurrentProcessCloudImages) {
                std_msgs::Header header;
                header.stamp = ros::Time(image.timestamp);
                header.frame_id = image.type;
                string encode;
                if (image.img.channels() == 3)
                    encode = "bgr8";
                else if (image.img.channels() == 1)
                    encode = "mono8";
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, encode, image.img).toImageMsg();

                imageSeqMsg.images.push_back(*msg);
                imageSeqMsg.timestamps.push_back(image.timestamp);
            }

            // debug bandwidth: write seq
            // WriteSeqBag(imageSeqMsg, "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/test_seq.bag");

            goal.sequence = imageSeqMsg;
            mpCloudImagesActionClient->sendGoal(goal,
                                                boost::bind(&Grabber::ActionFinishCb, this, _1, _2),
                                                CloudClient::SimpleActiveCallback(),
                                                CloudClient::SimpleFeedbackCallback());

            // @note 等待CloudMerging结束再继续
            if (mbWaitCloudResult) {
                mpCloudImagesActionClient->waitForResult();
                usleep(1000 * 1e3);
                while (mpSLAM->GetCloudMerger()->isRunning()) {
                    usleep(100 * 1e3);
                }
            }

            mpSLAM->SetTrackLostTimestamp(timestamp);
        }
        mpSLAM->ResetCloudProcessImages();
    }
}

void Grabber::RunTxt(const string &txt_path) {
    string data_dir = txt_path.substr(0, txt_path.rfind('/'));
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(txt_path, vstrImageFilenames, vTimestamps);
    mDataDuration = vTimestamps[vTimestamps.size() - 1] - vTimestamps[0];
    mStartTimestamp = vTimestamps[0];

    int nImages = vstrImageFilenames.size();
    float imageScale = mpSLAM->GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    double t_resize = 0.f;
    double t_track = 0.f;

    // Main loop
    cv::Mat im;
    for (int ni = 0; ni < nImages; ni++) {
        // Read image from file
        im = cv::imread(string(data_dir) + "/" + vstrImageFilenames[ni], cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (im.empty()) {
            cerr << endl
                 << "Failed to load image at: "
                 << string(data_dir) << "/" << vstrImageFilenames[ni] << endl;
            return;
        }
        // Pass the image to the SLAM system
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        TrackImage(im, tframe, imageScale);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T) {
            usleep(min((T - ttrack), 0.5) * 1e6);
        }

        if (mnMainLoopSleep != 0) {
            usleep(mnMainLoopSleep * 1e3);
        }

        // Save Traj
        if (ni == nImages - 1) {
            while (mpSLAM->GetCloudMerger()->isRunning()) {
                usleep(10000);
            }
        }

        ros::spinOnce();
    }
}

void Grabber::RunBag(const string &bag_path) {
    ROS_ERROR_STREAM("Begin read bag");
    rosbag::Bag input_bag;
    input_bag.open(bag_path, rosbag::bagmode::Read);
    ROS_ERROR_STREAM("End read bag");

    std::vector<std::string> topics;
    // const string image_topic_name = "/camera/rgb/image_color";
    const string image_topic_name = "/camera/color/image_raw";
    topics.push_back(image_topic_name);

    int nImages = 0;
    float imageScale = mpSLAM->GetImageScale();
    vector<double> vTimestamps;
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    for (auto m : view) {
        sensor_msgs::Image::ConstPtr pImage = m.instantiate<sensor_msgs::Image>();
        if (pImage == nullptr) {
            cerr << endl
                 << "read finish or fail" << endl;
            break;
        }
        nImages++;
        ROS_ERROR_STREAM("load image index: " << nImages);
        vTimestamps.push_back(pImage->header.stamp.toSec());
    }
    mDataDuration = vTimestamps[vTimestamps.size() - 1] - vTimestamps[0];
    mStartTimestamp = vTimestamps[0];

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    ROS_ERROR_STREAM("bag image size: " << vTimestamps.size());

    int ni = 0;
    // rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    for (auto m : view) {
        double tframe = vTimestamps[ni];
        sensor_msgs::Image::ConstPtr pImage = m.instantiate<sensor_msgs::Image>();
        if (pImage == nullptr) {
            cerr << endl
                 << "read finish or fail" << endl;
            break;
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(pImage);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Pass the image to the SLAM system
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        TrackImage(cv_ptr->image, tframe, imageScale);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T) {
            usleep(min((T - ttrack), 0.5) * 1e6);
        }

        if (mnMainLoopSleep != 0) {
            usleep(mnMainLoopSleep * 1e3);
        }

        // Save Traj
        if (ni == nImages - 1) {
            while (mpSLAM->GetCloudMerger()->isRunning()) {
                usleep(10000);
            }
        }

        // usleep(30 * 1e3);

        ni++;

        ros::spinOnce();
    }
}

void Grabber::GrabImage(const sensor_msgs::ImageConstPtr &msg) {
    id = msg->header.seq;
    // ROS_WARN_STREAM("image seq: " << id);

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    float imageScale = mpSLAM->GetImageScale();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    TrackImage(cv_ptr->image, msg->header.stamp.toSec(), imageScale);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
}

void Grabber::ActionFinishCb(const actionlib::SimpleClientGoalState &state, const cloud_edge_slam::CloudSlamResultConstPtr &result) {
    // cout << "begin sleep" << endl;
    // usleep(10000 * 1e3);
    // while (1) {
    //     usleep(1000 * 1e3);
    // }
    // cout << "end sleep" << endl;

    // @note print
    cloud_edge_slam::CloudMapConstPtr mapPtr(new cloud_edge_slam::CloudMap(result->map));
    mpSLAM->InsertCloudMap(ROSMapToORBMap(mapPtr));
    ROS_INFO_STREAM("Cloud Process Finish!");
}

void Grabber::MemoryCb(const std_msgs::Float32ConstPtr &msg) {
    mvMemory.push_back(msg->data);
}

void Grabber::GrabCloudMapCb(const cloud_edge_slam::CloudMapConstPtr &msg) {
    ROS_ERROR_STREAM("Read Cloud Map Start!");

    ORB_SLAM3::Map *cloudMap = ROSMapToORBMap(msg);

    mpSLAM->InsertCloudMap(cloudMap);

    ROS_ERROR_STREAM("Read Cloud Map End!");
}

inline geometry_msgs::Pose toRosPose(const Sophus::SE3f pose) {
    Eigen::Matrix4f T = pose.matrix();

    geometry_msgs::Pose rosPose;
    rosPose.position.x = T(0, 3);
    rosPose.position.y = T(1, 3);
    rosPose.position.z = T(2, 3);
    Eigen::Quaternionf q;
    q = T.block<3, 3>(0, 0);
    rosPose.orientation.x = q.x();
    rosPose.orientation.y = q.y();
    rosPose.orientation.z = q.z();
    rosPose.orientation.w = q.w();

    return rosPose;
}

void Grabber::PubORBMapCb(const std_msgs::Int16ConstPtr &msg) {
    int mapMnId = msg->data;
    ORB_SLAM3::Map *pMap;
    if (mapMnId == -1) {
        pMap = mpSLAM->GetAtlas()->GetCurrentMap();
    } else {
        pMap = mpSLAM->GetAtlas()->GetSpecifyMap(mapMnId);
    }
    mpOrbMapPub->publish(ORBMapToROSMap(pMap));
}

cloud_edge_slam::CloudMap Grabber::ORBMapToROSMap(ORB_SLAM3::Map *pMap) {
    cloud_edge_slam::CloudMap pubTestEdgeMap;

    std::vector<ORB_SLAM3::KeyFrame *> allKeyFrames = pMap->GetAllKeyFrames();
    std::vector<ORB_SLAM3::MapPoint *> allMapPoints = pMap->GetAllMapPoints();

    pubTestEdgeMap.header.seq = 0;
    pubTestEdgeMap.edge_front_map_mnid = pMap->GetId();
    pubTestEdgeMap.edge_back_map_mnid = pMap->GetId() + 1;

    // 提前取得指针与index的对应关系
    std::map<ORB_SLAM3::MapPoint *, int> mapPointMap;
    std::map<ORB_SLAM3::KeyFrame *, int> keyFrameMap;
    for (int mapPoint_i = 0; mapPoint_i < allMapPoints.size(); ++mapPoint_i) { mapPointMap[allMapPoints[mapPoint_i]] = mapPoint_i; }
    for (int keyFrame_i = 0; keyFrame_i < allKeyFrames.size(); ++keyFrame_i) { keyFrameMap[allKeyFrames[keyFrame_i]] = keyFrame_i; }

    for (int mapPoint_i = 0; mapPoint_i < allMapPoints.size(); ++mapPoint_i) {
        ORB_SLAM3::MapPoint *mapPoint = allMapPoints[mapPoint_i];
        std::map<ORB_SLAM3::KeyFrame *, std::tuple<int, int>> observations = mapPoint->GetObservations();

        cloud_edge_slam::MapPoint rosMapPoint;
        rosMapPoint.mnId = mapPoint->mnId;
        rosMapPoint.ref_keyframe_id = keyFrameMap[mapPoint->GetReferenceKeyFrame()]; // TODO check是否是这个
        Eigen::Vector3f point = mapPoint->GetWorldPos();
        rosMapPoint.point.x = point(0);
        rosMapPoint.point.y = point(1);
        rosMapPoint.point.z = point(2);

        for (auto observation : observations) {
            cloud_edge_slam::Observation rosObservation;
            if (keyFrameMap.count(observation.first) >= 1) {
                rosObservation.keyframe_id = keyFrameMap[observation.first]; // Bug! 即使keyFrameMap中没有该KeyFrame，std::map依然会返回0
                rosObservation.refer_keypoint_index = (short)std::get<0>(observation.second);
                if ((short)std::get<0>(observation.second) > observation.first->mvKeys.size()) {
                    ROS_ERROR_STREAM("observation keypoint index: " << (short)std::get<0>(observation.second) << "  " << observation.first->mvKeys.size());
                }
                assert((short)std::get<0>(observation.second) < observation.first->mvKeys.size());
                rosMapPoint.observations.push_back(rosObservation);
            }
        }
        rosMapPoint.num_obs = rosMapPoint.observations.size();
        pubTestEdgeMap.map_points.push_back(rosMapPoint);
    }

    // Add All KeyFrames
    for (auto KF : allKeyFrames) {
        Sophus::SE3f pose = KF->GetPose();
        std::vector<cv::KeyPoint> keyPoints = KF->mvKeys;
        std::vector<ORB_SLAM3::MapPoint *> matchMapPoint = KF->GetMapPointMatches();
        std::vector<cv::Mat> descriptors = ORB_SLAM3::Converter::toDescriptorVector(KF->mDescriptors);
        assert(matchMapPoint.size() == keyPoints.size() && keyPoints.size() == descriptors.size());

        cloud_edge_slam::KeyFrame rosKeyFrame;
        rosKeyFrame.mTimeStamp = KF->mTimeStamp;
        rosKeyFrame.mnId = KF->mnId;
        rosKeyFrame.pose_cw = toRosPose(pose);

        for (const auto &descriptor : descriptors) {
            cloud_edge_slam::Descriptor rosDescriptor;
            //            ROS_ERROR_STREAM("descriptor shape: " << descriptor.size);
            std::vector<double> descriptor_vector;
            descriptor.col(0).copyTo(descriptor_vector);
            std::copy(descriptor_vector.begin(), descriptor_vector.end(), rosDescriptor.descriptor.begin());
            rosKeyFrame.descriptors.push_back(rosDescriptor);
        }

        for (const auto &keyPoint : keyPoints) {
            cloud_edge_slam::KeyPoint rosKeyPoint;
            rosKeyPoint.x = keyPoint.pt.x;
            rosKeyPoint.y = keyPoint.pt.y;
            rosKeyFrame.key_points.push_back(rosKeyPoint);
        }

        for (const auto &mapPoint : matchMapPoint) {
            int matchMapPointIndex = -1;
            if (mapPoint) {
                matchMapPointIndex = mapPointMap[mapPoint];
            }
            rosKeyFrame.mvp_map_points_index.push_back(matchMapPointIndex);
        }
        pubTestEdgeMap.key_frames.push_back(rosKeyFrame);
    }

    //    ROS_ERROR_STREAM("TEST pub msg: " << pubTestEdgeMap.key_frames.size());
    //    ROS_ERROR_STREAM("TEST pub msg: " << pubTestEdgeMap.map_points.size());
    return pubTestEdgeMap;
}

ORB_SLAM3::Map *Grabber::ROSMapToORBMap(cloud_edge_slam::CloudMapConstPtr pMap) {
    ROS_ERROR_STREAM("ROS Map To ORB Map Start!");

    // Cloud Image
    std::map<std::string, cv::Mat> dLastCloudProcessImages = mpSLAM->GetCloudImageSampler()->mdLastCloudProcessImages;

    // hype parameters
    const bool bIncludeDescriptor = false;
    // const bool bIncludeDescriptor = true;

    // SLAM System Info
    auto pVocabulary = mpSLAM->GetVocabulary();
    auto pExtractor = mpSLAM->GetExtractor();
    auto pCamera = mpSLAM->GetCamera();
    auto distCoef = mpSLAM->GetDistCoef();
    auto bf = mpSLAM->Getbf();
    auto thDepth = mpSLAM->GetThDepth();
    auto imuCalib = mpSLAM->GetImcCalib();

    static long unsigned int nCloudMapId = 1000;
    ORB_SLAM3::Map *cloudMap = new ORB_SLAM3::Map(nCloudMapId++, true); // TODO 放入Database中，否则会消亡
    // ORB_SLAM3::Map *cloudMap = new ORB_SLAM3::Map(); // TODO 放入Database中，否则会消亡
    // TODO 将msg的信息打包进Map中
    // TODO 需要添加KeyFrameDatabase，其实初步也不用，只用构建完成后放入队列
    cloudMap->edgeFrontMapMnId = pMap->edge_front_map_mnid;
    cloudMap->edgeBackMapMnId = pMap->edge_back_map_mnid;

    std::vector<ORB_SLAM3::KeyFrame *> vKeyFrames;
    static int gen_start_id = 3565536;
    gen_start_id -= 300000;
    if (gen_start_id < 100000) {
        gen_start_id = 3565536;
    }
    long unsigned int gen_mnId = gen_start_id; // 由于只是临时使用的CloudMap，设置了一个很大的数，只要和Online的不重叠就行
    for (auto &rosKeyFrame : pMap->key_frames) {
        // mvKeys
        std::vector<cv::KeyPoint> vKeyPoints;
        for (auto &rosKeyPoint : rosKeyFrame.key_points) {
            cv::KeyPoint keyPoint;
            keyPoint.pt.x = rosKeyPoint.x;
            keyPoint.pt.y = rosKeyPoint.y;
            vKeyPoints.push_back(keyPoint);
        }

        // Descriptors
        cv::Mat descriptors;
        if (bIncludeDescriptor) { // 是否包含descriptor信息
            std::vector<cv::Mat> vDescriptors;
            for (auto &rosDescriptor : rosKeyFrame.descriptors) {
                std::vector<float> tmpDescriptor(rosDescriptor.descriptor.size());
                copy(rosDescriptor.descriptor.begin(), rosDescriptor.descriptor.end(), tmpDescriptor.begin());
                cv::Mat descriptor(tmpDescriptor);
                vDescriptors.push_back(descriptor);
            }
            descriptors = vDescriptors[0];
            for (int descriptor_i = 1; descriptor_i < vDescriptors.size(); ++descriptor_i) {
                cv::hconcat(descriptors, vDescriptors[descriptor_i], descriptors);
            }
            descriptors = descriptors.t();
        } else { // 若没有就生成假的，保证Keyframe内部的一些调用计算正常运行
            descriptors = cv::Mat::zeros(vKeyPoints.size(), 32, CV_32FC1);
        }

        // pose
        Sophus::SE3f pose = toSophusPose(rosKeyFrame.pose_cw);

        // @note construct frame
        ORB_SLAM3::Frame frame(vKeyPoints, rosKeyFrame.mTimeStamp, pExtractor, pVocabulary, pCamera, distCoef, bf, thDepth);
        frame.mnId = gen_mnId--;
        frame.mDescriptors = descriptors;
        frame.mvpMapPoints = std::vector<ORB_SLAM3::MapPoint *>(rosKeyFrame.mvp_map_points_index.size(), static_cast<ORB_SLAM3::MapPoint *>(NULL));
        ORB_SLAM3::KeyFrame *keyFrame = new ORB_SLAM3::KeyFrame(frame, cloudMap, nullptr); // TODO KeyFrameDatabase
        keyFrame->SetPose(pose);
        keyFrame->SetCloudFlag();

        // @note add img
        if (dLastCloudProcessImages.count(std::to_string(frame.mTimeStamp))) {
            keyFrame->SetImgGray(dLastCloudProcessImages[std::to_string(frame.mTimeStamp)]);
        }

        vKeyFrames.push_back(keyFrame); // 因为还差公视图部分，先不填充到Map中
    }
    std::vector<ORB_SLAM3::MapPoint *> vMapPoints;
    gen_mnId = gen_start_id; // 由于只是临时使用的CloudMap，设置了一个很大的数，只要和Online的不重叠就行
    for (auto &rosMapPoint : pMap->map_points) {
        Eigen::Vector3f pos(rosMapPoint.point.x, rosMapPoint.point.y, rosMapPoint.point.z);
        ORB_SLAM3::KeyFrame *refKeyFrame = vKeyFrames[rosMapPoint.ref_keyframe_id];
        ORB_SLAM3::MapPoint *mapPoint = new ORB_SLAM3::MapPoint(gen_mnId--, pos, refKeyFrame, cloudMap);
        mapPoint->isEdge = false;
        vMapPoints.push_back(mapPoint);
    }

    // 完成初步KeyFrame和MapPoint对象的初始化，接着建立两者间的联系
    // 参考LocalMapping中CreateNewMapPoints、ProcessNewKeyFrame对两者进行处理
    for (int mapPoint_i = 0; mapPoint_i < vMapPoints.size(); ++mapPoint_i) {
        ORB_SLAM3::MapPoint *mapPoint = vMapPoints[mapPoint_i];
        for (auto &observation : pMap->map_points[mapPoint_i].observations) {
            mapPoint->AddObservation(vKeyFrames[observation.keyframe_id], observation.refer_keypoint_index);
            if ((short)observation.refer_keypoint_index > vKeyFrames[observation.keyframe_id]->mvKeys.size()) {
                ROS_ERROR_STREAM("observation keypoint index: " << (short)observation.refer_keypoint_index << "  " << vKeyFrames[observation.keyframe_id]->mvKeys.size());
            }
            assert((short)observation.refer_keypoint_index < vKeyFrames[observation.keyframe_id]->mvKeys.size());
            //            ROS_ERROR_STREAM("observation keypoint index: " << vKeyFrames[observation.keyframe_id]->mvKeys.size() << "  " << observation.refer_keypoint_index);
        }
        mapPoint->ComputeDistinctiveDescriptors();
        mapPoint->UpdateNormalAndDepth();
    }
    for (int keyFrame_i = 0; keyFrame_i < vKeyFrames.size(); ++keyFrame_i) {
        ORB_SLAM3::KeyFrame *keyFrame = vKeyFrames[keyFrame_i];
        keyFrame->ComputeBoW();
        int i = 0;
        for (auto &matchMapPointIndex : pMap->key_frames[keyFrame_i].mvp_map_points_index) {
            if (matchMapPointIndex != -1)
                keyFrame->AddMapPoint(vMapPoints[matchMapPointIndex], i);
            i++;
        }
        keyFrame->UpdateCloudConnections();
    }

    // 处理完成后，添加到Map中
    for (auto &keyFrame : vKeyFrames) {
        cloudMap->AddKeyFrame(keyFrame);
    }
    for (auto &mapPoint : vMapPoints) {
        cloudMap->AddMapPoint(mapPoint);
    }

    ROS_ERROR_STREAM("ROS Map To ORB Map End!");

    return cloudMap;
}

void Grabber::LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps) {
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    while (!f.eof()) {
        string s;
        getline(f, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

void Grabber::SaveORBMapCb(const std_msgs::Int16ConstPtr &msg) {
    int mapMnId = msg->data;
    ORB_SLAM3::Map *pMap;
    if (mapMnId == -1) {
        pMap = mpSLAM->GetAtlas()->GetCurrentMap();
    } else {
        pMap = mpSLAM->GetAtlas()->GetSpecifyMap(mapMnId);
    }
    WriteCloudMapBag(ORBMapToROSMap(pMap), "/home/red0orange/github_projects/Cloud-Edge-SLAM/src/cloud_edge_slam/TestData/All_ORB_Test_Offline_Map/" + to_string(mapMnId) + ".bag");
}

void Grabber::WriteCloudMapBag(cloud_edge_slam::CloudMap map, const std::string &save_path) {
    rosbag::Bag bag;
    bag.open(save_path, rosbag::bagmode::Write);
    bag.write("/test_cloud_map", ros::Time::now(), map);
    bag.close();
}

void Grabber::WriteCloudImagesBag(std::vector<ORB_SLAM3::CloudImage> &vImages, const std::string &save_path) {
    rosbag::Bag bag;
    bag.open(save_path, rosbag::bagmode::Write);
    for (auto &image : vImages) {
        std_msgs::Header header;
        header.stamp = ros::Time(image.timestamp);
        header.frame_id = image.type;
        string encode;
        if (image.img.channels() == 3)
            encode = "bgr8";
        else if (image.img.channels() == 1)
            encode = "mono8";
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, encode, image.img).toImageMsg();
        bag.write("/test_cloud_images", ros::Time(image.timestamp), msg);
    }
    bag.close();
}

void Grabber::WriteSeqBag(const cloud_edge_slam::Sequence &seq, const std::string &save_path) {
    rosbag::Bag bag;
    bag.open(save_path, rosbag::bagmode::Write);
    auto curTime = ros::Time::now();
    bag.write("/test_seq", curTime, seq);
    bag.close();
}
