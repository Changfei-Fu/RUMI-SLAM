#ifndef CLOUDIMAGESAMPLER_H
#define CLOUDIMAGESAMPLER_H

#include <deque>
#include <map>
#include <opencv2/core/core.hpp>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "KFDSample.h"
#include "System.h"

namespace ORB_SLAM3 {
class CloudImage {
public:
    CloudImage(cv::Mat img, double timestamp, std::string type);

public:
    cv::Mat img;
    double timestamp;
    std::string type;
};

class CloudImageSampler {
public:
    CloudImageSampler(System *pSys, int nTrackLast, int nNewTrackFirst, float nTrackLastMinTime, float nNewTrackFirstMinTime, float Kp, float Kd, float th);
    ~CloudImageSampler();

    // 主函数，保持和Tracking线程同等节拍，通过该函数约束信息输入
    void TrackStep(const int &status, const bool &bIsKF, const int &nKFInMap, const int &curMapId, const cv::Mat &img, const double &timestamp);

    // KF过少或持续时间过短的Map会被重置，重置的KF将作为Lost Images
    void UpdateResetMapImages(Map *pMap);

    // const std::vector<CloudImage> &GetCurrentCloudProcessImages();
    // const std::vector<CloudImage> &GetLastCloudProcessImages();

    // 给Cloud Map填充Images
    std::map<std::string, cv::Mat> mdLastCloudProcessImages;

private:
    // 当一次Cloud Map上传完成，Reset current相关的变量
    void Reset();

private:
    enum eSamplerState {
        OLD_MAP_SAMPLE = 0,
        NEW_MAP_SAMPLE = 1,
    };

    System *mpSystem;
    KFDSample *mpKFDSampler;

    int mState;
    int mTrackingLastState;

    std::vector<CloudImage> mvLostImages;
    std::vector<CloudImage> mvLostNoSamplingImages;
    int mEdgeFrontMapId = -1;
    int mEdgeBackMapId = -1;

    // 拼接三个阶段的Images，得到最终上传的Image，作为当前Cloud处理的Image
    std::vector<CloudImage> mvCurrentCloudProcessImages;
    std::vector<CloudImage> mvCurrentCloudProcessNoSamplingImages;
    // 由于vCurrentCloudProcessImages在upload后会马上clear以保证vTrackLastNImages继续正常工作，但Cloud download时有可能使用原来的Images，这里保存Last Images
    std::vector<CloudImage> mvLastCloudProcessImages;

public:
    // param
    int nTrackLast;
    int nNewTrackFirst;
    float nNewTrackFirstMinTime;
    float nTrackLastMinTime;
    int nMaxTrackLast = 50;
    int nMaxNewTrackFirst = 50;
    // float nNewTrackFirstMinTrajCurvature = 1.2;
    // float nTrackLastMinTrajCurvature = 1.2;
    float nNewTrackFirstMinTrajCurvature = 0.0;
    float nTrackLastMinTrajCurvature = 0.0;
};

} // namespace ORB_SLAM3

#endif