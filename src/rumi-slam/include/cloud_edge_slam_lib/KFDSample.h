#ifndef KFDSAMPLER_HPP
#define KFDSAMPLER_HPP

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBextractor.h"
#include "pd.hpp"

using namespace cv;
using namespace std;

void Printinfo(float Kp, float Kd, float setpoint);
float Calmoptflmag(vector<Point2f> prvs, vector<Point2f> next, const int nkpt);

class KFDSample {
private:
    // Tracking points
    vector<Point2f> old, next, good_old, good_next;
    // ORBextractor
    ORB_SLAM3::ORBextractor *mpORBextractor;
    // ORBextractor Parameters Default for TUM1 Dataset
    float scaleFactor = 1.2;
    int nfeatures = 2000, nlevels = 8, iniThFAST = 20, minThFAST = 7;
    // PD Controller Param
    float Kp = 0.8, Kd = 0.005;
    // Tracking Keypoints and Descriptors
    vector<KeyPoint> mvKeys;
    Mat mDescriptors;
    // Image
    Mat frame1, imprvs;
    Mat frame2, imnext;
    // PD controller
    PD *mpPDcontroller;
    // Timestamp
    double ltframe = 0;
    // Mean Optical Flow
    float moptf = 0, th = 10;
    // status
    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 20, 0.03);
    // Display
    bool show = false;
    // keyframe Set
    vector<Mat> KFset;
    //
    vector<int> vLapping = {0, 0};

public:
    // Constructor Initialization
    KFDSample(int nfeatures, int nlevels, int iniThFAST, int minThFAST, float scaleFactor, Mat &Frame, float TimeStamp);
    KFDSample();
    // Destructor
    ~KFDSample() = default;
    // ORBextractor Init
    void InitORBextractor(const int nfeatures, const float scaleFactor, const int nlevels, const int iniThFAST, const int minThFAST) {
        this->mpORBextractor = new ORB_SLAM3::ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
    }
    // Init PD keyframe select controller
    void InitPDKFselector(float Kp, float Kd, float th);
    void InitPDKFselector();
    void SetPDKFselectorParams(float Kp, float Kd, float th);

    Mat GetKF();
    vector<Mat> GetAllKF();

    // set threshold
    void SetThreshold(const float TH);

    void SetDisplay();

    // Input Frame
    void SetNextFrame(const Mat &InputArray);
    // Select Good points After tracking
    void SelectGoodPts();

    void Reset();
    bool Step(const Mat &inputIm, double timeStamp);
};

#endif