#ifndef UMEYAMASIM3SOLVER_H
#define UMEYAMASIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"

namespace ORB_SLAM3 {

class UmeyamaSim3Solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    UmeyamaSim3Solver();
    UmeyamaSim3Solver(const std::vector<KeyFrame *> &map1KFs, const std::vector<KeyFrame *> &map2KFs, const std::vector<std::vector<std::pair<int, int>>> &avpValidKPMatches);

    void SetRansacParameters(double probability = 0.99, float minInliersRatio = 0.3, int maxIterations = 300, int selectKFNum = 15);

    g2o::Sim3 iterate(int nIterations, bool &bNoMore, float &nInliersRatio, bool &bConverge);

    static g2o::Sim3 umeyamaSolve(const vector<Eigen::Vector3d> &srcMatchPoints, const vector<Eigen::Vector3d> &dstMatchPoints);
    static float ComputeInliersNum(const std::vector<KeyFrame *> &map1KFs, const std::vector<KeyFrame *> &map2KFs, const std::vector<std::vector<std::pair<int, int>>> &avpValidKPMatches, g2o::Sim3 &gSw1w2);

private:
    // Indices for random selection
    std::vector<size_t> mvAllIndices;
    std::vector<std::pair<KeyFrame *, KeyFrame *>> mvpKFPair;
    std::vector<std::vector<std::pair<int, int>>> mavpValidKPMatches;

    int mnIterations;
    int mnInliersiRatio;

private:
    int N;
    int mN1;

    // Select KF Pair Num
    int mSelectKFNum;
    int mnBestInliersRatio;

    // RANSAC probability
    double mRansacProb;

    // RANSAC min inliers
    float mRansacMinInliersRatio;

    // RANSAC max iterations
    int mRansacMaxIts;

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;
    float mSigma2;

    // Best Sim3
    g2o::Sim3 mBestSim3;
};

} // namespace ORB_SLAM3

#endif