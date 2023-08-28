#include "UmeyamaSim3Solver.h"

#include <Eigen/src/Core/ArithmeticSequence.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Umeyama.h>
#include <cstddef>
#include <cstdlib>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace ORB_SLAM3 {

UmeyamaSim3Solver::UmeyamaSim3Solver() {
}

UmeyamaSim3Solver::UmeyamaSim3Solver(const std::vector<KeyFrame *> &map1KFs, const std::vector<KeyFrame *> &map2KFs, const std::vector<std::vector<std::pair<int, int>>> &avpValidKPMatches) :
    mnInliersiRatio(0), mnBestInliersRatio(0) {
    assert(map1KFs.size() == map2KFs.size());
    mN1 = map1KFs.size();
    mvpKFPair.reserve(mN1);
    mvAllIndices.reserve(mN1);

    mavpValidKPMatches = avpValidKPMatches;
    size_t idx = 0;
    for (int i = 0; i < map1KFs.size(); i++) {
        if (!map1KFs[i] || !map2KFs[i])
            exit(100001);
        mvpKFPair.push_back(make_pair(map1KFs[i], map2KFs[i]));
        mvAllIndices.push_back(idx);
        idx++;
    }
}

g2o::Sim3 UmeyamaSim3Solver::umeyamaSolve(const vector<Eigen::Vector3d> &srcMatchPoints, const vector<Eigen::Vector3d> &dstMatchPoints) {
    assert(srcMatchPoints.size() == dstMatchPoints.size());
    Eigen::Matrix<double, 3, Eigen::Dynamic> srcMapPosePosition(3, srcMatchPoints.size());
    Eigen::Matrix<double, 3, Eigen::Dynamic> dstMapPosePosition(3, dstMatchPoints.size());
    for (unsigned long i = 0; i < srcMatchPoints.size(); i++) {
        auto srcPosition = srcMatchPoints[i];
        auto dstPosition = dstMatchPoints[i];

        srcMapPosePosition(0, i) = srcPosition(0);
        srcMapPosePosition(1, i) = srcPosition(1);
        srcMapPosePosition(2, i) = srcPosition(2);
        dstMapPosePosition(0, i) = dstPosition(0);
        dstMapPosePosition(1, i) = dstPosition(1);
        dstMapPosePosition(2, i) = dstPosition(2);
    }
    Eigen::Matrix4d result = Eigen::umeyama(srcMapPosePosition, dstMapPosePosition, true);

    Sophus::Sim3d sSc1c2(result.cast<double>());
    Eigen::Matrix3d tmp_R = sSc1c2.rotationMatrix();
    Eigen::Vector3d tmp_T = sSc1c2.translation();
    double tmp_s = sSc1c2.scale();
    g2o::Sim3 gUmeyamaSw1w2 = g2o::Sim3(tmp_R, tmp_T, tmp_s); // g2o::Sim3: kf camera 2 -> kf camera 1，是输入求解的两个关键帧之间的Sim3变换，注意是2->1
    return gUmeyamaSw1w2;
}

float UmeyamaSim3Solver::ComputeInliersNum(const std::vector<KeyFrame *> &map1KFs, const std::vector<KeyFrame *> &map2KFs, const std::vector<std::vector<std::pair<int, int>>> &avpValidKPMatches, g2o::Sim3 &gSw1w2) {
    vector<vector<Eigen::Vector3f>> avpOriginX3D1;
    vector<vector<Eigen::Vector2f>> avpOriginlim1;
    vector<vector<Eigen::Vector2f>> avpProjectlim1;
    vector<vector<Eigen::Vector3f>> avpOriginX3D2;
    vector<vector<Eigen::Vector2f>> avpOriginlim2;
    vector<vector<Eigen::Vector2f>> avpProjectlim2;

    vector<float> avInliersRatio;
    vector<vector<bool>> avbInliersi;

    for (size_t kf_i = 0; kf_i < map1KFs.size(); kf_i++) {
        vector<Eigen::Vector3f> vpOriginX3D1;
        vector<Eigen::Vector2f> vpOriginlim1;
        vector<Eigen::Vector2f> vpProjectlim1;
        vector<Eigen::Vector3f> vpOriginX3D2;
        vector<Eigen::Vector2f> vpOriginlim2;
        vector<Eigen::Vector2f> vpProjectlim2;
        vector<bool> vbInliersi;

        KeyFrame *KF1 = map1KFs[kf_i];
        KeyFrame *KF2 = map2KFs[kf_i];

        if (!KF1 || !KF2) {
            cerr << "Empty KF, error if large num! " << endl;
            continue;
        }

        const std::vector<std::pair<int, int>> vpValidKPMatches = avpValidKPMatches[kf_i];

        Sophus::SE3f Tc1w1 = KF1->GetPose();
        Sophus::SE3f Tc2w2 = KF2->GetPose();
        g2o::Sim3 gSc1w1(Tc1w1.rotationMatrix().cast<double>(), Tc1w1.translation().cast<double>(), 1.0);
        g2o::Sim3 gSc2w2(Tc2w2.rotationMatrix().cast<double>(), Tc2w2.translation().cast<double>(), 1.0);
        const vector<MapPoint *> vpMPs1 = KF1->GetMapPointMatches();
        const vector<MapPoint *> vpMPs2 = KF2->GetMapPointMatches();
        const std::vector<cv::KeyPoint> vKPs1 = KF1->mvKeys;
        const std::vector<cv::KeyPoint> vKPs2 = KF2->mvKeys;

        int nInliersi = 0;
        for (size_t matchMP_i = 0; matchMP_i < vpValidKPMatches.size(); matchMP_i++) {
            int mp1_i = vpValidKPMatches[matchMP_i].first;
            int mp2_i = vpValidKPMatches[matchMP_i].second;

            if (!vpMPs1[mp1_i] || !vpMPs2[mp2_i]) {
                cerr << "Empty Map Points, error if large num! " << endl;
                continue;
            }

            vpOriginlim1.push_back(Eigen::Vector2f(vKPs1[mp1_i].pt.x, vKPs1[mp1_i].pt.y));
            vpOriginX3D1.push_back(vpMPs1[mp1_i]->GetWorldPos());
            vpOriginlim2.push_back(Eigen::Vector2f(vKPs2[mp2_i].pt.x, vKPs2[mp2_i].pt.y));
            vpOriginX3D2.push_back(vpMPs2[mp2_i]->GetWorldPos());

            g2o::Sim3 gSc2w1 = gSc2w2 * gSw1w2.inverse();
            g2o::Sim3 gSc1w2 = gSc1w1 * gSw1w2;

            vpProjectlim1.push_back(KF1->mpCamera->project(gSc1w2.map(vpMPs2[mp2_i]->GetWorldPos().cast<double>())).cast<float>());
            Eigen::Vector2f dist1 = vpOriginlim1[vpOriginlim1.size() - 1] - vpProjectlim1[vpProjectlim1.size() - 1];
            const float err1 = dist1.dot(dist1);

            vpProjectlim2.push_back(KF2->mpCamera->project(gSc2w1.map(vpMPs1[mp1_i]->GetWorldPos().cast<double>())).cast<float>());
            Eigen::Vector2f dist2 = vpOriginlim2[vpOriginlim2.size() - 1] - vpProjectlim2[vpProjectlim2.size() - 1];
            const float err2 = dist2.dot(dist2);

            // Cloud部分投影Inlier即可
            if ((err1 < (2 * 9.210 * KF1->mvLevelSigma2[vKPs1[mp1_i].octave]) || vpMPs2[mp2_i]->isEdge) && (err2 < (2 * 9.210 * KF2->mvLevelSigma2[vKPs2[mp2_i].octave]) || vpMPs1[mp1_i]->isEdge)) {
                vbInliersi.push_back(true);
                nInliersi++;
            } else
                vbInliersi.push_back(false);
        }

        avpOriginX3D1.push_back(vpOriginX3D1);
        avpOriginlim1.push_back(vpOriginlim1);
        avpProjectlim1.push_back(vpProjectlim1);
        avpOriginX3D2.push_back(vpOriginX3D2);
        avpOriginlim2.push_back(vpOriginlim2);
        avpProjectlim2.push_back(vpProjectlim2);
        avbInliersi.push_back(vbInliersi);
        if (vpValidKPMatches.size())
            avInliersRatio.push_back((float)nInliersi / (float)vpValidKPMatches.size());
        else
            avInliersRatio.push_back(0);
    }
    // 返回所有match KF的Inliers ratio的中位数
    struct compFunctor {
        inline bool operator()(float elem1, float elem2) {
            return elem1 < elem2;
        }
    };
    vector<float> avMedianInliersRatio(avInliersRatio.begin(), avInliersRatio.end());
    sort(avMedianInliersRatio.begin(), avMedianInliersRatio.end(), compFunctor());
    cerr << "Inliers Ratio: ";
    for (auto &i : avMedianInliersRatio)
        cerr << i << " ";
    cerr << endl;

    // return avMedianInliersRatio[int(avMedianInliersRatio.size() / 2)];
    return avMedianInliersRatio[avMedianInliersRatio.size() - 1];
}

void UmeyamaSim3Solver::SetRansacParameters(double probability, float minInliersRatio, int maxIterations, int selectKFNum) {
    mRansacProb = probability;
    mRansacMinInliersRatio = minInliersRatio;
    mRansacMaxIts = maxIterations;
    mSelectKFNum = selectKFNum;

    N = mvpKFPair.size(); // number of correspondences

    // Adjust Parameters according to number of correspondences
    float epsilon = minInliersRatio;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if (minInliersRatio == 1)
        nIterations = 1;
    else
        nIterations = ceil(log(1 - mRansacProb) / log(1 - pow(epsilon, 3)));

    mRansacMaxIts = max(1, min(nIterations, mRansacMaxIts));

    mnIterations = 0;
}

g2o::Sim3 UmeyamaSim3Solver::iterate(int nIterations, bool &bNoMore, float &nInliersRatio, bool &bConverge) {
    bNoMore = false;
    bConverge = false;
    nInliersRatio = 0;

    int nCurrentIterations = 0;

    g2o::Sim3 bestSim3;
    while (mnIterations < mRansacMaxIts && nCurrentIterations < nIterations) {
        nCurrentIterations++;
        mnIterations++;

        // Rand Select KF Pairs
        std::vector<std::vector<std::pair<int, int>>> avpKPMatches;
        vector<size_t> vAvailableIndices = vector<size_t>(mvAllIndices.begin(), mvAllIndices.end());
        std::vector<std::pair<KeyFrame *, KeyFrame *>> vpAvailableKFPair;
        for (short i = 0; i < mSelectKFNum; ++i) {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
            int idx = vAvailableIndices[randi];

            vpAvailableKFPair.push_back(mvpKFPair[idx]);
            avpKPMatches.push_back(mavpValidKPMatches[idx]);

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // Compute Sim3
        vector<Eigen::Vector3d> map1MatchKFPoints;
        vector<Eigen::Vector3d> map2MatchKFPoints;
        for (auto &iter : vpAvailableKFPair) {
            map1MatchKFPoints.push_back(iter.first->GetPoseInverse().translation().cast<double>());
            map2MatchKFPoints.push_back(iter.second->GetPoseInverse().translation().cast<double>());
        }
        g2o::Sim3 gSw1w2 = umeyamaSolve(map2MatchKFPoints, map1MatchKFPoints);

        // Compute Inliers
        std::vector<KeyFrame *> map1KFs;
        std::vector<KeyFrame *> map2KFs;
        for (auto &iter : vpAvailableKFPair) {
            map1KFs.push_back(iter.first);
            map2KFs.push_back(iter.second);
        }

        // Compute Inliers
        mnInliersiRatio = ComputeInliersNum(map1KFs, map2KFs, avpKPMatches, gSw1w2);

        // Judge if good enough
        if (mnInliersiRatio >= mnBestInliersRatio) {
            mnBestInliersRatio = mnInliersiRatio;

            nInliersRatio = mnInliersiRatio;
            if (mnInliersiRatio > mRansacMinInliersRatio) {
                bConverge = true;
                return mBestSim3;
            } else {
                bestSim3 = mBestSim3;
            }
        }
    }

    if (mnIterations >= mRansacMaxIts)
        bNoMore = true;

    return bestSim3;
}

} // namespace ORB_SLAM3