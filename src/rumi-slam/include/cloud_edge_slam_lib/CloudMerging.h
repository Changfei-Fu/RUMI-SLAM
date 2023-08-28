#ifndef CLOUDMERGING_H
#define CLOUDMERGING_H

#include "FrameDrawer.h"
#include "KeyFrame.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "LocalMapping.h"
#include "Tracking.h"
#include "LoopClosing.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3 {

class Tracking;
class LocalMapping;
class LoopClosing;
class KeyFrameDatabase;
class Map;
class MapDrawer;
class FrameDrawer;

class CloudMerging {
public:
    typedef pair<set<KeyFrame *>, int> ConsistentGroup;
    typedef map<KeyFrame *, g2o::Sim3, std::less<KeyFrame *>,
                Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3>>>
        KeyFrameAndPose;

public:
    CloudMerging(
        Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, const bool bActiveLC,
        const bool bWork, const bool bMergeAnyway,       // for parameters
        MapDrawer *pMapDrawer, FrameDrawer *pFrameDrawer // for debug
    );

    void SetTracker(Tracking *pTracker);
    void SetLocalMapper(LocalMapping *pLocalMapper);
    void SetLoopClosing(LoopClosing *pLoopClosing);

    // Get Variables
    Map *GetEdgeFrontMap();
    Map *GetEdgeBackMap();
    Map *GetCloudMap();
    std::map<KeyFrame *, KeyFrame *> mdSolveSim3MatchKeyFrames; // Debug: 用于测试Solve Sim3
    std::map<MapPoint *, MapPoint *> mdSolveSim3MatchMapPoints; // Debug: 用于测试Solve Sim3

    // Main function
    void Run(bool bOnline = true);

    void InsertCloudMap(Map *pMap);

    void RequestReset();
    void RequestResetActiveMap(Map *pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(Map *pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA() {
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA() {
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }

    void RequestFinish();

    bool isFinished();

    bool isRunning();

    Viewer *mpViewer;

    std::vector<double> mvpTestCloudMapTimeStamps; // For Test

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    bool CheckNewCloudMap(bool bOnline);

    // Our Methods to implement the Cloud-Ground Map Match
    static bool ComputeSubmapSim3(
        Map *pMap1, Map *pMap2, const std::map<int, int> &kfMatch12,                   // input
        const std::map<int, int> &vRandSelectKfMatch12,                                // input
        bool bFixScale,                                                                // input
        g2o::Sim3 &gSw1w2,                                                             // output
        std::map<KeyFrame *, std::vector<std::pair<int, int>>> &mvpMatchedKeyPoints12, // output
        MapDrawer *pMapDrawer                                                          // debug
    );

    // Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();
    bool DetectAndReffineSim3FromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<KeyFrame *> &vpBowCand, KeyFrame *&pMatchedKF, KeyFrame *&pLastCurrentKF, g2o::Sim3 &g2oScw,
                                    int &nNumCoincidences, std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs);
    bool DetectCommonRegionsFromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                       std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs);
    int FindMatchesByProjection(KeyFrame *pCurrentKF, KeyFrame *pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<MapPoint *> &spMatchedMPinOrigin, vector<MapPoint *> &vpMapPoints,
                                vector<MapPoint *> &vpMatchedMapPoints);

    static void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint *> &vpMapPoints);
    void SearchAndFuse(const vector<KeyFrame *> &vConectedKFs, vector<MapPoint *> &vpMapPoints);

    static void CloudMergeMap(
        Map *pMainMap, Map *pMergedMap,
        g2o::Sim3 gSwMainMerged,
        const std::map<int, int> &pMainMergedKeyFrameMatch,
        std::map<KeyFrame *, std::vector<std::pair<int, int>>> &vpMainMergedMapPointsMatch,
        LocalMapping *pLocalMapper, bool blockLocalMapper);

    void CheckObservations(set<KeyFrame *> &spKFsMap1, set<KeyFrame *> &spKFsMap2);

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    Map *mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    bool mbRunning;
    std::mutex mMutexFinish;

    Atlas *mpAtlas;
    Tracking *mpTracker;

    KeyFrameDatabase *mpKeyFrameDB;
    ORBVocabulary *mpORBVocabulary;

    LocalMapping *mpLocalMapper;
    LoopClosing *mpLoopClosing;

    MapDrawer *mpMapDrawer;
    FrameDrawer *mpFrameDrawer;

    std::list<Map *> mlpCloudMapQueue;
    std::list<Map *> dmlpCloudMapQueue; // for debug
    std::mutex mMutexCloudQueue;        // mutex 当队列在使用，这个队列是涉及多线程所以要这样

    // Start Cloud variables
    Map *mpCurrentCloudMap;
    Map *mpCurrentEdgeFrontMap;
    Map *mpCurrentEdgeBackMap;

    g2o::Sim3 mgSwEdgeFrontCloud;
    g2o::Sim3 mgSwNewCloudEdgeBack;

    // End Cloud variables

    std::map<int, int> mpEdgeFrontCloudKeyFrameMatch;
    std::map<KeyFrame *, std::vector<std::pair<int, int>>> mvpEdgeFrontCloudMatchedKeyPoints;
    std::map<int, int> mpNewEdgeFrontEdgeBackKeyFrameMatch;
    std::map<KeyFrame *, std::vector<std::pair<int, int>>> mvpNewEdgeFrontEdgeBackMatchedKeyPoints;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame *mpCurrentKF;
    KeyFrame *mpLastCurrentKF;
    KeyFrame *mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame *> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame *> mvpCurrentConnectedKFs;
    std::vector<MapPoint *> mvpCurrentMatchedPoints;
    std::vector<MapPoint *> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    //-------
    Map *mpLastMap;

    bool mbWork;
    bool mbMergeAnyway;

    // ***************** Map merge关键变量，和上面Loop closure保持一致 *****************
    bool mbMergeDetected;
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    KeyFrame *mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;
    g2o::Sim3 mg2oMergeScw;
    KeyFrame *mpMergeMatchedKF;
    std::vector<MapPoint *> mvpMergeMPs;
    std::vector<MapPoint *> mvpMergeMatchedMPs;
    std::vector<KeyFrame *> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread *mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;

    bool mnFullBAIdx;

    vector<double> vdPR_CurrentTime;
    vector<double> vdPR_MatchedTime;
    vector<int> vnPR_TypeRecogn;

    //DEBUG
    string mstrFolderSubTraj;
    int mnNumCorrection;
    int mnCorrectionGBA;

    // To (de)activate LC
    bool mbActiveCM = true;

#ifdef REGISTER_LOOP
    string mstrFolderLoop;
#endif
};

} // namespace ORB_SLAM3
#endif