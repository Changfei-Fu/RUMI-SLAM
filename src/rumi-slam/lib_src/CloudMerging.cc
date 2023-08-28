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

#include "CloudMerging.h"
#include "globals.h"

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Sim3Solver.h"
#include "UmeyamaSim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include "G2oTypes.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "opencv2/core/persistence.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "sophus/sim3.hpp"

#include <Eigen/src/Core/ArithmeticSequence.h>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <sstream>
#include <utility>

namespace ORB_SLAM3 {

CloudMerging::CloudMerging(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, const bool bActiveLC, const bool bWork, const bool bMergeAnyway, MapDrawer *pMapDrawer, FrameDrawer *pFrameDrawer) :
    mbResetRequested(false), mbResetActiveMapRequested(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(nullptr), mbFixScale(bFixScale), mnFullBAIdx(0), mnMergeNumCoincidences(0),
    mbMergeDetected(false), mnMergeNumNotFound(0), mbActiveCM(bActiveLC), mbMergeAnyway(bMergeAnyway), mbWork(bWork),
    mpMapDrawer(pMapDrawer), mpFrameDrawer(pFrameDrawer) {
    mnCovisibilityConsistencyTh = 3;
    mpLastCurrentKF = static_cast<KeyFrame *>(NULL);

    mstrFolderSubTraj = "SubTrajectories/";
    mnNumCorrection = 0;
    mnCorrectionGBA = 0;

    // TODO 临时用于测试
    // const std::string testCloudMapTimeStampsPath = "/home/red0orange/Data/TUM/rgbd_dataset_freiburg3_structure_texture_near/rgb_cloudedge_dvoid.txt";
    // ifstream infile;
    // infile.open(testCloudMapTimeStampsPath, ios::in);
    // if (!infile.is_open()) {
    //     cerr << "读取文件失败" << endl;
    //     exit(100);
    // }
    // vector<string> vLines;
    // string tmpLine;
    // while (getline(infile, tmpLine))
    //     vLines.push_back(tmpLine);
    // infile.close();
    // for (int i = 0; i < vLines.size(); ++i) {
    //     string line = vLines[i];
    //     istringstream str(line);
    //     string timeStamp;
    //     str >> timeStamp;
    //     mvpTestCloudMapTimeStamps.push_back(stod(timeStamp));
    // }
}

void CloudMerging::SetTracker(Tracking *pTracker) {
    mpTracker = pTracker;
}

void CloudMerging::SetLocalMapper(LocalMapping *pLocalMapper) {
    mpLocalMapper = pLocalMapper;
}

void CloudMerging::SetLoopClosing(LoopClosing *pLoopClosing) {
    mpLoopClosing = pLoopClosing;
}

void CloudMerging::Run(bool bOnline) {
    mbFinished = false;

    // 限制最大的match KF num，防止Cloud SLAM运行时本地进行了Loop Closure的清空
    int nLimitMaxMatchKFNum = 40;

    while (1) {
        // ***************** 第一步，检查是否有CloudMap *****************
        if (CheckNewCloudMap(bOnline) && mbWork) {
            static int nCloudMerge = 0;
            nCloudMerge++;

            mbRunning = true;
            // ***************** 第二步，准备数据，包括取出地图、匹配timestamp *****************
            {
                unique_lock<mutex> lock(mMutexCloudQueue);
                if (bOnline) {
                    mpCurrentCloudMap = mlpCloudMapQueue.front();
                    mlpCloudMapQueue.pop_front();
                    mpCurrentEdgeFrontMap = mpAtlas->GetSpecifyMap(mpCurrentCloudMap->edgeFrontMapMnId);
                    mpCurrentEdgeBackMap = mpAtlas->GetSpecifyMap(mpCurrentCloudMap->edgeBackMapMnId);
                    // @note 当前测试固定是0、1 Map
                    // mpCurrentEdgeFrontMap = mpAtlas->GetSpecifyMap(0);
                    // mpCurrentEdgeBackMap = mpAtlas->GetSpecifyMap(1);
                } else {
                    // 取出地图
                    mpCurrentCloudMap = mlpCloudMapQueue.front();
                    mlpCloudMapQueue.pop_front();
                    // TODO Debug For Offline
                    mpCurrentEdgeFrontMap = mlpCloudMapQueue.front();
                    mlpCloudMapQueue.pop_front();
                    mpCurrentEdgeBackMap = mlpCloudMapQueue.front();
                    mlpCloudMapQueue.pop_front();

                    mpAtlas->InsertNewMap(mpCurrentCloudMap);
                    mpAtlas->InsertNewMap(mpCurrentEdgeBackMap);
                    mpAtlas->InsertNewMap(mpCurrentEdgeFrontMap);
                }
            }

            if (!mpCurrentEdgeFrontMap || !mpCurrentEdgeBackMap) {
                cerr << "can't find the refer submap !" << endl;
                exit(1000);
            }

            SaveKeyFrameTrajectoryTUM(mpCurrentCloudMap, (full_path_ / "before_cloud.txt").string());
            SaveKeyFrameTrajectoryTUM(mpCurrentEdgeFrontMap, (full_path_ / "before_edge_front.txt").string());
            SaveKeyFrameTrajectoryTUM(mpCurrentEdgeBackMap, (full_path_ / "before_edge_back.txt").string());

            // 这里取出Map的KeyFrames进行匹配，可能会和LoopClosing线程Loop Closing或者Merge时的Map Update冲突
            // 使用time stamp匹配keyframe
            double matchToleranceTime = 0.0001;
            const vector<KeyFrame *> &edgeFrontMapKeyFrames = mpCurrentEdgeFrontMap->GetAllKeyFrames();
            const vector<KeyFrame *> &cloudMapKeyFrames = mpCurrentCloudMap->GetAllKeyFrames();
            vector<bool> edgeFrontHaveMatchFlag(cloudMapKeyFrames.size(), false); // 避免重复匹配

            // @note keyframe matched by timestamp
            // output: mpEdgeFrontCloudKeyFrameMatch
            for (unsigned long edgeFrontKF_i = 0; edgeFrontKF_i < edgeFrontMapKeyFrames.size(); edgeFrontKF_i++) {
                for (unsigned long cloudKF_i = 0; cloudKF_i < cloudMapKeyFrames.size(); cloudKF_i++) {
                    if (edgeFrontHaveMatchFlag[cloudKF_i]) { continue; }
                    double deltaTime = abs(edgeFrontMapKeyFrames[edgeFrontKF_i]->mTimeStamp - cloudMapKeyFrames[cloudKF_i]->mTimeStamp);
                    if (deltaTime < matchToleranceTime) {
                        mpEdgeFrontCloudKeyFrameMatch[edgeFrontKF_i] = cloudKF_i;
                        edgeFrontHaveMatchFlag[cloudKF_i] = true;
                        break;
                    }
                }
            }
            if (mpEdgeFrontCloudKeyFrameMatch.size() < 1) {
                cerr << "匹配EdgeFront和Cloud Map的KeyFrames数量过少" << endl;
                return;
            }

            // ***************** 第三步，对齐Cloud Map和Ground Map，计算Sim3 *****************
            std::vector<int> vEdgeFrontMatchIndexes;
            for (auto &iter : mpEdgeFrontCloudKeyFrameMatch) vEdgeFrontMatchIndexes.push_back(iter.first);
            sort(vEdgeFrontMatchIndexes.begin(), vEdgeFrontMatchIndexes.end(), [&](int x, int y) { return edgeFrontMapKeyFrames[x]->mTimeStamp < edgeFrontMapKeyFrames[y]->mTimeStamp; });
            // @note 暂时关闭这个处理
            // std::vector<int> vEdgeFrontMatchPartIndexes(vEdgeFrontMatchIndexes.begin(), vEdgeFrontMatchIndexes.begin() + int(mpEdgeFrontCloudKeyFrameMatch.size() * (8. / 10.)));
            std::vector<int> vEdgeFrontMatchPartIndexes(vEdgeFrontMatchIndexes.begin(), vEdgeFrontMatchIndexes.begin() + min(int(mpEdgeFrontCloudKeyFrameMatch.size()), nLimitMaxMatchKFNum));
            std::map<int, int> mEdgeFrontCloudKeyFrameRandSelectMatch;
            for (auto &iter : vEdgeFrontMatchPartIndexes) mEdgeFrontCloudKeyFrameRandSelectMatch[iter] = mpEdgeFrontCloudKeyFrameMatch[iter];

            // @note 保存匹配的KeyFrame
            std::vector<int> vSelectedIndex;
            for (auto const& element : mEdgeFrontCloudKeyFrameRandSelectMatch) {
                vSelectedIndex.push_back(element.first);
            }
            SaveSelectedKeyFrameTrajectoryTUM(mpCurrentEdgeFrontMap, vSelectedIndex, (full_path_ / "edge_front_cloud_match.txt").string());

            // std::vector<KeyFrame *> tmpDebugKF1;
            // for (auto &iter : vEdgeFrontMatchPartIndexes) tmpDebugKF1.push_back(edgeFrontMapKeyFrames[iter]);
            // mpMapDrawer->dvpKFs1 = tmpDebugKF1;
            // std::vector<KeyFrame *> tmpDebugKF3;
            // for (auto &iter : vEdgeFrontMatchIndexes) tmpDebugKF3.push_back(edgeFrontMapKeyFrames[iter]);
            // mpMapDrawer->dvpKFs3 = tmpDebugKF3;

            cerr << "First Merge EdgeFront Map KF num: " << mpCurrentEdgeFrontMap->GetAllKeyFrames().size() << endl;
            cerr << "First Merge Cloud Map KF num: " << mpCurrentCloudMap->GetAllKeyFrames().size() << endl;
            cerr << "First Merge Rand Select KF ratio: " << mEdgeFrontCloudKeyFrameRandSelectMatch.size() << " / " << mpEdgeFrontCloudKeyFrameMatch.size() << endl;
            bool bComputeEdgeFront = CloudMerging::ComputeSubmapSim3(mpCurrentEdgeFrontMap, mpCurrentCloudMap, mpEdgeFrontCloudKeyFrameMatch, mEdgeFrontCloudKeyFrameRandSelectMatch, false, mgSwEdgeFrontCloud, mvpEdgeFrontCloudMatchedKeyPoints, mpMapDrawer);
            // ***************** 第四步，Merge Map *****************
            if (bComputeEdgeFront || mbMergeAnyway) {
                // w -> world, m -> match keyframe camera, c -> current keyframe camera
                // l -> last keyframe camera, 但注意这里的mg2oMergeSlw中的 l 已经变成了当前帧，作者没有用一个新的变量
                // 在CloudMergeLocal未改好的情况下，临时测试用
                // auto iter = mpEdgeFrontCloudKeyFrameMatch.begin();
                // std::advance(iter, rand() % mpEdgeFrontCloudKeyFrameMatch.size());
                // mpRandEdgeFrontKF = edgeFrontMapKeyFrames[iter->first];
                // mpRandEdgeFrontCloudKF = cloudMapKeyFrames[iter->second];

                if (!bComputeEdgeFront) {
                    cerr << "===========================================" << endl;
                    cerr << "============front merge error==============" << endl;
                    cerr << "===========================================" << endl;
                }

                // mpTracker->SetStepByStep(true);

                Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);

                // ***************** 子地图合并函数 *****************
                // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
                bool bRelaunchBA = false;

                // Verbose::PrintMess("MERGE-VISUAL: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
                //  If a Global Bundle Adjustment is running, abort it
                if (isRunningGBA()) {
                    unique_lock<mutex> lock(mMutexGBA);
                    mbStopGBA = true;

                    mnFullBAIdx++;

                    if (mpThreadGBA) {
                        mpThreadGBA->detach();
                        delete mpThreadGBA;
                    }
                    bRelaunchBA = true;
                }

                // Merge !!!
                CloudMergeMap(mpCurrentEdgeFrontMap, mpCurrentCloudMap, mgSwEdgeFrontCloud, mpEdgeFrontCloudKeyFrameMatch, mvpEdgeFrontCloudMatchedKeyPoints, mpLocalMapper, true);

                // Atlas删除没有的Map
                mpAtlas->RemoveBadMaps();

                if (bRelaunchBA) {
                    // Launch a new thread to perform Global Bundle Adjustment
                    mbRunningGBA = true;
                    mbFinishedGBA = false;
                    mbStopGBA = false;
                    // TODO 目前输入参数的nLoopKF不确定是否有影响，初步看上去只是区分是否是MergeLocal引发的GlobalBA，只要mnId不是Map Origin即可
                    mpThreadGBA = new thread(&CloudMerging::RunGlobalBundleAdjustment, this, mpCurrentEdgeFrontMap, mpCurrentEdgeFrontMap->GetOriginKF()->mnId);
                }
                Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
                // ***************** 子地图合并完成 *****************
                // SaveKeyFrameTrajectoryTUM(mpCurrentEdgeFrontMap, "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/first_merge.txt");

                // @note Debug
                // return;

                // ***************** 开始edgeback部分 *****************
                // 这里取出Map的KeyFrames进行匹配，可能会和LoopClosing线程Loop Closing或者Merge时的Map Update冲突
                const vector<KeyFrame *> &newEdgeFrontMapKeyFrames = mpCurrentEdgeFrontMap->GetAllKeyFrames();
                const vector<KeyFrame *> &edgeBackMapKeyFrames = mpCurrentEdgeBackMap->GetAllKeyFrames();
                std::vector<bool> edgeBackHaveMatchFlag(edgeBackMapKeyFrames.size(), false);
                // 由于EdgeFront和Cloud已经被合并了，KeyFrames也改变了，重新匹配，应该匹配数量不会变，只是KeyFrame ID变了
                for (unsigned long newEdgeFrontKF_i = 0; newEdgeFrontKF_i < newEdgeFrontMapKeyFrames.size(); newEdgeFrontKF_i++) {
                    for (unsigned long edgeBackKF_i = 0; edgeBackKF_i < edgeBackMapKeyFrames.size(); edgeBackKF_i++) {
                        if (edgeBackHaveMatchFlag[edgeBackKF_i]) { continue; }
                        double deltaTime = abs(edgeBackMapKeyFrames[edgeBackKF_i]->mTimeStamp - newEdgeFrontMapKeyFrames[newEdgeFrontKF_i]->mTimeStamp);
                        if (deltaTime < matchToleranceTime) {
                            mpNewEdgeFrontEdgeBackKeyFrameMatch[newEdgeFrontKF_i] = edgeBackKF_i;
                            edgeBackHaveMatchFlag[edgeBackKF_i] = true;
                            break;
                        }
                    }
                }
                if (mpNewEdgeFrontEdgeBackKeyFrameMatch.size() < 1) {
                    cerr << "匹配EdgeBack和New Cloud Map的KeyFrames数量过少" << endl;
                    return;
                }

                std::vector<int> vNewEdgeFrontMatchIndexes;
                for (auto &iter : mpNewEdgeFrontEdgeBackKeyFrameMatch) vNewEdgeFrontMatchIndexes.push_back(iter.first);
                sort(vNewEdgeFrontMatchIndexes.begin(), vNewEdgeFrontMatchIndexes.end(), [&](int x, int y) { return newEdgeFrontMapKeyFrames[x]->mTimeStamp < newEdgeFrontMapKeyFrames[y]->mTimeStamp; });
                // @note 暂时关闭这个处理
                // std::vector<int> vNewEdgeFrontMatchPartIndexes(vNewEdgeFrontMatchIndexes.end() - int(mpNewEdgeFrontEdgeBackKeyFrameMatch.size() * (8. / 10.)), vNewEdgeFrontMatchIndexes.end());
                std::vector<int> vNewEdgeFrontMatchPartIndexes(vNewEdgeFrontMatchIndexes.end() - min(int(vNewEdgeFrontMatchIndexes.size()), nLimitMaxMatchKFNum), vNewEdgeFrontMatchIndexes.end());
                std::map<int, int> mNewEdgeFrontEdgeBackKeyFrameRandSelectMatch;
                for (auto &iter : vNewEdgeFrontMatchPartIndexes) mNewEdgeFrontEdgeBackKeyFrameRandSelectMatch[iter] = mpNewEdgeFrontEdgeBackKeyFrameMatch[iter];

                // @note 保存匹配的KeyFrame
                std::vector<int> vSelectedIndex;
                for (auto const& element : mNewEdgeFrontEdgeBackKeyFrameRandSelectMatch) {
                    vSelectedIndex.push_back(element.second);
                }
                SaveSelectedKeyFrameTrajectoryTUM(mpCurrentEdgeBackMap, vSelectedIndex, (full_path_ / "edge_back_cloud_match.txt").string());

                // std::vector<KeyFrame *> tmpDebugKF1;
                // for (auto &iter : vNewEdgeFrontMatchPartIndexes) tmpDebugKF1.push_back(edgeBackMapKeyFrames[mpNewEdgeFrontEdgeBackKeyFrameMatch[iter]]);
                // mpMapDrawer->dvpKFs1 = tmpDebugKF1;
                // std::vector<KeyFrame *> tmpDebugKF3;
                // for (auto &iter : vNewEdgeFrontMatchIndexes) tmpDebugKF3.push_back(edgeBackMapKeyFrames[mpNewEdgeFrontEdgeBackKeyFrameMatch[iter]]);
                // mpMapDrawer->dvpKFs3 = tmpDebugKF3;

                // for (auto iter : mNewEdgeFrontEdgeBackKeyFrameRandSelectMatch)
                //     cerr << "select match: " << iter.first << " " << iter.second << endl;

                // After Merge EdgeFront and Cloud Map, Merge New Cloud Map(still EdgeFront) and EdgeBack Map
                cerr << "Second Merge Main Map KF num: " << mpCurrentEdgeFrontMap->GetAllKeyFrames().size() << endl;
                cerr << "Second Merge EdgeBack Map KF num: " << mpCurrentEdgeBackMap->GetAllKeyFrames().size() << endl;
                cerr << "Second Merge Rand Select KF ratio: " << mNewEdgeFrontEdgeBackKeyFrameRandSelectMatch.size() << " / " << mpNewEdgeFrontEdgeBackKeyFrameMatch.size() << endl;
                bool bComputeEdgeBack = CloudMerging::ComputeSubmapSim3(mpCurrentEdgeFrontMap, mpCurrentEdgeBackMap, mpNewEdgeFrontEdgeBackKeyFrameMatch, mNewEdgeFrontEdgeBackKeyFrameRandSelectMatch, false, mgSwNewCloudEdgeBack, mvpNewEdgeFrontEdgeBackMatchedKeyPoints, mpMapDrawer);
                if (bComputeEdgeBack || mbMergeAnyway) {
                    // w -> world, m -> match keyframe camera, c -> current keyframe camera
                    // l -> last keyframe camera, 但注意这里的mg2oMergeSlw中的 l 已经变成了当前帧，作者没有用一个新的变量
                    // 在CloudMergeLocal未改好的情况下，临时测试用
                    // auto iter = mpNewEdgeFrontEdgeBackKeyFrameMatch.begin();
                    // std::advance(iter, rand() % mpNewEdgeFrontEdgeBackKeyFrameMatch.size());
                    // mpRandEdgeBackNewCloudKF = newEdgeFrontMapKeyFrames[iter->first];
                    // mpRandEdgeBackKF = edgeBackMapKeyFrames[iter->second];

                    if (!bComputeEdgeBack) {
                        cerr << "===========================================" << endl;
                        cerr << "============back merge error==============" << endl;
                        cerr << "===========================================" << endl;
                    }

                    // mpTracker->SetStepByStep(true);

                    Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);

                    // ***************** 子地图合并函数 *****************
                    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
                    bool bRelaunchBA = false;

                    // Verbose::PrintMess("MERGE-VISUAL: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
                    //  If a Global Bundle Adjustment is running, abort it
                    if (isRunningGBA()) {
                        unique_lock<mutex> lock(mMutexGBA);
                        mbStopGBA = true;

                        mnFullBAIdx++;

                        if (mpThreadGBA) {
                            mpThreadGBA->detach();
                            delete mpThreadGBA;
                        }
                        bRelaunchBA = true;
                    }

                    // Merge !!!
                    CloudMergeMap(mpCurrentEdgeFrontMap, mpCurrentEdgeBackMap, mgSwNewCloudEdgeBack, mpNewEdgeFrontEdgeBackKeyFrameMatch, mvpNewEdgeFrontEdgeBackMatchedKeyPoints, mpLocalMapper, true);

                    // 改变Active Map为合并后的Map
                    // 旧的Map不会删除，但是会设置为Bad
                    delete mpCurrentCloudMap; // Cloud Map没有添加到Atlas里，所以不需要管，可以考虑直接delete掉清理内存
                    // @note EdgeFrontMap本身就在里面，不需要改变CurrentMap
                    if (mpAtlas->GetCurrentMap() == mpCurrentEdgeBackMap) {
                        mpAtlas->ChangeMap(mpCurrentEdgeFrontMap);
                    }
                    mpCurrentEdgeFrontMap->ChangeId(mpCurrentEdgeBackMap->GetId());
                    mpCurrentEdgeFrontMap->ResetHaveMerged();

                    mpAtlas->SetMapBad(mpCurrentEdgeBackMap);
                    mpAtlas->RemoveBadMaps();

                    if (bRelaunchBA) {
                        // Launch a new thread to perform Global Bundle Adjustment
                        mbRunningGBA = true;
                        mbFinishedGBA = false;
                        mbStopGBA = false;
                        // TODO 目前输入参数的nLoopKF不确定是否有影响，初步看上去只是区分是否是MergeLocal引发的GlobalBA，只要mnId不是Map Origin即可
                        mpThreadGBA = new thread(&CloudMerging::RunGlobalBundleAdjustment, this, mpCurrentEdgeFrontMap, mpCurrentEdgeFrontMap->GetOriginKF()->mnId);
                    }
                    Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
                    // ***************** 子地图合并完成 *****************
                    // SaveKeyFrameTrajectoryTUM(mpCurrentEdgeFrontMap, "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results/second_merge.txt");
                } else {
                    mpCurrentEdgeBackMap->ResetHaveMerged();
                }
            } else {
                mpCurrentEdgeBackMap->ResetHaveMerged();
            }
            cerr << "测试：退出此次Merge" << endl;
            mbRunning = false;

            // Reset Var
            mpCurrentCloudMap = static_cast<Map *>(NULL);
            mpCurrentEdgeFrontMap = static_cast<Map *>(NULL);
            mpCurrentEdgeBackMap = static_cast<Map *>(NULL);

            mpEdgeFrontCloudKeyFrameMatch.clear();
            mvpEdgeFrontCloudMatchedKeyPoints.clear();
            mgSwEdgeFrontCloud = g2o::Sim3();
            mpNewEdgeFrontEdgeBackKeyFrameMatch.clear();
            mvpNewEdgeFrontEdgeBackMatchedKeyPoints.clear();
            mgSwNewCloudEdgeBack = g2o::Sim3();

            cerr << "==================================" << endl;
            cerr << "CloudMerge Time: " << nCloudMerge << endl;
            cerr << "==================================" << endl;
        }

        ResetIfRequested();

        if (CheckFinish()) {
            break;
        }

        usleep(5000);
    }

    SetFinish();
}

void CloudMerging::InsertCloudMap(Map *pMap) {
    unique_lock<mutex> lock(mMutexCloudQueue);
    mlpCloudMapQueue.push_back(pMap);
    // TODO add to debug queue
}

bool CloudMerging::CheckNewCloudMap(bool bOnline) {
    unique_lock<mutex> lock(mMutexCloudQueue);

    if (bOnline) {
        return (!mlpCloudMapQueue.empty());
    } else {
        return (mlpCloudMapQueue.size() == 3);
    }
}

/*!
 * @param 
 * @return
 */
bool CloudMerging::ComputeSubmapSim3(
    Map *pMap1, Map *pMap2, const std::map<int, int> &mKfMatch12,                  // input
    const std::map<int, int> &mRandSelectKfMatch12,                                // input
    bool bFixScale,                                                                // input
    g2o::Sim3 &gSw1w2,                                                             // output
    std::map<KeyFrame *, std::vector<std::pair<int, int>>> &mvpMatchedKeyPoints12, // output
    MapDrawer *pMapDrawer                                                          // debug
) {
    // check input
    if (!pMap1 || !pMap2) {
        cerr << "Map Input Error!" << endl;
        return 0;
    }
    float keyPointMatchTolerancePixel = 3;
    float nSolveSim3Inliers = 5;
    float nUmeyamaSolveSim3InliersRatio = 0.3;
    int nUmeyamaSolveSim3KFNum = 15;

    const vector<KeyFrame *> &vpMap1KeyFrames = pMap1->GetAllKeyFrames();
    const vector<KeyFrame *> &vpMap2KeyFrames = pMap2->GetAllKeyFrames();

    // 计算所有KF之间的MapPoint匹配情况
    int matchMapPointNum = 0; // For Debug
    std::map<KeyFrame *, std::vector<MapPoint *>> mvpMatchedPoints12;
    // std::map<KeyFrame *, std::vector<std::pair<int, int>>> mvpMatchedKeyPoints12;
    std::map<KeyFrame *, std::vector<std::pair<int, int>>> mvpValidMatchedKeyPoints12; // 只记录有对应mappoint对应的keypoint match
    std::map<KeyFrame *, int> mvpMatchedPointsNum12;

    std::chrono::steady_clock::time_point timeStartMatchMapPoints = std::chrono::steady_clock::now();

    // for (auto iter = mKfMatch12.begin(); iter != mKfMatch12.end(); iter++) { // TODO 三层For循环，最好统计一下耗时
    //     KeyFrame *pMap1KF = vpMap1KeyFrames[iter->first];
    //     KeyFrame *pMap2KF = vpMap2KeyFrames[iter->second];

    //     const std::vector<MapPoint *> &vpMap1MapPoints = pMap1KF->GetMapPointMatches();
    //     const std::vector<MapPoint *> &vpMap2MapPoints = pMap2KF->GetMapPointMatches();
    //     std::vector<MapPoint *> vpMatchedPoints12(vpMap1MapPoints.size(), static_cast<MapPoint *>(NULL));
    //     std::vector<std::pair<int, int>> vpMatchedKeyPoints12;
    //     std::vector<std::pair<int, int>> vpValidMatchedKeyPoints12;

    //     // 开始匹配MapPoints
    //     int matchNum = 0;
    //     const std::vector<cv::KeyPoint> &map1KFKPs = pMap1KF->mvKeys;
    //     const std::vector<cv::KeyPoint> &map2KFKPs = pMap2KF->mvKeys;
    //     for (unsigned long map1KFKP_i = 0; map1KFKP_i < map1KFKPs.size(); ++map1KFKP_i) { // TODO 这里可能有现成函数直接匹配，这里暴力实现
    //         for (unsigned long map2KFKP_i = 0; map2KFKP_i < map2KFKPs.size(); ++map2KFKP_i) {
    //             const float map1_u = map1KFKPs[map1KFKP_i].pt.x;
    //             const float map1_v = map1KFKPs[map1KFKP_i].pt.y;
    //             const float map2_u = map2KFKPs[map2KFKP_i].pt.x;
    //             const float map2_v = map2KFKPs[map2KFKP_i].pt.y;
    //             const auto delta_pixel = (float)sqrt(pow(map1_u - map2_u, 2) + pow(map1_v - map2_v, 2));
    //             if (delta_pixel < keyPointMatchTolerancePixel) {
    //                 matchNum++;
    //                 vpMatchedPoints12[map1KFKP_i] = vpMap2MapPoints[map2KFKP_i];
    //                 vpMatchedKeyPoints12.push_back(std::pair<int, int>(map1KFKP_i, map2KFKP_i));
    //                 if (vpMap1MapPoints[map1KFKP_i] && vpMap2MapPoints[map2KFKP_i]) // 两个mappoint非空，需要debug确认NULL pointer
    //                     vpValidMatchedKeyPoints12.push_back(std::pair<int, int>(map1KFKP_i, map2KFKP_i));
    //             }
    //         }
    //     }

    //     matchMapPointNum += matchNum;
    //     mvpMatchedPointsNum12[vpMap1KeyFrames[iter->first]] = matchNum;
    //     mvpMatchedPoints12[vpMap1KeyFrames[iter->first]] = vpMatchedPoints12;
    //     mvpMatchedKeyPoints12[vpMap1KeyFrames[iter->first]] = vpMatchedKeyPoints12;
    //     mvpValidMatchedKeyPoints12[vpMap1KeyFrames[iter->first]] = vpValidMatchedKeyPoints12;
    // }

    /*********** 利用ORBSLAM构造Frame时固有的keypoint网格划分来加速匹配MapPoint***********/
    for (auto iter = mKfMatch12.begin(); iter != mKfMatch12.end(); iter++) { // TODO 三层For循环，最好统计一下耗时
        KeyFrame *pMap1KF = vpMap1KeyFrames[iter->first];
        KeyFrame *pMap2KF = vpMap2KeyFrames[iter->second];

        const std::vector<MapPoint *> &vpMap1MapPoints = pMap1KF->GetMapPointMatches();
        const std::vector<MapPoint *> &vpMap2MapPoints = pMap2KF->GetMapPointMatches();
        std::vector<MapPoint *> vpMatchedPoints12(vpMap1MapPoints.size(), static_cast<MapPoint *>(NULL));
        std::vector<std::pair<int, int>> vpMatchedKeyPoints12;
        std::vector<std::pair<int, int>> vpValidMatchedKeyPoints12;

        // 开始匹配MapPoints
        int matchNum = 0;
        const std::vector<cv::KeyPoint> &map1KFKPs = pMap1KF->mvKeys;
        const std::vector<cv::KeyPoint> &map2KFKPs = pMap2KF->mvKeys;
        for (unsigned long map1KFKP_i = 0; map1KFKP_i < map1KFKPs.size(); ++map1KFKP_i) {
            const float map1_u = map1KFKPs[map1KFKP_i].pt.x;
            const float map1_v = map1KFKPs[map1KFKP_i].pt.y;
            const vector<size_t> vIndices = pMap2KF->GetFeaturesInArea(map1_u, map1_v, keyPointMatchTolerancePixel);

            if (vIndices.empty())
                continue;

            float best_dist = keyPointMatchTolerancePixel;
            int best_idx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) { // 选择3X3网格中距离query MapPoint最近的MapPoint in Map2
                const size_t map2KFKP_i = *vit;
                const float map2_u = map2KFKPs[map2KFKP_i].pt.x;
                const float map2_v = map2KFKPs[map2KFKP_i].pt.y;
                const float delta_pixel = (float)sqrt(pow(map1_u - map2_u, 2) + pow(map1_v - map2_v, 2));
                if (delta_pixel < best_dist && vpMap1MapPoints[map1KFKP_i] && vpMap2MapPoints[map2KFKP_i]) {
                    best_idx = map2KFKP_i;
                    best_dist = delta_pixel;
                }
            }
            if (best_idx == -1)
                continue;

            matchNum++;
            vpMatchedPoints12[map1KFKP_i] = vpMap2MapPoints[best_idx];
            vpMatchedKeyPoints12.push_back(std::pair<int, int>(map1KFKP_i, best_idx));
            vpValidMatchedKeyPoints12.push_back(std::pair<int, int>(map1KFKP_i, best_idx));
        }

        matchMapPointNum += matchNum;
        mvpMatchedPointsNum12[vpMap1KeyFrames[iter->first]] = matchNum;
        mvpMatchedPoints12[vpMap1KeyFrames[iter->first]] = vpMatchedPoints12;
        mvpMatchedKeyPoints12[vpMap1KeyFrames[iter->first]] = vpMatchedKeyPoints12;
        mvpValidMatchedKeyPoints12[vpMap1KeyFrames[iter->first]] = vpValidMatchedKeyPoints12;
    }

    static int i = 0;
    if (i == 0) {
        nFrontCloudMPMatchNum = matchMapPointNum;
    } else if (i == 1) {
        nBackCloudMPMatchNum = matchMapPointNum;
    }
    i++;

    cerr << "测试：匹配KeyPoint的数量：" << matchMapPointNum << endl;
    std::chrono::steady_clock::time_point timeEndMatchMapPoints = std::chrono::steady_clock::now();
    double time = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEndMatchMapPoints - timeStartMatchMapPoints).count();
    // cerr << "Test: Match Map Point Time " << time << endl;

    std::vector<KeyFrame *> matchMap1KeyFrames;
    std::vector<KeyFrame *> matchMap2KeyFrames;
    std::vector<std::vector<MapPoint *>> avpMatchedPoints;
    std::vector<std::vector<std::pair<int, int>>> avpValidKPMatches;
    matchMap1KeyFrames.reserve(mKfMatch12.size());
    matchMap2KeyFrames.reserve(mKfMatch12.size());
    avpMatchedPoints.reserve(mKfMatch12.size());
    avpValidKPMatches.reserve(mKfMatch12.size());
    for (auto iter = mKfMatch12.begin(); iter != mKfMatch12.end(); iter++) {
        matchMap1KeyFrames.push_back(vpMap1KeyFrames[iter->first]);
        matchMap2KeyFrames.push_back(vpMap2KeyFrames[iter->second]);
        avpMatchedPoints.push_back(mvpMatchedPoints12[vpMap1KeyFrames[iter->first]]);
        avpValidKPMatches.push_back(mvpValidMatchedKeyPoints12[vpMap1KeyFrames[iter->first]]);
    }

    // @note Umeyama Solve Sim3
    vector<Eigen::Vector3d> map1MatchKFPoints;
    vector<Eigen::Vector3d> map2MatchKFPoints;
    // for (auto &iter : mKfMatch12) {
    for (auto &iter : mRandSelectKfMatch12) {
        map1MatchKFPoints.push_back(vpMap1KeyFrames[iter.first]->GetPoseInverse().translation().cast<double>());
        map2MatchKFPoints.push_back(vpMap2KeyFrames[iter.second]->GetPoseInverse().translation().cast<double>());
    }
    assert(map1MatchKFPoints.size() == map2MatchKFPoints.size());
    Eigen::Matrix4d umeyamaResult = Sim3Solver::umeyamaSolve(map2MatchKFPoints, map1MatchKFPoints);
    Sophus::Sim3d sSc1c2(umeyamaResult.cast<double>());
    Eigen::Matrix3d tmp_R = sSc1c2.rotationMatrix();
    Eigen::Vector3d tmp_T = sSc1c2.translation();
    double tmp_s = sSc1c2.scale();
    g2o::Sim3 gUmeyamaSw1w2 = g2o::Sim3(tmp_R, tmp_T, tmp_s); // g2o::Sim3: kf camera 2 -> kf camera 1，是输入求解的两个关键帧之间的Sim3变换，注意是2->1

    // float nInliersRatio;
    // bool bUmeyamaNoMore;
    // bool bUmeyamaConverge;
    // UmeyamaSim3Solver umeyamaSolver(matchMap1KeyFrames, matchMap2KeyFrames, avpValidKPMatches);
    // umeyamaSolver.SetRansacParameters(0.99, nUmeyamaSolveSim3InliersRatio, 30, nUmeyamaSolveSim3KFNum);
    // g2o::Sim3 gUmeyamaSw1w2 = umeyamaSolver.iterate(20, bUmeyamaNoMore, nInliersRatio, bUmeyamaConverge);
    // if (bUmeyamaConverge) {
    //     cerr << "Umeyama RANSAC Converge: " << nInliersRatio << endl;
    // } else {
    //     cerr << "Umeyama RANSAC Converge Fail: " << nInliersRatio << endl;
    // }

    std::chrono::steady_clock::time_point timeStartComputeInliers = std::chrono::steady_clock::now();

    float umeyamaInliersRatio = Sim3Solver::ComputeInliersNum(matchMap1KeyFrames, matchMap2KeyFrames, avpValidKPMatches, gUmeyamaSw1w2);

    std::chrono::steady_clock::time_point timeEndComputeInliers = std::chrono::steady_clock::now();
    time = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEndComputeInliers - timeStartComputeInliers).count();
    // cerr << "Test: umeyama Compute Inliers Time " << time << endl;

    // @note 初步实现先随机抽取Edge Map和Cloud Map对应的两帧，求解初步的Sim3
    bool bFixedScale = bFixScale;

    bool bNoMore = false;
    vector<bool> vbInliers;
    int nInliers;
    bool bConverge = false;
    Eigen::Matrix4f mTcm;
    Eigen::Matrix3d solveR;
    Eigen::Vector3d solveT;
    double solveS;

    g2o::Sim3 gRANSACSw1w2;
    bool bRANSACConverge;

    /**********设置一系列best变量，记录循环中最优的Sim3初值***********/
    float bestInlierRatio = 0;
    Eigen::Matrix3f bestRotation;
    Eigen::Vector3f bestTranslation;
    float bestScale;
    int bestMatch1Index = 1;

    std::vector<int> vMatch1Indexes;
    for (auto &iter : mRandSelectKfMatch12) vMatch1Indexes.push_back(iter.first);
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    shuffle(vMatch1Indexes.begin(), vMatch1Indexes.end(), std::default_random_engine(seed));
    for (auto &iter : vMatch1Indexes) {
        int match1Index = iter;
        int match2Index = mRandSelectKfMatch12.at(iter);
        // std::cerr << "rand index: " << match1Index << "   index: " << match2Index << std::endl;
        KeyFrame *pRandSelectMap1KF = vpMap1KeyFrames[match1Index];
        KeyFrame *pRandSelectMap2KF = vpMap2KeyFrames[match2Index];

        // static int flag = 0;
        // if (flag == 0) {
        //     std::vector<KeyFrame *> tmpDebugKF2;
        //     tmpDebugKF2.push_back(pRandSelectMap1KF);
        //     pMapDrawer->dvpKFs2 = tmpDebugKF2;
        // }
        // if (flag == 1) {
        //     std::vector<KeyFrame *> tmpDebugKF2;
        //     tmpDebugKF2.push_back(pRandSelectMap2KF);
        //     pMapDrawer->dvpKFs2 = tmpDebugKF2;
        // }
        // flag++;

        if (mvpMatchedPointsNum12[pRandSelectMap1KF] < 5) { // @note 如果当前帧匹配pixel数量<5则放弃
            cerr << "当前选取的匹配KF对的匹配Map Points数量是 " << mvpMatchedPointsNum12[pRandSelectMap1KF] << ", 小于5，放弃当前帧" << endl;
            continue;
        }
        std::vector<MapPoint *> &vpMatchedPoints12 = mvpMatchedPoints12[pRandSelectMap1KF];
        std::vector<KeyFrame *> vpKeyFrameMatchedMP12(vpMatchedPoints12.size(), pRandSelectMap2KF);

        // @note save image for draw
        // static bool once = false;
        // std::vector<std::pair<int, int>> &vpMatchedKeyPoints12 = mvpMatchedKeyPoints12[pRandSelectMap1KF];
        // if (!once) {
        //     std::string save_dir = "/home/ruanjh/Workspace/NewSpace/Cloud-Edge-SLAM/results";

        //     // save image
        //     cv::Mat img1 = pRandSelectMap1KF->imgGray.clone();
        //     cv::Mat img2 = pRandSelectMap2KF->imgGray.clone();
        //     cv::imwrite(save_dir + "/" + "img1.png", img1);
        //     cv::imwrite(save_dir + "/" + "img2.png", img1);

        //     cv::FileStorage file(save_dir + "/" + "keypoint_match.yml", cv::FileStorage::WRITE);
        //     std::vector<cv::Point2i> img1_keypoints;
        //     std::vector<cv::Point2i> img2_keypoints;
        //     std::vector<cv::Point2i> keypoint_match_indexes;
        //     for (auto &keypoint : pRandSelectMap1KF->mvKeys) {
        //         img1_keypoints.push_back(cv::Point2i(keypoint.pt.x, keypoint.pt.y));
        //     }
        //     for (auto &keypoint : pRandSelectMap2KF->mvKeys) {
        //         img2_keypoints.push_back(cv::Point2i(keypoint.pt.x, keypoint.pt.y));
        //     }
        //     for (auto &match_index : vpMatchedKeyPoints12) {
        //         keypoint_match_indexes.push_back(cv::Point2i(match_index.first, match_index.second));
        //     }
        //     cv::Mat img1_keypoint_mat(img1_keypoints);
        //     cv::Mat img2_keypoint_mat(img2_keypoints);
        //     cv::Mat keypoint_match_index_mat(keypoint_match_indexes);

        //     file << "img1_keypoint" << img1_keypoint_mat;
        //     file << "img2_keypoint" << img2_keypoint_mat;
        //     file << "keypoint_match_indexes" << keypoint_match_index_mat;

        //     file.release();

        //     once = true;
        // }

        Sim3Solver solver = Sim3Solver(pRandSelectMap1KF, pRandSelectMap2KF, vpMatchedPoints12, bFixedScale, vpKeyFrameMatchedMP12);
        solver.SetRansacParameters(0.99, nSolveSim3Inliers, 300);

        bConverge = false;
        bNoMore = false;
        float oldBestRatio = bestInlierRatio;
        while (!bConverge && !bNoMore) {
            // ***************** 根据初步得到的Map Points匹配求解Sim3 *****************
            // mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
            mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge, matchMap1KeyFrames, matchMap2KeyFrames, avpValidKPMatches,
                                  bestInlierRatio, bestRotation, bestTranslation, bestScale);
            // cerr << "BoW guess: Solver achieve " + to_string(nInliers) + " min geometrical inliers " + to_string(nSolveSim3Inliers) + " BoW matches" << endl;
        }

        // 更新最优inlierRatio对应KF对的index
        if (bestInlierRatio > oldBestRatio) {
            bestMatch1Index = iter;
        }

        // cerr << "测试：求解Sim3是否不再优化：" << bNoMore << endl;
        // cerr << "测试：求解Sim3是否收敛：" << bConverge << endl;
        if (bConverge) {
            bRANSACConverge = true;
            // get solve result
            solveR = solver.GetEstimatedRotation().cast<double>();
            solveT = solver.GetEstimatedTranslation().cast<double>();
            solveS = (double)solver.GetEstimatedScale();
            g2o::Sim3 gSc1c2(solveR, solveT, solveS);
            g2o::Sim3 gSc1w(pRandSelectMap1KF->GetRotation().cast<double>(), pRandSelectMap1KF->GetTranslation().cast<double>(), 1.0); // g2o::Sim3: kf 1 world -> kf 1 camera
            g2o::Sim3 gSc2w(pRandSelectMap2KF->GetRotation().cast<double>(), pRandSelectMap2KF->GetTranslation().cast<double>(), 1.0); // g2o::Sim3: kf 2 world -> kf 2 camera
            gRANSACSw1w2 = gSc1w.inverse() * gSc1c2 * gSc2w;
            break;
        }
    }
    // cerr << "测试：RANSAC求解Sim3是否收敛：" << bRANSACConverge << endl;

    float RANSACInliersRatio = 0;
    if (bRANSACConverge) {
        std::chrono::steady_clock::time_point timeStartComputeInliers = std::chrono::steady_clock::now();

        RANSACInliersRatio = Sim3Solver::ComputeInliersNum(matchMap1KeyFrames, matchMap2KeyFrames, avpValidKPMatches, gRANSACSw1w2);

        std::chrono::steady_clock::time_point timeEndComputeInliers = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEndComputeInliers - timeStartComputeInliers).count();
        // cerr << "Test: RANSAC Compute Inliers Time " << time << endl;
    }
    // else{ // 若没有收敛，则选择循环中inlierRatio最优的Sim3作为初始值
    //     cout << "best mathch index: " << bestMatch1Index << endl;
    //     int bestMatch2Index = mRandSelectKfMatch12.at(bestMatch1Index);
    //     // std::cerr << "rand index: " << match1Index << "   index: " << match2Index << std::endl;
    //     KeyFrame *pRandSelectMap1KF = vpMap1KeyFrames[bestMatch1Index];
    //     KeyFrame *pRandSelectMap2KF = vpMap2KeyFrames[bestMatch2Index];

    //     solveR = bestRotation.cast<double>();
    //     solveT = bestTranslation.cast<double>();
    //     solveS = (double)bestScale;
    //     cout << "RANSAC Solve Scale: " << solveS << endl;
    //     g2o::Sim3 gSc1c2(solveR, solveT, solveS);
    //     g2o::Sim3 gSc1w(pRandSelectMap1KF->GetRotation().cast<double>(), pRandSelectMap1KF->GetTranslation().cast<double>(), 1.0); // g2o::Sim3: kf 1 world -> kf 1 camera
    //     g2o::Sim3 gSc2w(pRandSelectMap2KF->GetRotation().cast<double>(), pRandSelectMap2KF->GetTranslation().cast<double>(), 1.0); // g2o::Sim3: kf 2 world -> kf 2 camera
    //     gRANSACSw1w2 = gSc1w.inverse() * gSc1c2 * gSc2w;

    //     std::chrono::steady_clock::time_point timeStartComputeInliers = std::chrono::steady_clock::now();

    //     RANSACInliersRatio = Sim3Solver::ComputeInliersNum(matchMap1KeyFrames, matchMap2KeyFrames, avpValidKPMatches, gRANSACSw1w2);
    //     // RANSACInliersRatio = ComputeInliersNum(matchMap1KeyFrames, matchMap2KeyFrames, avpValidKPMatches, gRANSACSw1w2);

    //     std::chrono::steady_clock::time_point timeEndComputeInliers = std::chrono::steady_clock::now();
    //     double time = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEndComputeInliers - timeStartComputeInliers).count();
    //     cerr << "Test: RANSAC Compute Inliers Time " << time << endl;
    // }

    cerr << "测试：Umeyama Solve Sim3 Inliers Ratio：" << umeyamaInliersRatio << endl;
    cerr << "测试：RANSAC Solve Sim3 Inliers Ratio：" << RANSACInliersRatio << endl;

    // 比较两个方法的InliersRatio，选择更好的那一个
    float bestInliersRatio;
    if (umeyamaInliersRatio >= RANSACInliersRatio) {
        gSw1w2 = gUmeyamaSw1w2;
        bestInliersRatio = umeyamaInliersRatio;
    } else {
        gSw1w2 = gRANSACSw1w2;
        bestInliersRatio = RANSACInliersRatio;
    }
    // gSw1w2 = gUmeyamaSw1w2;
    // gSw1w2 = gRANSACSw1w2;

    // For debug
    // Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
    // Eigen::Vector3d t(0, 0, 0);
    // g2o::Sim3 gSc1c2(r, t, 1.0);

    Eigen::Matrix<double, 7, 7> mHessian7x7;

    // float numOptMatchesRatio = Optimizer::OptimizeCloudSim3(matchMap1KeyFrames, matchMap2KeyFrames, avpMatchedPoints, gSc1c2, 10, bFixedScale, mHessian7x7, true);
    float numOptMatchesRatio = Optimizer::OptimizeCloudSim3(matchMap1KeyFrames, matchMap2KeyFrames, avpMatchedPoints, gSw1w2, 10, true, mHessian7x7, true);
    // cerr << "测试：优化Sim3的点匹配比例：" << numOptMatchesRatio << endl;
    // cerr << "测试：Optimize Sim3的尺度变换值：" << gSw1w2.scale() << endl;

    timeStartComputeInliers = std::chrono::steady_clock::now();

    float OptimizeInliersRatio = Sim3Solver::ComputeInliersNum(matchMap1KeyFrames, matchMap2KeyFrames, avpValidKPMatches, gSw1w2);

    timeEndComputeInliers = std::chrono::steady_clock::now();
    time = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEndComputeInliers - timeStartComputeInliers).count();
    // cerr << "Test: Optimize Compute Inliers Time " << time << endl;

    if (OptimizeInliersRatio > 0.1) { // @note Todo
                                      // if (1) { // @note 暂时不管优化结果，调试整体流程
        cerr << "测试：优化Sim3的点匹配数量是否超过阈值： true, 值为 " << OptimizeInliersRatio << endl;
        return true;
    } else {
        cerr << "测试：优化Sim3的点匹配数量是否超过阈值： false, 值为" << OptimizeInliersRatio << endl;
        return false;
    }

    return false;
}

bool CloudMerging::NewDetectCommonRegions() {
    // To deactivate cloud merging. No CloudMerging  will be performed
    if (!mbActiveCM)
        return false;

    {
        // TODO For Test
        //        unique_lock<mutex> lock(mMutexLoopQueue);
        //        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        //        mlpLoopKeyFrameQueue.pop_front();
        //        // Avoid that a keyframe can be erased while it is being process by this thread
        //        mpCurrentKF->SetNotErase();
        //        mpCurrentKF->mbCurrentPlaceRecognition = true;
        //
        //        mpLastMap = mpCurrentKF->GetMap();
    }

    if (mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) //12
    {
        // cerr << "LoopClousure: Stereo KF inserted without check: " << mpCurrentKF->mnId << endl;
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // 如果当前关键帧的地图（其实也就是Active Map吧？）中关键帧数量小于12，则不进行Place Recognition
    // 为什么呢？好像也合理，如果是Active Map内部的Loop closure，太小没有意义。如果是multi map间的子地图合并，才刚跟踪回来几帧，确实也不太可能出现回到旧的地方的情况，不过这个可能性还是有的，如果遇到这种相关的离奇的情况，需要记得这个地方。@note warnings
    if (mpLastMap->GetAllKeyFrames().size() < 12) {
        // cerr << "LoopClousure: Stereo KF inserted without check, map is small: " << mpCurrentKF->mnId << endl;
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    //cerr << "LoopClousure: Checking KF: " << mpCurrentKF->mnId << endl;

    //Check the last candidates with geometric validation
    // Loop candidates
    bool bLoopDetectedInKF = false;
    bool bCheckSpatial = false;

    // ***************** 第三步（Map Merge），当上一个关键帧有匹配Place Recognition成功的迹象，这一帧进行确认检查 *****************
    // Merge candidates
    bool bMergeDetectedInKF = false;
    if (mnMergeNumCoincidences > 0) {
        // Find from the last KF candidates
        // 上一帧camera坐标系 -> 当前帧camera坐标系
        Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();
        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);

        g2o::Sim3 gScw = gScl * mg2oMergeSlw;
        int numProjMatches = 0;
        vector<MapPoint *> vpMatchedMPs;
        // mpLoopMatchedKF是第二步中从候选关键帧选择的最佳匹配关键帧，这里直接用新一个关键帧与其进行匹配，用于进一步地保证Place Recognition的Precision
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs);
        if (bCommonRegion) {
            bMergeDetectedInKF = true;

            mnMergeNumCoincidences++; // 若新一关键帧也匹配成功，则计数+1
            mpMergeLastCurrentKF->SetErase();
            mpMergeLastCurrentKF = mpCurrentKF;
            mg2oMergeSlw = gScw;
            mvpMergeMatchedMPs = vpMatchedMPs;

            mbMergeDetected = mnMergeNumCoincidences >= 3;
        } else {
            mbMergeDetected = false;
            bMergeDetectedInKF = false;

            mnMergeNumNotFound++;
            // ***************** 若第一次成功后，再连续两个关键帧丢失（失败）两次，则重置整次Place Recognition进程 *****************
            if (mnMergeNumNotFound >= 2) {
                // ***************** 可以在这里看到Place Recognition关键的变量 *****************
                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
            }
        }
    }

    // ***************** 第五步，若Loop closure或是Map Merge的Place Recognition检测成功，返回true *****************
    if (mbMergeDetected) {
        mpKeyFrameDB->add(mpCurrentKF);
        return true;
    }

    //TODO: This is only necessary if we use a minimun score for pick the best candidates
    const vector<KeyFrame *> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

    // Extract candidates from the bag of words
    vector<KeyFrame *> vpMergeBowCand, vpLoopBowCand;
    if (!bMergeDetectedInKF || !bLoopDetectedInKF) {
        // Search in BoW
        // ***************** 第一步，从BoW中检测出候选帧，默认选择3个最好的候选帧 *****************
        // 3个Active Map内的最优候选帧，3个Multi Map内的最优候选帧
        mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand, 3);
    }

    // Check the BoW candidates if the geometric candidate list is empty
    // ***************** 第二步，BoW得到候选之后，从候选中尝试选择最终的结果 *****************
    if (!bMergeDetectedInKF && !vpMergeBowCand.empty()) {
        mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
    }

    mpKeyFrameDB->add(mpCurrentKF);

    if (mbMergeDetected) {
        return true;
    }

    mpCurrentKF->SetErase();
    mpCurrentKF->mbCurrentPlaceRecognition = false;

    return false;
}

bool CloudMerging::DetectAndReffineSim3FromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                  std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs) {
    set<MapPoint *> spAlreadyMatchedMPs;
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

    int nProjMatches = 30;
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    if (nNumProjMatches >= nProjMatches) {
        //Verbose::PrintMess("Sim3 reffine: There are " + to_string(nNumProjMatches) + " initial matches ", Verbose::VERBOSITY_DEBUG);
        Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
        g2o::Sim3 gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
        g2o::Sim3 gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale = mbFixScale; // TODO CHECK; Solo para el monocular inertial
        if (mpTracker->mSensor == System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale = false;
        int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);

        //Verbose::PrintMess("Sim3 reffine: There are " + to_string(numOptMatches) + " matches after of the optimization ", Verbose::VERBOSITY_DEBUG);

        if (numOptMatches > nProjOptMatches) {
            g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(), 1.0);

            vector<MapPoint *> vpMatchedMP;
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));

            nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
            if (nNumProjMatches >= nProjMatchesRep) {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}

bool CloudMerging::DetectCommonRegionsFromBoW(std::vector<KeyFrame *> &vpBowCand, KeyFrame *&pMatchedKF2, KeyFrame *&pLastCurrentKF, g2o::Sim3 &g2oScw,
                                              int &nNumCoincidences, std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs) {
    int nBoWMatches = 20;
    int nBoWInliers = 15;
    int nSim3Inliers = 20;
    int nProjMatches = 50;
    int nProjOptMatches = 80;

    // spConnectedKeyFrames代表当前关键帧所有有共视的关键帧
    set<KeyFrame *> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

    int nNumCovisibles = 10;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);

    // Varibles to select the best numbe
    KeyFrame *pBestMatchedKF;
    // ***************** nBestMatchesReproj：遍历候选关键帧时，记录最终匹配最好的候选关键帧所匹配的Map Points数量，遍历结束后即可直接用它作为判定依据，判定是否达到了我们希望达到的数量，达到了则成功进行了一次Place Recognition *****************
    int nBestMatchesReproj = 0;
    // ***************** nBestNumCoindicendes：遍历候选关键帧时，记录最终匹配最好的候选关键帧 与 当前关键帧共视的10个关键帧中有多少个匹配成功了，例如3个匹配成功了，则3/10，遍历结束后即可直接用它作为判定依据，判定是否达到了我们希望达到的数量，达到了则成功进行了一次Place Recognition *****************
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    std::vector<MapPoint *> vpBestMapPoints;
    std::vector<MapPoint *> vpBestMatchedMapPoints;

    int numCandidates = vpBowCand.size();
    vector<int> vnStage(numCandidates, 0);
    vector<int> vnMatchesStage(numCandidates, 0);

    int index = 0;
    // Verbose::PrintMess("BoW candidates: There are " + to_string(vpBowCand.size()) + " possible candidates ", Verbose::VERBOSITY_DEBUG);
    // ***************** 遍历每个候选关键帧 开始 *****************
    // ***************** 在下列的循环中，操作对象都是 当前关键帧 + 当前遍历到的第i个候选关键帧 *****************
    for (KeyFrame *pKFi : vpBowCand) {
        if (!pKFi || pKFi->isBad())
            continue;

        // std::cerr << "KF candidate: " << pKFi->mnId << std::endl;
        // Current KF against KF with covisibles version
        // 取nNumCovisibles个最近的共视帧
        std::vector<KeyFrame *> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
        if (vpCovKFi.empty()) {
            std::cerr << "Covisible list empty" << std::endl;
            vpCovKFi.push_back(pKFi);
        } else {
            // 将Matched KF也放入列表内
            vpCovKFi.push_back(vpCovKFi[0]);
            vpCovKFi[0] = pKFi;
        }

        bool bAbortByNearKF = false;
        for (int j = 0; j < vpCovKFi.size(); ++j) {
            // 这样find == end代表没有找到，而不是指向最后一个元素
            // 如果有找到，说明找到的 候选关键帧+其最近的10个关键帧 存在与当前关键帧比较近的，不满足Loop closure或者Map merge的需求
            if (spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end()) {
                bAbortByNearKF = true;
                break;
            }
        }
        if (bAbortByNearKF) {
            //std::cerr << "Check BoW aborted because is close to the matched one " << std::endl;
            continue;
        }
        //std::cerr << "Check BoW continue because is far to the matched one " << std::endl;

        std::vector<std::vector<MapPoint *>> vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<MapPoint *> spMatchedMPi;
        int numBoWMatches = 0;

        // ***************** pMostBoWMatchesKF就是当前遍历的 候选关键帧 *****************
        KeyFrame *pMostBoWMatchesKF = pKFi;
        int nMostBoWNumMatches = 0;

        // 好奇怪的size
        std::vector<MapPoint *> vpMatchedPoints = std::vector<MapPoint *>(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));
        std::vector<KeyFrame *> vpKeyFrameMatchedMP = std::vector<KeyFrame *>(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame *>(NULL));

        int nIndexMostBoWMatchesKF = 0;
        for (int j = 0; j < vpCovKFi.size(); ++j) {
            if (!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;

            // 使用BoW匹配，获得当前关键帧 与 候选关键帧+其最近的10个关键帧 的MapPoints匹配
            int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
            if (num > nMostBoWNumMatches) {
                // 记录BoW匹配最多的关键帧
                nMostBoWNumMatches = num;
                nIndexMostBoWMatchesKF = j;
            }
        }

        for (int j = 0; j < vpCovKFi.size(); ++j) {
            for (int k = 0; k < vvpMatchedMPs[j].size(); ++k) // 遍历匹配上的每个Map Point
            {
                MapPoint *pMPi_j = vvpMatchedMPs[j][k];
                if (!pMPi_j || pMPi_j->isBad())
                    continue;

                if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end()) // 保存所有匹配上的Map Point到set
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;

                    vpMatchedPoints[k] = pMPi_j;          // 这里不是有可能覆盖吗？
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j]; //
                }
            }
        }

        //pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];  // 貌似作者本身还想要根据前面计算的情况修改最匹配的关键帧为当前遍历的候选关键帧中的共视关键帧之一

        if (numBoWMatches >= nBoWMatches) // TODO pick a good threshold
        {
            // Geometric validation
            bool bFixedScale = mbFixScale;

            Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers

            bool bNoMore = false;
            vector<bool> vbInliers;
            int nInliers;
            bool bConverge = false;
            Eigen::Matrix4f mTcm;
            while (!bConverge && !bNoMore) {
                // ***************** 根据初步得到的Map Points匹配求解Sim3 *****************
                mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
                //Verbose::PrintMess("BoW guess: Solver achieve " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
            }

            if (bConverge) {
                //std::cerr << "Check BoW: SolverSim3 converged" << std::endl;

                //Verbose::PrintMess("BoW guess: Convergende with " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
                // Match by reprojection
                vpCovKFi.clear();
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                vpCovKFi.push_back(pMostBoWMatchesKF);
                set<KeyFrame *> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                //std::cerr << "There are " << vpCovKFi.size() <<" near KFs" << std::endl;

                set<MapPoint *> spMapPoints;
                vector<MapPoint *> vpMapPoints;
                vector<KeyFrame *> vpKeyFrames;
                for (KeyFrame *pCovKFi : vpCovKFi) {
                    for (MapPoint *pCovMPij : pCovKFi->GetMapPointMatches()) {
                        if (!pCovMPij || pCovMPij->isBad())
                            continue;

                        if (spMapPoints.find(pCovMPij) == spMapPoints.end()) {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }

                //std::cerr << "There are " << vpKeyFrames.size() <<" KFs which view all the mappoints" << std::endl;

                g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(), solver.GetEstimatedTranslation().cast<double>(), (double)solver.GetEstimatedScale());
                g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(), pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
                g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current from the world position
                Sophus::Sim3f mScw = Converter::toSophus(gScw);

                vector<MapPoint *> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));
                vector<KeyFrame *> vpMatchedKF;
                vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame *>(NULL));
                // ***************** 根据求解好的Sim3，再根据投影寻找优化的Map Points匹配 *****************
                int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);
                //cerr <<"BoW: " << numProjMatches << " matches between " << vpMapPoints.size() << " points with coarse Sim3" << endl;

                if (numProjMatches >= nProjMatches) {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    bool bFixedScale = mbFixScale;
                    if (mpTracker->mSensor == System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                        bFixedScale = false;

                    // ***************** 根据投影优化后的Map Points进行约束优化 *****************
                    int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);

                    if (numOptMatches >= nSim3Inliers) {
                        g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(), pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
                        g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current from the world position
                        Sophus::Sim3f mScw = Converter::toSophus(gScw);

                        vector<MapPoint *> vpMatchedMP;
                        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));
                        // ***************** 约束优化后，再进行一次根据投影寻找优化的Map Points匹配 *****************
                        int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

                        if (numProjOptMatches >= nProjOptMatches) {
                            int max_x = -1, min_x = 1000000;
                            int max_y = -1, min_y = 1000000;
                            for (MapPoint *pMPi : vpMatchedMP) {
                                if (!pMPi || pMPi->isBad()) {
                                    continue;
                                }

                                tuple<size_t, size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
                                int index = get<0>(indexes);
                                if (index >= 0) {
                                    int coord_x = pKFi->mvKeysUn[index].pt.x;
                                    if (coord_x < min_x) {
                                        min_x = coord_x;
                                    }
                                    if (coord_x > max_x) {
                                        max_x = coord_x;
                                    }
                                    int coord_y = pKFi->mvKeysUn[index].pt.y;
                                    if (coord_y < min_y) {
                                        min_y = coord_y;
                                    }
                                    if (coord_y > max_y) {
                                        max_y = coord_y;
                                    }
                                }
                            }

                            int nNumKFs = 0;
                            //vpMatchedMPs = vpMatchedMP;
                            //vpMPs = vpMapPoints;
                            // Check the Sim3 transformation with the current KeyFrame covisibles
                            // ***************** 取当前关键帧最共视的10个关键帧 *****************
                            vector<KeyFrame *> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

                            int j = 0;
                            while (nNumKFs < 3 && j < vpCurrentCovKFs.size()) // 判定 当前关键帧 最共视的10个关键帧是否和pMostBoWMatchesKF(即当前遍历的候选关键帧)关键帧存在足够多的匹配，提升Place Recognition的Precision
                            {
                                KeyFrame *pKFj = vpCurrentCovKFs[j];
                                Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                                g2o::Sim3 gSjc(mTjc.unit_quaternion(), mTjc.translation(), 1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<MapPoint *> vpMatchedMPs_j;
                                // ***************** 进行判定 *****************
                                bool bValid = DetectCommonRegionsFromLastKF(pKFj, pMostBoWMatchesKF, gSjw, numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                                if (bValid) {
                                    Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
                                    Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
                                    Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                    Eigen::Vector3f vector_dist = Tc_cj.translation();
                                    nNumKFs++;
                                }
                                j++;
                            }

                            if (nNumKFs < 3) {
                                vnStage[index] = 8;
                                vnMatchesStage[index] = nNumKFs;
                            }

                            if (nBestMatchesReproj < numProjOptMatches) {
                                nBestMatchesReproj = numProjOptMatches;
                                nBestNumCoindicendes = nNumKFs;
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                // ***************** 两个关键的结果变量 *****************
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                            }
                        }
                    }
                }
            }
            /*else
            {
                Verbose::PrintMess("BoW candidate: it don't match with the current one", Verbose::VERBOSITY_DEBUG);
            }*/
        }
        index++;
    }
    // ***************** 遍历每个候选关键帧 结束 *****************

    if (nBestMatchesReproj > 0) // 不使用这个最佳匹配Map Point数量判定依据，只要有匹配就认为成功
    {
        pLastCurrentKF = mpCurrentKF;
        nNumCoincidences = nBestNumCoindicendes;
        // ***************** 关键的结果变量 *****************
        // pMatchedKF2：对应Loop closing类中的mpLoopMatchedKF，用于记录当前关键帧的最佳匹配关键帧
        pMatchedKF2 = pBestMatchedKF;
        pMatchedKF2->SetNotErase();
        g2oScw = g2oBestScw; // mg2oMergeSlw
        // ***************** 两个关键的结果变量 *****************
        // vpMapPoints：保存了所有 候选关键帧+其共视帧 的Map Points
        // vpMatchedMP：保存了当前关键帧的Map Points匹配的Map Points，大小固定，无匹配的元素则为NULL
        // 按理说所有vpMatchedMP中匹配的Map Points都可以在vpMapPoints中找到
        vpMPs = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;

        // ***************** 使用nNumCoincidences判定依据，由于选取当前关键帧最共视关键帧的数量(nNumCovisibles)为10，因此这里对应当达到3/10时就认为Place Recognition成功 *****************
        return nNumCoincidences >= 3;
    } else {
        int maxStage = -1;
        int maxMatched;
        for (int i = 0; i < vnStage.size(); ++i) {
            if (vnStage[i] > maxStage) {
                maxStage = vnStage[i];
                maxMatched = vnMatchesStage[i];
            }
        }
    }
    return false;
}

bool CloudMerging::DetectCommonRegionsFromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                 std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs) {
    set<MapPoint *> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

    int nProjMatches = 30;
    if (nNumProjMatches >= nProjMatches) {
        return true;
    }

    return false;
}

int CloudMerging::FindMatchesByProjection(KeyFrame *pCurrentKF, KeyFrame *pMatchedKFw, g2o::Sim3 &g2oScw,
                                          set<MapPoint *> &spMatchedMPinOrigin, vector<MapPoint *> &vpMapPoints,
                                          vector<MapPoint *> &vpMatchedMapPoints) {
    int nNumCovisibles = 10;
    vector<KeyFrame *> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFrame *> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFrame *> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
    if (nInitialCov < nNumCovisibles) {
        for (int i = 0; i < nInitialCov; ++i) {
            vector<KeyFrame *> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
            int nInserted = 0;
            int j = 0;
            while (j < vpKFs.size() && nInserted < nNumCovisibles) {
                if (spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end()) {
                    spCheckKFs.insert(vpKFs[j]);
                    ++nInserted;
                }
                ++j;
            }
            vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
        }
    }
    set<MapPoint *> spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
    for (KeyFrame *pKFi : vpCovKFm) {
        for (MapPoint *pMPij : pKFi->GetMapPointMatches()) {
            if (!pMPij || pMPij->isBad())
                continue;

            if (spMapPoints.find(pMPij) == spMapPoints.end()) {
                spMapPoints.insert(pMPij);
                vpMapPoints.push_back(pMPij);
            }
        }
    }

    Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
    ORBmatcher matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));
    int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

    return num_matches;
}

/*!
 * @param [in] pMainMap 作为Main Map
 * @param [in] pMergedMap  作为Merge Map，将被合并到Main Map
 * @param [in] gSwMainMerged  Sim3变换，Merged Map World -> Main Map World
 * @param [in] pLocalMapper  用于暂停、开启LocalMapper，保证LocalMapper的工作时间
 * @param [in] blockLocalMapper  是否暂停Local Mapper，如果是EdgeFront 和 Cloud，不涉及当前运行的Map，就不需要暂停
 */
void CloudMerging::CloudMergeMap(
    Map *pMainMap, Map *pMergedMap,
    g2o::Sim3 gSwMainMerged,
    const std::map<int, int> &pMainMergedKeyFrameMatch,
    std::map<KeyFrame *, std::vector<std::pair<int, int>>> &vpMainMergedMapPointsMatch,
    LocalMapping *pLocalMapper, bool blockLocalMapper) {
    // 抽取用于求解Sim3的支点关键帧
    const vector<KeyFrame *> &vpMainMapKeyFrames = pMainMap->GetAllKeyFrames();
    const vector<KeyFrame *> &vpMergedMapKeyFrames = pMergedMap->GetAllKeyFrames();
    auto iter = pMainMergedKeyFrameMatch.begin();
    std::advance(iter, rand() % pMainMergedKeyFrameMatch.size());
    KeyFrame *pRandMainKF = vpMainMapKeyFrames[iter->first];
    KeyFrame *pRandMergedKF = vpMergedMapKeyFrames[iter->second];

    // init Sim3
    Sophus::SE3d mTCameraMergedWorld = pRandMergedKF->GetPose().cast<double>(); // 取出当前关键帧
    g2o::Sim3 gSCameraMergedWorld(mTCameraMergedWorld.unit_quaternion(), mTCameraMergedWorld.translation(), 1.0);
    g2o::Sim3 gSMergedCameraMainWorld = gSCameraMergedWorld * gSwMainMerged.inverse();

    if (blockLocalMapper) {
        // ***************** 准备工作：因为需要处理Map，暂停Local Mapping线程，Local Mapping线程将会在处理完正在执行的那次循环后暂停 *****************
        pLocalMapper->RequestStop();
        // Wait until Local Mapping has effectively stopped
        while (!pLocalMapper->isStopped()) // 等待Local Mapping线程处理完正在执行的那次循环
        {
            usleep(1000);
        }
        //cerr << "Local Map stopped" << endl;

        pLocalMapper->EmptyQueue(); // 将队列中的关键帧全部添加到局部地图中，但不进行Local Mapping操作
    }

    // ***************** 开始工作 *****************
    // Ensure current keyframe is updated
    pRandMergedKF->UpdateConnections();

    // spLocalWindowKFs：保存Welding Windows中Active Map部分的所有关键帧
    // spLocalWindowMPs：保存Welding Windows中Active Map部分相关的所有地图点
    set<KeyFrame *> spMergedMapWindowKFs;
    set<MapPoint *> spMergedMapWindowMPs;

    set<KeyFrame *> spMainMapWindowKFs;
    set<MapPoint *> spMainMapWindowMPs;

    // ****** 构建Welding Window ******
    // 添加所有match的KF到welding window，并添加所有match KF各自的5个最佳公视帧
    int numTemporalKFs = 5;
    for (iter = pMainMergedKeyFrameMatch.begin(); iter != pMainMergedKeyFrameMatch.end(); iter++) {
        KeyFrame *pCurMainKF = vpMainMapKeyFrames[iter->first];
        KeyFrame *pCurMergedKF = vpMergedMapKeyFrames[iter->second];

        spMergedMapWindowKFs.insert(pCurMergedKF);
        vector<KeyFrame *> vpCurMergedCovisibleKFs = pCurMergedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
        spMergedMapWindowKFs.insert(vpCurMergedCovisibleKFs.begin(), vpCurMergedCovisibleKFs.end());

        spMainMapWindowKFs.insert(pCurMainKF);
        vector<KeyFrame *> vpCurMainCovisibleKFs = pCurMainKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
        spMainMapWindowKFs.insert(vpCurMainCovisibleKFs.begin(), vpCurMainCovisibleKFs.end());
    }

    // 添加所有看到的Map point
    for (KeyFrame *pKFi : spMergedMapWindowKFs) {
        if (!pKFi || pKFi->isBad())
            continue;

        set<MapPoint *> spMPs = pKFi->GetMapPoints();
        spMergedMapWindowMPs.insert(spMPs.begin(), spMPs.end());
    }

    for (KeyFrame *pKFi : spMainMapWindowKFs) {
        if (!pKFi || pKFi->isBad())
            continue;

        set<MapPoint *> vpMPs = pKFi->GetMapPoints();
        spMainMapWindowMPs.insert(vpMPs.begin(), vpMPs.end());
    }

    // 手动为两边match的keypoints对应的mappoints添加observations
    int nAddObservation = 0;
    for (iter = pMainMergedKeyFrameMatch.begin(); iter != pMainMergedKeyFrameMatch.end(); iter++) {
        KeyFrame *pCurMainKF = vpMainMapKeyFrames[iter->first];
        KeyFrame *pCurMergedKF = vpMergedMapKeyFrames[iter->second];

        if (!(vpMainMergedMapPointsMatch.count(pCurMainKF))) {
            cerr << "Error! " << endl;
            exit(200000);
        }
        std::vector<std::pair<int, int>> vpMapPointsMatch = vpMainMergedMapPointsMatch[pCurMainKF];
        for (auto &iter : vpMapPointsMatch) {
            MapPoint *pCurMainMP = pCurMainKF->GetMapPoint(iter.first);
            MapPoint *pCurMergedMP = pCurMergedKF->GetMapPoint(iter.second);
            if (pCurMainMP && !pCurMainMP->isBad()) {
                pCurMainMP->AddObservation(pCurMergedKF, iter.second);
                nAddObservation++;
            }
            if (pCurMergedMP && !pCurMergedMP->isBad()) {
                pCurMergedMP->AddObservation(pCurMainKF, iter.first);
                nAddObservation++;
            }
        }
    }
    cerr << "Test: Add observation num: " << nAddObservation << endl;

    // 原来的SearchAndFuse是有添加observations功能的，但因为我们CloudMap没有描述子，因此不用它
    // vector<MapPoint *> vpCheckFuseMapPoint;
    // vpCheckFuseMapPoint.reserve(spMainMapWindowMPs.size());
    // std::copy(spMainMapWindowMPs.begin(), spMainMapWindowMPs.end(), std::back_inserter(vpCheckFuseMapPoint));

    // 取出Place Recognition中得到的两个关键帧之间的Sim3相似变化
    Sophus::SE3d Twc = pRandMergedKF->GetPoseInverse().cast<double>();
    g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
    g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
    // Main World->Merged Camera
    g2o::Sim3 g2oCorrectedScw = gSMergedCameraMainWorld; //TODO Check the transformation

    // vCorrectedSim3：保存所有 当前关键帧+其共视关键帧 转到合并地图的Sim3变换
    // vNonCorrectedSim3：保存所有 当前关键帧+其共视关键帧 自己地图内的Sim3变换（只是将原先有的SE3转为Sim3而已）
    KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
    vCorrectedSim3[pRandMergedKF] = g2oCorrectedScw;
    vNonCorrectedSim3[pRandMergedKF] = g2oNonCorrectedScw;

    for (KeyFrame *pKFi : spMergedMapWindowKFs) // 遍历spLocalWindowKFs
    {
        if (!pKFi || pKFi->isBad()) {
            Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
            continue;
        }

        if (pKFi->GetMap() != pMergedMap) // 这种情况也可能发生吗？？
            Verbose::PrintMess("Other map KF, this should't happen", Verbose::VERBOSITY_DEBUG);

        g2o::Sim3 g2oCorrectedSiw;

        // 根据两个关键帧已知的Sim3变换，计算其他共视关键帧的Sim3变换（SE3 + Sim3）
        if (pKFi != pRandMergedKF) {
            Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            //Pose without correction
            vNonCorrectedSim3[pKFi] = g2oSiw;

            Sophus::SE3d Tic = Tiw * Twc;                                    // 计算
            g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0); // SE3转换为Sim3
            // 匹配世界->共视关键帧
            g2oCorrectedSiw = g2oSic * gSMergedCameraMainWorld;
            vCorrectedSim3[pKFi] = g2oCorrectedSiw;
        } else {
            g2oCorrectedSiw = g2oCorrectedScw;
        }
        pKFi->mTcwMerge = pKFi->GetPose();

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        double s = g2oCorrectedSiw.scale();
        // pKFi->mfScale保存关键帧变换的尺度
        pKFi->mfScale = s;
        Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);
        // pKFi->mTcwMerge保存关键帧变换的SE3
        pKFi->mTcwMerge = correctedTiw.cast<float>();

        //TODO DEBUG to know which are the KFs that had been moved to the other map
    }

    // 地图点在Merge Map中的坐标值
    int numPointsWithCorrection = 0;

    //for(MapPoint* pMPi : spLocalWindowMPs)
    set<MapPoint *>::iterator itMP = spMergedMapWindowMPs.begin();
    while (itMP != spMergedMapWindowMPs.end()) {
        MapPoint *pMPi = *itMP;
        // 删除坏点
        if (!pMPi || pMPi->isBad()) {
            itMP = spMergedMapWindowMPs.erase(itMP);
            continue;
        }

        KeyFrame *pKFref = pMPi->GetReferenceKeyFrame();
        if (vCorrectedSim3.find(pKFref) == vCorrectedSim3.end()) {
            itMP = spMergedMapWindowMPs.erase(itMP);
            numPointsWithCorrection++;
            continue;
        }
        g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

        // Project with non-corrected pose and project back with corrected pose
        Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
        Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
        Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

        // 结果保存至Map Point内部的成员变量
        pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
        pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

        itMP++;
    }

    // ***************** 迁移关键帧和Map Point，根据Active Map中关键帧和Map Point计算好的Merge Map坐标系下的Pose、坐标，进行迁移 *****************
    {
        // 这里不加锁也没事吧，Map线程已经Stop了：No! 不加锁有事，Tracking线程会使用这个锁，必须暂停Tracking线程
        unique_lock<mutex> currentLock(pMergedMap->mMutexMapUpdate); // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(pMainMap->mMutexMapUpdate);     // We remove the Kfs and MPs in the merged area from the old map

        //std::cerr << "Merge local window: " << spLocalWindowKFs.size() << std::endl;
        //std::cerr << "[Merge]: init merging maps " << std::endl;

        // 更新Active Map中Welding window中的关键帧
        for (KeyFrame *pKFi : spMergedMapWindowKFs) {
            if (!pKFi || pKFi->isBad()) {
                //std::cerr << "Bad KF in correction" << std::endl;
                continue;
            }

            //std::cerr << "KF id: " << pKFi->mnId << std::endl;

            pKFi->mTcwBefMerge = pKFi->GetPose();        // mTcw Before Merge
            pKFi->mTwcBefMerge = pKFi->GetPoseInverse(); // mTwc Before Merge
            // 将关键帧的Pose更新为Merge Map坐标系下的Pose
            pKFi->SetPose(pKFi->mTcwMerge);

            // Make sure connections are updated
            // 将关键帧添加到Merge Map中，并从Active Map中删去
            pKFi->UpdateMap(pMainMap);
            pKFi->mnMergeCorrectedForKF = pRandMergedKF->mnId;
            pMainMap->AddKeyFrame(pKFi);
            pMergedMap->EraseKeyFrame(pKFi);
        }

        // 更新Active Map中所有的Map Point
        // 注意，到此MapPoint都不用管重复的问题，都是直接Sim3变换得到结果
        for (MapPoint *pMPi : spMergedMapWindowMPs) {
            if (!pMPi || pMPi->isBad())
                continue;

            // 将MapPoint的坐标更新为Merge Map坐标系下的坐标
            pMPi->SetWorldPos(pMPi->mPosMerge);
            pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
            // 将MapPoint添加到Merge Map中，并从Active Map中删去
            pMPi->UpdateMap(pMainMap);
            pMainMap->AddMapPoint(pMPi);
            pMergedMap->EraseMapPoint(pMPi);
        }

        pMainMap->IncreaseChangeIndex();
        //TODO for debug
        //        pMainMap->ChangeId(pMergedMap->GetId());

        std::cerr << "[Merge]: merging maps finished" << std::endl;
        // @note 添加Merged Map Id到Main Map中
        pMainMap->mvMergedMapIds.push_back(pMergedMap->GetId());
    }

    // ***************** 前面完成Welding Window的迁移后，构建新的Essential Graph *****************
    // 在Active Map中，Welding window外的关键帧、MapPoint目前都还没有迁移过来
    // Rebuild the essential graph in the local window
    // 下面在拓展树中父子颠倒的操作是什么意思？
    // @note 颠倒拓展树
    KeyFrame *pNewChild;
    KeyFrame *pNewParent;
    pMergedMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = pRandMergedKF->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = pRandMergedKF;             // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    pRandMergedKF->ChangeParent(pRandMainKF);
    while (pNewChild) // TODO Bug 会卡在这里
    {
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        KeyFrame *pOldParent = pNewChild->GetParent();

        pNewChild->ChangeParent(pNewParent);

        pNewParent = pNewChild;
        pNewChild = pOldParent;
    }

    //Update the connections between the local window
    // ***************** Active Map中的Map Point虽然迁移到Merge Map了，但是没看到添加observation，这样update不应该有效 *****************
    pRandMainKF->UpdateConnections();

    // Project MapPoints observed in the neighborhood of the merge keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    // ***************** 第七步，进行Map Point的融合去重 *****************
    // @note SearchAndFuse
    // std::cerr << "[Merge]: start fuse points" << std::endl;
    // 原来的SearchAndFuse是有添加observations功能的，但因为我们CloudMap没有描述子，因此不用它
    // SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);
    // std::cerr << "[Merge]: fuse points finished" << std::endl;

    // Update connectivity
    // 在Map Point的Observation更新完成后，共视图的更新就有效了
    for (KeyFrame *pKFi : spMergedMapWindowKFs) {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for (KeyFrame *pKFi : spMainMapWindowKFs) {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    std::cerr << "[Merge]: Start welding bundle adjustment" << std::endl;

    bool bStop = false;
    vector<KeyFrame *> vpMergedMapWindowKFs;
    vector<KeyFrame *> vpMainMapKFs;
    std::copy(spMergedMapWindowKFs.begin(), spMergedMapWindowKFs.end(), std::back_inserter(vpMergedMapWindowKFs));
    std::copy(spMainMapWindowKFs.begin(), spMainMapWindowKFs.end(), std::back_inserter(vpMainMapKFs));
    std::cerr << "[Merge]: Local bundle adjustment, spLocalWindowKFs size: " << spMergedMapWindowKFs.size() << std::endl;
    std::cerr << "[Merge]: Local bundle adjustment, spMergeConnectedKFs size: " << spMainMapWindowKFs.size() << std::endl;

    // 和Local Map线程中利用Local Map进行Local BA保持一致
    // @note LocalBA
    Optimizer::LocalBundleAdjustment(pRandMergedKF, vpMergedMapWindowKFs, vpMainMapKFs, &bStop);

    std::cerr << "[Merge]: Welding bundle adjustment finished" << std::endl;

    if (blockLocalMapper) {
        pLocalMapper->Release();
    }

    // ***************** 第八步，将Welding window外的原Active Map的关键帧、Map Point迁移到Merged map *****************
    // Update the non critical area from the current map to the merged map
    // 取出原Active Map的所有剩下的关键帧、Map points（Welding window内的都已经迁移过去了）
    vector<KeyFrame *> vpCurrentMergedMapKFs = pMergedMap->GetAllKeyFrames();
    vector<MapPoint *> vpCurrentMergedMapMPs = pMergedMap->GetAllMapPoints();

    if (vpCurrentMergedMapKFs.size() == 0) {
    } else {
        // ***************** 和Welding window的迁移一致，先计算关键帧、再计算MapPoint，更新它们在Merged Map中的信息 *****************
        {
            unique_lock<mutex> currentLock(
                pMergedMap->mMutexMapUpdate); // We update the current map with the Merge information

            for (KeyFrame *pKFi : vpCurrentMergedMapKFs) {
                if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergedMap) {
                    continue;
                }

                g2o::Sim3 g2oCorrectedSiw;

                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
                //Pose without correction
                vNonCorrectedSim3[pKFi] = g2oSiw; // 更新vNonCorrectedSim3

                Sophus::SE3d Tic = Tiw * Twc;
                g2o::Sim3 g2oSim(Tic.unit_quaternion(), Tic.translation(), 1.0);
                g2oCorrectedSiw = g2oSim * gSMergedCameraMainWorld;
                vCorrectedSim3[pKFi] = g2oCorrectedSiw; // 更新vCorrectedSim3

                // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                double s = g2oCorrectedSiw.scale();

                pKFi->mfScale = s;

                Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

                pKFi->mTcwBefMerge = pKFi->GetPose();
                pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                pKFi->SetPose(correctedTiw.cast<float>());
            }
            for (MapPoint *pMPi : vpCurrentMergedMapMPs) {
                if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergedMap)
                    continue;

                KeyFrame *pKFref = pMPi->GetReferenceKeyFrame();
                g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                // Project with non-corrected pose and project back with corrected pose
                Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
                pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());

                pMPi->UpdateNormalAndDepth();
            }
        }

        if (blockLocalMapper) {
            // 再次请求暂停Local Map线程，等待Local Map线程结束
            pLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
            while (!pLocalMapper->isStopped()) {
                usleep(1000);
            }
        }

        {
            // ***************** 将剩下所有关键帧、Map Point迁移过去，但是不进行Local BA，而是放进去之后等待Global BA慢慢进行更新 *****************
            // Get Merge Map Mutex
            unique_lock<mutex> currentLock(pMergedMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMainMap->mMutexMapUpdate);     // We remove the Kfs and MPs in the merged area from the old map

            //std::cerr << "Merge outside KFs: " << vpCurrentMapKFs.size() << std::endl;
            for (KeyFrame *pKFi : vpCurrentMergedMapKFs) {
                if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergedMap) {
                    continue;
                }
                //std::cerr << "KF id: " << pKFi->mnId << std::endl;

                // Make sure connections are updated
                pKFi->UpdateMap(pMainMap);
                pMainMap->AddKeyFrame(pKFi);
                pMergedMap->EraseKeyFrame(pKFi);
            }

            for (MapPoint *pMPi : vpCurrentMergedMapMPs) {
                if (!pMPi || pMPi->isBad())
                    continue;

                pMPi->UpdateMap(pMainMap);
                pMainMap->AddMapPoint(pMPi);
                pMergedMap->EraseMapPoint(pMPi);
            }
        }
    }
    if (blockLocalMapper) {
        pLocalMapper->Release();
    }
}

void CloudMerging::CheckObservations(set<KeyFrame *> &spKFsMap1, set<KeyFrame *> &spKFsMap2) {
    cerr << "----------------------" << endl;
    for (KeyFrame *pKFi1 : spKFsMap1) {
        map<KeyFrame *, int> mMatchedMP;
        set<MapPoint *> spMPs = pKFi1->GetMapPoints();

        for (MapPoint *pMPij : spMPs) {
            if (!pMPij || pMPij->isBad()) {
                continue;
            }

            map<KeyFrame *, tuple<int, int>> mMPijObs = pMPij->GetObservations();
            for (KeyFrame *pKFi2 : spKFsMap2) {
                if (mMPijObs.find(pKFi2) != mMPijObs.end()) {
                    if (mMatchedMP.find(pKFi2) != mMatchedMP.end()) {
                        mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
                    } else {
                        mMatchedMP[pKFi2] = 1;
                    }
                }
            }
        }

        if (mMatchedMP.size() == 0) {
            cerr << "CHECK-OBS: KF " << pKFi1->mnId << " has not any matched MP with the other map" << endl;
        } else {
            cerr << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with " << mMatchedMP.size() << " KF from the other map" << endl;
            for (pair<KeyFrame *, int> matchedKF : mMatchedMP) {
                cerr << "   -KF: " << matchedKF.first->mnId << ", Number of matches: " << matchedKF.second << endl;
            }
        }
    }
    cerr << "----------------------" << endl;
}

void CloudMerging::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint *> &vpMapPoints) {
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    //cerr << "[FUSE]: Initially there are " << vpMapPoints.size() << " MPs" << endl;
    //cerr << "FUSE: Intially there are " << CorrectedPosesMap.size() << " KFs" << endl;
    for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend; mit++) {
        int num_replaces = 0;
        KeyFrame *pKFi = mit->first;
        Map *pMap = pKFi->GetMap();

        g2o::Sim3 g2oScw = mit->second;
        Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

        vector<MapPoint *> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint *>(NULL));
        int numFused = matcher.Fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints); // Fuse中会为Map Point添加Observation

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++) {
            MapPoint *pRep = vpReplacePoints[i];
            if (pRep) {
                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }

        total_replaces += num_replaces;
    }
    //cerr << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
}

void CloudMerging::SearchAndFuse(const vector<KeyFrame *> &vConectedKFs, vector<MapPoint *> &vpMapPoints) {
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    //cerr << "FUSE-POSE: Initially there are " << vpMapPoints.size() << " MPs" << endl;
    //cerr << "FUSE-POSE: Intially there are " << vConectedKFs.size() << " KFs" << endl;
    for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end(); mit != mend; mit++) {
        int num_replaces = 0;
        KeyFrame *pKF = (*mit);
        Map *pMap = pKF->GetMap();
        Sophus::SE3f Tcw = pKF->GetPose();
        Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
        Scw.setScale(1.f);
        /*std::cerr << "These should be zeros: " <<
            Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
            Scw.translation() - Tcw.translation() << std::endl <<
            Scw.scale() - 1.f << std::endl;*/
        vector<MapPoint *> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint *>(NULL));
        matcher.Fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++) {
            MapPoint *pRep = vpReplacePoints[i];
            if (pRep) {
                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }
        /*cerr << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaces << " MPs fused" << endl;
        total_replaces += num_replaces;*/
    }
    //cerr << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;
}

void CloudMerging::RequestReset() {
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while (1) {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetRequested)
                break;
        }
        usleep(5000);
    }
}

void CloudMerging::RequestResetActiveMap(Map *pMap) {
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMapRequested = true;
        mpMapToReset = pMap;
    }

    while (1) {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetActiveMapRequested)
                break;
        }
        usleep(3000);
    }
}

void CloudMerging::ResetIfRequested() {
    //    // TODO need to change
    //    unique_lock<mutex> lock(mMutexReset);
    //    if(mbResetRequested)
    //    {
    //        cerr << "Loop closer reset requested..." << endl;
    //        mlpCloudMapQueue.clear();
    //        mLastLoopKFid=0;  //TODO old variable, it is not use in the new algorithm
    //        mbResetRequested=false;
    //        mbResetActiveMapRequested = false;
    //    }
    //    else if(mbResetActiveMapRequested)
    //    {
    //
    //        for (list<KeyFrame*>::const_iterator it=mlpCloudMapQueue.begin(); it != mlpCloudMapQueue.end();)
    //        {
    //            KeyFrame* pKFi = *it;
    //            if(pKFi->GetMap() == mpMapToReset)
    //            {
    //                it = mlpCloudMapQueue.erase(it);
    //            }
    //            else
    //                ++it;
    //        }
    //
    //        mLastLoopKFid=mpAtlas->GetLastInitKFid(); //TODO old variable, it is not use in the new algorithm
    //        mbResetActiveMapRequested=false;
    //
    //    }
}

void CloudMerging::RunGlobalBundleAdjustment(Map *pActiveMap, unsigned long nLoopKF) {
    Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFGBA = std::chrono::steady_clock::now();

    nFGBA_exec += 1;

    vnGBAKFs.push_back(pActiveMap->GetAllKeyFrames().size());
    vnGBAMPs.push_back(pActiveMap->GetAllMapPoints().size());
#endif

    const bool bImuInit = pActiveMap->isImuInitialized();

    if (!bImuInit)
        Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF, false);
    else
        Optimizer::FullInertialBA(pActiveMap, 7, false, nLoopKF, &mbStopGBA);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndGBA = std::chrono::steady_clock::now();

    double timeGBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndGBA - time_StartFGBA).count();
    vdGBA_ms.push_back(timeGBA);

    if (mbStopGBA) {
        nFGBA_abort += 1;
    }
#endif

    int idx = mnFullBAIdx;
    // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if (idx != mnFullBAIdx)
            return;

        if (!bImuInit && pActiveMap->isImuInitialized())
            return;

        if (!mbStopGBA) {
            Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
            Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()) {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
            // cerr << "LC: Update Map Mutex adquired" << endl;

            //pActiveMap->PrintEssentialGraph();
            // Correct keyframes starting at map first keyframe
            list<KeyFrame *> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(), pActiveMap->mvpKeyFrameOrigins.end());

            while (!lpKFtoCheck.empty()) {
                KeyFrame *pKF = lpKFtoCheck.front();
                const set<KeyFrame *> sChilds = pKF->GetChilds();
                //cerr << "---Updating KF " << pKF->mnId << " with " << sChilds.size() << " childs" << endl;
                //cerr << " KF mnBAGlobalForKF: " << pKF->mnBAGlobalForKF << endl;
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                //cerr << "Twc: " << Twc << endl;
                //cerr << "GBA: Correct KeyFrames" << endl;
                for (set<KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
                    KeyFrame *pChild = *sit;
                    if (!pChild || pChild->isBad())
                        continue;

                    if (pChild->mnBAGlobalForKF != nLoopKF) {
                        //cerr << "++++New child with flag " << pChild->mnBAGlobalForKF << "; LoopKF: " << nLoopKF << endl;
                        //cerr << " child id: " << pChild->mnId << endl;
                        Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                        //cerr << "Child pose: " << Tchildc << endl;
                        //cerr << "pKF->mTcwGBA: " << pKF->mTcwGBA << endl;
                        pChild->mTcwGBA = Tchildc * pKF->mTcwGBA; //*Tcorc*pKF->mTcwGBA;

                        Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                        if (pChild->isVelocitySet()) {
                            pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                        } else
                            Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);

                        //cerr << "Child bias: " << pChild->GetImuBias() << endl;
                        pChild->mBiasGBA = pChild->GetImuBias();

                        pChild->mnBAGlobalForKF = nLoopKF;
                    }
                    lpKFtoCheck.push_back(pChild);
                }

                //cerr << "-------Update pose" << endl;
                pKF->mTcwBefGBA = pKF->GetPose();
                //cerr << "pKF->mTcwBefGBA: " << pKF->mTcwBefGBA << endl;
                pKF->SetPose(pKF->mTcwGBA);
                /*cv::Mat Tco_cn = pKF->mTcwBefGBA * pKF->mTcwGBA.inv();
                cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
                double dist = cv::norm(trasl);
                cerr << "GBA: KF " << pKF->mnId << " had been moved " << dist << " meters" << endl;
                double desvX = 0;
                double desvY = 0;
                double desvZ = 0;
                if(pKF->mbHasHessian)
                {
                    cv::Mat hessianInv = pKF->mHessianPose.inv();

                    double covX = hessianInv.at<double>(3,3);
                    desvX = std::sqrt(covX);
                    double covY = hessianInv.at<double>(4,4);
                    desvY = std::sqrt(covY);
                    double covZ = hessianInv.at<double>(5,5);
                    desvZ = std::sqrt(covZ);
                    pKF->mbHasHessian = false;
                }
                if(dist > 1)
                {
                    cerr << "--To much distance correction: It has " << pKF->GetConnectedKeyFrames().size() << " connected KFs" << endl;
                    cerr << "--It has " << pKF->GetCovisiblesByWeight(80).size() << " connected KF with 80 common matches or more" << endl;
                    cerr << "--It has " << pKF->GetCovisiblesByWeight(50).size() << " connected KF with 50 common matches or more" << endl;
                    cerr << "--It has " << pKF->GetCovisiblesByWeight(20).size() << " connected KF with 20 common matches or more" << endl;

                    cerr << "--STD in meters(x, y, z): " << desvX << ", " << desvY << ", " << desvZ << endl;


                    string strNameFile = pKF->mNameFile;
                    cv::Mat imLeft = cv::imread(strNameFile, CV_LOAD_IMAGE_UNCHANGED);

                    cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);

                    vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
                    int num_MPs = 0;
                    for(int i=0; i<vpMapPointsKF.size(); ++i)
                    {
                        if(!vpMapPointsKF[i] || vpMapPointsKF[i]->isBad())
                        {
                            continue;
                        }
                        num_MPs += 1;
                        string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                        cv::circle(imLeft, pKF->mvKeys[i].pt, 2, cv::Scalar(0, 255, 0));
                        cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                    }
                    cerr << "--It has " << num_MPs << " MPs matched in the map" << endl;

                    string namefile = "./test_GBA/GBA_" + to_string(nLoopKF) + "_KF" + to_string(pKF->mnId) +"_D" + to_string(dist) +".png";
                    cv::imwrite(namefile, imLeft);
                }*/

                if (pKF->bImu) {
                    //cerr << "-------Update inertial values" << endl;
                    pKF->mVwbBefGBA = pKF->GetVelocity();
                    //if (pKF->mVwbGBA.empty())
                    //    Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                    //assert(!pKF->mVwbGBA.empty());
                    pKF->SetVelocity(pKF->mVwbGBA);
                    pKF->SetNewBias(pKF->mBiasGBA);
                }

                lpKFtoCheck.pop_front();
            }

            //cerr << "GBA: Correct MapPoints" << endl;
            // Correct MapPoints
            const vector<MapPoint *> vpMPs = pActiveMap->GetAllMapPoints();

            for (size_t i = 0; i < vpMPs.size(); i++) {
                MapPoint *pMP = vpMPs[i];

                if (pMP->isBad())
                    continue;

                if (pMP->mnBAGlobalForKF == nLoopKF) {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                } else {
                    // Update according to the correction of its reference keyframe
                    KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                    if (pRefKF->mnBAGlobalForKF != nLoopKF)
                        continue;

                    /*if(pRefKF->mTcwBefGBA.empty())
                        continue;*/

                    // Map to non-corrected camera
                    // cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    // cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                    // Backproject using corrected camera
                    pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
                }
            }

            pActiveMap->InformNewBigChange();
            pActiveMap->IncreaseChangeIndex();

            // TODO Check this update
            // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(), mpTracker->GetLastKeyFrame());

            mpLocalMapper->Release();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndUpdateMap = std::chrono::steady_clock::now();

            double timeUpdateMap = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndUpdateMap - time_EndGBA).count();
            vdUpdateMap_ms.push_back(timeUpdateMap);

            double timeFGBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndUpdateMap - time_StartFGBA).count();
            vdFGBATotal_ms.push_back(timeFGBA);
#endif
            Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void CloudMerging::RequestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    // cerr << "LC: Finish requested" << endl;
    mbFinishRequested = true;
}

bool CloudMerging::CheckFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void CloudMerging::SetFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool CloudMerging::isFinished() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

bool CloudMerging::isRunning() {
    return mbRunning;
}

Map *CloudMerging::GetEdgeFrontMap() {
    return mpCurrentEdgeFrontMap;
}

Map *CloudMerging::GetEdgeBackMap() {
    return mpCurrentEdgeBackMap;
}

Map *CloudMerging::GetCloudMap() {
    return mpCurrentCloudMap;
}

} // namespace ORB_SLAM3
