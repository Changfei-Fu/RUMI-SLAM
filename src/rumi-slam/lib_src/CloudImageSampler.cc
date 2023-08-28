#include "CloudImageSampler.h"
#include "globals.h"
#include "KFDSample.h"
#include "KeyFrame.h"
#include "Tracking.h"
#include <cstdlib>
#include <opencv2/core.hpp>
#include <string>
#include <utility>

namespace ORB_SLAM3 {

CloudImage::CloudImage(cv::Mat img, double timestamp, std::string type) :
    img(img), timestamp(timestamp), type(type) {
}

CloudImageSampler::CloudImageSampler(System *pSys, int nTrackLast, int nNewTrackFirst, float nTrackLastMinTime, float nNewTrackFirstMinTime, float Kp, float Kd, float th) :
    mpSystem(pSys), mState(OLD_MAP_SAMPLE), mTrackingLastState(Tracking::NO_IMAGES_YET), nTrackLast(nTrackLast), nNewTrackFirst(nNewTrackFirst), nTrackLastMinTime(nTrackLastMinTime), nNewTrackFirstMinTime(nNewTrackFirstMinTime) {
    mpKFDSampler = new KFDSample();
    mpKFDSampler->SetPDKFselectorParams(Kp, Kd, th);
}

void CloudImageSampler::Reset() {
    // mEdgeFrontMapId = -1;
    // mEdgeBackMapId = -1;
    // mState = OLD_MAP_SAMPLE;
    mvLostImages.clear();
    mvCurrentCloudProcessImages.clear();
    mvCurrentCloudProcessNoSamplingImages.clear();
}

void CloudImageSampler::UpdateResetMapImages(Map *pMap) {
    for (auto &kf : pMap->GetAllKeyFrames()) {
        mvLostImages.push_back(CloudImage(kf->imgGray, kf->mTimeStamp, "lost"));
    }
}

float inline ComputeKFDistance(KeyFrame *pKF1, KeyFrame *pKF2) {
    auto prevKFCenter = pKF1->GetCameraCenter();
    auto curKFCenter = pKF2->GetCameraCenter();
    return sqrt(pow(prevKFCenter.x() - curKFCenter.x(), 2) + pow(prevKFCenter.y() - curKFCenter.y(), 2) + pow(prevKFCenter.z() - curKFCenter.z(), 2));
}

void CloudImageSampler::TrackStep(const int &curTrackingState, const bool &bIsKF, const int &nKFInMap, const int &curMapId, const cv::Mat &img, const double &timestamp) {
    // 记录RECENTLY_LOST和LOST的图像
    if (curTrackingState == Tracking::LOST || curTrackingState == Tracking::NOT_INITIALIZED || curTrackingState == Tracking::RECENTLY_LOST) {
        mvLostNoSamplingImages.push_back(CloudImage(img, timestamp, "lost"));
        bool bSelectKF = mpKFDSampler->Step(img, timestamp);
        if (bSelectKF) {
            mvLostImages.push_back(CloudImage(img, timestamp, "lost"));
        }
    } else {
        mpKFDSampler->Reset();
    }

    // 是否Merge过，是否Map数量大于2，若不是则跳过
    int nCurAtlasMapNum = mpSystem->GetAtlas()->CountMaps();
    if (nCurAtlasMapNum < 2) {
        return;
    }

    const std::vector<Map *> vpMaps = mpSystem->GetAtlas()->GetAllMaps();
    Map *pEdgeFrontMap = vpMaps[vpMaps.size() - 2];
    Map *pEdgeBackMap = vpMaps[vpMaps.size() - 1];

    if (pEdgeFrontMap->IsHaveMerged() || pEdgeBackMap->IsHaveMerged()) {
        if (pEdgeFrontMap->IsHaveMerged()) {
            // cerr << "Edge Front Map Have Merge !!!!!!!!!!!!!!!!!!!!!!" << endl;
        }
        if (pEdgeBackMap->IsHaveMerged()) {
            // cerr << "Edge Back Map Have Merge !!!!!!!!!!!!!!!!!!!!!!" << endl;
        }
        return;
    }

    int nCurMapKFNum = pEdgeBackMap->KeyFramesInMap();
    double nCurMapDuration = pEdgeBackMap->KeyFramesDuration();
    cerr << "secondFrameTime: " << nCurMapDuration << endl;

    // 当Map KF Num大于20且Duration大于3则进行一次Merge
    if (nCurMapKFNum > nNewTrackFirst && nCurMapDuration > nNewTrackFirstMinTime && pEdgeBackMap->ComputeTrajCurvature() > nNewTrackFirstMinTrajCurvature) {
        nLostTime++;
        assert(pEdgeFrontMap == mpSystem->GetAtlas()->GetCurrentMap());
        assert(pEdgeFrontMap != nullptr);
        assert(pEdgeBackMap != nullptr);

        // 一些情况需要跳过
        if (mvLostImages.size() < 5) {
            cerr << "Lost Images不足够，特殊情况" << endl;
            return;
        }

        // 添加Update Lock
        unique_lock<mutex> edgeFrontLock(pEdgeFrontMap->mMutexMapUpdate);
        // 不需要添加edgeBackLock，因为在Track线程里TrackStep前已经加锁了
        // unique_lock<mutex> edgeBackLock(pEdgeBackMap->mMutexMapUpdate);

        std::vector<CloudImage> vTrackLastNKeyFrameImages;
        std::vector<CloudImage> vLostImages;
        std::vector<CloudImage> vLostNoSamplingImages;
        std::vector<CloudImage> vNewTrackFirstNKeyFrameImages;
        double firstFrameTime, lastFrameTime;
        // 导出OLD Track Map KF
        std::vector<KeyFrame *> vpEdgeFrontKeyframes = pEdgeFrontMap->GetAllKeyFrames();
        Map::SortKF(vpEdgeFrontKeyframes);
        std::reverse(vpEdgeFrontKeyframes.begin(), vpEdgeFrontKeyframes.end());
        std::vector<KeyFrame *> tvpEdgeFrontKeyframes;
        double lastTime = vpEdgeFrontKeyframes[0]->mTimeStamp;
        nLostTimeStamp = lastTime;
        for (auto &pKF : vpEdgeFrontKeyframes) {
            firstFrameTime = pKF->mTimeStamp;
            cerr << "firstFrameTime: " << (lastTime - pKF->mTimeStamp) << endl;
            if (((lastTime - pKF->mTimeStamp) > nTrackLastMinTime) && (vTrackLastNKeyFrameImages.size() > nTrackLast) && (Map::ComputeKFCurvature(tvpEdgeFrontKeyframes) > nTrackLastMinTrajCurvature)) {
                // 如果Delta Time和KF Num都满足了才直接结束，否则取所有
                break;
            }
            if (vTrackLastNKeyFrameImages.size() > nMaxTrackLast) {
                break;
            }
            vTrackLastNKeyFrameImages.push_back(CloudImage(pKF->imgGray.clone(), pKF->mTimeStamp, "edge_front"));
            tvpEdgeFrontKeyframes.push_back(pKF);
        }
        mvCurrentCloudProcessImages.insert(mvCurrentCloudProcessImages.end(), vTrackLastNKeyFrameImages.begin(), vTrackLastNKeyFrameImages.end());
        mvCurrentCloudProcessNoSamplingImages.insert(mvCurrentCloudProcessNoSamplingImages.end(), vTrackLastNKeyFrameImages.begin(), vTrackLastNKeyFrameImages.end());

        // 导出NEW Track Map KF
        std::vector<KeyFrame *> vpEdgeBackKeyframes = pEdgeBackMap->GetAllKeyFrames();
        Map::SortKF(vpEdgeBackKeyframes);
        std::vector<KeyFrame *> tvpEdgeBackKeyframes;
        double firstTime = vpEdgeBackKeyframes[0]->mTimeStamp;
        nNewMapTimeStamp = firstTime;
        for (auto &pKF : vpEdgeBackKeyframes) {
            lastFrameTime = pKF->mTimeStamp;
            if (((pKF->mTimeStamp - firstTime) > nNewTrackFirstMinTime) && (vNewTrackFirstNKeyFrameImages.size() > nNewTrackFirst) && (Map::ComputeKFCurvature(tvpEdgeBackKeyframes) > nNewTrackFirstMinTrajCurvature)) {
                // 如果Delta Time和KF Num都满足了才直接结束，否则取所有
                break;
            }
            if (vNewTrackFirstNKeyFrameImages.size() > nMaxNewTrackFirst) {
                break;
            }
            vNewTrackFirstNKeyFrameImages.push_back(CloudImage(pKF->imgGray.clone(), pKF->mTimeStamp, "edge_back"));
            tvpEdgeBackKeyframes.push_back(pKF);
        }
        mvCurrentCloudProcessImages.insert(mvCurrentCloudProcessImages.end(), vNewTrackFirstNKeyFrameImages.begin(), vNewTrackFirstNKeyFrameImages.end());
        mvCurrentCloudProcessNoSamplingImages.insert(mvCurrentCloudProcessNoSamplingImages.end(), vNewTrackFirstNKeyFrameImages.begin(), vNewTrackFirstNKeyFrameImages.end());

        // 过滤并清空Lost Images
        for (auto &elem : mvLostImages) {
            if (elem.timestamp > firstFrameTime && elem.timestamp < lastFrameTime) {
                vLostImages.push_back(elem);
            }
        }
        mvCurrentCloudProcessImages.insert(mvCurrentCloudProcessImages.end(), vLostImages.begin(), vLostImages.end());
        for (auto &elem : mvLostNoSamplingImages) {
            if (elem.timestamp > firstFrameTime && elem.timestamp < lastFrameTime) {
                vLostNoSamplingImages.push_back(elem);
            }
        }
        mvCurrentCloudProcessNoSamplingImages.insert(mvCurrentCloudProcessNoSamplingImages.end(), vLostNoSamplingImages.begin(), vLostNoSamplingImages.end());

        // 最后排序
        {
            struct compFunctor {
                inline bool operator()(CloudImage elem1, CloudImage elem2) {
                    return elem1.timestamp < elem2.timestamp; // 从大到小
                }
            };
            sort(mvCurrentCloudProcessImages.begin(), mvCurrentCloudProcessImages.end(), compFunctor());
            sort(mvCurrentCloudProcessNoSamplingImages.begin(), mvCurrentCloudProcessNoSamplingImages.end(), compFunctor());
        }

        // 不允许有无图像的帧
        for (auto &iter : mvCurrentCloudProcessImages) {
            // cerr.precision(16);
            // cerr << iter.second << endl;
            assert(iter.img.dims != 0);
        }

        // 不允许无Lost Images，因为大概率不正常
        assert(vLostImages.size() > 0);

        // 要求至少有30帧，否则不正常
        assert(mvCurrentCloudProcessImages.size() > 30);

        // 最后整理
        // 记录map Id
        mEdgeFrontMapId = pEdgeFrontMap->GetId();
        mEdgeBackMapId = pEdgeBackMap->GetId();
        // 设置flag
        pEdgeFrontMap->SetHaveMerged();
        pEdgeBackMap->SetHaveMerged();

        // 最后发布
        mpSystem->PublishCloudImages(mvCurrentCloudProcessImages, mvCurrentCloudProcessNoSamplingImages, mEdgeFrontMapId, mEdgeBackMapId);

        mvLastCloudProcessImages = mvCurrentCloudProcessImages;
        for (auto &iter : mvLastCloudProcessImages) {
            mdLastCloudProcessImages[std::to_string(iter.timestamp)] = iter.img;
        }

        Reset();
    }

    // if (mTrackingLastState == Tracking::NO_IMAGES_YET) {
    //     // init
    //     mTrackingLastState = curTrackingState;
    //     return;
    // }

    // // @note 核心步骤：一旦有LOST且KF >= 10，就会保存旧图，新建新图
    // if (curTrackingState == Tracking::LOST && nKFInMap >= 10) {
    //     mState = NEW_MAP_SAMPLE;
    //     return;
    // }

    // if (mState == NEW_MAP_SAMPLE && curTrackingState == Tracking::LOST) {
    //     Map *pEdgeBackMap = mpSystem->GetAtlas()->GetSpecifyMap(curMapId);
    //     if (pEdgeBackMap->KeyFramesInMap() < 10) {
    //         mState = OLD_MAP_SAMPLE;
    //         return;
    //     }
    // }

    // // @note 核心步骤：每次LOST要重置光流，检测几种情况的跳变即可
    // if ((mTrackingLastState == Tracking::OK && curTrackingState == Tracking::RECENTLY_LOST) || (mTrackingLastState == Tracking::OK && curTrackingState == Tracking::LOST)) {
    //     mpKFDSampler->Reset();
    // }

    // bool bSelectKF = false;
    // if (mState == OLD_MAP_SAMPLE) {
    //     mEdgeFrontMapId = curMapId;
    //     if (curTrackingState == Tracking::OK) {
    //     } else if (curTrackingState == Tracking::RECENTLY_LOST) {
    //         bSelectKF = mpKFDSampler->Step(img, timestamp);
    //         if (bSelectKF) {
    //             mvLostImages.push_back(std::pair<cv::Mat, double>(img, timestamp));
    //         }
    //     } else {
    //         // 如果刚初始化，会有一段NOT_INITLIZED
    //         cerr << "Cloud Sampler OLD_MAP_SAMPLE state, meet unexpected: " << curTrackingState << endl;
    //     }
    // } else if (mState == NEW_MAP_SAMPLE) {
    //     mEdgeBackMapId = curMapId;
    //     if (curTrackingState == Tracking::LOST || curTrackingState == Tracking::NOT_INITIALIZED || curTrackingState == Tracking::RECENTLY_LOST) {
    //         bSelectKF = mpKFDSampler->Step(img, timestamp);
    //         if (bSelectKF) {
    //             mvLostImages.push_back(std::pair<cv::Mat, double>(img, timestamp));
    //         }
    //     } else if (curTrackingState == Tracking::OK) {
    //     } else {
    //         cerr << "Cloud Sampler OLD_MAP_SAMPLE state, meet unexpected: " << curTrackingState << endl;
    //     }
    // }

    // double nNewTrackFirstMinTime = 3.0;
    // double nTrackLastMinTime = 3.0;
    // // 在特定mState的特定条件下进行特定任务
    // if (mState == NEW_MAP_SAMPLE) {
    //     Map *pEdgeFrontMap = mpSystem->GetAtlas()->GetSpecifyMap(mEdgeFrontMapId);
    //     Map *pEdgeBackMap = mpSystem->GetAtlas()->GetSpecifyMap(mEdgeBackMapId);
    //     std::vector<KeyFrame *> vpEdgeBackKeyframes = pEdgeBackMap->GetAllKeyFrames();
    //     {
    //         struct compFunctor {
    //             inline bool operator()(KeyFrame *elem1, KeyFrame *elem2) {
    //                 return elem1->mTimeStamp < elem2->mTimeStamp; // 从小到大
    //             }
    //         };
    //         sort(vpEdgeBackKeyframes.begin(), vpEdgeBackKeyframes.end(), compFunctor());
    //     }

    //     double edgeBackDeltaTime = 0;
    //     if (vpEdgeBackKeyframes.size() > 2) {
    //         edgeBackDeltaTime = vpEdgeBackKeyframes[vpEdgeBackKeyframes.size() - 1]->mTimeStamp - vpEdgeBackKeyframes[0]->mTimeStamp;
    //     }

    //     if ((pEdgeBackMap->KeyFramesInMap() > nNewTrackFirst && edgeBackDeltaTime > nNewTrackFirstMinTime) || (curTrackingState == Tracking::LOST && pEdgeBackMap->KeyFramesInMap() >= 10)) {
    //         // @note print
    //         cout << "Cloud Image Sampler: Pub Cloud Images !" << endl;

    //         // concat images
    //         std::vector<std::pair<cv::Mat, double>> vTrackLastNKeyFrameImages;
    //         std::vector<std::pair<cv::Mat, double>> vNewTrackFirstNKeyFrameImages;

    //         // Track Edge Front
    //         std::vector<KeyFrame *> vpEdgeFrontKeyframes = pEdgeFrontMap->GetAllKeyFrames();
    //         {
    //             struct compFunctor {
    //                 inline bool operator()(KeyFrame *elem1, KeyFrame *elem2) {
    //                     return elem1->mTimeStamp > elem2->mTimeStamp; // 从大到小
    //                 }
    //             };
    //             sort(vpEdgeFrontKeyframes.begin(), vpEdgeFrontKeyframes.end(), compFunctor());
    //         }
    //         double lastTime = vpEdgeFrontKeyframes[0]->mTimeStamp;
    //         for (auto &pKF : vpEdgeFrontKeyframes) {
    //             vTrackLastNKeyFrameImages.push_back(std::pair<cv::Mat, double>(pKF->imgGray.clone(), pKF->mTimeStamp));
    //             if (((lastTime - pKF->mTimeStamp) > nTrackLastMinTime) && (vTrackLastNKeyFrameImages.size() > nTrackLast)) {
    //                 // 如果Delta Time和KF Num都满足了才直接结束，否则取所有
    //                 break;
    //             }
    //         }
    //         mvCurrentCloudProcessImages.insert(mvCurrentCloudProcessImages.end(), vTrackLastNKeyFrameImages.begin(), vTrackLastNKeyFrameImages.end());

    //         // Lost
    //         mvCurrentCloudProcessImages.insert(mvCurrentCloudProcessImages.end(), mvLostImages.begin(), mvLostImages.end());

    //         // Track Edge Back
    //         for (auto &pKF : vpEdgeBackKeyframes)
    //             vNewTrackFirstNKeyFrameImages.push_back(std::pair<cv::Mat, double>(pKF->imgGray.clone(), pKF->mTimeStamp));
    //         mvCurrentCloudProcessImages.insert(mvCurrentCloudProcessImages.end(), vNewTrackFirstNKeyFrameImages.begin(), vNewTrackFirstNKeyFrameImages.end());

    //         // sort by timestamp
    //         {
    //             struct compFunctor {
    //                 inline bool operator()(std::pair<cv::Mat, double> elem1, std::pair<cv::Mat, double> elem2) {
    //                     return elem1.second < elem2.second; // 从大到小
    //                 }
    //             };
    //             sort(mvCurrentCloudProcessImages.begin(), mvCurrentCloudProcessImages.end(), compFunctor());
    //         }
    //         for (auto &iter : mvCurrentCloudProcessImages) {
    //             // cerr.precision(16);
    //             // cerr << iter.second << endl;
    //             assert(!iter.first.empty());
    //         }

    //         // pub images
    //         mpSystem->PublishCloudImages(mvCurrentCloudProcessImages, mEdgeFrontMapId, mEdgeBackMapId);

    //         mvLastCloudProcessImages = mvCurrentCloudProcessImages;
    //         Reset();
    //     }
    // }

    // mTrackingLastState = curTrackingState;

    // // @note print
    // cout << "Cloud Image Sampler Tracker Cur State: " << curTrackingState << endl;
    // cout << "Cloud Image Sampler Sampler State: " << bSelectKF << endl;
    // cout << "Cloud Image Sampler State: " << mState << endl;
}

} // namespace ORB_SLAM3