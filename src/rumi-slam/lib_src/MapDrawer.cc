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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM3 {

MapDrawer::MapDrawer(Atlas *pAtlas, const string &strSettingPath, Settings *settings) :
    mpAtlas(pAtlas) {
    if (settings) {
        newParameterLoader(settings);
    } else {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if (!is_correct) {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try {
                throw -1;
            } catch (exception &e) {
            }
        }
    }
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings) {
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if (!node.empty()) {
        mKeyFrameSize = node.real();
    } else {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if (!node.empty()) {
        mKeyFrameLineWidth = node.real();
    } else {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if (!node.empty()) {
        mGraphLineWidth = node.real();
    } else {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if (!node.empty()) {
        mPointSize = node.real();
    } else {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if (!node.empty()) {
        mCameraSize = node.real();
    } else {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if (!node.empty()) {
        mCameraLineWidth = node.real();
    } else {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void MapDrawer::SetCloudMerging(CloudMerging *pCloudMerging) {
    mpCloudMerging = pCloudMerging;
}

void MapDrawer::DrawCloudEdgeMapPoints(const bool bDrawEdgeFrontMP, const bool bDrawEdgeBackMP, const bool bDrawCloudMP, const bool bDrawSolveMatch) {
    Map *pEdgeFrontMap = mpCloudMerging->GetEdgeFrontMap();
    Map *pEdgeBackMap = mpCloudMerging->GetEdgeBackMap();
    Map *pCloudMap = mpCloudMerging->GetCloudMap();

    if (bDrawEdgeFrontMP && pEdgeFrontMap && !pEdgeFrontMap->IsBad()) {
        const vector<MapPoint *> &vpMPs = pEdgeFrontMap->GetAllMapPoints();
        // const vector<MapPoint*> &vpRefMPs = pEdgeFrontMap->GetReferenceMapPoints();
        // set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 1.0);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
    if (bDrawEdgeBackMP && pEdgeBackMap && !pEdgeBackMap->IsBad()) {
        const vector<MapPoint *> &vpMPs = pEdgeBackMap->GetAllMapPoints();
        // const vector<MapPoint*> &vpRefMPs = pEdgeFrontMap->GetReferenceMapPoints();
        // set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
    if (bDrawCloudMP && pCloudMap && !pCloudMap->IsBad()) {
        const vector<MapPoint *> &vpMPs = pCloudMap->GetAllMapPoints();
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
    // if (bDrawSolveMatch && mpCloudMerging->mdSolveSim3MatchMapPoints.size()) {
    //     glLineWidth(mGraphLineWidth);
    //     glColor4f(0.0f,1.0f,1.0f,0.6f);
    //     glBegin(GL_LINES);
    //
    //     std::map<MapPoint*, MapPoint*>::iterator iter;
    //     // for (iter = mpCloudMerging->mdSolveSim3MatchMapPoints.begin(); iter != mpCloudMerging->mdSolveSim3MatchMapPoints.end(); iter++) {
    //     //     if (!iter->first || !iter->second || iter->first->isBad() || iter->second->isBad()){
    //     //         std::cout << "bad match mappoint draw" << std::endl;
    //     //         continue;
    //     //     }
    //     //     Eigen::Vector3f Ow1 = iter->first->GetWorldPos();
    //     //     Eigen::Vector3f Ow2 = iter->second->GetWorldPos();
    //     //     glVertex3f(Ow1(0),Ow1(1),Ow1(2));
    //     //     glVertex3f(Ow2(0),Ow2(1),Ow2(2));
    //     // }
    //
    //     glEnd();
    // }
}

void MapDrawer::DrawCloudEdgeKeyFrames(const bool bDrawEdgeFront, const bool bDrawEdgeBack, const bool bDrawCloud, const bool bDrawSolveMatch) {
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    Map *pEdgeFrontMap = mpCloudMerging->GetEdgeFrontMap();
    Map *pEdgeBackMap = mpCloudMerging->GetEdgeBackMap();
    Map *pCloudMap = mpCloudMerging->GetCloudMap();

    if (bDrawEdgeFront && pEdgeFrontMap && !pEdgeFrontMap->IsBad()) {
        const vector<KeyFrame *> vpKFs = pEdgeFrontMap->GetAllKeyFrames();
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            if (!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth * 5);
                glColor3f(0.0f, 0.0f, 1.0f);
                glBegin(GL_LINES);
            } else {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                glBegin(GL_LINES);
            }

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }
    if (bDrawEdgeBack && pEdgeBackMap && !pEdgeBackMap->IsBad()) {
        const vector<KeyFrame *> vpKFs = pEdgeBackMap->GetAllKeyFrames();
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            if (!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth * 5);
                glColor3f(0.0f, 1.0f, 0.0f);
                glBegin(GL_LINES);
            } else {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.2f, 1.0f, 0.0f); // Basic color
                glBegin(GL_LINES);
            }

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }
    if (bDrawCloud && pCloudMap && !pCloudMap->IsBad()) {
        const vector<KeyFrame *> vpKFs = pCloudMap->GetAllKeyFrames();
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            if (!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth * 5);
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
            } else {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                glColor3f(1.0f, 0.0f, 0.0f); // Basic color
                glBegin(GL_LINES);
            }

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }
    if (bDrawSolveMatch && mpCloudMerging->mdSolveSim3MatchKeyFrames.size()) {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);

        std::map<KeyFrame *, KeyFrame *>::iterator iter;
        for (iter = mpCloudMerging->mdSolveSim3MatchKeyFrames.begin(); iter != mpCloudMerging->mdSolveSim3MatchKeyFrames.end(); iter++) {
            Eigen::Vector3f Ow1 = iter->first->GetCameraCenter();
            Eigen::Vector3f Ow2 = iter->second->GetCameraCenter();
            glVertex3f(Ow1(0), Ow1(1), Ow1(2));
            glVertex3f(Ow2(0), Ow2(1), Ow2(2));
        }

        glEnd();
    }
}

void MapDrawer::DebugDrawMapPointsField1() {
    if (dvpMPs1.size() != 0) {
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);

        for (size_t i = 0, iend = dvpMPs1.size(); i < iend; i++) {
            if (dvpMPs1[i]->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = dvpMPs1[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
}

void MapDrawer::DebugDrawMapPointsField2() {
    if (dvpMPs2.size() != 0) {
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);

        for (size_t i = 0, iend = dvpMPs2.size(); i < iend; i++) {
            if (dvpMPs2[i]->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = dvpMPs2[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
}

void MapDrawer::DebugDrawMapPointsField3() {
    if (dvpMPs3.size() != 0) {
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 1.0);

        for (size_t i = 0, iend = dvpMPs3.size(); i < iend; i++) {
            if (dvpMPs3[i]->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = dvpMPs3[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
}

void MapDrawer::DebugDrawPointsField1() {
    if (dvpPoints1.size() != 0) {
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);

        for (size_t i = 0, iend = dvpPoints1.size(); i < iend; i++) {
            Eigen::Matrix<float, 3, 1> pos = dvpPoints1[i].cast<float>();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
}

void MapDrawer::DebugDrawPointsField2() {
    if (dvpPoints2.size() != 0) {
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);

        for (size_t i = 0, iend = dvpPoints2.size(); i < iend; i++) {
            Eigen::Matrix<float, 3, 1> pos = dvpPoints2[i].cast<float>();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
}

void MapDrawer::DebugDrawPointsField3() {
    if (dvpPoints3.size() != 0) {
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 1.0);

        for (size_t i = 0, iend = dvpPoints3.size(); i < iend; i++) {
            Eigen::Matrix<float, 3, 1> pos = dvpPoints3[i].cast<float>();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();
    }
}

void MapDrawer::DebugDrawKeyFramesField1() {
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;
    if (dvpKFs1.size() != 0) {
        for (size_t i = 0; i < dvpKFs1.size(); i++) {
            KeyFrame *pKF = dvpKFs1[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            //cout << "Child KF: " << vpKFs[i]->mnId << endl;
            glLineWidth(mKeyFrameLineWidth);
            glColor3f(1.0f, 0.0f, 0.0f); // Basic color
            glBegin(GL_LINES);

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }
}

void MapDrawer::DebugDrawKeyFramesField2() {
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;
    if (dvpKFs2.size() != 0) {
        for (size_t i = 0; i < dvpKFs2.size(); i++) {
            KeyFrame *pKF = dvpKFs2[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            //cout << "Child KF: " << vpKFs[i]->mnId << endl;
            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f, 1.0f, 0.0f); // Basic color
            glBegin(GL_LINES);

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }
}

void MapDrawer::DebugDrawKeyFramesField3() {
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;
    if (dvpKFs3.size() != 0) {
        for (size_t i = 0; i < dvpKFs3.size(); i++) {
            KeyFrame *pKF = dvpKFs3[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            glLineWidth(mKeyFrameLineWidth * 5);
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }
}

void MapDrawer::DrawMapPoints() {
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    if (!pActiveMap)
        return;

    const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0), pos(1), pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
        if ((*sit)->isBad())
            continue;
        Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0), pos(1), pos(2));
    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba) {
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    Map *pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if (!pActiveMap)
        return;

    const vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();

    if (bDrawKF) {
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            if (!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth * 5);
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
            } else {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if (sOptKFs.find(pKF->mnId) != sOptKFs.end()) {
                        glColor3f(0.0f, 1.0f, 0.0f); // Green -> Opt KFs
                    } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
                        glColor3f(1.0f, 0.0f, 0.0f); // Red -> Fixed KFs
                    } else {
                        glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                    }
                } else {
                    glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if (bDrawGraph) {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for (size_t i = 0; i < vpKFs.size(); i++) {
            // Covisibility Graph
            const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if (!vCovKFs.empty()) {
                for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++) {
                    if ((*vit)->mnId < vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Ow2(0), Ow2(1), Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame *pParent = vpKFs[i]->GetParent();
            if (pParent) {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }

            // Loops
            set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                if ((*sit)->mnId < vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owl(0), Owl(1), Owl(2));
            }
        }

        glEnd();
    }

    if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKFi = vpKFs[i];
            Eigen::Vector3f Ow = pKFi->GetCameraCenter();
            KeyFrame *pNext = pKFi->mNextKF;
            if (pNext) {
                Eigen::Vector3f Owp = pNext->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }
        }

        glEnd();
    }

    vector<Map *> vpMaps = mpAtlas->GetAllMaps();

    if (bDrawKF) {
        for (Map *pMap : vpMaps) {
            if (pMap == pActiveMap)
                continue;

            vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat *)Twc.data());

                if (!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth * 5);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                } else {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0], mfFrameColors[index_color][1], mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
    const float &w = mCameraSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw) {
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw) {
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (int i = 0; i < 4; i++) {
        M.m[4 * i] = Twc(0, i);
        M.m[4 * i + 1] = Twc(1, i);
        M.m[4 * i + 2] = Twc(2, i);
        M.m[4 * i + 3] = Twc(3, i);
    }

    MOw.SetIdentity();
    MOw.m[12] = Twc(0, 3);
    MOw.m[13] = Twc(1, 3);
    MOw.m[14] = Twc(2, 3);
}

} // namespace ORB_SLAM3
