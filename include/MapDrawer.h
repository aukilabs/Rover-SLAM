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


#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Atlas.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include "Settings.h"
#include<Eigen/Core>
#include<pangolin/pangolin.h>

#include<array>
#include<mutex>
#include<vector>

namespace ORB_SLAM3
{

class Settings;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings);

    struct PortalSquare
    {
        std::string id;
        std::array<std::array<float, 3>, 4> corners;
    };

    struct CustomPoint
    {
        std::string type;
        std::string id;
        std::array<float, 3> point;
        float diameter_m = 0.1f;
    };

    struct MeshTriangle
    {
        std::array<std::array<float, 3>, 3> vertices;
    };

    void newParameterLoader(Settings* settings);

    Atlas* mpAtlas;

    void DrawMapPoints();
    void DrawPortalSquares();
    void DrawCustomPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const Sophus::SE3f &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);
    void SetViewerAlignment(const Eigen::Matrix3f &R_domain_slam, const Eigen::Vector3f &t_domain_slam, const float scale);
    void ClearViewerAlignment();
    void SetViewerPortalSquares(const std::vector<PortalSquare> &squares);
    void ClearViewerPortalSquares();
    void SetViewerTruthPortalSquares(const std::vector<PortalSquare> &squares);
    void ClearViewerTruthPortalSquares();
    void SetViewerCustomPoints(const std::vector<CustomPoint> &points);
    void ClearViewerCustomPoints();
    void SetViewerTopographyMesh(const std::vector<MeshTriangle> &triangles);
    void ClearViewerTopographyMesh();
    void DrawTopographyMesh();

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    struct ViewerAlignment
    {
        bool enabled = false;
        float scale = 1.0f;
        Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
        Eigen::Vector3f t_domain_slam = Eigen::Vector3f::Zero();
    };

    ViewerAlignment GetViewerAlignment();
    Eigen::Vector3f ApplyViewerAlignment(const Eigen::Vector3f &p_slam, const ViewerAlignment &alignment);
    Sophus::SE3f ApplyViewerAlignment(const Sophus::SE3f &Twc_slam, const ViewerAlignment &alignment);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Sophus::SE3f mCameraPose;

    std::mutex mMutexCamera;
    std::mutex mMutexViewerAlignment;
    std::mutex mMutexViewerPortalSquares;
    std::mutex mMutexViewerCustomPoints;
    std::mutex mMutexViewerTopographyMesh;
    ViewerAlignment mViewerAlignment;
    std::vector<PortalSquare> mViewerPortalSquares;
    std::vector<PortalSquare> mViewerTruthPortalSquares;
    std::vector<CustomPoint> mViewerCustomPoints;
    std::vector<MeshTriangle> mViewerTopographyMesh;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};

};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
