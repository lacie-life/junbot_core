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

#include "Atlas.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Settings.h"
#include "Converter.h"

#include "YoloDetection.h"
#include "Detector.h"
#include "Merge2d3d.h"
#include "MergeSG.h"

#include <pangolin/pangolin.h>

#include <mutex>

#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class Merge2d3d;
class MergeSG;

namespace ORB_SLAM3
{

class Settings;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings);

    ~MapDrawer();

    void newParameterLoader(Settings* settings);

    Atlas* mpAtlas;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const Sophus::SE3f &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);

    bool GetCurrentCameraPos(cv::Mat &Rcw, cv::Mat  &Ow);
    void DrawGrid();
    void DrawObs(void);
    void DrawOctoMap();
    void DrawObject();

    void SaveOctoMap(const char*filename);
    void RegisterObs(pcl::PointCloud<pcl::PointXYZRGB> mvObs);

    MergeSG* mpMerge2d3d;

protected:

    void GeneratePointCloud(KeyFrame* kf,
                            pcl::PointCloud<pcl::PointXYZRGB>& ground,
                            pcl::PointCloud<pcl::PointXYZRGB>& nonground);

    void GeneratePointCloud(KeyFrame* kf,
                            pcl::PointCloud<pcl::PointXYZRGB>& ground,
                            pcl::PointCloud<pcl::PointXYZRGB>& nonground,
                            std::vector<Object>& objects);

    void InsertScan(octomap::point3d sensorOrigin,
                    pcl::PointCloud<pcl::PointXYZRGB>& ground,
                    pcl::PointCloud<pcl::PointXYZRGB>& nonground);

    bool isSpeckleNode(const octomap::OcTreeKey &nKey);

    void UpdateOctomap(vector<KeyFrame*> vKFs);

    void heightMapColor(double h, double& r, double &g, double& b);

    void LoadOctoMap();

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Sophus::SE3f mCameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};

private:
    // int last_obj_size;

    pcl::PointCloud<pcl::PointXYZRGB> observation;

    uint16_t  lastKeyframeSize =0;

    octomap::ColorOcTree *m_octree;
    octomap::KeyRay m_keyRay; // temp storage for casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    double m_maxRange;
    bool m_useHeightMap;

    double m_colorFactor;
    double m_res;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    bool bIsLocalization;


    octomap::OcTreeKey m_paddedMinKey, m_paddedMaxKey;
    inline static void updateMinKey(const octomap::OcTreeKey&in,
                                    octomap::OcTreeKey& min)
    {
        for(unsigned int i=0; i<3; i++)
            min[i] = std::min(in[i], min[i]);
    }
    inline static void updateMaxKey(const octomap::OcTreeKey&in,
                                    octomap::OcTreeKey& max)
    {
        for(unsigned int i=0; i<3; i++)
            max[i] = std::max(in[i], max[i]);
    }
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
