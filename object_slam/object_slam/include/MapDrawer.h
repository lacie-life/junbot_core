

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

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

#include "Atlas.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Settings.h"
#include "Converter.h"

#include "Detector.h"
#include "MergeSG.h"

#include <pangolin/pangolin.h>

using namespace pcl;

class MergeSG;

namespace semantic_slam
{

class Settings;
class Atlas;

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

// For 3D cuboid testing
public:
    void DrawMapCuboids();
    void DrawMapCuboids2();

private:
    std::vector<Eigen::Vector3f> box_colors;
    Eigen::MatrixXd all_edge_pt_ids; // for object drawing
    Eigen::MatrixXd front_edge_pt_ids;

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

    // Pending
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

} //namespace semantic_slam

#endif // MAPDRAWER_H
