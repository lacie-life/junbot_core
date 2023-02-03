/*
 *--------------------------------------------------------------------------------------------------
 * DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments
　*　Author(s):
 * Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei Qiao qiaofei@mail.tsinghua.edu.cn
 * Created by Yu Chao@2018.12.03
 * --------------------------------------------------------------------------------------------------
 * DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2. If you haven't learn ORB_SLAM2 code, 
 * you'd better to be familiar with ORB_SLAM2 project first. Compared to ORB_SLAM2, 
 * we add anther two threads including semantic segmentation thread and densemap creation thread. 
 * You should pay attention to Frame.cc, ORBmatcher.cc, Pointcloudmapping.cc and Segment.cc.
 * 
 *　@article{murORB2,
 *　title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
　*　author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
　* journal={IEEE Transactions on Robotics},
　*　volume={33},
　* number={5},
　* pages={1255--1262},
　* doi = {10.1109/TRO.2017.2705103},
　* year={2017}
 *　}
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2018, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/) and 
 * Advanced Mechanism and Roboticized Equipment Lab. All rights reserved.
 *
 * Licensed under the GPLv3 License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://github.com/ivipsourcecode/DS-SLAM/blob/master/LICENSE
 *--------------------------------------------------------------------------------------------------
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include "Converter.h"
#include "ObjectDatabase.h"
#include "MergeSG.h"
#include "YoloDetection.h"

#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>

#include <condition_variable>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace ORB_SLAM3;

class PointCloudMapping
{

public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    /**
     * @brief Construct a new Point Cloud Mapping object
     * 
     * @param resolution_ 
     */
    PointCloudMapping(double resolution_, double octoResolution_, std::string modelPath);

    /**
     * @brief Insert new keyframe and RGB-D image, and trigger pointcloud generation process
     * 
     * @param kf keyframe 
     * @param color rgb image 
     * @param depth depth image
     */
    void insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);

    void generateOctoMap();
    void genrerateObjectBondingBox();

    void SaveOctoMap(const char* filename);
    void LoadOctoMap(const char* filename);
    void RegisterObs(pcl::PointCloud<pcl::PointXYZRGB> mvObs);

    /**
     * @brief Shuts down all threads properly
     */
    void shutdown();

    MergeSG* mpMergeSG;
    YoloDetection* mDetector;

protected:
    PointCloud::Ptr globalMap;

    boost::shared_ptr<thread> viewerThread;
    boost::shared_ptr<thread> pclThread;

    bool shutDownFlag = false;
    std::mutex shutDownMutex;
    std::mutex keyFrameUpdateMutex;
    std::condition_variable keyFrameUpdated;
    std::condition_variable newMaskArrived;

    // Data to generate point clouds
    std::map<int, cv::Mat> maskMap;
    std::vector<KeyFrame *> keyframes;
    std::vector<cv::Mat> colorImgs;
    std::vector<cv::Mat> depthImgs;
    std::vector<int> imgMasksSeq;
    std::mutex keyframeMutex;
    std::mutex maskMutex;
    uint16_t lastKeyframeSize = 0;

    double resolution = 0.005;
    pcl::VoxelGrid<PointT> voxel;
    pcl::StatisticalOutlierRemoval<PointT> sor;

protected:

    // OctoMap
    // int last_obj_size;

    PointCloud::Ptr generatePointCloud(KeyFrame *kf, cv::Mat color, cv::Mat depth);
    PointCloud::Ptr generatePointCloud(KeyFrame *kf, cv::Mat color, cv::Mat depth, std::vector<Object>& objects);
    PointCloud::Ptr generatePointCloudWithDynamicObject(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

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

    pcl::PointCloud<pcl::PointXYZRGB> observation;

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

    /**
     * @brief Runs all functionalities in a new thread
     */
    void publisher();

    /**
     * @brief Generates and publishes the pointcloud in a separate thread
     * 
     * @param N Total number of all (keyframes, RGB-D images) inserted
     */
    void generateAndPublishPointCloud(size_t N);

    /**
     * @brief Calculates transformation matrix based on camera pose
     * and publishes it as a ROS tf message to transfrom the pointcloud to world coordinates
     * 
     * @param cameraPose camera pose generated by ORB-SLAM3
     */
    void broadcastTransformMat(Eigen::Isometry3d cameraPose);

    bool checkDynamicPoint(cv::Point2f pt, std::vector<Object> objects);
};

#endif // POINTCLOUDMAPPING_H
