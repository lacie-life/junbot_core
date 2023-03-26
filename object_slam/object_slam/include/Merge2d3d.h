//
// Created by lacie on 28/01/2023.
//

#ifndef SEMANTIC_SLAM_ROS_MERGE2D3D_H
#define SEMANTIC_SLAM_ROS_MERGE2D3D_H

#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

#include <Eigen/Core>

#include <vector>

#include "ObjectDatabase.h"
#include "YoloDetection.h"

using namespace pcl;

struct Cluster;
class ObjectDatabase;

class Merge2d3d {
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    Merge2d3d();
    ~Merge2d3d();
    void merge(std::vector<Object>& objects, cv::Mat depth, PointCloud::Ptr pclMap);

    ObjectDatabase* mpOD;

protected:
    bool mergeOne(Object& object, Cluster& cluster, cv::Mat depth_img, PointCloud::Ptr pclMap);

    pcl::ExtractIndices<PointT> mExtractInd;
    pcl::VoxelGrid<PointT>  mVoxel;
    pcl::StatisticalOutlierRemoval<PointT> mStat;

    // ObjectDatabase mOD;
    // ObjectDatabase* mpOD;
};

#endif //SEMANTIC_SLAM_ROS_MERGE2D3D_H
