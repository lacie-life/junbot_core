//
// Created by lacie on 28/01/2023.
//

#ifndef SEMANTIC_SLAM_ROS_MERGESG_H
#define SEMANTIC_SLAM_ROS_MERGESG_H

#include <pcl/features/impl/integral_image_normal.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/organized_multi_plane_segmentation.hpp>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>

#include <Eigen/Core>
#include <vector>

#include "ObjectDatabase.h"
#include "YoloDetection.h"

using namespace pcl;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct Cluster;
class ObjectDatabase;

// PCL newly defined point type 3d point coordinate + 2d pixel coordinate value 3d-2d point pair
struct PointXYZPixel
{
    PCL_ADD_POINT4D;
    uint32_t pixel_x; // Pixel values
    uint32_t pixel_y;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // NOLINT

// registration point type
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZPixel,                // xyz + pixel x, y as fields
                                  (float, x, x)                 // field x
                                  (float, y, y)                 // field y
                                  (float, z, z)                 // field z
                                  (uint32_t, pixel_x, pixel_x)  // field pixel x
                                  (uint32_t, pixel_y, pixel_y)  // field pixel y
)

typedef enum NewComparator
{
    kPlaneCoefficientComparator,            // Plane Coefficient RANSAC Sampling
    kEuclideanPlaneCoefficientComparator,   // Euclidean distance Plane segmentation
    kRGBPlaneCoefficientComparator,         // Color Distance Plane Segmentation
    kEdgeAwarePlaneComaprator               // Edge Plane Segmentation
} NewComparator;

// Test result class
typedef struct Object3d
{
    cv::Rect_<float> rect;                       // frame
    Eigen::Vector3f minPt;                       // Smallest x-value, y-value, z-value among all points
    Eigen::Vector3f maxPt;                       // Maximum x-value, y-value, z-value among all points
    Eigen::Vector3f centroid;                    // Center point of point cloud Homogeneous representation
    Eigen::Vector3f sizePt;                      // Length, width and height
    Eigen::Vector3f boxCenter;                   // Bounding box center point

} Object3d;

class MergeSG {

public:
    /** Default constructor */
    MergeSG();

    /** Default destructor */
    ~MergeSG();

    void merge(std::vector<Object>& objects, cv::Mat depth, PointCloudT::Ptr pclMap);

    ObjectDatabase* mpOD;

private:

    void extract(std::vector<Object>& objects,
                 PointCloudT::Ptr point_ptr,
                 std::vector<Cluster>& clusters);

    void findMaxIntersectionRelationships(std::vector<Object>& objects,
                                          std::vector<Object3d>& object3d,
                                          std::vector<Cluster>& clusters);

    bool getProjectedROI(const pcl::PointCloud<PointXYZPixel>::ConstPtr& point_cloud,
                         cv::Rect_<float> & roi);

    double getMatch(const cv::Rect_<float> & r1, const cv::Rect_<float> & r2);


    void segment(const PointCloudT::ConstPtr& cloud,
                 PointCloudT::Ptr& cloud_segment,
                 std::vector<pcl::PointIndices>& cluster_indices);

    void applyConfig();

    void estimateNormal(const PointCloudT::ConstPtr& cloud,
                        pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud);


    void segmentPlanes(
            const PointCloudT::ConstPtr& cloud,
            const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud,
            std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions,
            pcl::PointCloud<pcl::Label>::Ptr labels,
            std::vector<pcl::PointIndices>& label_indices);

    void segmentObjects(
            const PointCloudT::ConstPtr& cloud,
            std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions,
            pcl::PointCloud<pcl::Label>::Ptr labels,
            std::vector<pcl::PointIndices>& label_indices,
            std::vector<pcl::PointIndices>& cluster_indices);

    void copyPointCloud(const PointCloudT::ConstPtr& original,
                        const std::vector<int>& indices,
                        pcl::PointCloud<PointXYZPixel>::Ptr& dest);

private:

    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> normal_estimation_;

    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> plane_segmentation_;
    pcl::PlaneCoefficientComparator<PointT, pcl::Normal>::Ptr plane_comparator_;
    pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr euclidean_comparator_;
    pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr rgb_comparator_;
    pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>::Ptr edge_aware_comparator_;

    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

    NewComparator mPlanComparator;
    double   mPlanNormal_angle_threshold;//  2.0 0.01 45.0
    double   normal_distance_threshold;  //      0.02  0.001  0.1
    int mMin_plane_inliers;// 100 200 500 1000
    int mPlane_minimum_points;//  300 500 1000 2000

    double mNormal_max_depth_change;//        0.02  0.001  0.1
    double mNormal_smooth_size;     //          30.0  1.0  100.0

    double mEuclidean_distance_threshold;//  0.02 0.001 0.1
    int mObject_minimum_points;//50 100 200 500
};

#endif //SEMANTIC_SLAM_ROS_MERGESG_H
