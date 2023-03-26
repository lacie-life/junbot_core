//
// Created by lacie on 12/02/2023.
//

#ifndef SEMANTIC_SLAM_ROS_MAPPUBLISHER_H
#define SEMANTIC_SLAM_ROS_MAPPUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>

#include "Atlas.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Object.h"
#include "Converter.h"
#include <random>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/eigen.hpp>

namespace semantic_slam
{
    class Atlas;
    class Object_Map;

    // TODO: Publish ObjectDatabase to ROS
    class MapPublisher {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        MapPublisher(Atlas* mAtlas, const string &strSettingPath);

        Atlas* mpAtlas;

        void Refresh();
        void PublishMapPoints(const std::vector<MapPoint*> &vpMPs, const std::vector<MapPoint*> &vpRefMPs);
        void PublishKeyFrames(const std::vector<KeyFrame*> &vpKFs);
        void PublishCurrentCamera(const cv::Mat &Tcw);
        void PublishObject(const vector<Object_Map*> &vpObjs );
        void PublishObject2Map(const std::vector<Object_Map*> &vpObjs);
        void PublishIE(const vector<Object_Map*> &vObjs );
        geometry_msgs::Point corner_to_marker(Eigen::Vector3d& v);
        geometry_msgs::Point corner_to_marker(const std::vector<float>& v);
        geometry_msgs::Point corner_to_marker(const std::vector<double>& v);
        void SetCurrentCameraPose(const cv::Mat &Tcw);

    private:

        cv::Mat GetCurrentCameraPose();
        bool isCamUpdated();
        void ResetCamFlag();

        ros::NodeHandle nh;
        ros::Publisher publisher;
        ros::Publisher publisher_curframe;
        ros::Publisher publisher_KF;
        ros::Publisher publisher_CoView;
        ros::Publisher publisher_object;
        ros::Publisher publisher_object2map;
        ros::Publisher publisher_object_points;
        ros::Publisher publisher_IE;
        ros::Publisher publisher_robotpose;

        //tf tree
        tf::TransformBroadcaster odom_broadcaster;
        tf::TransformBroadcaster camera_broadcaster;

        visualization_msgs::Marker mPoints;
        visualization_msgs::Marker mReferencePoints;
        visualization_msgs::Marker mKeyFrames;
        visualization_msgs::Marker mReferenceKeyFrames;
        visualization_msgs::Marker mCovisibilityGraph;
        visualization_msgs::Marker mMST;
        visualization_msgs::Marker mCurrentCamera;

        int object_id_init;
        int IE_id;
        float fCameraSize;
        float fPointSize;

        cv::Mat mCameraPose;// = cv::Mat::eye(4,1,CV_32F);
        bool mbCameraUpdated;

        std::mutex mMutexCamera;

        const char* MAP_FRAME_ID = "map"; //  odom   imu_link   /ORB_SLAM/World    map
        const char* POINTS_NAMESPACE = "MapPoints";
        const char* KEYFRAMES_NAMESPACE = "KeyFrames";
        const char* PLANES_NAMESPACE = "MapPlanes";
        const char* OBJECTS_NAMESPACE = "MapObjects";
        const char* GRAPH_NAMESPACE = "Graph";
        const char* CAMERA_NAMESPACE = "Camera";
    public:
        cv::Mat mT_body_cam;
        geometry_msgs::Quaternion mQuaternion_robot_camera;
        geometry_msgs::Vector3_<float> mTranslation_robot_camera;
    };
}

#endif //SEMANTIC_SLAM_ROS_MAPPUBLISHER_H
