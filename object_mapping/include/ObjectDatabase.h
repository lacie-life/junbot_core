//
// Created by lacie on 27/03/2023.
//

#ifndef OBJECT_MAPPING_OBJECTDATABASE_H
#define OBJECT_MAPPING_OBJECTDATABASE_H

#include <Eigen/Core>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sl/Camera.hpp>

typedef struct ObjectMap
{
    std::vector<sl::float3> bounding_box;   // 3D cuboid
    sl::float3 size;
    Eigen::Vector3f centroid;// object centroid
    float prob;              // Confidence
    std::string object_name; // object class name
    int class_id;            // Corresponding category ID
    int object_id;           // object number
    sl::String zed_unique_object_id;
    bool operator ==(const std::string &x);
} ObjectMap;

class ObjectDatabase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ObjectDatabase(bool rViz_view = false);
    ~ObjectDatabase();

    void updateObjectDatabase(sl::Objects& objects, sl::Pose &cam_w_pose);

    void addObject(sl::ObjectData& object);

    // Return the object data with the same name in the database
    std::vector<ObjectMap>  getObjectByName(std::string objectName);

    geometry_msgs::Point corner_to_marker(const sl::float3& v);

    // Publish to ROS
    void updateROSMap();
    void updaterVizView();

    // Get all objects
    std::vector<ObjectMap> getAllObject();

    // Objects array
    std::vector<ObjectMap> mObjects;

    bool rviz_visual = false;

protected:
    // the color of each object
    std::vector<cv::Scalar> mvColors;

    // the size of each object
    std::vector<float>      mvSizes;
    std::vector<int> mvInterestNames;

    int DataBaseSize;

    ros::NodeHandle nh;
    ros::Publisher publisher_object2map;
    ros::Publisher publisher_object;
    ros::Publisher publisher_cameraPose;
    tf::TransformBroadcaster camera_broadcaster;
    sl::Pose currPose;
};

#endif //OBJECT_MAPPING_OBJECTDATABASE_H
