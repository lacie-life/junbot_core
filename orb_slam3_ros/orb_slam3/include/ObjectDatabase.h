//
// Created by lacie on 28/01/2023.
//

#ifndef ORB_SLAM3_ROS_OBJECTDATABASE_H
#define ORB_SLAM3_ROS_OBJECTDATABASE_H

#include "System.h"
#include "Object.h"

#include <Eigen/Core>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// target semantic information
typedef struct Cluster
{
    Eigen::Vector3f size;    // 3D frame size
    Eigen::Vector3f centroid;// point cloud center point
    float prob;              // Confidence
    std::string object_name; // object class name
    int class_id;            // Corresponding category ID
    int object_id;           // object number
    bool operator ==(const std::string &x);
} Cluster;

class ObjectDatabase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ObjectDatabase();
    ~ObjectDatabase();
    void addObject(Cluster& cluster);
    void addObject(ORB_SLAM3::Object_Map& object);
    cv::Scalar  getObjectColor(int class_id); // defined object color
    cv::Scalar  getObjectColor(string class_name); // defined object color
    float getObjectSize(int class_id);        // defined object size

    // Return the object data with the same name in the database
    std::vector<Cluster>  getObjectByName(std::string objectName);
    std::vector<ORB_SLAM3::Object_Map> getObjectMapByName(std::string objectName);

    // Semantic point cloud target array
    std::vector<Cluster> mClusters;

    std::vector<ORB_SLAM3::Object_Map> mObjects;
protected:
    // the color of each object
    std::vector<cv::Scalar> mvColors;

    // the size of each object
    std::vector<float>      mvSizes;
    std::vector<int> mvInterestNames;

    int DataBaseSize;
};

#endif //ORB_SLAM3_ROS_OBJECTDATABASE_HmpDetector
