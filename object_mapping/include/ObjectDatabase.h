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
#include <sl/Camera.hpp>

typedef struct ObjectMap
{
    Eigen::Vector3f size;    // 3D frame size
    Eigen::Vector3f centroid;// point cloud center point
    float prob;              // Confidence
    std::string object_name; // object class name
    int class_id;            // Corresponding category ID
    int object_id;           // object number
    bool operator ==(const std::string &x);
} ObjectMap;

class ObjectDatabase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ObjectDatabase();
    ~ObjectDatabase();
    void addObject(ObjectMap& object);
    cv::Scalar  getObjectColor(int class_id); // defined object color
    cv::Scalar  getObjectColor(std::string class_name); // defined object color
    float getObjectSize(int class_id);        // defined object size

    // Return the object data with the same name in the database
    std::vector<ObjectMap>  getObjectByName(std::string objectName);

    // Get all objects
    std::vector<ObjectMap> getAllObject();

    // Semantic point cloud target array
    std::vector<ObjectMap> mObjects;

protected:
    // the color of each object
    std::vector<cv::Scalar> mvColors;

    // the size of each object
    std::vector<float>      mvSizes;
    std::vector<int> mvInterestNames;

    int DataBaseSize;
};

#endif //OBJECT_MAPPING_OBJECTDATABASE_H
