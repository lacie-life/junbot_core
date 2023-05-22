//
// Created by lacie on 27/03/2023.
//

#include "ObjectDatabase.h"

#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/Marker.h>

#include <random>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>

bool ObjectMap::operator ==(const std::string &x){
    return(this->object_name == x);
}

ObjectDatabase::ObjectDatabase(bool rViz_view)
{
    DataBaseSize = 0;
    mObjects.clear();
    rviz_visual = rViz_view;

    mvInterestNames = {-1, 0, 62, 58, 41, 77,
                       66, 75, 64, 45, 56, 60};

    // Different colors correspond to different objects
    for (int i = 0; i < mvInterestNames.size(); i++) // background with
    {
        mvColors.push_back(cv::Scalar( i*10 + 40, i*10 + 40, i*10 + 40));
    }
    mvColors[1] = cv::Scalar(255,0,255);

    // object size
    for (int i = 0; i < mvInterestNames.size(); i++)
    {
        // voc dataset 20 types of objects
        mvSizes.push_back(1.0);
    }
    mvSizes[5] = 1.0;   // Bottles within 0.06 meters are considered to be the same object
    mvSizes[10] = 1.0;    // Chair
    mvSizes[2] = 1.0;  // monitor

    // ROS Publisher
    publisher_object2map = nh.advertise<vision_msgs::Detection3DArray>("object_list", 1000);
    publisher_object = nh.advertise<visualization_msgs::Marker>("objectmap", 1000);
    publisher_cameraPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
}

ObjectDatabase::~ObjectDatabase()
{

}

std::vector<ObjectMap> ObjectDatabase::getObjectByName(std::string objectName)
{

    std::vector<ObjectMap>::iterator iter   = mObjects.begin()-1;
    std::vector<ObjectMap>::iterator it_end = mObjects.end();
    std::vector<ObjectMap> sameName;
    while(true)
    {
        iter = find(++iter, it_end, objectName);
        if (iter != it_end )
            sameName.push_back(*iter);
        else
            break;
    }
    return sameName;
}

std::vector<ObjectMap> ObjectDatabase::getAllObject()
{
    return std::vector<ObjectMap>(mObjects.begin(), mObjects.end());
}

// Create object map from Zed tracked objects
// TODO: Fixing here
void ObjectDatabase::addObject(sl::ObjectData &object)
{
    // 1. Convert sl::ObjectData to ObjectMap
    ObjectMap temp;

    // Class data
    temp.zed_unique_object_id = object.unique_object_id;
    temp.class_id = object.raw_label;
    temp.prob = object.confidence;
    temp.object_name = class_names[object.raw_label];

    // Position data
    temp.centroid = Eigen::Vector3f(object.position.x, object.position.y, object.position.z);
    temp.bounding_box = object.bounding_box;
    temp.size = object.dimensions;

    // 2. Check the total quantity, if the database is empty, join directly
    if(!mObjects.size())
    {
        DataBaseSize++;
        temp.object_id = DataBaseSize;
        mObjects.push_back(temp);

        std::cout << "[addObject] Case 1 \n";

        // Update to object map layer
        updateROSMap();

        return;
    }

    // 3. The object already exists in the database
    // find out whether the new object already exists in the database
    std::vector<ObjectMap>::iterator iter   = mObjects.begin()-1;
    std::vector<ObjectMap>::iterator it_end = mObjects.end();
    std::vector<std::vector<ObjectMap>::iterator> likely_obj;// iterator over objects with the same name
    while(true)
    {
        iter = find(++iter, it_end, temp.object_name);// search by name
        if (iter != it_end )// find one, store it
            likely_obj.push_back(iter);
        else // can't find it
            break;
    }

    std::vector<ObjectMap>::iterator best_close;// most recent index
    float center_distance = 100;// Corresponding distance

    // 4. If not found, add it directly to the database
    if(!likely_obj.size())
    {
        DataBaseSize++;
        temp.object_id = DataBaseSize;
        mObjects.push_back(temp);

        std::cout << "[addObject] Case 2 \n";

        // Update to object map layer
        updateROSMap();

        return;
    }
    else // Find multiple objects with the same name in the database
    {
        std::cout << "[addObject] Case 3 \n";

        std::cout << "Likely objects: " << likely_obj.size() << std::endl;

        // TODO: Add check moving object
        // 5. Go through each object with the same name and find the one closest to the center point
        for(unsigned int j = 0; j < likely_obj.size(); j++)
        {
            std::vector<ObjectMap>::iterator& temp_iter = likely_obj[j];
            ObjectMap & temp_cluster = *temp_iter;

            Eigen::Vector3f dis_vec = temp.centroid - temp_cluster.centroid;// center point connection vector
            float dist = dis_vec.norm();
            if(dist < center_distance)
            {
                center_distance = dist; // shortest distance
                best_close      = temp_iter;// corresponding index
            }
        }
        // 6. If the distance is smaller than the object size,
        // it is considered to be the same object in the same space,
        // and the information of the object in the database is updated
        for (int i = 0; i < mvInterestNames.size(); i++)
        {
            std::cout << "Object name: " << class_names[mvInterestNames.at(i)] << " " << center_distance << std::endl;

            if(temp.object_name == std::to_string(mvInterestNames[i]))
            {
                if(center_distance < mvSizes[i])
                {
                    best_close->prob    = (best_close->prob + temp.prob)/2.0; // Comprehensive confidence
                    best_close->centroid = (best_close->centroid + temp.centroid)/2.0; // center mean
                    best_close->size     = (best_close->size + temp.size)/2.0; // center size
                }
                else
                {
                    // 7. If the distance exceeds the size of the object,
                    // it is considered to be the same object in a different position,
                    // and it is directly put into the database
                    DataBaseSize++;
                    temp.object_id = DataBaseSize;
                    mObjects.push_back(temp);

                    std::cout << "[addObject] Case 3.1 \n";
                }
            }
        }
    }

    // Update to object map layer
    updateROSMap();

    return;
}

void ObjectDatabase::updateObjectDatabase(sl::Objects &objects, sl::Pose &cam_w_pose)
{
    currPose = cam_w_pose;

    for(int i = 0; i < objects.object_list.size(); i++)
    {
        sl::ObjectData obj;
        objects.getObjectDataFromId(obj, i);
        addObject(obj);
    }
}

geometry_msgs::Point ObjectDatabase::corner_to_marker(const sl::float3& v){
    geometry_msgs::Point point;
    point.x = v.x;
    point.y = v.y;
    point.z = v.z;

//    sl::Matrix4f T = currPose.pose_data;
//    sl::Matrix3f R = currPose.getRotationMatrix();
//
//    auto cam_to_world = currPose;
//
//    cam_to_world.pose_data.inverse();
//
//    // TODO: Coding here
//    sl::Matrix3f temp;
//    temp(0, 0) = v.x;
//    temp(0, 1) = v.x;
//    temp(0, 2) = v.x;
//    sl::Matrix3f p_world = R * temp;
//    geometry_msgs::Point p;
//    p.x= p_world(0, 0) + T(0, 3);
//    p.y= p_world(0, 1) + T(1, 3);
//    p.z= p_world(0, 2) + T(2, 3);

    return point;
}

void ObjectDatabase::updaterVizView()
{
    // Update object pose
    for(size_t i=0; i< mObjects.size(); i++)
    {
        // Generate color
        std::vector<std::vector<float> > colors_bgr{
            {135,0,248},
            {255,0,253},
            {4,254,119},
            {255,126,1},
            {0,112,255},
            {0,250,250}   };

        std::vector<float> color = colors_bgr[mObjects.at(i).class_id % 6];

        // The color used for the object is random
        std::default_random_engine e;
        std::uniform_real_distribution<double>  random(0.5,1);
        float r = random(e); float g = random(e); float b = random(e);

        // object
        visualization_msgs::Marker marker;
        marker.id = mObjects.at(i).object_id;
        marker.lifetime = ros::Duration(0.1);
        marker.header.frame_id= "map";
        marker.header.stamp=ros::Time::now();

        marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = color[2]/255.0;
        marker.color.g = color[1]/255.0;
        marker.color.b = color[0]/255.0;
        marker.color.a = 1.0;
        marker.scale.x = 0.01;

        std::cout << "Object " << i <<  " have " << mObjects.at(i).bounding_box.size() << " points" << std::endl;
        std::cout << "Class: " << mObjects.at(i).object_name << std::endl;

        if(mObjects.at(i).bounding_box.size() != 8)
        {
            std::cout << "Skipping ... \n";
            continue;
        }

        // ZED 3D bounding box
        //     1------2
        //    /|     /|
        //   / |    / |
        //  0------3  |
        //  |  5---|--6
        //  | /    | /
        //  4------7

        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(4)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(7)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(7)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(6)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(6)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(5)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(5)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(4)));

        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(4)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(0)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(7)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(3)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(6)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(2)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(5)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(1)));

        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(0)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(1)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(1)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(2)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(2)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(3)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(3)));
        marker.points.push_back(corner_to_marker(mObjects.at(i).bounding_box.at(0)));

        publisher_object.publish(marker);
    }
}

void ObjectDatabase::updateROSMap()
{
    if(mObjects.size() <= 0)
    {
        return;
    }

    std::cout << "Number object in DB: " << mObjects.size() << "\n";

    vision_msgs::Detection3DArray objDB;

    for(size_t i = 0; i < mObjects.size(); i++)
    {
        vision_msgs::Detection3D obj;
        ObjectMap temp = mObjects.at(i);

        obj.bbox.center.position.x = temp.centroid.x();
        obj.bbox.center.position.y = temp.centroid.y();
        obj.bbox.center.position.z = temp.centroid.z();
        obj.bbox.center.orientation.x = 0;
        obj.bbox.center.orientation.y = 0;
        obj.bbox.center.orientation.z = 0;
        obj.bbox.center.orientation.w = 1;
        obj.bbox.size.x = temp.size.x;
        obj.bbox.size.y = temp.size.y;
        obj.bbox.size.z = temp.size.z;

        vision_msgs::ObjectHypothesisWithPose hypo;
        hypo.id = temp.class_id;
        hypo.score = 1;

        obj.results.push_back(hypo);

        objDB.detections.push_back(obj);
    }
    objDB.header = std_msgs::Header();

    publisher_object2map.publish(objDB);

    if(rviz_visual)
    {
        updaterVizView();
    }
}
