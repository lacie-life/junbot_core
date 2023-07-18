//
// Created by lacie on 17/07/2023.
//

#ifndef JUNBOT_ADAPTIVE_SEMANTICPLANNER_H
#define JUNBOT_ADAPTIVE_SEMANTICPLANNER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <custom_msgs/Obstacles.h>
#include <custom_msgs/Form.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>

class SemanticPlanner {

public:
    SemanticPlanner(ros::NodeHandle &nh);
    ~SemanticPlanner();

    void calculateZone();

    void run();

private:
    void ObstacleCallback(const vision_msgs::Detection3DArray::ConstPtr& msg);

    void gridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg);

    inline double calculateDistance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
    }

private:
    ros::Publisher obstaclePub;

    ros::Subscriber pos_sub;
    ros::Subscriber obstacleSub;
    ros::Subscriber grid_map_sub;
    ros::Subscriber plan_sub;

    tf::TransformListener *robotPoselistener;
    nav_msgs::OccupancyGrid grid_map;
    nav_msgs::Path global_plan;
    custom_msgs::Obstacles m_obstacles;

    std::mutex mutex;
};

#endif //JUNBOT_ADAPTIVE_SEMANTICPLANNER_H
