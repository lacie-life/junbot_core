//
// Created by lacie on 17/02/2023.
//
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <custom_msgs/Obstacles.h>

#include <mutex>

double calculateDistance (double Xa, double Ya, double Xb, double Yb);
// TODO: Subscribe map infor
nav_msgs::Path path;
ros::Publisher pub;
custom_msgs::Obstacles object;
std::mutex m;

void globalPlanCallback(nav_msgs::Path::ConstPtr tempPath)
{
    m.lock();
    for (int i = 0; i < tempPath->poses.size(); i++)
    {
        path.poses.push_back(tempPath->poses[i]);
    }
    m.unlock();
//    path = tempPath;
}

void objectCallback(custom_msgs::Obstacles::ConstPtr objTemp)
{
    m.lock();
    object.list = objTemp->list;

    ROS_INFO("Object Callback");
    for(int i = 0; i < 4; i++)
    {
        std::cout << object.list.at(0).form.at(i).x << "\n";
        std::cout << object.list.at(0).form.at(i).y << "\n";
        std::cout << object.list.at(0).form.at(i).z << "\n";
    }
    m.unlock();
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber subPlan = n.subscribe("/move_base/DWAPlannerROS/global_plan", 10000, globalPlanCallback);
    ros::Subscriber subObj = n.subscribe("/object_costmap_layer/obsctacles_temp", 10000, objectCallback);
    ros::Publisher pubObj = n.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obsctacles", 1000);
    float coner[3][3] = {{-1.77,-0.77,0},
                            {1.03,-0.77,0},
                            {-2,1.55}};
                            // {-6.3, 0.85, 0}
    ros::Rate rate(100);
    while (ros::ok())
    {
        m.lock();
        double distance = 99;
        custom_msgs::Obstacles objectNew = object;
        for (int i = 0; i < object.list.size(); ++i) {
            for (int j = 0; j < 4; ++j) {
                std::vector<geometry_msgs::Point> waypointError;
                custom_msgs::Form temp;
                for (int k = 0; k < path.poses.size(); ++k) {
                    distance = calculateDistance(object.list[i].form[j].x, object.list[i].form[j].y,path.poses[k].pose.position.x, path.poses[k].pose.position.y);
                    if (distance <= 0.3) {
                        geometry_msgs::Point p;
                        p.x = path.poses[k].pose.position.x;
                        p.y = path.poses[k].pose.position.y;
                        p.z = path.poses[k].pose.position.z;
                        waypointError.push_back(p);
                    }
                }
                if (waypointError.size()>1)
                {
                    geometry_msgs::Point _begin = waypointError.at(0);
                    geometry_msgs::Point _end = waypointError.at(waypointError.size() - 1);
                    temp.form.push_back(_begin);
                    temp.form.push_back(_end);
                    temp.form.push_back(object.list[i].form[j]);
                    temp.id = "zone";
                    objectNew.list.push_back(temp);
                }
            }
        }
        for (int j = 0; j < 3; ++j) {
            std::vector<geometry_msgs::Point> waypointError;
            custom_msgs::Form temp;
            for (int k = 0; k < path.poses.size(); ++k) {
                distance = calculateDistance(coner[j][0],coner[j][1],path.poses[k].pose.position.x, path.poses[k].pose.position.y);                    if (distance <= 0.3) {
                geometry_msgs::Point p;
                p.x = path.poses[k].pose.position.x;
                p.y = path.poses[k].pose.position.y;
                p.z = path.poses[k].pose.position.z;
                waypointError.push_back(p);
                }
            }
            if (waypointError.size()>1)
            {
                geometry_msgs::Point _begin = waypointError.at(0);
                geometry_msgs::Point _end = waypointError.at(waypointError.size() - 1);
                geometry_msgs::Point coner_;
                coner_.x = coner[j][0];
                coner_.y = coner[j][1];
                coner_.z = coner[j][2];
                temp.form.push_back(_begin);
                temp.form.push_back(_end);
                temp.form.push_back(coner_);
                temp.id = "zone";
                objectNew.list.push_back(temp);
            }
        }
        pubObj.publish(objectNew);
        m.unlock();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

double calculateDistance (double Xa, double Ya, double Xb, double Yb)
{
    return sqrt((Xb-Xa)*(Xb-Xa)+(Yb-Ya)*(Yb-Ya));
}
