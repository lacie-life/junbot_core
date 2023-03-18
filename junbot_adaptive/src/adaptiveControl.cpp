//
// Created by lacie on 17/02/2023.
//

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

// TODO: Subscribe map infor
ros::Subscriber plannerPathSub;
ros::Subscriber map_sub;

void mapCallback(nav_msgs::OccupancyGrid::ConstPtr map);
void plannerPathCallback(nav_msgs::Path::ConstPtr path);

int main(int argc, char** argv) {

    std::cout << "Hello world \n";

    return 0;
}

