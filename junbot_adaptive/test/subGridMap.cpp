#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>


nav_msgs::OccupancyGrid Map_;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void mapCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    Map_.info.width = msg->info.width;
    Map_.info.height = msg->info.height;
    Map_.data = msg->data;
    std::cout << Map_.info.width << "   :";
    std::cout << Map_.info.height << "   :";
    std::cout << Map_.data.size() << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/move_base/global_costmap/costmap", 10000, mapCallback);
    ros::spin();
    return 0;
}