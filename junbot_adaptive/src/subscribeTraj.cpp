#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>


nav_msgs::Path tempTraj;
ros::Publisher pub;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void globalPlanCallback(nav_msgs::Path path)
{
    for (int i = 0; i < path.poses.size(); i++)
    {
        tempTraj.poses.push_back(path.poses[i]);
        std::cout << tempTraj.poses[i].pose.position.x << "\n";
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/move_base/DWAPlannerROS/global_plan", 10000, globalPlanCallback);
    return 0;
}