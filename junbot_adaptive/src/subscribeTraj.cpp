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
void chatterCallback(nav_msgs::Path path)
{
    for (int i = 0; i < path.poses.size(); i++)
    {
        tempTraj.poses[i].pose.position.x = path.poses[i].pose.position.x;
        tempTraj.poses[i].pose.position.y = path.poses[i].pose.position.y;
        ROS_INFO("%f", tempTraj.poses[i].pose.position.x );
    }
}

int main(int argc, char **argv)
{
    actionlib_msgs::GoalID tempCancel;
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Publisher  cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        int c = getchar();   // call your non-blocking input function
        if (c == 'a')
        {
            ros::Subscriber sub = n.subscribe("/move_base/DWAPlannerROS/global_plan", 1000, chatterCallback);
            tempCancel.stamp = {};
            tempCancel.id = {};
            cancel.publish(tempCancel);
        }
        else if (c == 'b')
        {
            pub = n.advertise<nav_msgs::Path>("/move_base/DWAPlannerROS/global_plan", 1000);
            pub.publish(tempTraj);
        }
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}