#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>


move_base_msgs::MoveBaseActionGoal tempGoal;
ros::Publisher pub;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg)
{
  tempGoal.goal_id = msg->goal_id;
  tempGoal.goal = msg->goal;
//  ROS_INFO("%f", tempGoal.goal.target_pose.pose.position.x);
}

int main(int argc, char **argv)
{
    actionlib_msgs::GoalID tempCancel;
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/move_base/goal", 1000, chatterCallback);
    ros::Publisher  cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        int c = getchar();   // call your non-blocking input function
        if (c == 'a')
        {
            tempCancel.stamp = {};
            tempCancel.id = {};
            cancel.publish(tempCancel);
        }
        else if (c == 'b')
        {
            pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
            pub.publish(tempGoal);
        }
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
}