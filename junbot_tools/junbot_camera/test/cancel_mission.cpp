#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>


move_base_msgs::MoveBaseActionGoal tempGoal;
actionlib_msgs::GoalID tempCancel;
ros::Publisher pub;
std::string msg_cancel = "0";
ros::Publisher  cancel;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg)
{
  tempGoal.goal_id = msg->goal_id;
  tempGoal.goal = msg->goal;
//  ROS_INFO("%f", tempGoal.goal.target_pose.pose.position.x);
}
void cancelCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string tmp = msg->data.c_str();

  if (tmp == "0")
  {
    if (msg_cancel == "1"){
      tempCancel.stamp = {};
      tempCancel.id = {};
      cancel.publish(tempCancel);
      msg_cancel = "0";
    }
  }
  else if (tmp == "1")
  {
    if (msg_cancel == "0")
    {
      if(tempGoal.goal.target_pose.pose.position.x != FLT_MAX && tempGoal.goal.target_pose.pose.position.y != FLT_MAX){
        pub.publish(tempGoal);
      }
      
      msg_cancel = "1";
    }
  }
  msg_cancel = tmp;
}
int main(int argc, char **argv)
{
    tempGoal.goal.target_pose.pose.position.x = FLT_MAX;
    tempGoal.goal.target_pose.pose.position.y = FLT_MAX;
    ros::init(argc, argv, "listener1");
    ros::NodeHandle n;
    cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
    ros::Subscriber sub = n.subscribe("/move_base/goal", 1000, chatterCallback);
    ros::Subscriber sub_cancel = n.subscribe("/cancel_mission", 1000, cancelCallback);
    pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
}