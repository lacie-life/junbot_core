#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>


move_base_msgs::MoveBaseActionGoal tempGoal;
ros::Publisher pub;

int main(int argc, char **argv)
{
    int num=0;
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
            std::cin >> num;

        }
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
}