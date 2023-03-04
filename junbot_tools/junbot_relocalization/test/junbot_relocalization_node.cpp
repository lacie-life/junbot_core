#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sstream>

nav_msgs::Odometry odom;
nav_msgs::Odometry last_odom;
double current_RPY[3];
double current_Yaw;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    odom = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "amcl_relocalization");

    ros::NodeHandle n;

    // TODO: Change topic with t265 config
    ros::Subscriber m_odomSub = n.subscribe("/camera/odom/sample", 1, odomCallback);

    // TODO: Check topic
    ros::Publisher m_initialposePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "initialpose", 10);

    ros::Rate loop_rate(50);

    double roll, pitch, yaw;

    while (ros::ok())
    {
        // ROS_INFO("Seq: [%d]", odom.header.seq);
        auto _orientation = odom.pose.pose.orientation;
        auto _position = odom.pose.pose.position;
        tf::Quaternion q(_orientation.x, _orientation.y, _orientation.z, _orientation.w);
        tf::Matrix3x3 m(q);

        m.getRPY(current_RPY[0], current_RPY[1], current_RPY[2]);

        ROS_INFO("RPY -> r: [%f], p: [%f], y: [%f]", current_RPY[0], current_RPY[1], current_RPY[2]);

        // TODO: Check distance between last_RPY and current_RPY
        float x = odom.pose.pose.position.x;
        float y = odom.pose.pose.position.y;

        float last_x = last_odom.pose.pose.position.x;
        float last_y = last_odom.pose.pose.position.y;

        float distance = sqrt((x - last_x) * (x - last_x) + (y - last_y) * (y - last_y));

        if (distance >= 10)
        {
            geometry_msgs::PoseWithCovarianceStamped re_init_pose;

            re_init_pose.header.frame_id = "map";
            re_init_pose.header.stamp = ros::Time::now();
            re_init_pose.pose.pose.position.x = odom.pose.pose.position.x;
            re_init_pose.pose.pose.position.y = odom.pose.pose.position.x;
            re_init_pose.pose.pose.position.z = 0;
            re_init_pose.pose.pose.orientation = odom.pose.pose.orientation;

            m_initialposePub.publish(re_init_pose);

            last_odom = odom;
        }

        ros::spinOnce();
    }

    return 0;
}
