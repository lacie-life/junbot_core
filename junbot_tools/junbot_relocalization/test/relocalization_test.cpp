#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sstream>
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "relocalization_test");

    ros::NodeHandle n;

    // ROS_INFO("Input -> x: [%f], y: [%f], angle: [%f]", atof(argv[1]), atof(argv[2]), atof(argv[3]));

    // TODO: Check topic
    ros::Publisher m_initialposePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "initialpose", 10);

    int count = 0;
    ros::Rate rate(1.0);

    while (n.ok())
    {
        double x, y, angle;

        std::cin >> x >> y >> angle;

        geometry_msgs::PoseWithCovarianceStamped re_init_pose;

        re_init_pose.header.frame_id = "map";
        re_init_pose.header.stamp = ros::Time::now();
        re_init_pose.pose.pose.position.x = x;
        re_init_pose.pose.pose.position.y = y;
        re_init_pose.pose.pose.position.z = 0;
        re_init_pose.pose.pose.orientation =
            tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, angle);

        // ROS_INFO(re_init_pose);

        m_initialposePub.publish(re_init_pose);

        ros::spinOnce();
        rate.sleep();

        count++;
    }

    return 0;
}
