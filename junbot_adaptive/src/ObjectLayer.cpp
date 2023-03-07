//
// Created by lacie on 07/03/2023.
//

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <custom_msgs/Obstacles.h>
#include <custom_msgs/Form.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <iostream>

ros::Publisher obstaclePub;

void ObjDBCallBack(const vision_msgs::Detection3DArray::ConstPtr& msg)
{
    ROS_INFO("I received: [%s] objects", msg->detections.size());

    custom_msgs::Obstacles temp;

    for (int i = 0; i < msg->detections.size(); i++)
    {
        custom_msgs::Form obs;

        //     8------7
        //    /|     /|
        //   / |    / |
        //  5------6  |
        //  |  4---|--3
        //  | /    | /
        //  1------2

        vision_msgs::Detection3D detect = msg->detections.at(i);

        // TODO: Convert vision_msgs::Detection3D to custom_msgs::Obstacles
        // TODO: Convert to map coordinate ?

        // 1
        geometry_msgs::Point p1;
        p1.x = detect.bbox.center.position.x - detect.bbox.size.x/2;
        p1.y = detect.bbox.center.position.y + detect.bbox.size.y/2;
        p1.z = 0.0;
        obs.form.push_back(p1);

        // 2
        geometry_msgs::Point p2;
        p2.x = detect.bbox.center.position.x + detect.bbox.size.x/2;
        p2.y = detect.bbox.center.position.y + detect.bbox.size.y/2;
        p2.z = 0.0;
        obs.form.push_back(p2);

        // 3
        geometry_msgs::Point p3;
        p2.x = detect.bbox.center.position.x + detect.bbox.size.x/2;
        p2.y = detect.bbox.center.position.y - detect.bbox.size.y/2;
        p2.z = 0.0;
        obs.form.push_back(p3);

        // 4
        geometry_msgs::Point p4;
        p2.x = detect.bbox.center.position.x - detect.bbox.size.x/2;
        p2.y = detect.bbox.center.position.y - detect.bbox.size.y/2;
        p2.z = 0.0;
        obs.form.push_back(p4);

        obs.id = detect.results.at(0).id;
        obs.score = 1.0;

        temp.list.push_back(obs);
    }
    // Publish message msg to topic chatter
    obstaclePub.publish(temp);
}

int main(int argc, char **argv)
{
    // init ROS, This allows ROS to do name remapping through the command line and define node names
    ros::init(argc, argv, "addObject2Map");

    // Create a handle to this process node
    ros::NodeHandle nh;

    // Publisher is chatter_pub publishing to topic /object_costamp_layer/obsctacles with queue_size 1000
    ros::Publisher chatter_pub = nh.advertise<custom_msgs::Obstacles>("/object_costamp_layer/obsctacles", 1000);
    ros::Subscriber sub = nh.subscribe("detect_array", 1000, ObjDBCallBack);

    obstaclePub = nh.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obstacles", 1000);

    ros::spin();

    return 0;
}
