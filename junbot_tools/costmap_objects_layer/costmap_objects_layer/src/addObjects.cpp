//
// Created by lacie on 08/02/2023.
//

#include <ros/ros.h>
#include <custom_msgs/Obstacles.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>

ros::Publisher obstaclePub;

void ObjDBCallBack(const vision_msgs::Detection3DArray::ConstPtr& msg)
{
    ROS_INFO("I received: [%s] objects", msg->detections.size());

    custom_msgs::Obstacles temp;

    for (int i = 0; i < msg->detections.size(); i++)
    {
        custom_msgs::Form obs;

        // TODO: Convert vision_msgs::Detection3D to custom_msgs::Obstacles

        temp.list.push_back(obs);
    }
    // Publish message msg to topic chatter
    obstaclePub.publish(temp);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "addObject2Map");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("detect_array", 1000, ObjDBCallBack);
    obstaclePub = nh.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obstacles", 1000);

    ros::spin();

    return 0;
}