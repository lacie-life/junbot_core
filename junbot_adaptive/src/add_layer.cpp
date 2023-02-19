#include "ros/ros.h"
#include "std_msgs/String.h"
#include <custom_msgs/Obstacles.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <iostream>

// void ObjDBCallBack(const vision_msgs::Detection3DArray::ConstPtr& msg)
// {
//     ROS_INFO("I received: [%s] objects", msg->detections.size());

//     custom_msgs::Obstacles temp;

//     for (int i = 0; i < msg->detections.size(); i++)
//     {
//         custom_msgs::Form obs;

//         // TODO: Convert vision_msgs::Detection3D to custom_msgs::Obstacles

//         temp.list.push_back(obs);
//     }
//     // Publish message msg to topic chatter
//     obstaclePub.publish(temp);
// }

int main(int argc, char **argv)
{
    // init ROS, This allows ROS to do name remapping through the command line and define node names
    // ros::init(argc, argv, "addObject2Map");
    ros::init(argc, argv, "talker");
    // Create a handle to this process node
    ros::NodeHandle n;
    // Publisher is chatter_pub publishing to topic /object_costamp_layer/obsctacles with queue_size 1000
    ros::Publisher chatter_pub = n.advertise<custom_msgs::Obstacles>("/object_costamp_layer/obsctacles", 1000);
    // ros::Subscriber sub = nh.subscribe("detect_array", 1000, ObjDBCallBack);
    // obstaclePub = nh.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obstacles", 1000);
    int  count = 0;
    ros::Rate rate(1000);
    while (ros::ok())
    {
        // create message msg with date type is string
        std_msgs::String msg;
        // creat variable ss with data type is stringstream
        float point_[8][3] = {{-3,1,0},
                              {-4,1,0},
                              {-3,3,0},
                              {-4,3,0},
                              {-1.5, 0, 0},
                              {-2, 0, 0},
                              {-1.5, 2, 0},
                              {-2, 2, 0}};
        int number_objs = 2;

        custom_msgs::Obstacles temp;

        for (int i = 0; i < number_objs; i++)
        {
            custom_msgs::Form obs;
            for (int n = 0 ; n < 4; n++) {
                geometry_msgs::Point p;
                p.x = point_[n+i*4][0];
                p.y = point_[n+i*4][1];
                p.z = point_[n+i*4][2];

                obs.form.push_back(p);
            }
            temp.list.push_back(obs);
        }
        // Publish message msg to topic chatter
        chatter_pub.publish(temp);
        ros::spinOnce();
        ++count;
        rate.sleep();
    }
    return 0;
}