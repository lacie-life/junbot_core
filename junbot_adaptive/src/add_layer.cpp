#include "ros/ros.h"
#include <std_msgs/String.h>
#include <custom_msgs/Obstacles.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    // Create a handle to this process node
    ros::NodeHandle n;
    // Publisher is chatter_pub publishing to topic /object_costamp_layer/obsctacles with queue_size 1000
    ros::Publisher chatter_pub = n.advertise<custom_msgs::Obstacles>("/object_costamp_layer/obsctacles", 1000);

    int  count = 0;
    ros::Rate rate(100);
    while (ros::ok())
    {
        // create message msg with date type is string
        std_msgs::String msg;
        // creat variable ss with data type is stringstream
        //-4.729493, -5.122387
        float point_[8][3] = {{-3,0.75,0},
                              {-4,0.75,0},
                              {-3,6,0},
                              {-4,6,0},
                              {-1.5, 0, 0},
                              {-2, 0, 0},
                              {-1.5, 2, 0},
                              {-2, 2, 0}};

        int number_circle = 8;
        int number_objs = 2;

        custom_msgs::Obstacles temp;
        // add object layer
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
       // add circle
    //    for (int i = 0; i < number_circle; i++)
    //    {
    //        custom_msgs::Form obs;
    //        for (int n = 0 ; n < 1; n++) {
    //            geometry_msgs::Point p;
    //            p.x = circle_[i][0];
    //            p.y = circle_[i][1];
    //            p.z = circle_[i][2];
    //            obs.form.push_back(p);
    //        }
    //        temp.list.push_back(obs);
    //    }
        chatter_pub.publish(temp);
        ros::spinOnce();
        ++count;
        rate.sleep();
    }
    return 0;
}
