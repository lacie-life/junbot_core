#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Int16.h"
#include <string.h>
#include <stdio.h>
#include <string>

serial::Serial ser;

std::string charToString(char a){
    std::string s = ""; 
    s = s + a;
    return s;
}


void cmd_callback(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing from serial port " <<  msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;

    std_msgs::String status;
    status.data = "ok";
     
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 1000, cmd_callback);
    ros::Publisher status_pub = nh.advertise<std_msgs::String>("status", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    ros::Rate loop_rate(5);

    while(ros::ok()){

        ros::spinOnce();
	    std::string tmp;
	    fflush(stdin);

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            status_pub.publish(result);
            ser.flush();
        }
        loop_rate.sleep();
    }
}
