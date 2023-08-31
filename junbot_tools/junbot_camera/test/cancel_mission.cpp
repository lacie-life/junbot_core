#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <nlohmann/json.hpp>
#include "geometry_msgs/Twist.h"
#include <std_msgs/Int32.h>
#include <iostream>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

struct object {
    int id;
    int distance;
};

struct aruco_struct {
    int id;
    float x, y, z;
    float distance;
    int centerX, centerY;
};

struct pose_ {
    int ID_mission;
    float x,y,w;
};

enum State {
    free_ = 0, goal = 1, aruco = 2
};

enum State status = free_;
aruco_struct makerAruco;
move_base_msgs::MoveBaseActionGoal tempGoal;
actionlib_msgs::GoalID tempCancel;
ros::Publisher pub;
ros::Publisher pub_goal_arrived;
ros::Publisher cancel;
ros::Publisher pub_cmd_vel;
std::string msg_cancel = "0";
bool outPerson = true;
geometry_msgs::PoseWithCovariance amcl_pose;
std::vector<pose_> target;
geometry_msgs::Twist msg_cmd_vel;

void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg_goal) {
    tempGoal.goal = msg_goal->goal;
}

void arucoCallback(const std_msgs::String::ConstPtr &msg_aruco) {
    using json = nlohmann::json;
    std::string tmp = msg_aruco->data.c_str();
    json parsedJson = json::parse(tmp);
    int id_aruco_detect;
    id_aruco_detect = parsedJson["id"];
    if (id_aruco_detect != target.at(0).ID_mission){
        return;
    }
    makerAruco.id = id_aruco_detect;
    makerAruco.x = parsedJson["x"];
    makerAruco.y = parsedJson["y"];
    makerAruco.z = parsedJson["z"];
    makerAruco.centerX = parsedJson["centerX"];
    makerAruco.centerY = parsedJson["centerY"];
    //calculate distance
    float distance = sqrt(makerAruco.x*makerAruco.x+makerAruco.z*makerAruco.z);
    if (distance < 1.3){
        tempCancel.stamp = {};
        tempCancel.id = {};
        cancel.publish(tempCancel);
        status = aruco;
    }
}

void missionCallback(const std_msgs::String::ConstPtr &msg_mission) {
    using json = nlohmann::json;
    std::string tmp = msg_mission->data.c_str();
    json parsedJson = json::parse(tmp);
    target.at(0).ID_mission = parsedJson["id"];
    target.at(1).ID_mission = parsedJson["id"];
    target.at(1).x = parsedJson["target_x"];
    target.at(1).y = parsedJson["target_y"];
    target.at(1).w = parsedJson["target_w"];
    target.at(0).x = parsedJson["ref_x"];
    target.at(0).y = parsedJson["ref_y"];
    target.at(0).w = parsedJson["ref_w"];
    status = goal;
}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_amcl)
{
    amcl_pose = msg_amcl->pose;
}
void cancelCallback(const std_msgs::String::ConstPtr &msg) {
    using json = nlohmann::json;
    object obj;
    std::string tmp = msg->data.c_str();
    json parsedJson = json::parse(tmp);
    obj.id = parsedJson["id"];
    obj.distance = parsedJson["distance"];

    if (status == free_) {
        // check no mission or have mission from app

    }
        // Mission + Goal + ArUcO not detected
    else if (status == goal) {
        // Check distance
        if (obj.id == 0 && obj.distance < 0.6) {
            if (msg_cancel == "1") {
                tempCancel.stamp = {};
                tempCancel.id = {};
                cancel.publish(tempCancel);
                msg_cancel = "0";
                std::cout << "obs_goal";
            }
        } else {
            if (msg_cancel == "0") {
                if (tempGoal.goal.target_pose.pose.position.x != FLT_MAX &&
                    tempGoal.goal.target_pose.pose.position.y != FLT_MAX) {
                    pub.publish(tempGoal);
                    std::cout << "goal";
                }
                msg_cancel = "1";
            }
        }
    }
        // Mission + ArUcO detected
    else if (status == aruco) {
        // Check distance
        if (obj.id == 0 && obj.distance < 0.6) {
            msg_cmd_vel.linear.x = 0;
            msg_cmd_vel.angular.z = 0;
            pub_cmd_vel.publish(msg_cmd_vel);
            outPerson = false;
            std::cout << "obs_aruco";
        } else {
            outPerson = true;
            std::cout << "aruco";
        }
    }
}

void moveAruco() {
    tempCancel.stamp = {};
    tempCancel.id = {};
    cancel.publish(tempCancel);
    std::vector <float> vel;
    vel.at(0) = target.at(0).x - amcl_pose.pose.position.x;
    vel.at(1) = target.at(0).y - amcl_pose.pose.position.y;
    vel.at(2) = target.at(0).w - amcl_pose.pose.orientation.w;

    float distance = sqrt(vel.at(0) * vel.at(0)  + vel.at(1)  * vel.at(1));
    msg_cmd_vel.linear.x = 0.1 * distance;
    msg_cmd_vel.angular.z = std::atan2(vel.at(1) , vel.at(0)) - amcl_pose.pose.orientation.w;
    if (distance < 0.1){
        msg_cmd_vel.linear.x = 0;
        msg_cmd_vel.angular.z = 0.1*(target.at(0).w- amcl_pose.pose.orientation.w);
        if (makerAruco.centerX > 390 && makerAruco.centerX < 410){
            vel.at(0) = target.at(1).x - amcl_pose.pose.position.x;
            vel.at(1) = target.at(1).y - amcl_pose.pose.position.y;
            distance = sqrt(vel.at(0) * vel.at(0)  + vel.at(1)  * vel.at(1));
            msg_cmd_vel.linear.x = 0.1 * distance;
            msg_cmd_vel.angular.z = 0;
            if (distance < 0.1){
                msg_cmd_vel.linear.x = 0;
                msg_cmd_vel.angular.z = 0;
                std_msgs::String msg_goal_arrived;
                std::string tmp = "{";
                tmp += "\"id\":" + std::to_string(makerAruco.id) + ",";
                tmp += "\"status\":done";
                tmp += "}";
                msg_goal_arrived.data = tmp;
                pub_goal_arrived.publish(msg_goal_arrived);
            }
        }
    }
    std::cout << "move";
}



int main(int argc, char **argv) {

    tempGoal.goal.target_pose.pose.position.x = FLT_MAX;
    tempGoal.goal.target_pose.pose.position.y = FLT_MAX;
    ros::init(argc, argv, "cancel_mission_pub_sub");
    ros::NodeHandle n;
    cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
    ros::Subscriber sub_goal = n.subscribe("/move_base/goal", 1000, goalCallback);
    ros::Subscriber sub_cancel = n.subscribe("/object_detected", 1000, cancelCallback);
    pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    ros::Subscriber sub_aruco = n.subscribe("/detectionAruco", 1000, arucoCallback);
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    pub_goal_arrived = n.advertise<std_msgs::String>("/goal_arrived", 1000);
    ros::Subscriber sub_mission = n.subscribe("/robot_target_id", 1000, missionCallback);
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose", 1000, poseAMCLCallback);
    while (ros::ok()) {
        ROS_INFO("mission: ");
        std::cout << status;
        if (status == aruco  && outPerson) {
            moveAruco();
        }
        ros::Rate loop_rate(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}