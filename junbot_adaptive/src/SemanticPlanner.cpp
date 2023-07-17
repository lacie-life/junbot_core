//
// Created by lacie on 17/07/2023.
//

#include "SemanticPlanner.h"

SemanticPlanner::SemanticPlanner(ros::NodeHandle &nh)
{
    obstaclePub = nh.advertise<custom_msgs::Obstacles>("/object_costmap_layer/obsctacles", 1);

    obstacleSub = nh.subscribe("obstacles", 1, &SemanticPlanner::ObstacleCallback, this);
    grid_map_sub = nh.subscribe("grid_map", 1, &SemanticPlanner::gridMapCallback, this);
    plan_sub = nh.subscribe("/move_base/DWAPlannerROS/global_plan", 1, &SemanticPlanner::globalPlanCallback, this);

    robotPoselistener = new tf::TransformListener();
}

SemanticPlanner::~SemanticPlanner()
{
    delete robotPoselistener;
}

void SemanticPlanner::ObstacleCallback(const vision_msgs::Detection3DArray::ConstPtr &msg) {
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

        geometry_msgs::PointStamped center_point;

        center_point.point.x = detect.bbox.center.position.x;
        center_point.point.y = detect.bbox.center.position.y;
        center_point.point.z = 0.0;

        geometry_msgs::PointStamped map_point;

        center_point.header.frame_id = "/base_link";

        try {
            robotPoselistener->transformPoint("/map", center_point, map_point);

        } catch (tf::TransformException& ex) {
            ROS_WARN("robot pose tf listener wrong ");
            try {
                robotPoselistener->waitForTransform("/map", "/base_link", ros::Time(0),
                                                    ros::Duration(0.4));
            } catch (tf::TransformException& ex) {
                ROS_WARN("robot pose tf listener wrong ");
            }
        }

        // TODO: Convert vision_msgs::Detection3D to custom_msgs::Obstacles
        // TODO: Convert to map coordinate

        // 1
        geometry_msgs::Point p1;
        p1.x = center_point.point.x - detect.bbox.size.x/2;
        p1.y = center_point.point.y + detect.bbox.size.y/2;
        p1.z = 0.0;
        obs.form.push_back(p1);

        // 2
        geometry_msgs::Point p2;
        p2.x = center_point.point.x + detect.bbox.size.x/2;
        p2.y = center_point.point.y + detect.bbox.size.y/2;
        p2.z = 0.0;
        obs.form.push_back(p2);

        // 3
        geometry_msgs::Point p3;
        p2.x = center_point.point.x + detect.bbox.size.x/2;
        p2.y = center_point.point.y - detect.bbox.size.y/2;
        p2.z = 0.0;
        obs.form.push_back(p3);

        // 4
        geometry_msgs::Point p4;
        p2.x = center_point.point.x - detect.bbox.size.x/2;
        p2.y = center_point.point.y - detect.bbox.size.y/2;
        p2.z = 0.0;
        obs.form.push_back(p4);

        obs.id = detect.results.at(0).id;
        obs.score = 1.0;

        temp.list.push_back(obs);
    }
    // Publish message msg to topic chatter
    obstaclePub.publish(temp);
}

void SemanticPlanner::calculateZone() {
    // Pending
}

void SemanticPlanner::globalPlanCallback(const nav_msgs::Path::ConstPtr &msg) {

    mutex.lock();
    for (int i = 0; i < msg->poses.size(); i++)
    {
        global_plan.poses.push_back(msg->poses[i]);
    }
    mutex.unlock();
}

void SemanticPlanner::gridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    mutex.lock();
    grid_map.info.width = msg->info.width;
    grid_map.info.height = msg->info.height;
    grid_map.data = msg->data;
    mutex.unlock();
}

void SemanticPlanner::run() {
    ros::Rate loop_rate(40);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {
        calculateZone();
        loop_rate.sleep();
    }
}
