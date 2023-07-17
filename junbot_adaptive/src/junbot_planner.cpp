//
// Created by lacie on 17/07/2023.
//

#include <iostream>
#include <ros/ros.h>
#include "SemanticPlanner.h"

int main(int argc, char** argv)
{
    std::cout << "Hello, World!" << std::endl;

    ros::init(argc, argv, "junbot_planner");

    ros::NodeHandle nh;

    SemanticPlanner semanticPlanner(nh);
    semanticPlanner.run();

    return 0;
}
