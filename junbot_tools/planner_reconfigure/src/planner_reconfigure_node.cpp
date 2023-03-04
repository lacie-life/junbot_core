#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include "costmap_2d/InflationPluginConfig.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "planner_reconfigure_client");

    dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> local_client("/move_base/local_costmap/inflation_layer/");
    dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> global_client("/move_base/global_costmap/inflation_layer/");

    costmap_2d::InflationPluginConfig local_config;
    costmap_2d::InflationPluginConfig global_config;

    float count = 0.0;

    while(ros::ok())
    {
        count = count + 0.2;
        
        local_config.inflation_radius = count;
        local_client.setConfiguration(local_config);
        ros::Duration(1).sleep();

        global_config.inflation_radius = count;
        global_client.setConfiguration(global_config);
        ros::Duration(1).sleep();
    }
  return 0;
}