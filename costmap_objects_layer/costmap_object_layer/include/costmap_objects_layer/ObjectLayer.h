//
// Created by lacie on 08/02/2023.
//

#ifndef COSTMAP_OBJECTS_LAYER_OBJECTLAYER_H
#define COSTMAP_OBJECTS_LAYER_OBJECTLAYER_H

#include "costmap_objects_layer/ObjectLayer.h"

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace object_layer_namespace
{
    class ObjectLayer : public costmap_2d::Layer
    {
    public:
        ObjectLayer();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                      double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    private:
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

        double mark_x_, mark_y_;
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    };
}

#endif //COSTMAP_OBJECTS_LAYER_OBJECTLAYER_H
