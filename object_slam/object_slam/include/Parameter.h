//
// Created by lacie on 18/02/2023.
//

#ifndef PARAMETER_H
#define PARAMETER_H

#include <string>

namespace semantic_slam
{
    extern bool enable_viewer;
    extern bool enable_viewmap;
    extern bool enable_viewimage;

    extern bool parallel_mapping;

//    extern bool whether_detect_object;
    extern bool whether_read_offline_cuboidtxt;
    extern bool associate_point_with_object;
//
    extern bool whether_dynamic_object;
    extern bool remove_dynamic_features;
    extern bool use_dynamic_klt_features;

    extern bool mono_firstframe_truth_depth_init;
    extern bool mono_firstframe_Obj_depth_init;
    extern bool mono_allframe_Obj_depth_init;

    extern bool enable_ground_height_scale;
    extern bool build_worldframe_on_ground;

// for BA
    extern bool bundle_object_opti;
    extern double object_velocity_BA_weight;
    extern double camera_object_BA_weight;

//dynamic debug
    extern bool ba_dyna_pt_obj_cam;
    extern bool ba_dyna_obj_velo;
    extern bool ba_dyna_obj_cam;

// for gui
    extern bool draw_map_truth_paths;
    extern bool draw_nonlocal_mappoint;

    enum Scene_Name
    {
        voidtype = 0,
        kitti
    };
    extern Scene_Name scene_unique_id;
    extern std::string base_data_folder;

}

#endif //PARAMETER_H
