//
// Created by lacie on 18/02/2023.
//

#include "Parameter.h"

// TODO: Add to Settings.h

namespace semantic_slam
{
    bool parallel_mapping = true;

//    bool whether_detect_object = false;
    bool whether_read_offline_cuboidtxt = false;
    bool associate_point_with_object = true;
//
    bool whether_dynamic_object = false;
    bool remove_dynamic_features = false;
    bool use_dynamic_klt_features = false;

    bool mono_firstframe_truth_depth_init = false;
    bool mono_firstframe_Obj_depth_init = false;
    bool mono_allframe_Obj_depth_init = false;

    bool enable_ground_height_scale = false;
    bool build_worldframe_on_ground = false;

    // for BA
    bool bundle_object_opti = false;
    double object_velocity_BA_weight = 1.0;
    double camera_object_BA_weight = 1.0;

    // for gui
    bool draw_map_truth_paths = true;
    bool draw_nonlocal_mappoint = true;

    //dynamic debug
    bool ba_dyna_pt_obj_cam = false;
    bool ba_dyna_obj_velo = true;
    bool ba_dyna_obj_cam = true;

    std::string base_data_folder;

    Scene_Name scene_unique_id;
}
