#include <iostream>
#include <chrono>
#include <cmath>

#include <sl/Camera.hpp>

#include "GLViewer.hpp"
#include "Detector.h"

int main(int argc, char** argv) {

    std::cout << "Hello world \n";

    // Open ZED camera
    sl::Camera zed;
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
    init_parameters.sdk_verbose = true;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;

    // OpenGL's coordinate system is right_handed
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Open ZED Fail \n";
        return EXIT_FAILURE;
    }

    zed.enablePositionalTracking();
    // Custom OD
    sl::ObjectDetectionParameters detection_parameters;
    detection_parameters.enable_tracking = true;
    detection_parameters.enable_mask_output = false; // designed to give person pixel mask
    detection_parameters.detection_model = sl::DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    returned_state = zed.enableObjectDetection(detection_parameters);

    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cerr << " enableObjectDetection fails \n";
        zed.close();
        return EXIT_FAILURE;
    }

    auto camera_config = zed.getCameraInformation().camera_configuration;
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404));
    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;
    // Create OpenGL Viewer
    GLViewer viewer;
    viewer.init(argc, argv, camera_info.calibration_parameters.left_cam, true);

    Detector* detector = new Detector(argv[1]);

    sl::Mat left_sl, point_cloud;
    cv::Mat left_cv_rgb;
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    sl::Objects objects;
    sl::Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();

    while (viewer.isAvailable())
    {
        if(zed.grab() == sl::ERROR_CODE::SUCCESS)
        {
            // TODO: Process image

        }
    }

    return 0;
}
