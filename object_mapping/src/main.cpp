#include <iostream>
#include <chrono>
#include <cmath>

#include <sl/Camera.hpp>
#include <ros/ros.h>

#include "GLViewer.hpp"
#include "Detector.h"
#include "ObjectDatabase.h"
#include "yololayer.h"

int main(int argc, char** argv) {

    std::cout << "Hello world \n";

    ros::init(argc, argv, "object_mapping");
    ros::start();

    // Open ZED camera
    sl::Camera zed;
    sl::InitParameters init_parameters;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
    init_parameters.sdk_verbose = true;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;

    // ROS coordinate
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

    if (argc > 1 && std::string(argv[2]).find(".svo") != std::string::npos) {
        // SVO input mode
        init_parameters.input.setFromSVOFile(argv[2]);
        init_parameters.svo_real_time_mode=true;

        std::cout << "[Debug] Using SVO File input: " << argv[2] << std::endl;
    }

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
    ObjectDatabase* oDB = new ObjectDatabase(true);

    sl::Mat left_sl, point_cloud;
    cv::Mat left_cv_rgb;
//    sl::RuntimeParameters
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    sl::Objects objects;

    sl::Pose cam_w_pose;
    sl::Pose w_cam_pose;
    cam_w_pose.pose_data.setIdentity();
    w_cam_pose.pose_data.setIdentity();

    int frameCount = 0;

    while (viewer.isAvailable())
    {
        if(zed.grab() == sl::ERROR_CODE::SUCCESS)
        {
            // TODO: Process image
            zed.retrieveImage(left_sl, sl::VIEW::LEFT);

            // Preparing inference
            cv::Mat left_cv_rgba = slMat2cvMat(left_sl);
            cv::cvtColor(left_cv_rgba, left_cv_rgb, cv::COLOR_BGRA2BGR);

            if (left_cv_rgb.empty())
            {
                continue;
            }

            std::vector<Yolo::Detection> objs = detector->detectObject(left_cv_rgb);

            std::cout << "Number object in frame " << frameCount << "th: " << objs.size() << "\n";

            // Preparing for ZED SDK ingesting
            std::vector<sl::CustomBoxObjectData> objects_in;
            for (auto &it : objs) {
                sl::CustomBoxObjectData tmp;
                cv::Rect r = get_rect(left_cv_rgb, it.bbox);
                // Fill the detections into the correct format
                tmp.unique_object_id = sl::generate_unique_id();
                tmp.probability = it.conf;
                tmp.label = (int) it.class_id;
                tmp.bounding_box_2d = cvt(r);
                tmp.is_grounded = ((int) it.class_id == 0); // Only the first class (person) is grounded, that is moving on the floor plane
                // others are tracked in full 3D space
                objects_in.push_back(tmp);
            }
            // Send the custom detected boxes to the ZED
            zed.ingestCustomBoxObjects(objects_in);

            // Displaying 'raw' objects
            for (size_t j = 0; j < objs.size(); j++) {
                cv::Rect r = get_rect(left_cv_rgb, objs[j].bbox);
                cv::rectangle(left_cv_rgb, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(left_cv_rgb, std::to_string((int) objs[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }

            cv::resize(left_cv_rgb, left_cv_rgb, cv::Size(640, 480), cv::INTER_LINEAR);

            cv::imshow("Objects", left_cv_rgb);
            cv::waitKey(10);

            // Retrieve the tracked objects, with 2D and 3D attributes
            zed.retrieveObjects(objects, objectTracker_parameters_rt);

            zed.getPosition(w_cam_pose, sl::REFERENCE_FRAME::WORLD);
            // Update object in object database
            oDB->updateObjectDatabase(objects, w_cam_pose);

            // GL Viewer
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::GPU, pc_resolution);
            zed.getPosition(cam_w_pose, sl::REFERENCE_FRAME::WORLD);
            viewer.updateData(point_cloud, objects.object_list, cam_w_pose.pose_data);

            frameCount++;
        }
    }

    zed.disableObjectDetection();
    zed.disablePositionalTracking();
    zed.close();

    return 0;
}
