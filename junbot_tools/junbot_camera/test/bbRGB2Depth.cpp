//
// Created by lacie on 03/06/2023.
//

#include "yolov5_detection.h"
#include "postprocess.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <iostream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

using namespace boost::filesystem;
using namespace cv;


int main( int argc, char** argv ) try
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Publisher cancel_pub = n.advertise<std_msgs::String>("cancel_mission", 1000);
    ros::Rate loop_rate(100);
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    rs2::pipeline_profile selection = pipe.start();
    using namespace cv;
    const auto window_name = "Display Image";
    std::string model= "/home/junbot/junbot_ws/src/JunBot/junbot_tools/junbot_camera/Model/yolov5s.engine";

    namedWindow(window_name, 1);

    YoLoObjectDetection det(model);

    rs2::stream_profile  depth_stream = selection.get_stream(RS2_STREAM_DEPTH);
    rs2::stream_profile  color_stream = selection.get_stream(RS2_STREAM_COLOR);
    rs2_extrinsics depth_to_color_extrin = depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics color_to_depth_extrin = color_stream.get_extrinsics_to(depth_stream);
    // Apply extrinsics to the origin
    float origin[3] { 0.f, 0.f, 0.f };
    float target[3];
    rs2_transform_point_to_point(target, &depth_to_color_extrin, origin);
    rs2_transform_point_to_point(target, &color_to_depth_extrin, origin);

    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    float depth_scale =  sensor.get_depth_scale();

    auto depth_min = 0.11; //meter
    auto depth_max = 1.0; //meter
    rs2::video_stream_profile  depth_stream_ = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2_intrinsics depth_intrin = depth_stream_.get_intrinsics();
    rs2::video_stream_profile  color_stream_ = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics color_intrin = color_stream_.get_intrinsics();

    
    while ((ros::ok()))
    {
        std_msgs::String msg;
        std::stringstream ss;
        

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame rgb = data.get_color_frame().apply_filter(color_map);
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::depth_frame df = data.get_depth_frame();

        // Query frame size (width and height)
        const int w = rgb.as<rs2::video_frame>().get_width();
        const int h = rgb.as<rs2::video_frame>().get_height();

        const int w_depth = depth.as<rs2::video_frame>().get_width();
        const int h_depth = depth.as<rs2::video_frame>().get_height();
        std::cout << w << "  " << h << '\n';
        std::cout << w_depth << "  " << h_depth << '\n';
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat frame_rgb(Size(w, h), CV_8UC3, (void*)rgb.get_data(), Mat::AUTO_STEP);
        cv::Mat frame_depth(Size(w_depth, h_depth), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        // // Update the window with new data
        cv::cvtColor(frame_rgb, frame_rgb, COLOR_BGR2RGB);
        // imshow(window_name, frame);

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

        std::vector<Detection> res;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        det.detectObject(frame_rgb, res);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        std::cout<<"Time: " << time_used.count() << "seconds" << std::endl;
        int fps = 1.0/time_used.count();
        std::string string_fps = "FPS: " + std::to_string(fps);
        bool check_obs = 0;
        for(auto object:res) {

            cv::Rect r = get_rect(frame_rgb, object.bbox);

            cv::rectangle(frame_rgb, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(frame_rgb, std::to_string((int) object.class_id), cv::Point(r.x + r.width - 15, r.y),
                        cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xC1, 0xFF), 2);
            cv::putText(frame_rgb, string_fps, cv::Point(11, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2,
                        cv::LINE_AA);

            cv::Rect r_depth = get_rect(frame_rgb, object.bbox);
            
            float center[2] = {(float) r.x, (float) r.y};
            float center_depth[2];

            rs2_project_color_pixel_to_depth_pixel(
                    center_depth, reinterpret_cast<const uint16_t *>(df.get_data()), depth_scale,
                    depth_min, depth_max,
                    &depth_intrin, &color_intrin, &depth_to_color_extrin, &color_to_depth_extrin, center);

            r_depth.x = center_depth[0];
            r_depth.y = center_depth[1];

            cv::rectangle(frame_depth, r_depth, cv::Scalar(0x27, 0xC1, 0x36), 2);
            
            float distance_min = 999;
            float distance_medium = 0;
            float count = 0;
            int w_temp = r_depth.x + r_depth.width;
            int h_temp = r_depth.y + r_depth.height;
            if (w_temp >= w_depth) {
                w_temp = w_depth;
            }
            if (h_temp >= h_depth) {
                h_temp = h_depth;
            }
            for (int i = r_depth.x; i < w_temp; i++) {
                if (i < 0) {
                    i = 0;
                }
                for (int j = r_depth.y; j < h_temp; j++) {
                    if (j < 0) {
                        j = 0;
                    }
                    if (df.get_distance(i, j) < 0.2 || df.get_distance(i, j) > 2) {
                        break;
                    }
                    distance_medium = distance_medium + df.get_distance(i, j);
                    count++;
                    if (df.get_distance(i, j) < distance_min) {
                        distance_min = df.get_distance(i, j);
                    }
                }
            }
            object.distance = distance_medium/count;
            if (object.class_id == 0 && object.distance < 1) {
                std::string tmp = "0";
                msg.data = tmp;
                cancel_pub.publish(msg);
                check_obs = 1;
                // cv::putText(frame_depth, std::to_string((float)object.distance), cv::Point(r_depth.x, r_depth.y + 20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                // cv::putText(frame_rgb, std::to_string((float)object.distance), cv::Point(r.x, r.y +20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
            ros::spinOnce();
            loop_rate.sleep();
            if (object.class_id == 0)
            {
                cv::putText(frame_depth, std::to_string((float) object.distance), cv::Point(r_depth.x, r_depth.y +20),cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                cv::putText(frame_depth, std::to_string((float) object.class_id), cv::Point(r_depth.x, r_depth.y +40),cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
            if (object.class_id == 56 && object.distance < 5)
            {
                float positon_[3]; // From point (in 3D)
                float pixel[2];

                pixel[0] = r_depth.x;
                pixel[1] = r_depth.y;
                rs2_deproject_pixel_to_point(positon_, &depth_intrin, pixel, object.distance);

                std::cout << positon_[0] << " " << positon_[1] << " " << positon_[2] << " " << object.class_id << '\n';
            }
            // cv::putText(frame_rgb, std::to_string((float)distance_min), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }
        if (check_obs == 0)
        {
            std::string tmp = "1";
            msg.data = tmp;
            cancel_pub.publish(msg);
        }
        // cv::imshow(window_name, frame_rgb);
        cv::imshow("depth", frame_depth);

        if(cv::waitKey(1) == 27)
        {
            break;
        }

    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
