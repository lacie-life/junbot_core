//
// Created by lacie on 03/06/2023.
//

#include "yolov5_detection.h"
#include "postprocess.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

using namespace boost::filesystem;
using namespace cv;

static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A,
                                         0x92CC17, 0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF,
                                         0x344593, 0x6473FF, 0x0018EC, 0x8438FF, 0x520085, 0xCB38FF,
                                         0xFF95C8, 0xFF37C7};

class D455_camera{
public:
    rs2::colorizer color_map;
    rs2::pipeline pipelines;
    rs2::pipeline_profile selections;
    std::string serials;
    rs2::stream_profile  depth_stream;
    rs2::stream_profile  color_stream;
    rs2_extrinsics depth_to_color_extrin;
    rs2_extrinsics color_to_depth_extrin;

    // Apply extrinsics to the origin
    float origin[3] { 0.f, 0.f, 0.f };
    float target[3];
    float depth_scale;
    rs2::video_stream_profile  depth_stream_;
    rs2::video_stream_profile  color_stream_;
    rs2_intrinsics depth_intrin;
    rs2_intrinsics color_intrin;
};

int main( int argc, char** argv ) try
{
    ros::init(argc, argv, "D455_Multi_Camera");
    ros::NodeHandle n;
    ros::Publisher cancel_pub = n.advertise<std_msgs::String>("cancel_mission", 1000);
    ros::Rate loop_rate(100);

    rs2::context ctx;
    std::vector<D455_camera> D455;
    D455.resize(2);

    // Capture serial numbers before opening streaming
    int count = 0;
    // serial ID D455: 117122250794
    // serial ID D455: 117122250006

    for (auto&& dev : ctx.query_devices()){
        std::string serials = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (serials != "908412110411"){
            std::cout << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
            D455.at(count).serials = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            count++;
        }
    }

    // Start a streaming pipe per each connected device
    for (int i=0; i<D455.size(); i++)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        rs2::pipeline_profile selection;

        cfg.enable_device(D455.at(i).serials);

        selection = pipe.start(cfg);
        D455.at(i).pipelines = pipe;
        D455.at(i).selections = selection;

        // Map from each device's serial number to a different colorizer
        D455.at(i).color_map = rs2::colorizer();
    }

    using namespace cv;
    const auto window_name = "Display Image";
    const auto window_name1 = "Display Image1";
    std::string model= "/home/junbot/junbot_ws/src/JunBot/junbot_tools/junbot_camera/Model/yolov5s_seg.engine"; // orin
    // std::string model= "/home/hn/junbot_ws/src/JunBot/junbot_tools/junbot_camera/Model/yolov5s_orin.engine"; //HN

    namedWindow(window_name, 1);
    namedWindow(window_name1, 1);
    YoLoObjectDetection det(model);

    // Get the depth stream's unique ID
    for (int i=0; i<D455.size(); i++)
    {
        D455.at(i).depth_stream = D455.at(i).selections.get_stream(RS2_STREAM_DEPTH);
        D455.at(i).color_stream = D455.at(i).selections.get_stream(RS2_STREAM_COLOR);
        D455.at(i).depth_to_color_extrin = D455.at(i).depth_stream.get_extrinsics_to(D455.at(i).color_stream);
        D455.at(i).color_to_depth_extrin = D455.at(i).color_stream.get_extrinsics_to(D455.at(i).depth_stream);
        rs2_transform_point_to_point(D455.at(i).target, &D455.at(i).depth_to_color_extrin,D455.at(i).origin);
        rs2_transform_point_to_point(D455.at(i).target, &D455.at(i).color_to_depth_extrin,D455.at(i).origin);
        rs2::depth_sensor sensor = D455.at(i).selections.get_device().first<rs2::depth_sensor>();;
        D455.at(i).depth_scale = sensor.get_depth_scale();
        D455.at(i).depth_stream_ = D455.at(i).selections.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        D455.at(i).depth_intrin = D455.at(i).depth_stream_.get_intrinsics();
        D455.at(i).color_stream_ = D455.at(i).selections.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        D455.at(i).color_intrin = D455.at(i).color_stream_.get_intrinsics();
    }
    double depth_min = 0.11; //meter
    double depth_max = 1.0; //meter

    while ((ros::ok()))
    {
        std_msgs::String msg;
        std::stringstream ss;

        // Collect the new frames from all the connected devices
        std::vector<rs2::frameset> new_frames;

        for (int i = 0; i < D455.size(); i++) {
            rs2::frameset fs;
            if (D455.at(i).pipelines.poll_for_frames(&fs))
            {
                new_frames.push_back(fs);
            }
        }
        for (int i = 0; i < new_frames.size(); i++) {
            rs2::frameset data = new_frames.at(i);
            auto serial = rs2::sensor_from_frame(data.get_color_frame())->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            int check_serial = 0;
            if (serial == D455.at(0).serials)
                check_serial = 0;
            else if (serial == D455.at(1).serials)
                check_serial = 1;
            rs2::frame rgb = data.get_color_frame().apply_filter(D455.at(check_serial).color_map);
            rs2::frame depth = data.get_depth_frame().apply_filter(D455.at(check_serial).color_map);
            rs2::depth_frame df = data.get_depth_frame();
            // Query frame size (width and height)
            const int w = rgb.as<rs2::video_frame>().get_width();
            const int h = rgb.as<rs2::video_frame>().get_height();

            const int w_depth = depth.as<rs2::video_frame>().get_width();
            const int h_depth = depth.as<rs2::video_frame>().get_height();

            // Create OpenCV matrix of size (w,h) from the colorized depth data
            cv::Mat frame_rgb(Size(w, h), CV_8UC3, (void*)rgb.get_data(), Mat::AUTO_STEP);
            cv::Mat frame_depth(Size(w_depth, h_depth), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

            // // Update the window with new data
            cv::cvtColor(frame_rgb, frame_rgb, COLOR_BGR2RGB);

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
                cv::Mat mask = det.get_mask(frame_rgb, object);

                auto color = colors[(int)object.class_id % colors.size()];
                auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);

                // Fill mask to image
                for (int x = r.x; x < r.x + r.width; x++) {
                    for (int y = r.y; y < r.y + r.height; y++) {
                        float val = mask.at<float>(y, x);
                        
                        if (val <= 0.5){
                            continue;    
                        } 
                        
                        frame_rgb.at<cv::Vec3b>(y, x)[0] = frame_rgb.at<cv::Vec3b>(y, x)[0] / 2 + bgr[0] / 2;
                        frame_rgb.at<cv::Vec3b>(y, x)[1] = frame_rgb.at<cv::Vec3b>(y, x)[1] / 2 + bgr[1] / 2;
                        frame_rgb.at<cv::Vec3b>(y, x)[2] = frame_rgb.at<cv::Vec3b>(y, x)[2] / 2 + bgr[2] / 2;
                    }
                }

                
                // cv::putText(frame_rgb, std::to_string((int) object.class_id), cv::Point(r.x + r.width - 15, r.y),
                //             cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xC1, 0xFF), 2);
                // cv::putText(frame_rgb, string_fps, cv::Point(11, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2,
                //             cv::LINE_AA);

                cv::Rect r_depth = get_rect(frame_rgb, object.bbox);

                float center[2] = {(float) r.x, (float) r.y};
                float center_depth[2];

                rs2_project_color_pixel_to_depth_pixel(
                        center_depth, reinterpret_cast<const uint16_t *>(df.get_data()), D455.at(check_serial).depth_scale,
                        depth_min, depth_max,
                        &D455.at(check_serial).depth_intrin, &D455.at(check_serial).color_intrin, &D455.at(check_serial).depth_to_color_extrin, &D455.at(check_serial).color_to_depth_extrin, center);

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
                            continue;
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
                    cv::rectangle(frame_rgb, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                    cv::putText(frame_depth, std::to_string((float) object.distance), cv::Point(r_depth.x, r_depth.y +20),cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                    cv::putText(frame_depth, std::to_string((float) object.class_id), cv::Point(r_depth.x, r_depth.y +40),cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                    cv::putText(frame_rgb, std::to_string((float)object.distance), cv::Point(r.x, r.y +20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);

                }
            }
            if (check_obs == 0)
            {
                std::string tmp = "1";
                msg.data = tmp;
                cancel_pub.publish(msg);
            }
            if (check_serial == 0){
                cv::imshow(window_name, frame_rgb);
                cv::imshow("depth", frame_depth);
            } else{
                cv::imshow(window_name1, frame_rgb);
                cv::imshow("depth", frame_depth);
            }


            if(cv::waitKey(1) == 27)
            {
                break;
            }
            ros::spinOnce();
            loop_rate.sleep();
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