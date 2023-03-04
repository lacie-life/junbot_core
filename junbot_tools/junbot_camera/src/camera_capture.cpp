#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include "ros/ros.h"       

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "d435i");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    rs2::rates_printer printer;
    std::vector<rs2::pipeline> pipelines;
    std::map<std::string, rs2::colorizer> colorizers;
    rs2::context ctx;

    // Capture serial numbers before opening streaming
    std::vector<std::string> serials;
    for (auto&& dev : ctx.query_devices())
    {
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        colorizers[serial] = rs2::colorizer();
    }

    
    std::map<int, rs2::frame> render_frames;

    cv::Mat image;

    int left = 0;
    int right = 0;
    char left_path[100];
    char right_path[100];

    while (1) 
    {
        std::vector<rs2::frame> new_frames;
        std::vector<cv::Mat> new_images;
        for (auto &&pipe : pipelines)
        {
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs))
            {
                rs2::frame color = fs.get_color_frame();
                new_frames.emplace_back(color);
            }
        }
        for (auto &frame : new_frames)
        {
            auto name = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            render_frames[frame.get_profile().unique_id()] = colorizers[name].process(frame);
            cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
            // cv::cvtColor(color, color, cv::COLOR_BGR2RGB);
            cv::imshow(name, color);
            new_images.emplace_back(color);
        }

        char c = (char)cv::waitKey(25);
        if(c == 27){
            sprintf(left_path, "/home/jun/catkin_ws/src/JunBot/jun_camera/images/left/0-%d.jpg", left);
            sprintf(right_path, "/home/jun/catkin_ws/src/JunBot/jun_camera/images/right/1-%d.jpg", right);
    
            cv::imwrite(left_path, new_images.at(0));
            cv::imwrite(right_path, new_images.at(1));

            left++;
            right++;
        }
    }

    return EXIT_SUCCESS;
}