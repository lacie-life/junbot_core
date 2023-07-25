//
// Created by lacie on 02/02/2023.
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

using namespace boost::filesystem;
using namespace cv;

int main( int argc, char** argv ) try
{
        // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    std::string folderPath = argv[1];
    std::vector<std::string> imageList;
    boost::filesystem::path p(folderPath);
    for (auto i = directory_iterator(p); i != directory_iterator(); i++)
    {
        if (!is_directory(i->path())) //we eliminate directories
        {
            std::cout << i->path().filename().string() << std::endl;
            imageList.emplace_back( i->path().filename().string());
        }
        else
            continue;
    }
    std::cout << imageList.size() << std::endl;

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame rgb = data.get_color_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = rgb.as<rs2::video_frame>().get_width();
        const int h = rgb.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat frame(Size(w, h), CV_8UC3, (void*)rgb.get_data(), Mat::AUTO_STEP);

        // // Update the window with new data
        cv::cvtColor(frame, frame, COLOR_BGR2RGB);
        imshow(window_name, frame);

        YoLoObjectDetection det(argv[2]);
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        for (int i = 0; i < imageList.size(); i++)
        {
            frame = cv::imread(folderPath + imageList[i]);
            std::vector<Detection> res;
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            det.detectObject(frame, res);
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
            std::cout<<"Time: " << time_used.count() << "seconds" << std::endl;
            int fps = 1.0/time_used.count();
            std::string string_fps = "FPS: " + std::to_string(fps);

            for(auto object:res)
            {
                cv::Rect r = get_rect(frame, object.bbox);
                cv::rectangle(frame, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(frame, std::to_string((int)object.class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                cv::putText(frame, string_fps, cv::Point(11,80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            }

            cv::imshow("detection", frame);
            cv::imwrite(folderPath + "/results/" + imageList[i], frame);
            if(cv::waitKey(1) == 27)
            {
                break;
            }
        }
        std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
        std::chrono::duration<double> all_time = std::chrono::duration_cast<std::chrono::duration<double>>(stop-start);
        std::cout<<"Time: " << all_time.count() << "seconds" << std::endl;
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