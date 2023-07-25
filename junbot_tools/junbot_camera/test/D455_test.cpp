//
// Created by lacie on 02/02/2023.
//

#include "yolov5_detection.h"
#include "postprocess.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <iostream>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;
using namespace cv;
using namespace std;

void ShowManyImages(std::string title, cv::Mat image1, cv::Mat image2) {
    int size;
    int i;
    int m, n;
    int x, y;

// w - Maximum number of images in a row
// h - Maximum number of images in a column
    int w, h;

// scale - How much we have to resize the image
    float scale;
    int max;
    int nArgs =2;

// If the number of arguments is lesser than 0 or greater than 12
// return without displaying
    if(nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if(nArgs > 14) {
        printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
        return;
    }
// Determine the size of the image,
// and the number of rows/cols
// from number of arguments
    else if (nArgs == 1) {
        w = h = 1;
        size = 300;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 600;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 300;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }

// Create a new 3 channel image
    Mat DispImage = Mat::zeros(Size(100 + size*w, 60 + size*h), CV_8UC3);
    
// Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {

        cv::Mat img;
        if(i == 0)
            img = image1.clone();
        else
            img = image2.clone();

        // Check whether it is NULL or not
        // If it is NULL, release the image, and return
        if(img.empty()) {
            printf("Invalid arguments");
            return;
        }

        // Find the width and height of the image
        x = img.cols;
        y = img.rows;

        // Find whether height or width is greater in order to resize the image
        max = (x > y)? x: y;

        // Find the scaling factor to resize the image
        scale = (float) ( (float) max / size );

        // Used to Align the images
        if( i % w == 0 && m!= 20) {
            m = 20;
            n+= 20 + size;
        }

        // Set the image ROI to display the current image
        // Resize the input image and copy the it to the Single Big Image
        Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
        Mat temp; resize(img,temp, Size(ROI.width, ROI.height));
        temp.copyTo(DispImage(ROI));
    }

// Create a new window, and show the Single Big Image
    namedWindow( title, 1 );
    imshow( title, DispImage);
    waitKey(1);
}


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

    std::string model= "/home/orin/catkin_ws/src/JunBot/junbot_tools/D455/Model/yolov5s.engine";
    YoLoObjectDetection det(model);

    namedWindow(window_name, WINDOW_AUTOSIZE);
    
    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame rgb = data.get_color_frame().apply_filter(color_map);
//        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::depth_frame depth = data.get_depth_frame();
        
        // Query frame size (width and height)
        const int w = rgb.as<rs2::video_frame>().get_width();
        const int h = rgb.as<rs2::video_frame>().get_height();
        const int w_depth = depth.as<rs2::video_frame>().get_width();
        const int h_depth = depth.as<rs2::video_frame>().get_height();
        
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat frame(Size(w, h), CV_8UC3, (void*)rgb.get_data(), Mat::AUTO_STEP);
        
        Mat frame_depth(Size(w_depth, h_depth), CV_8UC1, (void*)depth.get_data(), Mat::AUTO_STEP);

        std::cout << "1234" << '\n';

        if(frame.empty())
        {
            break;
        }

        cv::cvtColor(frame, frame, COLOR_BGR2RGB);

        
        
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        std::vector<Detection> res;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::cout << "1234" << '\n';
        det.detectObject(frame, res);
        std::cout << "1234" << '\n';
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        // std::cout<<"Time: " << time_used.count() << "seconds" << std::endl;
        int fps = 1.0/time_used.count();
        std::string string_fps = "FPS: " + std::to_string(fps);
        
        for(auto object:res)
        {
            cv::Rect r = get_rect(frame, object.bbox);
//            float bounding_[8];
//            bounding_[0] = 1;
            // insert bounding in image RGB
            cv::rectangle(frame, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(frame, std::to_string((int)object.class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            cv::putText(frame, string_fps, cv::Point(11,80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

            cv::Rect r_depth = get_rect(frame, object.bbox);
            ;
//            float bounding_[8];
//            bounding_[0] = 1;
                // insert bounding in image RGB
            cv::rectangle(frame_depth, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(frame_depth, std::to_string((int)object.class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            cv::putText(frame_depth, string_fps, cv::Point(11,80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            // insert bounding in image depth
//            cv::Rect r_depth = get_rect(frame_depth, object.bbox);
            // get pixel value
            float distance_median= 0;
            float distance_min = 999;
            float count = 0;
            int w_temp = r.x+r.width;
            int h_temp = r.y+r.height;
            if (w_temp >= w_depth)
            {
                w_temp = w_depth;
            }
            if (h_temp >= h_depth)
            {
                h_temp = h_depth;
            }
            for (int i=r.x; i<w_temp; i++) {
                if (i < 0) {
                    i = 0;
                }
                for (int j = r.y; j < h_temp; j++) {
                    if (j < 0) {
                        j = 0;
                    }
                    if (depth.get_distance(i, j) < distance_min) {
                        distance_min = depth.get_distance(i, j);
                    }
                    distance_median = distance_median + depth.get_distance(i, j);
                    count++;
                }
            }
            distance_median = distance_median/count;
            float lenght_bounding = abs(distance_median - distance_min)*2;
            float width_bounding = r.width;
        }
        cv::imshow(window_name, frame);
        cv::imshow("depth", frame_depth);
//        ShowManyImages("Image", frame, frame_depth);
//        cv::imwrite("/results/", frame);
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