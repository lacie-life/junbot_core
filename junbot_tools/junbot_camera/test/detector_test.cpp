//
// Created by lacie on 03/06/2023.
//

#include "yolov5_detection.h"
#include "postprocess.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;

static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A,
                                       0x92CC17, 0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF,
                                       0x344593, 0x6473FF, 0x0018EC, 0x8438FF, 0x520085, 0xCB38FF,
                                       0xFF95C8, 0xFF37C7};

int main( int argc, char** argv )
{
    cv::Mat input_image = cv::imread(argv[1]);

    std::string model= argv[2];

    YoLoObjectDetection det(model);

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    std::vector<Detection> res;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    det.detectObject(input_image, res);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);

    std::cout<<"Time: " << time_used.count() << " seconds" << std::endl;

    int fps = 1.0/time_used.count();
    std::string string_fps = "FPS: " + std::to_string(fps);
    bool check_obs = 0;

    for(auto object:res) {

        cv::Rect r = get_rect(input_image, object.bbox);
        cv::Mat mask = det.get_mask(input_image, object);

        auto color = colors[(int) object.class_id % colors.size()];
        auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);

        // Fill mask to image
        for (int x = r.x; x < r.x + r.width; x++) {
            for (int y = r.y; y < r.y + r.height; y++) {
                float val = mask.at<float>(y, x);

                if (val <= 0.5) {
                    continue;
                }
                input_image.at<cv::Vec3b>(y, x)[0] = input_image.at<cv::Vec3b>(y, x)[0] / 2 + bgr[0] / 2;
                input_image.at<cv::Vec3b>(y, x)[1] = input_image.at<cv::Vec3b>(y, x)[1] / 2 + bgr[1] / 2;
                input_image.at<cv::Vec3b>(y, x)[2] = input_image.at<cv::Vec3b>(y, x)[2] / 2 + bgr[2] / 2;
            }
        }

        cv::rectangle(input_image, r, bgr, 2);
    }

    cv::imshow("result", input_image);
    cv::waitKey(0);

    return 0;
}