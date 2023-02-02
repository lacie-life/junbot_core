//
// Created by lacie on 02/02/2023.
//

#include "yolov5_detection.h"
#include "include/postprocess.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <iostream>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

int main( int argc, char** argv )
{
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

    cv::Mat frame;

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

    return 0;
}
