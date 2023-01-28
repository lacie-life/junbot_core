//
// Created by lacie on 28/01/2023.
//

#ifndef ORB_SLAM3_ROS_DETECTOR_H
#define ORB_SLAM3_ROS_DETECTOR_H

#include "System.h"
#include "KeyFrame.h"
#include "YoloDetection.h"

#include <condition_variable>
#include <boost/make_shared.hpp>
#include <string>


class ORB_SLAM3::KeyFrame;
class YoloDetection;

class Detector {

public:
    void insertKFColorImg(ORB_SLAM3::KeyFrame* kf, cv::Mat color);
    void Run(void);
    Detector(std::string modelPath);
    ~Detector();

protected:
    std::shared_ptr<thread>  mRunThread;

    condition_variable  colorImgUpdated;

    mutex               colorImgMutex;
    std::vector<cv::Mat>     colorImgs;
    std::vector<ORB_SLAM3::KeyFrame*> mvKeyframes;
    mutex mvKeyframesMutex;

    //std::vector<std::vector<Object>> mvvObjects;
    //mutex  mvvObjectsMutex;


    uint16_t          lastKeyframeSize =0;
    YoloDetection* mDetector;
};

#endif //ORB_SLAM3_ROS_DETECTOR_H
