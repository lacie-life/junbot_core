//
// Created by lacie on 28/01/2023.
//

#ifndef SEMANTIC_SLAM_ROS_DETECTOR_H
#define SEMANTIC_SLAM_ROS_DETECTOR_H

#include "System.h"
#include "KeyFrame.h"
#include "YoloDetection.h"

#include <condition_variable>
#include <boost/make_shared.hpp>
#include <string>

class YoloDetection;

class Detector {

public:
    void insertKFColorImg(semantic_slam::KeyFrame* kf, cv::Mat color);
    void Run(void);
    Detector(std::string modelPath);
    ~Detector();

protected:
    std::shared_ptr<thread>  mRunThread;

    condition_variable  colorImgUpdated;

    mutex               colorImgMutex;
    std::vector<cv::Mat>     colorImgs;
    std::vector<semantic_slam::KeyFrame*> mvKeyframes;
    mutex mvKeyframesMutex;

    //std::vector<std::vector<Object>> mvvObjects;
    //mutex  mvvObjectsMutex;

    uint16_t  lastKeyframeSize =0;
    YoloDetection* mDetector;
};

#endif //SEMANTIC_SLAM_ROS_DETECTOR_H
