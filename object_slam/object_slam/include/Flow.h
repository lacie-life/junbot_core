//
// Created by lacie on 28/01/2023.
//

#ifndef ORB_SLAM3_ROS_FLOW_H
#define ORB_SLAM3_ROS_FLOW_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "Frame.h"

using namespace std;

namespace ORB_SLAM3
{
    class Flow {
    private:
        cv::Mat mImGrayLast;
        cv::Mat mImGrayCurrent;
        //cv::Mat mImSource;
        // float mBInaryThreshold;

    public:
        Flow();
        ~Flow() = default;
        void ComputeMask(const cv::Mat& GrayImg, cv::Mat& mask, float BInaryThreshold);
        void ComputeMask(const cv::Mat& GrayImg, const cv::Mat& Homo, cv::Mat& mask, float BInaryThreshold);
    private:
        // Pass in the homography transformation matrix
        void myWarpPerspective(const cv::Mat& src, cv::Mat& dst, const cv::Mat& H);
    };
}

#endif //ORB_SLAM3_ROS_FLOW_H
