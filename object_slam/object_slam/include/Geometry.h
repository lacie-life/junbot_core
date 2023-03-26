//
// Created by lacie on 28/01/2023.
//

#ifndef SEMANTIC_SLAM_ROS_GEOMETRY_H
#define SEMANTIC_SLAM_ROS_GEOMETRY_H

#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include "Frame.h"

#define MAX_DB_SIZE 20
#define MAX_REF_FRAMES 5
#define ELEM_INITIAL_MAP 5
#define MIN_DEPTH_THRESHOLD 0.2

namespace semantic_slam
{
    class Geometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    private:

        class DynKeyPoint
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        public:
            cv::Point2i mPoint;
            int mRefFrameLabel;
        };

        class DataBase
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            vector<semantic_slam::Frame> mvDataBase = vector<semantic_slam::Frame>(MAX_DB_SIZE);
            int mIni=0;
            int mFin=0;
            int mNumElem = 0;
            bool IsFull();
            void InsertFrame2DB(const semantic_slam::Frame &currentFrame);
        };

        vector<DynKeyPoint> ExtractDynPoints(const vector<semantic_slam::Frame> &vRefFrames,
                                             const semantic_slam::Frame &currentFrame);

        vector<semantic_slam::Frame> GetRefFrames(const semantic_slam::Frame &currentFrame);

//      void CombineMasks(const semantic_slam::Frame &currentFrame, cv::Mat &mask);


//      void FillRGBD(const semantic_slam::Frame &currentFrame,
//                               cv::Mat &mask,cv::Mat &imGray,cv::Mat &imDepth);
//      void FillRGBD(const semantic_slam::Frame &currentFrame,
//                               cv::Mat &mask,cv::Mat &imGray,
//                               cv::Mat &imDepth,cv::Mat &imRGB);

        cv::Mat DepthRegionGrowing(const vector<DynKeyPoint> &vDynPoints,
                                   const cv::Mat &imDepth);

        bool isRotationMatrix(const cv::Mat &R);
        cv::Mat rotm2euler(const cv::Mat &R);

        cv::Mat RegionGrowing(const cv::Mat &Image,
                              int &x,int &y,const float &threshold);

        cv::Mat RegionGrowingGaps(const cv::Mat &Image, int &x, int &y);

        int mnRefFrames;

        int mDmax;

        float mDepthThreshold;

        float mSegThreshold;

        double mSizeThreshold;

        float mVarThreshold;

        double mParallaxThreshold;

        DataBase mDB;

        cv::Mat vAllPixels;

        bool IsInFrame(const float &x, const float &y,
                       const semantic_slam::Frame &Frame);

        bool IsInImage(const float &x, const float &y, const cv::Mat image);

    public:
        Geometry();
        ~Geometry() = default;

        void GeometricModelCorrection(const semantic_slam::Frame &currentFrame,
                                      cv::Mat &imDepth, cv::Mat &mask);

        // void InpaintFrames(const ORB_SLAM3::Frame &currentFrame,
        //                      cv::Mat &imGray, cv::Mat &imDepth, cv::Mat &imRGB, cv::Mat &mask);

        void GeometricModelUpdateDB(const semantic_slam::Frame &mCurrentFrame);
    };
}

#endif //SEMANTIC_SLAM_ROS_GEOMETRY_H
