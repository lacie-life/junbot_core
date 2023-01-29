//
// Created by lacie on 28/01/2023.
//

#ifndef ORB_SLAM3_ROS_GEOMETRY_H
#define ORB_SLAM3_ROS_GEOMETRY_H

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

namespace ORB_SLAM3
{
    class Geometry {
    private:

        class DynKeyPoint
        {
        public:
            cv::Point2i mPoint;
            int mRefFrameLabel;
        };

        class DataBase
        {
        public:
            vector<ORB_SLAM3::Frame> mvDataBase = vector<ORB_SLAM3::Frame>(MAX_DB_SIZE);
            int mIni=0;
            int mFin=0;
            int mNumElem = 0;
            bool IsFull();
            void InsertFrame2DB(const ORB_SLAM3::Frame &currentFrame);
        };

        vector<DynKeyPoint> ExtractDynPoints(const vector<ORB_SLAM3::Frame> &vRefFrames,
                                             const ORB_SLAM3::Frame &currentFrame);

        vector<ORB_SLAM3::Frame> GetRefFrames(const ORB_SLAM3::Frame &currentFrame);

//      void CombineMasks(const ORB_SLAM3::Frame &currentFrame, cv::Mat &mask);


//      void FillRGBD(const ORB_SLAM3::Frame &currentFrame,
//                               cv::Mat &mask,cv::Mat &imGray,cv::Mat &imDepth);
//      void FillRGBD(const ORB_SLAM3::Frame &currentFrame,
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
                       const ORB_SLAM3::Frame &Frame);

        bool IsInImage(const float &x, const float &y, const cv::Mat image);

    public:
        Geometry();
        ~Geometry() = default;

        void GeometricModelCorrection(const ORB_SLAM3::Frame &currentFrame,
                                      cv::Mat &imDepth, cv::Mat &mask);

        // void InpaintFrames(const ORB_SLAM3::Frame &currentFrame,
        //                      cv::Mat &imGray, cv::Mat &imDepth, cv::Mat &imRGB, cv::Mat &mask);

        void GeometricModelUpdateDB(const ORB_SLAM3::Frame &mCurrentFrame);
    };
}

#endif //ORB_SLAM3_ROS_GEOMETRY_H
