/*
 *--------------------------------------------------------------------------------------------------
 * DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments
　*　Author(s):
 * Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei Qiao qiaofei@mail.tsinghua.edu.cn
 * Created by Yu Chao@2018.12.03
 * --------------------------------------------------------------------------------------------------
 * DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2. If you haven't learn ORB_SLAM2 code, 
 * you'd better to be familiar with ORB_SLAM2 project first. Compared to ORB_SLAM2, 
 * we add anther two threads including semantic segmentation thread and densemap creation thread. 
 * You should pay attention to Frame.cc, ORBmatcher.cc, Pointcloudmapping.cc and Segment.cc.
 * 
 *　@article{murORB2,
 *　title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
　*　author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
　* journal={IEEE Transactions on Robotics},
　*　volume={33},
　* number={5},
　* pages={1255--1262},
　* doi = {10.1109/TRO.2017.2705103},
　* year={2017}
 *　}
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2018, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/) and 
 * Advanced Mechanism and Roboticized Equipment Lab. All rights reserved.
 *
 * Licensed under the GPLv3 License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://github.com/ivipsourcecode/DS-SLAM/blob/master/LICENSE
 *--------------------------------------------------------------------------------------------------
 */

#include "PointCloudMapping.h"
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#define MAX_POINTCLOUD_DEPTH 10.0 // in meters

using namespace std;

ros::Publisher pclPub;
ros::Publisher fullMapPub;
sensor_msgs::PointCloud2 pclPoint;
sensor_msgs::PointCloud2 fullPCLPoint;

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize(resolution, resolution, resolution);
    this->sor.setMeanK(100);
    this->sor.setStddevMulThresh(0.1);
    globalMap = boost::make_shared<PointCloud>();
    viewerThread = boost::make_shared<thread>(bind(&PointCloudMapping::viewer, this));
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    std::cout << "There? \n";

    unique_lock<mutex> lck(keyframeMutex);
    
    keyframes.push_back(kf);
    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    keyFrameUpdated.notify_one();
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::generatePointCloud(KeyFrame *kf, cv::Mat color, cv::Mat depth)
{
    
    PointCloud::Ptr tmp(new PointCloud());

    // point cloud is null ptr
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d > MAX_POINTCLOUD_DEPTH)
                continue;

            PointT _p;
            _p.z = d;
            _p.x = (n - kf->cx) * _p.z * kf->invfx;
            _p.y = (m - kf->cy) * _p.z * kf->invfy;

            _p.b = color.ptr<uchar>(m)[n*3];
            _p.g = color.ptr<uchar>(m)[n*3+1];
            _p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(_p);
        }
    }

    std::cout << "Size: " << tmp->size() << std::endl;

    tmp->is_dense = false;

    return tmp;
}

void PointCloudMapping::generateAndPublishPointCloud(size_t N)
{
    Eigen::Matrix4d m;

    m << 0, 0, 1, 0,
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 0, 1;

    Eigen::Isometry3d axisTransform(m);

    for (size_t i = lastKeyframeSize; i < N; i++)
    {
        PointCloud::Ptr p = generatePointCloudWithDynamicObject(keyframes[i], colorImgs[i], depthImgs[i]);
        PointCloud::Ptr tmp1(new PointCloud());

        tmp1->resize(p->size());

        voxel.setInputCloud(p);
        voxel.filter(*tmp1);
        p->swap(*tmp1);

        // Convert to sensor_msgs 
        pcl::toROSMsg(*p, pclPoint);
        pclPoint.header.frame_id = "pointCloudFrame";
        Eigen::Isometry3d T = Converter::toSE3Quat(keyframes[i]->GetPose());
        broadcastTransformMat(T.inverse());

        pclPub.publish(pclPoint);

        // Merge full map
        PointCloud::Ptr _p(new PointCloud);
        _p->resize(p->size());
        pcl::transformPointCloud(*p, *_p, axisTransform * T.inverse().matrix());
        _p->is_dense = false;
        *globalMap += *_p;
    }

    pcl::toROSMsg(*globalMap, fullPCLPoint);
    fullPCLPoint.header.frame_id = "cameraToRobot";
    fullMapPub.publish(fullPCLPoint);

    lastKeyframeSize = N;
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::generatePointCloudWithDynamicObject(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr pointCloud_temp(new PointCloud);

    for (int v = 0; v < color.rows; v++)
    {
        for (int u = 0; u < color.cols; u++)
        {
            cv::Point2i pt(u, v);
            bool IsDynamic = false;
            for (auto area : kf->mvDynamicArea)
                if (area.contains(pt)) IsDynamic = true;
            if (!IsDynamic)
            {
                float d = depth.ptr<float>(v)[u];
                if (d<0.01 || d>10)
                    continue;
                PointT p;
                p.z = d;
                p.x = ( u - kf->cx) * p.z / kf->fx;
                p.y = ( v - kf->cy) * p.z / kf->fy;

                p.b = color.ptr<cv::Vec3b>(v)[u][0];
                p.g = color.ptr<cv::Vec3b>(v)[u][1];
                p.r = color.ptr<cv::Vec3b>(v)[u][2];
                pointCloud_temp->push_back(p);
            }
        }
    }
    std::cout << "Size: " << pointCloud_temp->size() << "\n";

    // Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());
    // PointCloud::Ptr pointCloud(new PointCloud);
    // pointCloud->resize(pointCloud_temp->size());
    // pcl::transformPointCloud(*pointCloud_temp, *pointCloud, T.inverse().matrix());

    pointCloud_temp->is_dense = false;

    return pointCloud_temp;
}

void PointCloudMapping::broadcastTransformMat(Eigen::Isometry3d cameraPose)
{
    // TODO: Need to check transform
    static tf::TransformBroadcaster transformBroadcaster;

    Eigen::Matrix4d m;

    m << 0, 0, 1, 0,
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 0, 1;

    Eigen::Isometry3d axisTransform(m);
    // Apply axis transformation to the camera pose
    Eigen::Isometry3d finalTransform = axisTransform * cameraPose;

    // Manually create the rotation and translation matrices based on the final transform
    // in the shape of 4x4 [R T]
    tf::Matrix3x3 rotationMat(
        finalTransform(0, 0), finalTransform(0, 1), finalTransform(0, 2),
        finalTransform(1, 0), finalTransform(1, 1), finalTransform(1, 2),
        finalTransform(2, 0), finalTransform(2, 1), finalTransform(2, 2));

    tf::Vector3 translationMat(
        finalTransform(0, 3), finalTransform(1, 3), finalTransform(2, 3));

    tf::Transform transform;
    transform.setOrigin(translationMat);
    transform.setBasis(rotationMat);

    std::cout << "Transform: " << finalTransform.matrix() << "\n"; 

    // Publish the transfrom with the parent frame = /cameraToRobot and create a new child frame pointCloudFrame
    transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "cameraToRobot", "pointCloudFrame"));
}

void PointCloudMapping::viewer()
{
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    pclPub = n->advertise<sensor_msgs::PointCloud2>("/slam_pointclouds", 100000);
    fullMapPub = n->advertise<sensor_msgs::PointCloud2>("/full_slam_pointclouds", 100000);

    while (ros::ok())
    {
        cout << "[PCL] Starting the PCL" << endl;
        {
            unique_lock<mutex> lckShutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lckKeyframeUpdated(keyFrameUpdateMutex);
            keyFrameUpdated.wait(lckKeyframeUpdated);
        }

        size_t N = 0;
        {
            unique_lock<mutex> lck(keyframeMutex);
            N = keyframes.size();
        }
        if (N == 0)
        {
            cout << "[PCL] Keyframes miss!" << endl;
            usleep(1000);
            continue;
        }

        pclThread = boost::make_shared<thread>(boost::bind(&PointCloudMapping::generateAndPublishPointCloud, this, _1), N);
        pclThread->join();
    }

    pcl::io::savePCDFile("result.pcd", *globalMap);
    std::cout << "Final point cloud saved!!!" << std::endl;
}
