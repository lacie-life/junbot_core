/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM3
{

FrameDrawer::FrameDrawer(Atlas* pAtlas):both(false),mpAtlas(pAtlas)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mImRight = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}


FrameDrawer::FrameDrawer(Atlas* pAtlas, MapDrawer* pMapDrawer, const string &strSettingPath):
                     mpAtlas(pAtlas),
                     mpMapDrawer(pMapDrawer)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    std::cout << "FrameDrawer Init \n";
}

cv::Mat FrameDrawer::DrawFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state
    vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0,255,0);
    cv::Scalar odometryColor(255,0,0);

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im,pt1,pt2,standardColor);
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
        {
            cv::Point2f pt1,pt2;
            if(imageScale != 1.f)
            {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            }
            else
            {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im,pt1,pt2, standardColor,5);
        }

    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = vCurrentKeys[i].pt / imageScale;
                    float px = vCurrentKeys[i].pt.x / imageScale;
                    float py = vCurrentKeys[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = vCurrentKeys[i].pt;
                    pt1.x=vCurrentKeys[i].pt.x-r;
                    pt1.y=vCurrentKeys[i].pt.y-r;
                    pt2.x=vCurrentKeys[i].pt.x+r;
                    pt2.y=vCurrentKeys[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,standardColor);
                    cv::circle(im,point,2,standardColor,-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,odometryColor);
                    cv::circle(im,point,2,odometryColor,-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

cv::Mat FrameDrawer::DrawRightFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mImRight.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeysRight;
        }
    } // destroy scoped mutex -> release mutex

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }

                cv::line(im,pt1,pt2,cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for(int i=0;i<n;i++)
        {
            if(vbVO[i + Nleft] || vbMap[i + Nleft])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = mvCurrentKeysRight[i].pt / imageScale;
                    float px = mvCurrentKeysRight[i].pt.x / imageScale;
                    float py = mvCurrentKeysRight[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = mvCurrentKeysRight[i].pt;
                    pt1.x=mvCurrentKeysRight[i].pt.x-r;
                    pt1.y=mvCurrentKeysRight[i].pt.y-r;
                    pt2.x=mvCurrentKeysRight[i].pt.x+r;
                    pt2.y=mvCurrentKeysRight[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i + Nleft])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,point,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,point,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::generatePC(void)
{
    cv::Mat Depth,ImRGB,Tcw;
    {
     unique_lock<mutex> lock(mMutex);
     mImDep.copyTo(Depth);
     mImRGB.copyTo(ImRGB);
     mTcw.copyTo(Tcw); 
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    int n,m;
    for ( m=0; m < Depth.rows; m+=1 )
    {
          for ( n=0; n < Depth.cols; n+=1 )
          {
              float d = Depth.ptr<float>(m)[n];
              if (d < 0.01 || d>2.0)
                 continue;
              pcl::PointXYZRGB p;
              p.z = d;
              p.x = ( n - mK.at<float>(0,0)) * p.z / mK.at<float>(0,2);
              p.y = ( m - mK.at<float>(1,1)) * p.z / mK.at<float>(1,2);
              if(p.y<-3.0 || p.y>3.0) continue; 
              p.b = ImRGB.ptr<uchar>(m)[n*3+0];
              p.g = ImRGB.ptr<uchar>(m)[n*3+1];
              p.r = ImRGB.ptr<uchar>(m)[n*3+2];
              cloud->points.push_back( p );
          }
    }

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01,0.01, 0.01);
    vg.filter(*cloud);

    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(Tcw);
    pcl::PointCloud<pcl::PointXYZRGB> temp;
    pcl::transformPointCloud( *cloud, temp, T.inverse().matrix());
    mpMapDrawer->RegisterObs(temp);
}



void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mThDepth = pTracker->mCurrentFrame.mThDepth;
    mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

    if(both){
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    }
    else{
        N = mvCurrentKeys.size();
    }

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    //Variables for the new visualization
    mCurrentFrame = pTracker->mCurrentFrame;
    mmProjectPoints = mCurrentFrame.mmProjectPoints;
    mmMatchedInImage.clear();

    mvpLocalMap = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;

                    mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
                }
                else
                {
                    mvpOutlierMPs.push_back(pMP);
                    mvOutlierKeys.push_back(mvCurrentKeys[i]);
                }
            }
        }

    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
