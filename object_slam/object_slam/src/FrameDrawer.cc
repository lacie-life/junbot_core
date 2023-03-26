
#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace semantic_slam
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
                    bool bInBox = false;
                    
                    // For 3D cuboid testing
                    // if(mbshow_yolo_result)
                    {
                        for (auto&box : Dboxes)
                        {
                            int left = box.x;
                            int right = box.x+box.width;
                            int top = box.y;
                            int bottom = box.y+box.height;

                            if((vCurrentKeys[i].pt.x > left)&&(vCurrentKeys[i].pt.x < right)
                                &&(vCurrentKeys[i].pt.y > top)&&(vCurrentKeys[i].pt.y < bottom))
                            {
                                cv::circle(im, vCurrentKeys[i].pt, 2, colors[box.m_class%6], -1);
                                bInBox = true;
                                break;
                            }
                        }
                    }
                    if(bInBox == false)
                    {
                        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,255),-1);
                    }

                    // cv::rectangle(im,pt1,pt2,standardColor);
                    // cv::circle(im,point,2,standardColor,-1);
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
    // DrawTextInfo(im,state, imWithInfo);

    imWithInfo = im.clone();
    DrawYoloInfo(imWithInfo, true);

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

    Eigen::Isometry3d T = semantic_slam::Converter::toSE3Quat(Tcw);

//    std::cout << "Trans Cam to World: " << Tcw << endl;

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

    pTracker->mImGray.copyTo(mIm);
    pTracker->mImMask.copyTo(mDynMask);
    pTracker->mImDepth.copyTo(mImDep);
    pTracker->mImRGB.copyTo(mImRGB);

    cv::Mat _Tcw = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(pTracker->mCurrentFrame.GetPose()));
    _Tcw.copyTo(mTcw);

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

    // For 3D cuboid testing
    // key line
    Dkeylines_raw_nouse = pTracker->mCurrentFrame.keylines_raw;
    Dkeylines_out_nouse = pTracker->mCurrentFrame.keylines_out;
    DTimeStamp_nouse = pTracker->mCurrentFrame.mTimeStamp;
    DObjsLines = pTracker->mCurrentFrame.vObjsLines;// lines in objects

    // colo image
    //pTracker->mCurrentFrame.mColorImage.copyTo(mRGBIm);
    pTracker->mCurrentFrame.mQuadricImage.copyTo(mQuadricIm);
    Dboxes = pTracker->mCurrentFrame.boxes;
    this->mTcw = Converter::toCvMat(pTracker->mCurrentFrame.GetPose());
    this->mK = pTracker->mCurrentFrame.mK;

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

// For 3D cuboid testing
cv::Mat FrameDrawer::DrawQuadricImage()
{

    cv::Mat imRGB = mQuadricIm.clone();
    // Projection Matrix K[R|t].  Camera frame to world change relationship
    cv::Mat Pcw(3, 4, CV_32F);
    if(mTcw.size().height!=4){
        std::cout << "Camera pose is empty" << std::endl;
        return imRGB;
    }
    const cv::Mat Rcw = mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mTcw.rowRange(0, 3).col(3);
    Rcw.copyTo(Pcw.rowRange(0, 3).colRange(0, 3));
    tcw.copyTo(Pcw.rowRange(0, 3).col(3));
    cv::Mat Pfw(3, 4, CV_32F);
    Pfw = mK * Pcw;

    Map* mpMap = mpAtlas->GetCurrentMap();

    const std::vector<Object_Map*> obj_3ds_new = mpMap->GetObjects();

    // std::cout << "Obj 3D in Map: " << obj_3ds_new.size() << "\n";

    for(int i = (int)obj_3ds_new.size() - 1; i >= 0; i--)
    {

        //cv::Mat DrawQuadricProject( cv::Mat &im,
        //                        const cv::Mat &P,
        //                        const cv::Mat &axe,
        //                        const cv::Mat &Twq,
        //                        int nClassid,
        //                        bool isGT=true,
        //                        int nLatitudeNum = 7,
        //                        int nLongitudeNum = 6);

        //ColorImage = DrawQuadricProject(this->mCurrentFrame.mQuadricImage,
        //                                    P,
        //                                    axe,
        //                                    Twq,
        //                                    obj_3ds_new[i]->mnClass);
        Object_Map* obj = obj_3ds_new[i];
        // Only display the objects in the past 20 frames, it will not be confusing
        if((obj->mnLastAddID + 20) < CurFrameId)
            continue;
        if(obj->bad_3d)
            continue;

        // Size
        cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
        axe.at<float>(0) = obj->mCuboid3D.lenth / 2;
        axe.at<float>(1) = obj->mCuboid3D.width / 2;
        axe.at<float>(2) = obj->mCuboid3D.height / 2;

        // object pose (world).
        cv::Mat Twq = obj->mCuboid3D.pose_mat;//Converter::toCvMat(obj_3ds_new[i]->mCuboid3D.pose);

        // draw params
        cv::Scalar sc = colors[ obj->mnClass % 6];
        int nLatitudeNum = 7, nLongitudeNum = 6;
        bool isGT=true;
        int nLineWidth = 2;

        // generate angluar grid -> xyz grid (vertical half sphere)
        vector<float> vfAngularLatitude;  // (-90, 90)
        vector<float> vfAngularLongitude; // [0, 180]
        cv::Mat pointGrid(nLatitudeNum + 2, nLongitudeNum + 1, CV_32FC4);

        for (int i = 0; i < nLatitudeNum + 2; i++)
        {
            float fThetaLatitude = -M_PI_2 + i * M_PI / (nLatitudeNum + 1);
            cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
            for (int j = 0; j < nLongitudeNum + 1; j++)
            {
                float fThetaLongitude = j * M_PI / nLongitudeNum;
                p[j][0] = axe.at<float>(0, 0) * cos(fThetaLatitude) * cos(fThetaLongitude);
                p[j][1] = axe.at<float>(1, 0) * cos(fThetaLatitude) * sin(fThetaLongitude);
                p[j][2] = axe.at<float>(2, 0) * sin(fThetaLatitude);
                p[j][3] = 1.;
            }
        }

        // draw latitude
        for (int i = 0; i < pointGrid.rows; i++)
        {
            cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
            // [0, 180]
            for (int j = 0; j < pointGrid.cols - 1; j++)
            {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = Pfw * Twq * spherePt0;
                cv::Mat conicPt1 = Pfw * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(imRGB, pt0, pt1, sc, nLineWidth); // [0, 180]
            }
            // [180, 360]
            for (int j = 0; j < pointGrid.cols - 1; j++)
            {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = Pfw * Twq * spherePt0;
                cv::Mat conicPt1 = Pfw * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(imRGB, pt0, pt1, sc, nLineWidth); // [180, 360]
            }
        }

        // draw longitude
        cv::Mat pointGrid_t = pointGrid.t();
        for (int i = 0; i < pointGrid_t.rows; i++)
        {
            cv::Vec4f *p = pointGrid_t.ptr<cv::Vec4f>(i);
            // [0, 180]
            for (int j = 0; j < pointGrid_t.cols - 1; j++)
            {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = Pfw * Twq * spherePt0;
                cv::Mat conicPt1 = Pfw * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(imRGB, pt0, pt1, sc, nLineWidth); // [0, 180]
            }
            // [180, 360]
            for (int j = 0; j < pointGrid_t.cols - 1; j++)
            {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = Pfw * Twq * spherePt0;
                cv::Mat conicPt1 = Pfw * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(imRGB, pt0, pt1, sc, nLineWidth); // [180, 360]
            }
        }
    }

    return imRGB;
}

cv::Mat FrameDrawer::GetQuadricImage()
{
    cv::Mat imRGB = DrawQuadricImage();

    return imRGB;
}

cv::Mat FrameDrawer::DrawYoloInfo(cv::Mat &im, bool bText)
{
    for (auto&box : Dboxes)
    {
        if(bText)
        {
            cv::putText(im,
                        class_names[box.m_class],
                        box.tl(),
                        cv::FONT_HERSHEY_DUPLEX  ,
                        1.0,
                        colors[box.m_class%4],
                        // cv::Scalar(0,255,0),
                        2);
        }

        // draw lines in the box
//        for(int obj_id = 0; obj_id < DObjsLines.size(); obj_id ++)
//        {
//            for(int line_id = 0; line_id < DObjsLines[obj_id].rows(); line_id++)
//            {
//                cv::Scalar lineColor;
//                int R = ( rand() % (int) ( 255 + 1 ) );
//                int G = ( rand() % (int) ( 255 + 1 ) );
//                int B = ( rand() % (int) ( 255 + 1 ) );
//                lineColor = cv::Scalar( R, G, B );
//
//                cv::line(   im,
//                            cv::Point2f( DObjsLines[obj_id](line_id, 0), DObjsLines[obj_id](line_id, 1)),
//                            cv::Point2f( DObjsLines[obj_id](line_id, 2), DObjsLines[obj_id](line_id, 3)),
////                            cv::Scalar( 255, 255, 0 ),
//                            lineColor,
//                            2.0);
//            }
//        }

        // draw bounding box.
        cv::rectangle(  im,
                        box,
                        colors[box.m_class%6],
                        2);

        // cv::putText( im, text, cv::Point(x, y + label_size.height),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.4, colors[box.m_class % 4], 1);
    }

    return im;
}

} //namespace semantic_slam
