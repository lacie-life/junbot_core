//
// Created by lacie on 28/01/2023.
//

#include "Geometry.h"
#include <algorithm>
#include "Frame.h"
#include "Tracking.h"

namespace semantic_slam
{

    Geometry::Geometry()
    {
        vAllPixels = cv::Mat(640*480,2,CV_32F);
        int m(0);
        for (int i(0); i < 640; i++){
            for (int j(0); j < 480; j++){
                vAllPixels.at<float>(m,0) = i;
                vAllPixels.at<float>(m,1) = j;
                m++;
            }
        }
    }

    /*
    void Geometry::GeometricModelCorrection(
                                        const semantic_slam::Frame &currentFrame,
                                        cv::Mat &imDepth,
                                        cv::Mat &mask)
    {
        if(currentFrame.mTcw.empty()){
            std::cout << "Geometry not working." << std::endl;
        }
        else if (mDB.mNumElem >= ELEM_INITIAL_MAP)// >5
        {
            vector<semantic_slam::Frame> vRefFrames = GetRefFrames(currentFrame);
            vector<DynKeyPoint> vDynPoints = ExtractDynPoints(vRefFrames,currentFrame);
            mask = DepthRegionGrowing(vDynPoints,imDepth);
            CombineMasks(currentFrame,mask);
        }
    }
    */

    void Geometry::GeometricModelCorrection(
            const semantic_slam::Frame &currentFrame,
            cv::Mat &imDepth,
            cv::Mat &mask)
    {
        cv::Mat _Tcw = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(currentFrame.GetPose()));
        if(_Tcw.empty()){
            std::cout << "Geometry not working." << std::endl;
        }
        else if (mDB.mNumElem >= ELEM_INITIAL_MAP)// >5
        {
            std::cout << "Geometry DB size ." << mDB.mNumElem << std::endl;
            vector<semantic_slam::Frame> vRefFrames = GetRefFrames(currentFrame);

            vector<DynKeyPoint> vDynPoints = ExtractDynPoints(vRefFrames,currentFrame);
            mask = DepthRegionGrowing(vDynPoints,imDepth);

            // CombineMasks(currentFrame,mask);
        }
    }

    void Geometry::GeometricModelUpdateDB(const semantic_slam::Frame &currentFrame)
    {
        if (currentFrame.mIsKeyFrame)
        {
            mDB.InsertFrame2DB(currentFrame);
        }
    }

    vector<semantic_slam::Frame> Geometry::GetRefFrames(const semantic_slam::Frame &currentFrame)
    {
        cv::Mat _Tcw = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(currentFrame.GetPose()));

        cv::Mat rot1      = _Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat eul1      = rotm2euler(rot1);
        cv::Mat trans1 = _Tcw.rowRange(0,3).col(3);
        cv::Mat vDist;
        cv::Mat vRot;

        for (int i(0); i < mDB.mNumElem; i++)
        {
            cv::Mat _Tcw = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(mDB.mvDataBase[i].GetPose()));

            cv::Mat rot2 = _Tcw.rowRange(0,3).colRange(0,3);
            cv::Mat eul2 = rotm2euler(rot2);
            double distRot = cv::norm(eul2,eul1,cv::NORM_L2);
            vRot.push_back(distRot);

            cv::Mat trans2 = _Tcw.rowRange(0,3).col(3);
            double dist = cv::norm(trans2,trans1,cv::NORM_L2);
            vDist.push_back(dist);
        }

        double minvDist, maxvDist;
        cv::minMaxLoc(vDist, &minvDist, &maxvDist);
        vDist /= maxvDist;

        double minvRot, maxvRot;
        cv::minMaxLoc(vRot, &minvRot, &maxvRot);
        vRot /= maxvRot;

        vDist = 0.7*vDist + 0.3*vRot;
        cv::Mat vIndex;
        cv::sortIdx(vDist,vIndex,cv::SORT_EVERY_COLUMN + cv::SORT_DESCENDING);

        mnRefFrames = std::min(MAX_REF_FRAMES,vDist.rows);

        vector<semantic_slam::Frame> vRefFrames;

        for (int i(0); i < mnRefFrames; i++)
        {
            int ind = vIndex.at<int>(0,i);
            vRefFrames.push_back(mDB.mvDataBase[ind]);
        }
        return(vRefFrames);
    }

    // Extract dynamic points ============== Is it possible to use optical flow to calculate dynamic points ===============
    // 1. Select the reference frame key point depth (0~6m), calculate the 3d point under the reference frame, and then transform it to the world coordinate system
    // 2. Keep the points whose angle between the current frame to the world point vector and the reference frame to the world point vector is less than 30, not too close
    // 3. Retain the world point and back-project it to the point with a depth value <7m in the current frame coordinate system
    // 4. Retain the world point and back-project it to the point in the enrichment plane (20～620 & 20～460=) in the pixel coordinate system of the current frame, and at this point, the depth of the current frame!=0
    // 5. According to the depth value of the projection point and the depth value of the current frame of the surrounding 20×20 domain points, filter out the depth value of the domain point with a smaller depth difference to update the depth value of the current frame
    // 6. The difference between the projected depth value of the point and the depth under the current frame of the feature point is too large, and the depth variance around the point is small, so it is determined that the point is a moving point
    vector<Geometry::DynKeyPoint> Geometry::ExtractDynPoints(
            const vector<semantic_slam::Frame> &vRefFrames,
            const semantic_slam::Frame &currentFrame)
    {

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = currentFrame.fx;
        K.at<float>(1,1) = currentFrame.fy;
        K.at<float>(0,2) = currentFrame.cx;
        K.at<float>(1,2) = currentFrame.cy;


        cv::Mat vAllMPw;
        cv::Mat vAllMatRefFrame;
        cv::Mat vAllLabels;
        cv::Mat vAllDepthRefFrame;

        for (int i(0); i < mnRefFrames; i++)
        {
            semantic_slam::Frame refFrame = vRefFrames[i];

            // Fill matrix with points
            cv::Mat matRefFrame(refFrame.N,3,CV_32F);
            cv::Mat matDepthRefFrame(refFrame.N,1,CV_32F);
            cv::Mat matInvDepthRefFrame(refFrame.N,1,CV_32F);
            cv::Mat vLabels(refFrame.N,1,CV_32F);
            int k(0);
            for(int j(0); j < refFrame.N; j++)
            {


                const cv::KeyPoint &kp = refFrame.mvKeys[j];
                const float &v = kp.pt.y;
                const float &u = kp.pt.x;
                const float d = refFrame.mImDepth.at<float>(v,u);
                if (d > 0 && d < 6)
                {
                    matRefFrame.at<float>(k,0) = refFrame.mvKeysUn[j].pt.x;
                    matRefFrame.at<float>(k,1) = refFrame.mvKeysUn[j].pt.y;
                    matRefFrame.at<float>(k,2) = 1.;
                    matInvDepthRefFrame.at<float>(k,0) = 1./d;
                    matDepthRefFrame.at<float>(k,0) = d;
                    vLabels.at<float>(k,0) = i;
                    k++;// k<= N
                }
            }

            matRefFrame = matRefFrame.rowRange(0,k);
            matInvDepthRefFrame = matInvDepthRefFrame.rowRange(0,k);
            matDepthRefFrame = matDepthRefFrame.rowRange(0,k);
            vLabels = vLabels.rowRange(0,k);
            cv::Mat vMPRefFrame = K.inv()*matRefFrame.t();
            // K = [fx, 0,  cx]
            //     [0,  fy, cy]
            //     [0,  0,  1 ]

            //K.inv() =
            //        [1/fx, 0,  -cx/fx]
            //        [0,  1/fy,  -cy/fy]
            //        [0,   0,       1 ]

            // vconcat（B,C，A）;
            cv::vconcat(vMPRefFrame,matInvDepthRefFrame.t(),vMPRefFrame);

            cv::Mat _RefTcw = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(refFrame.GetPose()));
            cv::Mat _CurTcw = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(currentFrame.GetPose()));

            cv::Mat vMPw = _RefTcw.inv() * vMPRefFrame;


            cv::Mat _vMPw = cv::Mat(4,vMPw.cols,CV_32F);
            cv::Mat _vLabels = cv::Mat(vLabels.rows,1,CV_32F);
            cv::Mat _matRefFrame = cv::Mat(matRefFrame.rows,3,CV_32F);
            cv::Mat _matDepthRefFrame = cv::Mat(matDepthRefFrame.rows,1,CV_32F);

            int h(0);
            mParallaxThreshold = 30;
            for (int j(0); j < k; j++)
            {
                cv::Mat mp = cv::Mat(3,1,CV_32F);
                mp.at<float>(0,0) = vMPw.at<float>(0,j)/matInvDepthRefFrame.at<float>(0,j);// X
                mp.at<float>(1,0) = vMPw.at<float>(1,j)/matInvDepthRefFrame.at<float>(0,j);// Y
                mp.at<float>(2,0) = vMPw.at<float>(2,j)/matInvDepthRefFrame.at<float>(0,j);// Z
                cv::Mat tRefFrame = _RefTcw.rowRange(0,3).col(3);

                cv::Mat tCurrentFrame = _CurTcw.rowRange(0,3).col(3);
                cv::Mat nMPRefFrame = mp - tRefFrame;
                cv::Mat nMPCurrentFrame = mp - tCurrentFrame;

                double dotProduct = nMPRefFrame.dot(nMPCurrentFrame);// ||A|| × ||B||×cos()
                double normMPRefFrame = cv::norm(nMPRefFrame,cv::NORM_L2);// ||A||
                double normMPCurrentFrame = cv::norm(nMPCurrentFrame,cv::NORM_L2);// ||B||
                double angle = acos(dotProduct/(normMPRefFrame*normMPCurrentFrame))*180/M_PI;
                if (angle < mParallaxThreshold)
                {
                    _vMPw.at<float>(0,h) = vMPw.at<float>(0,j);// X/Z
                    _vMPw.at<float>(1,h) = vMPw.at<float>(1,j);// Y/Z
                    _vMPw.at<float>(2,h) = vMPw.at<float>(2,j);// 1
                    _vMPw.at<float>(3,h) = vMPw.at<float>(3,j);// 1/Z
                    _vLabels.at<float>(h,0) = vLabels.at<float>(j,0);
                    _matRefFrame.at<float>(h,0) = matRefFrame.at<float>(j,0);// u
                    _matRefFrame.at<float>(h,1) = matRefFrame.at<float>(j,1);// v
                    _matRefFrame.at<float>(h,2) = matRefFrame.at<float>(j,2);// 1
                    _matDepthRefFrame.at<float>(h,0) = matDepthRefFrame.at<float>(j,0);// Z
                    h++;
                }
            }

            vMPw = _vMPw.colRange(0,h);       //    (X/Z，Y/Z，1，1/Z)
            vLabels = _vLabels.rowRange(0,h);//   h*1
            matRefFrame = _matRefFrame.rowRange(0,h);//   (u，v，1)   h*3
            matDepthRefFrame = _matDepthRefFrame.rowRange(0,h);// h*1

            if (vAllMPw.empty())
            {
                vAllMPw = vMPw;
                vAllMatRefFrame = matRefFrame;
                vAllLabels = vLabels;
                vAllDepthRefFrame = matDepthRefFrame;
            }
            else
            {
                if (!vMPw.empty())
                {
                    hconcat(vAllMPw,vMPw,vAllMPw);
                    vconcat(vAllMatRefFrame,matRefFrame,vAllMatRefFrame);
                    vconcat(vAllLabels,vLabels,vAllLabels);
                    vconcat(vAllDepthRefFrame,matDepthRefFrame,vAllDepthRefFrame);
                }
            }
        }

        cv::Mat vLabels = vAllLabels;//  M×1 ==

        if (!vAllMPw.empty())
        {
            cv::Mat temp = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(currentFrame.GetPose()));
            cv::Mat vMPCurrentFrame = temp * vAllMPw;

            // Divide by last column   (X/Z，Y/Z，1，1/Z) ----> (X,Y,Z,1)
            for (int i(0); i < vMPCurrentFrame.cols; i++)// 4×M  (X/Z，Y/Z，1，1/Z)
            {
                vMPCurrentFrame.at<float>(0,i) /= vMPCurrentFrame.at<float>(3,i);
                vMPCurrentFrame.at<float>(1,i) /= vMPCurrentFrame.at<float>(3,i);
                vMPCurrentFrame.at<float>(2,i) /= vMPCurrentFrame.at<float>(3,i);
                vMPCurrentFrame.at<float>(3,i) /= vMPCurrentFrame.at<float>(3,i);
            }
            cv::Mat matProjDepth = vMPCurrentFrame.row(2);

            cv::Mat _vMPCurrentFrame = cv::Mat(vMPCurrentFrame.size(),CV_32F);

            cv::Mat _vAllMatRefFrame = cv::Mat(vAllMatRefFrame.size(),CV_32F);

            cv::Mat _vLabels = cv::Mat(vLabels.size(),CV_32F);

            cv::Mat __vAllDepthRefFrame = cv::Mat(vAllDepthRefFrame.size(),CV_32F);

            int h(0);
            cv::Mat __matProjDepth = cv::Mat(matProjDepth.size(),CV_32F);
            for (int i(0); i < matProjDepth.cols; i++)
            {
                if (matProjDepth.at<float>(0,i) < 7)
                {
                    __matProjDepth.at<float>(0,h) = matProjDepth.at<float>(0,i);

                    _vMPCurrentFrame.at<float>(0,h) = vMPCurrentFrame.at<float>(0,i);

                    _vMPCurrentFrame.at<float>(1,h) = vMPCurrentFrame.at<float>(1,i);
                    _vMPCurrentFrame.at<float>(2,h) = vMPCurrentFrame.at<float>(2,i);
                    _vMPCurrentFrame.at<float>(3,h) = vMPCurrentFrame.at<float>(3,i);

                    _vAllMatRefFrame.at<float>(h,0) = vAllMatRefFrame.at<float>(i,0);

                    _vAllMatRefFrame.at<float>(h,1) = vAllMatRefFrame.at<float>(i,1);
                    _vAllMatRefFrame.at<float>(h,2) = vAllMatRefFrame.at<float>(i,2);

                    _vLabels.at<float>(h,0) = vLabels.at<float>(i,0);

                    __vAllDepthRefFrame.at<float>(h,0) = vAllDepthRefFrame.at<float>(i,0);

                    h++;
                }
            }

            matProjDepth = __matProjDepth.colRange(0,h);
            vMPCurrentFrame = _vMPCurrentFrame.colRange(0,h);

            vAllMatRefFrame = _vAllMatRefFrame.rowRange(0,h);
            vLabels = _vLabels.rowRange(0,h);
            vAllDepthRefFrame = __vAllDepthRefFrame.rowRange(0,h);

            cv::Mat aux;
            cv::hconcat(cv::Mat::eye(3,3,CV_32F),cv::Mat::zeros(3,1,CV_32F),aux);
            //
            //  [1 0 0 0]
            //  [0 1 0 0]
            //  [0 0 1 0]
            cv::Mat matCurrentFrame = K*aux*vMPCurrentFrame;

            cv::Mat mat2CurrentFrame(matCurrentFrame.cols,2,CV_32F);
            cv::Mat v2AllMatRefFrame(matCurrentFrame.cols,3,CV_32F);
            cv::Mat mat2ProjDepth(matCurrentFrame.cols,1,CV_32F);
            cv::Mat v2Labels(matCurrentFrame.cols,1,CV_32F);
            cv::Mat _vAllDepthRefFrame(matCurrentFrame.cols,1,CV_32F);

            int j = 0;
            for (int i(0); i < matCurrentFrame.cols; i++)
            {
                float x = ceil(matCurrentFrame.at<float>(0,i)/matCurrentFrame.at<float>(2,i));//  (U,V,W) ----->(U/W,V/W,1)---->(u,v)
                float y = ceil(matCurrentFrame.at<float>(1,i)/matCurrentFrame.at<float>(2,i));
                if (IsInFrame(x,y,currentFrame))//   20～620 & 20～460
                {
                    const float d = currentFrame.mImDepth.at<float>(y,x);
                    if (d > 0)
                    {
                        mat2CurrentFrame.at<float>(j,0) = x;
                        mat2CurrentFrame.at<float>(j,1) = y;
                        v2AllMatRefFrame.at<float>(j,0) = vAllMatRefFrame.at<float>(i,0);
                        v2AllMatRefFrame.at<float>(j,1) = vAllMatRefFrame.at<float>(i,1);
                        v2AllMatRefFrame.at<float>(j,2) = vAllMatRefFrame.at<float>(i,2);
                        _vAllDepthRefFrame.at<float>(j,0) = vAllDepthRefFrame.at<float>(i,0);
                        float d1 = matProjDepth.at<float>(0,i);
                        mat2ProjDepth.at<float>(j,0) = d1;
                        v2Labels.at<float>(j,0) = vLabels.at<float>(i,0);
                        j++;
                    }
                }
            }

            vAllDepthRefFrame = _vAllDepthRefFrame.rowRange(0,j);
            vAllMatRefFrame = v2AllMatRefFrame.rowRange(0,j);
            matProjDepth = mat2ProjDepth.rowRange(0,j);
            matCurrentFrame = mat2CurrentFrame.rowRange(0,j);
            vLabels = v2Labels.rowRange(0,j);

            cv::Mat u1((2*mDmax+1)*(2*mDmax+1),2,CV_32F); // mDmax = 20   IsInFrame()
            int m(0);
            for (int i(-mDmax); i <= mDmax; i++){    // -20～20
                for (int j(-mDmax); j <= mDmax; j++){// -20～20
                    u1.at<float>(m,0) = i;
                    u1.at<float>(m,1) = j;
                    m++;
                }
            }
            cv::Mat matDepthCurrentFrame(matCurrentFrame.rows,1,CV_32F);
            cv::Mat _matProjDepth(matCurrentFrame.rows,1,CV_32F);
            cv::Mat _matCurrentFrame(matCurrentFrame.rows,2,CV_32F);

            int _s(0);
            for (int i(0); i < matCurrentFrame.rows; i++)
            {
                int s(0);
                cv::Mat _matDiffDepth(u1.rows,1,CV_32F);
                cv::Mat _matDepth(u1.rows,1,CV_32F);
                for (int j(0); j < u1.rows; j++)
                {
                    int x = (int)matCurrentFrame.at<float>(i,0) + (int)u1.at<float>(j,0);
                    int y = (int)matCurrentFrame.at<float>(i,1) + (int)u1.at<float>(j,1);
                    float _d = currentFrame.mImDepth.at<float>(y,x);
                    if ((_d > 0) && (_d < matProjDepth.at<float>(i,0)))
                    {
                        _matDepth.at<float>(s,0) = _d;
                        _matDiffDepth.at<float>(s,0) = matProjDepth.at<float>(i,0) - _d;
                        s++;
                    }
                }

                if (s > 0)
                {
                    _matDepth = _matDepth.rowRange(0,s);
                    _matDiffDepth = _matDiffDepth.rowRange(0,s);
                    double minVal, maxVal;
                    cv::Point minIdx, maxIdx;
                    cv::minMaxLoc(_matDiffDepth,&minVal,&maxVal,&minIdx,&maxIdx);
                    int xIndex = minIdx.x;
                    int yIndex = minIdx.y;
                    matDepthCurrentFrame.at<float>(_s,0) = _matDepth.at<float>(yIndex,0);
                    _matProjDepth.at<float>(_s,0) = matProjDepth.at<float>(i,0);
                    _matCurrentFrame.at<float>(_s,0) = matCurrentFrame.at<float>(i,0);
                    _matCurrentFrame.at<float>(_s,1) = matCurrentFrame.at<float>(i,1);
                    _s++;
                }
            }
            matDepthCurrentFrame = matDepthCurrentFrame.rowRange(0,_s);
            matProjDepth = _matProjDepth.rowRange(0,_s);
            matCurrentFrame = _matCurrentFrame.rowRange(0,_s);

            mDepthThreshold = 0.6;
            cv::Mat matDepthDifference = matProjDepth - matDepthCurrentFrame;
            mVarThreshold = 0.001; //0.040;

            vector<Geometry::DynKeyPoint> vDynPoints;

            for (int i(0); i < matCurrentFrame.rows; i++)
            {
                if (matDepthDifference.at<float>(i,0) > mDepthThreshold)
                {
                    int xIni = (int)matCurrentFrame.at<float>(i,0) - mDmax;
                    int yIni = (int)matCurrentFrame.at<float>(i,1) - mDmax;
                    int xEnd = (int)matCurrentFrame.at<float>(i,0) + mDmax + 1;
                    int yEnd = (int)matCurrentFrame.at<float>(i,1) + mDmax + 1;
                    cv::Mat patch = currentFrame.mImDepth.rowRange(yIni,yEnd).colRange(xIni,xEnd);
                    cv::Mat mean, stddev;
                    cv::meanStdDev(patch,mean,stddev);
                    double _stddev = stddev.at<double>(0,0);
                    double var = _stddev*_stddev;
                    if (var < mVarThreshold)
                    {
                        DynKeyPoint dynPoint;
                        dynPoint.mPoint.x = matCurrentFrame.at<float>(i,0);
                        dynPoint.mPoint.y = matCurrentFrame.at<float>(i,1);
                        dynPoint.mRefFrameLabel = vLabels.at<float>(i,0);
                        vDynPoints.push_back(dynPoint);
                    }
                }
            }

            return vDynPoints;
        }
        else
        {
            vector<Geometry::DynKeyPoint> vDynPoints;
            return vDynPoints;
        }
    }


    cv::Mat Geometry::DepthRegionGrowing(
            const vector<DynKeyPoint> &vDynPoints,
            const cv::Mat &imDepth)
    {

        cv::Mat maskG = cv::Mat::zeros(480,640,CV_32F);

        if (!vDynPoints.empty())
        {
            mSegThreshold = 0.20;

            for (size_t i(0); i < vDynPoints.size(); i++)
            {

                int xSeed = vDynPoints[i].mPoint.x;
                int ySeed = vDynPoints[i].mPoint.y;

                const float d = imDepth.at<float>(ySeed,xSeed);
                if (maskG.at<float>(ySeed,xSeed) != 1. && d > 0)
                {

                    cv::Mat J = RegionGrowing(imDepth,xSeed,ySeed,mSegThreshold);
                    maskG = maskG | J;
                }
            }


            int dilation_size = 15;
            cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                   cv::Point( dilation_size, dilation_size ) );
            maskG.cv::Mat::convertTo(maskG,CV_8U);
            cv::dilate(maskG, maskG, kernel);
        }
        else
        {
            maskG.cv::Mat::convertTo(maskG,CV_8U);
        }

        cv::Mat _maskG = cv::Mat::ones(480,640,CV_8U);
        maskG = _maskG - maskG;// 1-mask

        return maskG;
    }

    float Area(float x1, float x2, float y1, float y2)
    {
        float xc1 = max(x1-0.5,x2-0.5);
        float xc2 = min(x1+0.5,x2+0.5);
        float yc1 = max(y1-0.5,y2-0.5);
        float yc2 = min(y1+0.5,y2+0.5);
        return (xc2-xc1)*(yc2-yc1);
    }

    void Geometry::DataBase::InsertFrame2DB(const semantic_slam::Frame &currentFrame)
    {

        if (!IsFull())
        {
            mvDataBase[mFin] = currentFrame;
            mFin = (mFin + 1) % MAX_DB_SIZE;
            mNumElem += 1;
        }
        else {
            mvDataBase[mIni] = currentFrame;
            mFin = mIni;
            mIni = (mIni + 1) % MAX_DB_SIZE;
        }
    }

    bool Geometry::DataBase::IsFull(){
        return (mIni == (mFin+1) % MAX_DB_SIZE);
    }

    cv::Mat Geometry::rotm2euler(const cv::Mat &R)
    {
        assert(isRotationMatrix(R));
        float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
        bool singular = sy < 1e-6;
        float x, y, z;
        if (!singular)
        {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        }
        else
        {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }
        cv::Mat res = (cv::Mat_<double>(1,3) << x, y, z);
        return res;
    }

    bool Geometry::isRotationMatrix(const cv::Mat &R)
    {
        cv::Mat Rt;
        transpose(R,Rt);
        cv::Mat shouldBeIdentity = Rt*R;
        cv::Mat I = cv::Mat::eye(3,3,shouldBeIdentity.type());
        return norm(I,shouldBeIdentity) < 1e-6;
    }

    bool Geometry::IsInFrame(
            const float &x, const float &y,
            const semantic_slam::Frame &Frame)
    {
        mDmax = 20;
        return (x > (mDmax + 1) && x < (Frame.mImDepth.cols - mDmax - 1) && y > (mDmax + 1) && y < (Frame.mImDepth.rows - mDmax - 1));
    }

    bool Geometry::IsInImage(
            const float &x, const float &y,
            const cv::Mat image)
    {
        return (x >= 0 && x < (image.cols) && y >= 0 && y < image.rows);
    }


    cv::Mat Geometry::RegionGrowing(
            const cv::Mat &im,
            int &x,int &y,
            const float &reg_maxdist)
    {

        cv::Mat J = cv::Mat::zeros(im.size(),CV_32F);

        float reg_mean = im.at<float>(y,x);
        int reg_size = 1;

        int _neg_free = 10000;
        int neg_free = 10000;
        int neg_pos = -1;
        cv::Mat neg_list = cv::Mat::zeros(neg_free,3,CV_32F);

        double pixdist=0;

        //Neighbor locations (footprint)
        cv::Mat neigb(4,2,CV_32F);
        neigb.at<float>(0,0) = -1;
        neigb.at<float>(0,1) = 0;

        neigb.at<float>(1,0) = 1;
        neigb.at<float>(1,1) = 0;

        neigb.at<float>(2,0) = 0;
        neigb.at<float>(2,1) = -1;

        neigb.at<float>(3,0) = 0;
        neigb.at<float>(3,1) = 1;

        while(pixdist < reg_maxdist && reg_size < im.total())
        {
            for (int j(0); j< 4; j++)
            {
                //Calculate the neighbour coordinate
                int xn = x + neigb.at<float>(j,0);
                int yn = y + neigb.at<float>(j,1);

                bool ins = ((xn >= 0) && (yn >= 0) && (xn < im.cols) && (yn < im.rows));
                if (ins && (J.at<float>(yn,xn) == 0.))
                {
                    neg_pos ++;
                    neg_list.at<float>(neg_pos,0) = xn;
                    neg_list.at<float>(neg_pos,1) = yn;
                    neg_list.at<float>(neg_pos,2) = im.at<float>(yn,xn);
                    J.at<float>(yn,xn) = 1.;
                }
            }

            // Add a new block of free memory
            if((neg_pos + 10) > neg_free){
                cv::Mat _neg_list = cv::Mat::zeros(_neg_free,3,CV_32F);
                neg_free += 10000;
                vconcat(neg_list,_neg_list,neg_list);
            }

            // Add pixel with intensity nearest to the mean of the region, to the region
            cv::Mat dist;
            for (int i(0); i < neg_pos; i++){
                double d = abs(neg_list.at<float>(i,2) - reg_mean);
                dist.push_back(d);
            }
            double max;
            cv::Point ind, maxpos;
            cv::minMaxLoc(dist, &pixdist, &max, &ind, &maxpos);
            int index = ind.y;

            if (index != -1)
            {
                J.at<float>(y,x) = -1.;
                reg_size += 1;

                // Calculate the new mean of the region
                reg_mean = (reg_mean*reg_size + neg_list.at<float>(index,2))/(reg_size+1);

                // Save the x and y coordinates of the pixel (for the neighbour add proccess)
                x = neg_list.at<float>(index,0);
                y = neg_list.at<float>(index,1);

                // Remove the pixel from the neighbour (check) list
                neg_list.at<float>(index,0) = neg_list.at<float>(neg_pos,0);
                neg_list.at<float>(index,1) = neg_list.at<float>(neg_pos,1);
                neg_list.at<float>(index,2) = neg_list.at<float>(neg_pos,2);
                neg_pos -= 1;
            }
            else
            {
                pixdist = reg_maxdist;
            }

        }

        J = cv::abs(J);
        return(J);
    }
}
