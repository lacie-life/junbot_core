//
// Created by lacie on 11/02/23.
//

#include "Object.h"
#include "Converter.h"
#include <exception>

std::string WORK_SPACE_PATH = "/home/lacie/slam_ws/src/junbot_planner/object_slam/";
std::string yamlfile_object = "TUM2.yaml";
bool MotionIou_flag = true;
bool NoPara_flag = true;
bool ProIou_flag = true;
bool Ttest_flag = true;
bool iforest_flag = true;
bool little_mass_flag = false;
bool ProIou_only30_flag = true;

// TODO: Improve cube merging
namespace semantic_slam
{

mutex Object_2D::mGlobalMutex;  //crash bug
mutex Object_Map::mMutex_front_back;

bool debug_iou_view = 0;
Object_2D::Object_2D() {
//    std::cout << "Object_2D  construct 1   ";
//    std::cout << ">>>   End" << std::endl;
}

Object_2D::Object_2D(Map* Map, Frame* CurrentFrame, const BoxSE &box) {
//    std::cout << "Object_2D  construct 3   ";
    mclass_id = box.m_class;
    mScore = box.m_score;
    mleft = box.x;
    mright = box.x + box.width;
    mtop = box.y;
    mbottom = box.y + box.height;
    mWidth = box.width;
    mHeight = box.height;
    // 2d center.

    mBoxCenter_2d = cv::Point2f(box.x + box.width / 2, box.y + box.height / 2);
    // opencv Rect format.
    mBox_cvRect = cv::Rect(box.x, box.y, box.width, box.height);

    this->mpCurrentFrame = CurrentFrame;
    this->mpMap = Map;

    this->sum_pos_3d = cv::Mat::zeros(3, 1, CV_32F);
//    std::cout << ">>>   End" << std::endl;
}

void Object_2D::AddYoloBoxes(const BoxSE &box) {
    
}

void Object_2D::ComputeMeanAndDeviation()
{
    unique_lock<mutex> lock(mMutexPos);
    unique_lock<mutex> lock2(mMutexObjMapPoints);  
    // remove bad points.
    vector<MapPoint *>::iterator pMP;
    sum_pos_3d = cv::Mat::zeros(3,1,CV_32F);
    for (pMP = mvMapPonits.begin();
         pMP != mvMapPonits.end();)
    {
        cv::Mat pos = Converter::toCvMat((*pMP)->GetWorldPos());
        if ((*pMP)->isBad())
        {
            //sum_pos_3d -= pos;
            pMP = mvMapPonits.erase(pMP);
        }
        else{
            sum_pos_3d += pos;
            ++pMP;
        }
    }
    
    mPos_world = sum_pos_3d / (mvMapPonits.size());
    
    //float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    //size_t i = 0;
    //for (; i < mvMapPonits.size(); i++)
    //{
    //    MapPoint *pMP = mvMapPonits[i];
    //    if (pMP->isBad())
    //        continue;
    //
    //    cv::Mat pos = pMP->GetWorldPos();
    //    cv::Mat pos_ave = mPos_world;
    //
    //    sum_x2 += (pos.at<float>(0)-pos_ave.at<float>(0))     *   (pos.at<float>(0)-pos_ave.at<float>(0));
    //    sum_y2 += (pos.at<float>(1)-pos_ave.at<float>(1))     *   (pos.at<float>(1)-pos_ave.at<float>(1));
    //    sum_z2 += (pos.at<float>(2)-pos_ave.at<float>(2))     *   (pos.at<float>(2)-pos_ave.at<float>(2));
    //}
    //mStandar_x = sqrt(sum_x2 / i);
    //mStandar_y = sqrt(sum_y2 / i);
    //mStandar_z = sqrt(sum_z2 / i);
}

void Object_2D::RemoveOutlier_ByHeightorDepth()
{
    unique_lock<mutex> lock(mMutexObjMapPoints);
    const cv::Mat Rcw = cv::Mat::zeros(3,3,CV_32F);
    const cv::Mat tcw = cv::Mat::eye(3,1,CV_32F);

    cv::Mat Tcw_ = Converter::toCvMat(mpCurrentFrame->GetPose());

    Tcw_.rowRange(0, 3).colRange(0, 3).copyTo(Rcw);
    Tcw_.rowRange(0, 3).col(3).copyTo(tcw);

    // world -> camera.
    vector<float> x_c;
    vector<float> y_c;
    vector<float> z_c;
    for (size_t i = 0; i < mvMapPonits.size(); i++)
    {
        MapPoint *pMP = mvMapPonits[i];

        cv::Mat PointPosWorld = Converter::toCvMat(pMP->GetWorldPos());
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        x_c.push_back(PointPosCamera.at<float>(0));
        y_c.push_back(PointPosCamera.at<float>(1));
        z_c.push_back(PointPosCamera.at<float>(2));
    }

    // sort.
    sort(x_c.begin(), x_c.end());
    sort(y_c.begin(), y_c.end());
    sort(z_c.begin(), z_c.end());
    
    if ((z_c.size() / 4 <= 0) || (z_c.size() * 3 / 4 >= z_c.size() - 1))
        return;
    
    float Q1 = z_c[(int)(z_c.size() / 4)];
    float Q3 = z_c[(int)(z_c.size() * 3 / 4)];
    float IQR = Q3 - Q1;
    float min_th = Q1 - 1.5 * IQR; // no use
    float max_th = Q3 + 1.5 * IQR;

    vector<MapPoint *>::iterator pMP;
    for (pMP = mvMapPonits.begin();
         pMP != mvMapPonits.end();)
    {
        cv::Mat PointPosWorld = Converter::toCvMat((*pMP)->GetWorldPos());
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;
        float z = PointPosCamera.at<float>(2);
        
        if (z > max_th){
            pMP = mvMapPonits.erase(pMP);
        }
        else
            ++pMP;
    }
    
    //this->ComputeMeanAndDeviation();
}

void Object_2D::MergeTwo_Obj2D(Object_2D *Old_Object2D)
{
    unique_lock<mutex> lock(mMutexObjMapPoints);
    for (size_t m = 0; m < Old_Object2D->mvMapPonits.size(); ++m)
    {
        bool bNewPoint = true;

        MapPoint *pMPLast = Old_Object2D->mvMapPonits[m];
        cv::Mat PosLast = Converter::toCvMat(pMPLast->GetWorldPos());

        // whether a new points.
        for (size_t n = 0; n < this->mvMapPonits.size(); ++n)
        {
            MapPoint *pMPCurr = this->mvMapPonits[n];
            cv::Mat PosCurr = Converter::toCvMat(pMPCurr->GetWorldPos());

            if (cv::countNonZero(PosLast - PosCurr) == 0)
            {
                bNewPoint = false;   
                break;
            }
        }

        if (bNewPoint)
        {
            this->mvMapPonits.push_back(pMPLast);  
        }
    }
    //this->ComputeMeanAndDeviation();
}

int Object_2D::Object2D_DataAssociationWith_Object3D()  //cv::Mat &image
{
    std::cout << "Check if it is connected to the old object" << std::endl;

    const cv::Mat image = mpCurrentFrame->mColorImage.clone();
    const cv::Mat Rcw = cv::Mat::zeros(3,3,CV_32F);
    const cv::Mat tcw = cv::Mat::eye(3,1,CV_32F);

    cv::Mat Tcw_ = Converter::toCvMat(mpCurrentFrame->GetPose());

    Tcw_.rowRange(0, 3).colRange(0, 3).copyTo(Rcw);
    Tcw_.rowRange(0, 3).col(3).copyTo(tcw);

    cv::Rect RectCurrent = mBox_cvRect;     // object bounding box in current frame.
    cv::Rect RectPredict;                   // predicted bounding box according to last frame and next to last frame.
    cv::Rect RectProject;                   // bounding box constructed by projecting points.

    const vector<Object_Map*> ObjectMaps  = mpMap->GetObjects();

    float IouMax = 0;                           //
    bool bAssoByMotionIou = false;              // 
    int AssoObjId_byIou = -1;                   // the associated map object ID.
    int ObjID_IouMax = -1;                      // temporary variable. 
    float IouThreshold = 0.5;                   // IoU threshold.

    // ****************************************************
    //         STEP 1. Motion  IoU  association.          *
    // ****************************************************
    if(MotionIou_flag)
    {
        for (int i = 0; i < (int)ObjectMaps.size(); i++)
        {
            Object_Map* obj3d = ObjectMaps[i];

            if (mclass_id != obj3d->mnClass)
                continue;

            if (obj3d->bad_3d)
                continue;

            if ((mpCurrentFrame->mnId-1) == obj3d->mnLastAddID)
            {
                // step 1.1 predict object bounding box according to last frame and next to last frame.
                if ((mpCurrentFrame->mnId-2) == obj3d->mnLastLastAddID)
                {   // The following assumptions are based on: Near three frames, the detection frame of the same object moves the same on the image
                    // 0____ll___l____c
                    // c = l - ll + l = 2 * l - ll

                    // left-top.
                    float left_top_x = obj3d->mLastRect.x * 2 - obj3d->mLastLastRect.x;     // cv::Rect The x represents the x-coordinate of the upper left corner of the square
                    if (left_top_x < 0)
                        left_top_x = 0;
                    float left_top_y = obj3d->mLastRect.y * 2 - obj3d->mLastLastRect.y;     // cv::Rect The y represents the y coordinate of the upper left corner of the square
                    if (left_top_y < 0)
                        left_top_y = 0;

                    // right-bottom.
                    float right_down_x = (obj3d->mLastRect.x + obj3d->mLastRect.width) * 2 - (obj3d->mLastLastRect.x + obj3d->mLastLastRect.width);
                    if (left_top_x > image.cols)
                        right_down_x = image.cols;
                    float right_down_y = (obj3d->mLastRect.y + obj3d->mLastRect.height) * 2 - (obj3d->mLastLastRect.y + obj3d->mLastLastRect.height);
                    if (left_top_y > image.rows)
                        right_down_y = image.rows;

                    float width = right_down_x - left_top_x;
                    float height = right_down_y - left_top_y;

                    // predicted bounding box.
                    RectPredict = cv::Rect(left_top_x, left_top_y, width, height);

                    // If two consecutive frames are observed, increase the threshold.
                    IouThreshold = 0.6;
                }
                else
                {
                    // If it is not close to three frames, the object detection frame is considered to be the same as the previous frame
                    RectPredict = obj3d->mLastRect;
                }

                // step 1.2 compute IoU, record the max IoU and the map object ID.
                float Iou = Converter::bboxOverlapratio(RectCurrent, RectPredict);

                if(debug_iou_view){
                    cv::Mat mat_test = mpCurrentFrame->mColorImage.clone();
                    cv::Scalar color = (200, 0, 0);
                    cv::rectangle(mat_test, RectCurrent, (0, 0, 255), 2);
                    cv::rectangle(mat_test, RectPredict, (255, 0, 0), 2);
                    cv::putText(mat_test, std::to_string(Iou), cv::Point(0, 500), cv::FONT_HERSHEY_DUPLEX, 1.0,
                                (0, 255, 0), 2);
                    cv::resize(mat_test, mat_test, cv::Size(640 * 0.5, 480 * 0.5), 0, 0, cv::INTER_CUBIC);
                    cv::imshow("[MotionIou]", mat_test);
                }

                std::cout << "[MotionIou] iou:" << Iou << std::endl;

                if ((Iou > IouThreshold) && (Iou > IouMax))
                {
                    IouMax = Iou;
                    ObjID_IouMax = i;
                }

            }
        }

        // step 1.3 if the association is successful, update the map object.
        Object_Map* obj3d_IouMax = ObjectMaps[ObjID_IouMax];
        if ((IouMax > 0) && (ObjID_IouMax >= 0))
        {
            bool bFlag = obj3d_IouMax->UpdateToObject3D(this, *mpCurrentFrame, MotionIou);

            if (bFlag)
            {
                bAssoByMotionIou = true;              // associated by IoU.
                AssoObjId_byIou = ObjID_IouMax;     // associated map object id.
            }
        }
    }
    // Iou data association

    // *************************************************
    //      STEP 2. Nonparametric data association     *
    // *************************************************
    bool bAssoByNp = false;
    int  AssoObjId_byNP  = -1; //;nAssoByNPId
    vector<int> vAssoObjIds_byNP;     // potential associated objects.

    if(NoPara_flag)
    {
        for (int i = (int)ObjectMaps.size() - 1; (i >= 0) ; i--)
        {
            Object_Map* obj3d = ObjectMaps[i];

            if (mclass_id != obj3d->mnClass)
                continue;

            if (obj3d->bad_3d)
                continue;

            // step 2.1 nonparametric test.
            int NoParaFlag = this->NoParaDataAssociation(obj3d);  

            if (NoParaFlag == 0) // 0: skip the nonparametric test and continue with the subsequent t-test.
                break;
            if (NoParaFlag == 2) // 2: association failed, compare next object.
                continue;
            else if (NoParaFlag == 1) // 1: association succeeded, but there may be more than one.
                vAssoObjIds_byNP.push_back(i);
        }

        // step 2.2 update data association and record potential associated objects.
        if (vAssoObjIds_byNP.size() >= 1)
        {
            // case 1: if associated by IoU, the objects judged by nonparametric-test are marked as potential association objects.
            if (bAssoByMotionIou)
            {
                Object_Map* obj3d_IOU = ObjectMaps[AssoObjId_byIou];
                
                for (int i = 0; i < vAssoObjIds_byNP.size(); i++)
                {
                    AssoObjId_byNP = vAssoObjIds_byNP[i];

                    
                    if (AssoObjId_byNP == AssoObjId_byIou)
                        continue;

                    // Record potential association objects and potential association times.
                    AddPotentialAssociatedObjects(ObjectMaps, AssoObjId_byIou  ,AssoObjId_byNP);
                }
            }

            // case 2: if association failed by IoU,
            else
            {
                for (int i = 0; i < vAssoObjIds_byNP.size(); i++)
                {
                    
                    AssoObjId_byNP = vAssoObjIds_byNP[i];
                    bool bFlag = ObjectMaps[AssoObjId_byNP]->UpdateToObject3D(this, *mpCurrentFrame, NoPara);

                    // if association successful, other objects are marked as potential association objects.
                    if (bFlag)
                    {
                        bAssoByNp = true;               // associated by NP.
                        AssoObjId_byNP = AssoObjId_byNP;    // associated map object id.

                        if (vAssoObjIds_byNP.size() > i + 1) 
                        {
                            for (int j = i + 1; j < vAssoObjIds_byNP.size(); j++)
                            {
                                int AssoObjId_byNP_2 = vAssoObjIds_byNP[j];
                                // Record potential association objects and potential association times.
                                AddPotentialAssociatedObjects(ObjectMaps, AssoObjId_byNP, AssoObjId_byNP_2);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
    // Nonparametric data association


    // ****************************************************
    //         STEP 3. Projected box data association     *
    // ****************************************************
    bool bAssoByProjectIou = false;
    int MaxAssoObjId_byProIou = -1;  //int nAssoByProId = -1;
    vector<int> vAssoObjIds_byProIou;

    if(ProIou_flag)
    {
        float fIouMax = 0.0;

        for (int i = (int)ObjectMaps.size() - 1; i >= 0; i--)
        {
            Object_Map* obj3D = ObjectMaps[i];
            if (mclass_id != obj3D->mnClass){
                continue;
            }

            if (obj3D->bad_3d){
                continue;
            }

            if(little_mass_flag)
            {
                int df = (int) obj3D->mvObject_2ds.size();
                if ((mvMapPonits.size() >= 10) && (df > 8)) {
                    continue;
                }
            }

            // step 3.1 compute IoU with bounding box constructed by projecting points.
            float fIou = Converter::bboxOverlapratio(RectCurrent, obj3D->mRect_byProjectPoints);
            float fIou2 = Converter::bboxOverlapratio(mBox_cvRect_FeaturePoints, obj3D->mRect_byProjectPoints);
            fIou = max(fIou, fIou2);

            // visualization for debug
            if(debug_iou_view)
            {
                cv::Mat mat_test = mpCurrentFrame->mColorImage.clone();
                cv::Scalar color = (200, 0, 0);
                cv::rectangle(mat_test, RectCurrent, cv::Scalar(255, 0, 0), 2);
                cv::rectangle(mat_test, mBox_cvRect_FeaturePoints, cv::Scalar(0, 255, 0), 2);  
                cv::rectangle(mat_test, obj3D->mRect_byProjectPoints, cv::Scalar(0, 0, 255), 2);
                cv::putText(mat_test, std::to_string(fIou), cv::Point(0, 500), cv::FONT_HERSHEY_DUPLEX, 1.0,
                            (0, 255, 0), 2);
                cv::resize(mat_test, mat_test, cv::Size(640 * 0.5, 480 * 0.5), 0, 0, cv::INTER_CUBIC);
                cv::imshow("[ProIou]", mat_test);
                std::cout << "[ProIou] " << fIou << " iou1:"
                          << Converter::bboxOverlapratio(RectCurrent, obj3D->mRect_byProjectPoints) << ", iou2:"
                          << Converter::bboxOverlapratio(mBox_cvRect_FeaturePoints, obj3D->mRect_byProjectPoints)
                          << std::endl;
            }
            // record the max IoU and map object id.
            if ((fIou >= 0.25) && (fIou > fIouMax))
            {
                fIouMax = fIou;
                MaxAssoObjId_byProIou = i;   //AssoObjId_byProIou  ProIouMaxObjId = i;
                vAssoObjIds_byProIou.push_back(i);
            }

        }
        // step 3.2 update data association and record potential associated objects.
        if (fIouMax >= 0.25)
        {
            sort(vAssoObjIds_byProIou.begin(), vAssoObjIds_byProIou.end());

            if (bAssoByMotionIou || bAssoByNp)
            { 
                for (int j = vAssoObjIds_byProIou.size() - 1; j >= 0; j--)
                {
                    int AssoObjId_byMotionIouOrNP;
                    if (bAssoByMotionIou)
                        AssoObjId_byMotionIouOrNP = AssoObjId_byIou;
                    if (bAssoByNp)
                        AssoObjId_byMotionIouOrNP = AssoObjId_byNP;

                    if (vAssoObjIds_byProIou[j] == AssoObjId_byMotionIouOrNP)
                        continue;

                    
                    int AssoObjId_byProIou = vAssoObjIds_byProIou[j];
                    AddPotentialAssociatedObjects(ObjectMaps, AssoObjId_byMotionIouOrNP, AssoObjId_byProIou);
                }
            }
            else
            {
                bool bFlag = ObjectMaps[MaxAssoObjId_byProIou]->UpdateToObject3D(this, *mpCurrentFrame, ProIou); // 4: project iou.

                // association succeeded.
                if (bFlag)
                {
                    bAssoByProjectIou = true;          // associated by projecting box.
                }

                for (int j = vAssoObjIds_byProIou.size() - 1; j >= 0; j--)
                {
                    std::cout << "[Object2D_DataAssociationWith_Object3D] J: " << j << " "
                    << MaxAssoObjId_byProIou << " "
                    << ObjectMaps[vAssoObjIds_byProIou[j]]->mnId << "\n";

                    if (vAssoObjIds_byProIou[j] == MaxAssoObjId_byProIou)
                    {
                        continue;
                    }

                    AddPotentialAssociatedObjects(ObjectMaps, MaxAssoObjId_byProIou, ObjectMaps[vAssoObjIds_byProIou[j]]->mnId);
                }
            }
        }
    }
    // Projected box data association

    // ************************************************
    //          STEP 4. t-test data association       *
    // ************************************************
    // step 4.1 Read t-distribution boundary value.
    float tTestData[122][9] = {0};
    ifstream infile;
    std::string filePath = WORK_SPACE_PATH + "/data/t_test.txt";
    infile.open(filePath);
    for (int i = 0; i < 122; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            infile >> tTestData[i][j];
        }
    }
    infile.close();

    // step 4.2 t-test.
    bool bAssoByTtest = false;
    int nAssoByTId = -1;
    vector<int> vObjByTId;
    vector<int> vObjByTIdLower; // potential association.

    if(Ttest_flag)
    {
        for (int i = (int)ObjectMaps.size() - 1; i >= 0; i--)
        {
            Object_Map* obj3d = ObjectMaps[i];
            if (mclass_id != obj3d->mnClass)
                continue;

            if (obj3d->bad_3d)
                continue;

            // t-test results in 3 directions.
            float t_test;
            float t_test_x, t_test_y, t_test_z;

            // Degrees of freedom.
            int df = (int)obj3d->mvObject_2ds.size();
            
            if (df <= 8)
                continue;

            // Iou.
            float fIou = Converter::bboxOverlapratio(RectCurrent, obj3d->mRect_byProjectPoints);
            float fIou2 = Converter::bboxOverlapratio(mBox_cvRect_FeaturePoints, obj3d->mRect_byProjectPoints);
            fIou = max(fIou, fIou2);

            // The distance from points to the object center.
            float dis_x, dis_y, dis_z;
            cv::Mat pos_points = mPos_world;
            dis_x = abs(obj3d->mAveCenter3D.at<float>(0, 0) - pos_points.at<float>(0, 0));
            dis_y = abs(obj3d->mAveCenter3D.at<float>(1, 0) - pos_points.at<float>(1, 0));
            dis_z = abs(obj3d->mAveCenter3D.at<float>(2, 0) - pos_points.at<float>(2, 0));

            // t-test.
            
            t_test_x = dis_x / (obj3d->mCenterStandar_x / sqrt(df));
            t_test_y = dis_y / (obj3d->mCenterStandar_y / sqrt(df));
            t_test_z = dis_z / (obj3d->mCenterStandar_z / sqrt(df));

            // Satisfy t test.  // 5->0.05.
            // notes: curr_t_test < t_{n-1, \alpha /2} 
            if ((t_test_x < tTestData[min((df - 1), 121)][5]) &&
                (t_test_y < tTestData[min((df - 1), 121)][5]) &&
                (t_test_z < tTestData[min((df - 1), 121)][5]))
            {
                vObjByTId.push_back(i);
            }
            // If the T-test is not satisfied, but the IOU is large, reducing the significance.
            else if (fIou > 0.25)
            {
                if ((t_test_x < tTestData[min((df - 1), 121)][8]) &&
                    (t_test_y < tTestData[min((df - 1), 121)][8]) &&
                    (t_test_z < tTestData[min((df - 1), 121)][8]))
                {
                    vObjByTId.push_back(i);
                }

                else if ((fIou > 0.25) && ((t_test_x + t_test_y + t_test_z) / 3 < 10))
                {
                    vObjByTId.push_back(i);
                }
                else
                {
                    vObjByTIdLower.push_back(i);
                }
            }
            else if ((t_test_x + t_test_y + t_test_z) / 3 < 4)
            {
                obj3d->ComputeProjectRectFrameToCurrentFrame(*mpCurrentFrame);

                float fIou_force = Converter::bboxOverlapratio(RectCurrent, obj3d->mRect_byProjectPoints);
                float fIou2_force = Converter::bboxOverlapratio(mBox_cvRect_FeaturePoints, obj3d->mRect_byProjectPoints);
                fIou_force = max(fIou_force, fIou2_force);

                if (fIou_force > 0.25){
                    vObjByTIdLower.push_back(i);
                }
            }
        }

        // step 4.2 update data association and record potential associated objects.
        if (bAssoByMotionIou || bAssoByNp || bAssoByProjectIou)
        {
            int ReId;
            if (bAssoByMotionIou)
                ReId = AssoObjId_byIou;
            if (bAssoByNp)
                ReId = AssoObjId_byNP;
            if (bAssoByProjectIou)
                ReId = MaxAssoObjId_byProIou;
            
            if (vObjByTId.size() >= 1)
            {
                for (int j = 0; j < vObjByTId.size(); j++)
                {
                    if (vObjByTId[j] == ReId)
                        continue;

                    AddPotentialAssociatedObjects(ObjectMaps, ReId, ObjectMaps[vObjByTId[j]]->mnId);
                }
            }

            if (vObjByTIdLower.size() >= 0)
            {
                for (int j = 0; j < vObjByTIdLower.size(); j++)
                {
                    if (vObjByTIdLower[j] == ReId)
                        continue;

                    AddPotentialAssociatedObjects(ObjectMaps, ReId, ObjectMaps[vObjByTIdLower[j]]->mnId);
                }
            }
        }
        else
        {
            if (vObjByTId.size() >= 1)
            {
                for (int i = 0; i < vObjByTId.size(); i++)
                {
                    bool bFlag = ObjectMaps[vObjByTId[i]]->UpdateToObject3D(this, *mpCurrentFrame, t_test);

                    if (bFlag)
                    {
                        bAssoByTtest = true;
                        nAssoByTId = vObjByTId[i];

                        if (vObjByTId.size() > i)
                        {
                            for (int j = i + 1; j < vObjByTId.size(); j++)
                            {
                                AddPotentialAssociatedObjects(ObjectMaps, nAssoByTId, ObjectMaps[vObjByTId[j]]->mnId);
                            }
                        }

                        if (vObjByTIdLower.size() >= 0)
                        {
                            for (int j = 0; j < vObjByTIdLower.size(); j++)
                            {
                                if (vObjByTIdLower[j] == nAssoByTId)
                                    continue;

                                AddPotentialAssociatedObjects(ObjectMaps, nAssoByTId, ObjectMaps[vObjByTIdLower[j]]->mnId);
                            }
                        }

                        break;
                    }
                }
            }
        }
    }
    // t-test data association

    // *************************************************
    //             STEP 4. create a new object         *
    // *************************************************
     if (bAssoByMotionIou)
        return MotionIou;
     if (bAssoByNp)
        return NoPara;
     if (bAssoByProjectIou)
        return ProIou;
     if (bAssoByTtest)
        return t_test;
     return 0;
}

int Object_2D::creatObject()
{
    unique_lock<mutex> lock1(mMutexObjMapPoints);   
    unique_lock<mutex> lock2(mGlobalMutex);

    const cv::Mat ColorImage = mpCurrentFrame->mColorImage.clone();
    int associate = Object2D_DataAssociationWith_Object3D();    // data association with object3d in map.

    switch (associate) {
                case MotionIou:   cout << "[creatObject] Association：MotionIou. " << endl; return 0;
                case NoPara:    cout << "[creatObject] Association：NoPara. " << endl;  return 0;
                case ProIou:    cout << "[creatObject] Association：ProIou. " << endl;  return 0;
                case t_test:    cout << "[creatObject] Association：t_test. " << endl;  return 0;
    }
    if(associate)
        return 0;  // Association succeeded

    // If the object appears at the edge of the image, ignore.
    if ((this->mBox_cvRect.x < 10) || (this->mBox_cvRect.y < 10) ||
        (this->mBox_cvRect.x + this->mBox_cvRect.width > ColorImage.cols - 10) ||
        (this->mBox_cvRect.y + this->mBox_cvRect.height > ColorImage.rows - 10))
    {
        this->bad = true;
        return -1;  
    }

    // TODO: Improve Association method
    // create a 3d object in the map.
    Object_Map *Object3D = new Object_Map;  
    Object3D->mvObject_2ds.push_back(this);
    const vector<Object_Map*> ObjectMaps  = mpMap->GetObjects();
    Object3D->mnId = ObjectMaps.size();
    Object3D->mnClass = mclass_id;

    Object3D->mnConfidence_foractive = 1;
    Object3D->mnAddedID_nouse = mpCurrentFrame->mnId;
    Object3D->mnLastAddID = mpCurrentFrame->mnId;
    Object3D->mnLastLastAddID = mpCurrentFrame->mnId;
    Object3D->mLastRect = mBox_cvRect;
    //Object3D->mPredictRect = obj->mBoxRect;       // for iou.
    // add properties of the point and save it to the object.

    for (size_t i = 0; i < mvMapPonits.size(); i++)
    {
        MapPoint *pMP = mvMapPonits[i];
        pMP->object_mnId = Object3D->mnId;
        pMP->object_class = Object3D->mnClass;
        pMP->viewdCount_forObjectId.insert(make_pair(Object3D->mnId, 1));  

        // save to the object.
        Object3D->mvpMapObjectMappoints.push_back(pMP);
        Object3D->mvpMapObjectMappoints_NewForActive.push_back(pMP);
    }

    mnId = Object3D->mnId;

    // save this 2d object to current frame (associates with a 3d object in the map).
    mpCurrentFrame->mvObject_2ds.push_back(this);
    mpCurrentFrame->AppearNewObject = true;                  
    //mpCurrentFrame->AppearNewObject = true;

    // update object map.
    Object3D->ComputeMeanAndDeviation_3D();
    Object3D->IsolationForestDeleteOutliers();
    Object3D->ComputeMeanAndDeviation_3D();
    //mpMap->mvObjectMap.push_back(ObjectMapSingle);
    mpMap->AddObject(Object3D);

    std::cout << "[Object] Save in map" << std::endl;

    return 1;  
}

//  Nonparametric test
int Object_2D::NoParaDataAssociation(Object_Map *Object3D)
{
    // step 1. sample size.
    // 2d object points in the frame -- m.
    int m = (int)mvMapPonits.size();
    int OutPointNum1 = 0;
    for (int i = 0; i < (int)mvMapPonits.size(); i++)
    {
        MapPoint *p1 = mvMapPonits[i];
        if (p1->isBad())
        {
            OutPointNum1++;
            continue;
        }
    }
    m = m - OutPointNum1;

    // 3d object points in the object map -- n.
    int n = (int)Object3D->mvpMapObjectMappoints.size();
    int OutPointNum2 = 0;
    for (int i = 0; i < (int)Object3D->mvpMapObjectMappoints.size(); i++) 
    {
        MapPoint *p2 = Object3D->mvpMapObjectMappoints[i];
        if (p2->isBad())
        {
            OutPointNum2++;
            continue;
        }
    }
    n = n - OutPointNum2;

    // 0: skip the nonparametric test and continue with the subsequent t-test.
    if (m < 20)
        return 0;

    // 2: association failed, compare next object.
    if (n < 20)
        return 2;

    // Homogenization to avoid too many points of map object; n = 3 * m.
    bool bSampleMapPoints = true;

    vector<float> x_pt_map_sample;
    vector<float> y_pt_map_sample;
    vector<float> z_pt_map_sample;

    if (bSampleMapPoints)
    {
        int step = 1;
        if (n > 3 * m)   
        {
            n = 3 * m;
            step = (int)Object3D->mvpMapObjectMappoints.size() / n;
           
            vector<float> x_pt;
            vector<float> y_pt;
            vector<float> z_pt;
            for (int i = 0; i < (int)Object3D->mvpMapObjectMappoints.size(); i++)
            {
                MapPoint *p2 = Object3D->mvpMapObjectMappoints[i];
                if (p2->isBad())
                {
                    continue;
                }

                cv::Mat x3D2 = Converter::toCvMat(p2->GetWorldPos());
                x_pt.push_back(x3D2.at<float>(0, 0));
                y_pt.push_back(x3D2.at<float>(1, 0));
                z_pt.push_back(x3D2.at<float>(2, 0));
            }
            
            sort(x_pt.begin(), x_pt.end());
            sort(y_pt.begin(), y_pt.end());
            sort(z_pt.begin(), z_pt.end());
            for (int i = 0; i < x_pt.size(); i += step)
            {
                x_pt_map_sample.push_back(x_pt[i]);
                y_pt_map_sample.push_back(y_pt[i]);
                z_pt_map_sample.push_back(z_pt[i]);
            }
            n = x_pt_map_sample.size();
        }
        else    
        {
            for (int jj = 0; jj < (int)Object3D->mvpMapObjectMappoints.size(); jj++) 
            {
                MapPoint *p2 = Object3D->mvpMapObjectMappoints[jj];
                if (p2->isBad())
                {
                    continue;
                }

                cv::Mat x3D2 = Converter::toCvMat(p2->GetWorldPos());
                x_pt_map_sample.push_back(x3D2.at<float>(0, 0));
                y_pt_map_sample.push_back(x3D2.at<float>(1, 0));
                z_pt_map_sample.push_back(x3D2.at<float>(2, 0));
            }

            n = x_pt_map_sample.size();
        }
    }

    float w_x_2d_bigger_3d = 0.0;
    float w_y_2d_bigger_3d = 0.0;
    float w_z_2d_bigger_3d = 0.0;
    float w_x_3d_bigger_2d = 0.0;
    float w_y_3d_bigger_2d = 0.0;
    float w_z_3d_bigger_2d = 0.0;
    float w_x_3d_equal_2d = 0.0;
    float w_y_3d_equal_2d = 0.0;
    float w_z_3d_equal_2d = 0.0;
    float w_x = 0.0;
    float w_y = 0.0;
    float w_z = 0.0;

    for (int ii = 0; ii < (int)mvMapPonits.size(); ii++)
    {
        MapPoint *p1 = mvMapPonits[ii];
        if (p1->isBad() )
            continue;

        cv::Mat x3D1 = Converter::toCvMat(p1->GetWorldPos());
        double x_2d = x3D1.at<float>(0, 0);
        double y_2d = x3D1.at<float>(1, 0);
        double z_2d = x3D1.at<float>(2, 0);

        
        if (!bSampleMapPoints)
        {
            // Re-extract point from Object3D->mvpMapObjectMappoints, and perform
            for (int jj = 0; jj < (int)Object3D->mvpMapObjectMappoints.size(); jj++)
            {
                MapPoint *p2 = Object3D->mvpMapObjectMappoints[jj];
                if (p2->isBad())
                    continue;

                cv::Mat x3D2 = Converter::toCvMat(p2->GetWorldPos());
                double x2 = x3D2.at<float>(0, 0);
                double y2 = x3D2.at<float>(1, 0);
                double z2 = x3D2.at<float>(2, 0);

                if (x_2d > x2)
                    w_x_2d_bigger_3d++;
                else if (x_2d < x2)
                    w_x_3d_bigger_2d++;
                else if (x_2d == x2)
                    w_x_3d_equal_2d++;

                if (y_2d > y2)
                    w_y_2d_bigger_3d++;
                else if (y_2d < y2)
                    w_y_3d_bigger_2d++;
                else if (y_2d == y2)
                    w_y_3d_equal_2d++;

                if (z_2d > z2)
                    w_z_2d_bigger_3d++;
                else if (z_2d < z2)
                    w_z_3d_bigger_2d++;
                else if (z_2d == z2)
                    w_z_3d_equal_2d++;
            }
        }

        // will 3d point
        if (bSampleMapPoints)
        {
            for (int jj = 0; jj < (int)x_pt_map_sample.size(); jj++)
            {
                double x_3d = x_pt_map_sample[jj];
                double y_3d = y_pt_map_sample[jj];
                double z_3d = z_pt_map_sample[jj];

                if (x_2d > x_3d)
                    w_x_2d_bigger_3d++;
                else if (x_2d < x_3d)
                    w_x_3d_bigger_2d++;
                else if (x_2d == x_3d)
                    w_x_3d_equal_2d++;

                if (y_2d > y_3d)
                    w_y_2d_bigger_3d++;
                else if (y_2d < y_3d)
                    w_y_3d_bigger_2d++;
                else if (y_2d == y_3d)
                    w_y_3d_equal_2d++;

                if (z_2d > z_3d)
                    w_z_2d_bigger_3d++;
                else if (z_2d < z_3d)
                    w_z_3d_bigger_2d++;
                else if (z_2d == z_3d)
                    w_z_3d_equal_2d++;
            }
        }
    }

    // step 2. `compute the rank sum.`
    // notes: Note that when we calculate the ranks of several equivalent elements,
    // the average of the ranks of these elements is used as their rank in the entire sequence.
    // This is why w_x_00 / 2 is added
    // w_x_12 + w_x_00 / 2 The sort corresponding to m; w_x_21 + w_x_00 / 2 is the sort corresponding to n. It feels wrong
    // W = min(W_p, W_q)
    // zhang: w = (n*n - R)   + n(n+1)/2
    w_x = min(w_x_2d_bigger_3d + m * (m + 1) / 2, w_x_3d_bigger_2d + n * (n + 1) / 2) + w_x_3d_equal_2d / 2;
    w_y = min(w_y_2d_bigger_3d + m * (m + 1) / 2, w_y_3d_bigger_2d + n * (n + 1) / 2) + w_y_3d_equal_2d / 2;
    w_z = min(w_z_2d_bigger_3d + m * (m + 1) / 2, w_z_3d_bigger_2d + n * (n + 1) / 2) + w_z_3d_equal_2d / 2;

    // step 3. compute the critical value.
    // TODO: Modified to the standard form on the wiki
    // notes: The formula here is actually not right, the formula in the code is r1 = r_l = m + s * sqrt(\sigma)
    // in \sigma = m * n * (m + n + 1) / 12
    // But in this case, there are repeated ranks, \sigma should use the formula in the paper
    // But the formula in the paper is not consistent with the wiki
    // The comment given, that is, the value of s, does not seem to be correct. In the standard normal distribution table, 80% corresponds to 0.85, and 1.28 corresponds to 90%
    // 97.5% corresponds to 1.96
    float r1 = 0.5 * m * (m + n + 1) - 1.282 * sqrt(m * n * (m + n + 1) / 12); // 80%：1.282  85%:1.96
    float r2 = 0.5 * m * (m + n + 1) + 1.282 * sqrt(m * n * (m + n + 1) / 12); // 80%：1.282  85%:1.96

    // step 4. whether the 3 directions meet the nonparametric test.
    bool old_np = false;
    int add = 0;
    if (w_x > r1 && w_x < r2)
        add++;
    if (w_y > r1 && w_y < r2)
        add++;
    if (w_z > r1 && w_z < r2)
        add++;

    if (add == 3)
        old_np = true;  // Nonparametric Association succeeded.

    if (old_np == 1)
        return 1;       // success.
    else
        return 2;       // failure.
} // Object_2D::NoParaDataAssociation()

void Object_2D::AddObjectPoint(semantic_slam::MapPoint *pMP) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mvMapPonits.push_back(pMP);
    const cv::Mat PointPosWorld = Converter::toCvMat(pMP->GetWorldPos());                 // world frame.
    sum_pos_3d += PointPosWorld;  
}


void Object_2D::AddPotentialAssociatedObjects(vector<Object_Map*> obj3ds, int AssoId, int beAssoId){

    unique_lock<mutex> lock2(mMutexObj);

    map<int, int>::iterator sit;

    for(sit = obj3ds[AssoId]->mReObj.begin(); sit != obj3ds[AssoId]->mReObj.begin(); sit++)
    {
        int nObjId = sit->first;

        if(nObjId == obj3ds[beAssoId]->mnId)
        {

            if (sit != obj3ds[AssoId]->mReObj.end())
            {
                int sit_sec = sit->second;
                obj3ds[AssoId]->mReObj.erase(obj3ds[beAssoId]->mnId);
                obj3ds[AssoId]->mReObj.insert(std::make_pair(obj3ds[beAssoId]->mnId, sit_sec + 1));
            }
            else
            {
                // TODO: Crash here

                obj3ds[AssoId]->mReObj.insert(std::make_pair(obj3ds[beAssoId]->mnId, 1));
            }
        }
    }
}

// ************************************
// object3d
// ************************************
void Object_Map::ComputeMeanAndDeviation_3D() {

    mSumPointsPos = cv::Mat::zeros(3,1,CV_32F);

    // remove bad points.
    {
        unique_lock<mutex> lock(mMutexMapPoints);
        vector<MapPoint *>::iterator pMP;
        int i = 0;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            i++;

            cv::Mat pos = Converter::toCvMat((*pMP)->GetWorldPos());

            if ((*pMP)->isBad()) {
                pMP = mvpMapObjectMappoints.erase(pMP);
            } else {
                mSumPointsPos += pos;
                ++pMP;
            }
        }
    }

    // step 0. mean(3d center).
    mAveCenter3D = mSumPointsPos / (mvpMapObjectMappoints.size());

    // step 1. standard deviation in 3 directions.
    float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    vector<float> x_pt, y_pt, z_pt;
    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++) {
        cv::Mat pos = Converter::toCvMat(mvpMapObjectMappoints[i]->GetWorldPos());
        cv::Mat pos_ave = mAveCenter3D;

        // （x-x^）^2
        sum_x2 += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
        sum_y2 += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
        sum_z2 += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));

        x_pt.push_back(pos.at<float>(0));
        y_pt.push_back(pos.at<float>(1));
        z_pt.push_back(pos.at<float>(2));
    }
    mStandar_x = sqrt(sum_x2 / (mvpMapObjectMappoints.size()));
    mStandar_y = sqrt(sum_y2 / (mvpMapObjectMappoints.size()));
    mStandar_z = sqrt(sum_z2 / (mvpMapObjectMappoints.size()));

    if (x_pt.size() == 0)
        return;

    // step 2. standard deviation of centroids (observations from different frames).
    float sum_x2_c = 0, sum_y2_c = 0, sum_z2_c = 0;
    vector<float> x_c, y_c, z_c;

    for (size_t i = 0; i < this->mvObject_2ds.size(); i++)
    {
        cv::Mat pos = this->mvObject_2ds[i]->mPos_world.clone();
        cv::Mat pos_ave = this->mAveCenter3D.clone();

        // （x-x^）^2
        sum_x2_c += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
        sum_y2_c += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
        sum_z2_c += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));
    }

    mCenterStandar_x = sqrt(sum_x2_c / (this->mvObject_2ds.size()));
    mCenterStandar_y = sqrt(sum_y2_c / (this->mvObject_2ds.size()));
    mCenterStandar_z = sqrt(sum_z2_c / (this->mvObject_2ds.size()));

    // step 3. update object center and scale.
    if (this->mvObject_2ds.size() < 5)
    {
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        sort(z_pt.begin(), z_pt.end());

        if ((x_pt.size() == 0) || (y_pt.size() == 0) || (z_pt.size() == 0)) {
            this->bad_3d = true;
            return;
        }

        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];

        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        float z_min = z_pt[0];
        float z_max = z_pt[z_pt.size() - 1];

        // centre.
        mCuboid3D.cuboidCenter = Eigen::Vector3d((x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2);

        mCuboid3D.x_min = x_min;
        mCuboid3D.x_max = x_max;
        mCuboid3D.y_min = y_min;
        mCuboid3D.y_max = y_max;
        mCuboid3D.z_min = z_min;
        mCuboid3D.z_max = z_max;

        mCuboid3D.lenth = x_max - x_min;
        mCuboid3D.width = y_max - y_min;
        mCuboid3D.height = z_max - z_min;

        mCuboid3D.corner_1 = Eigen::Vector3d(x_min, y_min, z_min);
        mCuboid3D.corner_2 = Eigen::Vector3d(x_max, y_min, z_min);
        mCuboid3D.corner_3 = Eigen::Vector3d(x_max, y_max, z_min);
        mCuboid3D.corner_4 = Eigen::Vector3d(x_min, y_max, z_min);
        mCuboid3D.corner_5 = Eigen::Vector3d(x_min, y_min, z_max);
        mCuboid3D.corner_6 = Eigen::Vector3d(x_max, y_min, z_max);
        mCuboid3D.corner_7 = Eigen::Vector3d(x_max, y_max, z_max);
        mCuboid3D.corner_8 = Eigen::Vector3d(x_min, y_max, z_max);

        mCuboid3D.corner_1_w = Eigen::Vector3d(x_min, y_min, z_min);
        mCuboid3D.corner_2_w = Eigen::Vector3d(x_max, y_min, z_min);
        mCuboid3D.corner_3_w = Eigen::Vector3d(x_max, y_max, z_min);
        mCuboid3D.corner_4_w = Eigen::Vector3d(x_min, y_max, z_min);
        mCuboid3D.corner_5_w = Eigen::Vector3d(x_min, y_min, z_max);
        mCuboid3D.corner_6_w = Eigen::Vector3d(x_max, y_min, z_max);
        mCuboid3D.corner_7_w = Eigen::Vector3d(x_max, y_max, z_max);
        mCuboid3D.corner_8_w = Eigen::Vector3d(x_min, y_max, z_max);
    }

    // step 4. update object pose
    Update_Twobj();

    // step 5. Calculate the world coordinates of 8 fixed points
    vector<float> x_pt_obj, y_pt_obj, z_pt_obj;
    g2o::SE3Quat pose =  Converter::toSE3Quat(this->mCuboid3D.pose_mat);
    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++) {
        // world frame.
        Eigen::Vector3d PointPos_world = Converter::toVector3d(Converter::toCvMat(mvpMapObjectMappoints[i]->GetWorldPos()));

        // object frame.   Twobj.inv * point = Tobjw * point
        Eigen::Vector3d PointPos_object = pose.inverse() * PointPos_world;
        x_pt_obj.push_back(PointPos_object[0]);
        y_pt_obj.push_back(PointPos_object[1]);
        z_pt_obj.push_back(PointPos_object[2]);
    }

    if (x_pt_obj.size() == 0)
        return;

    // rank.
    int s = x_pt_obj.size();
    sort(x_pt_obj.begin(), x_pt_obj.end());
    sort(y_pt_obj.begin(), y_pt_obj.end());
    sort(z_pt_obj.begin(), z_pt_obj.end());

    float x_min_obj = x_pt_obj[0];
    float x_max_obj = x_pt_obj[s - 1];
    float y_min_obj = y_pt_obj[0];
    float y_max_obj = y_pt_obj[s - 1];
    float z_min_obj = z_pt_obj[0];
    float z_max_obj = z_pt_obj[s - 1];

    // update object vertices and translate it to world frame.
    // g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
    mCuboid3D.corner_1 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_2 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_3 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_4 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_5 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_6 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_7 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    mCuboid3D.corner_8 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);

    //// object frame -> world frame (without yaw, parallel to world frame).
    //g2o::SE3Quat pose_without_yaw =  Converter::toSE3Quat(this->mCuboid3D.pose_noyaw_mat);
    //mCuboid3D.corner_1_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    //mCuboid3D.corner_2_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    //mCuboid3D.corner_3_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    //mCuboid3D.corner_4_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    //mCuboid3D.corner_5_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    //mCuboid3D.corner_6_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    //mCuboid3D.corner_7_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    //mCuboid3D.corner_8_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);


    // step 6. Calculate the length, width, height and radius of the cubic
    this->mCuboid3D.lenth = x_max_obj - x_min_obj;
    this->mCuboid3D.width = y_max_obj - y_min_obj;
    this->mCuboid3D.height = z_max_obj - z_min_obj;
    this->mCuboid3D.cuboidCenter = (mCuboid3D.corner_2 + mCuboid3D.corner_8) / 2;
    Update_Twobj();

    // maximum radius.
    float fRMax = 0.0;
    vector<cv::Mat> vCornerMat;
    vCornerMat.resize(8);
    for (int i = 0; i < 8; i++) {
        cv::Mat mDis = cv::Mat::zeros(3, 1, CV_32F);
        if (i == 0)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_1);
        if (i == 1)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_2);
        if (i == 2)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_3);
        if (i == 3)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_4);
        if (i == 4)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_5);
        if (i == 5)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_6);
        if (i == 6)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_7);
        if (i == 7)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_8);

        mDis = mAveCenter3D - vCornerMat[i];
        float fTmp = sqrt(mDis.at<float>(0) * mDis.at<float>(0) + mDis.at<float>(1) * mDis.at<float>(1) +
                          mDis.at<float>(2) * mDis.at<float>(2));
        fRMax = max(fRMax, fTmp);
    }
    mCuboid3D.mfRMax = fRMax;


    // step 7. Calculate the deviation between the cubic center and the center of each object2d point cloud cluster on the straight-line distance
    // standard deviation of distance.
    float dis = 0;
    for (size_t i = 0; i < mvObject_2ds.size(); i++) {
        float center_sum_x2 = 0, center_sum_y2 = 0, center_sum_z2 = 0;

        const cv::Mat pos = mvObject_2ds[i]->mPos_world.clone();
        const cv::Mat pos_ave = mAveCenter3D.clone();

        center_sum_x2 =
                (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0)); // dis_x^2
        center_sum_y2 =
                (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1)); // dis_y^2
        center_sum_z2 =
                (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2)); // dis_z^2

        dis += sqrt(center_sum_x2 + center_sum_y2 + center_sum_z2);
    }
    mCenterStandar = sqrt(dis / (mvObject_2ds.size()));

    // step 8. Compute IE
    this->ComputeIE();
}

// Remove the outliers in object3d and re-optimize the coordinates and scale of the object
// Question: What is the difference between this and ComputeMeanAndStandard?
// Answer: It seems to be exclusive to ComputeMeanAndStandard under biForest
// remove outliers and refine the object position and scale by IsolationForest.
void Object_Map::IsolationForestDeleteOutliers(){
    if(!iforest_flag)
        return;

    //if ((this->mnClass == 75) || (this->mnClass == 64) || (this->mnClass == 65))
    //    return;

    float th = 0.6;
    if (this->mnClass == 62)
        th = 0.65;

    // notes: Compared with the traditional srand(), std::mt19937 has better performance.
    //std::mt19937 rng(12345);
    std::vector<std::array<float, 3>> data; // uint32_t

    if (mvpMapObjectMappoints.size() < 30)
        return;

    // Convert the coordinates of point from cv::mat to std::array
    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++)
    {
        MapPoint *pMP = mvpMapObjectMappoints[i];
        cv::Mat pos = Converter::toCvMat(pMP->GetWorldPos());

        std::array<float, 3> temp;
        temp[0] = pos.at<float>(0);
        temp[1] = pos.at<float>(1);
        temp[2] = pos.at<float>(2);
        data.push_back(temp);
    }

    // STEP 3 Build a random forest filter
    iforest::IsolationForest<float, 3> forest; // uint32_t
    if (!forest.Build(50, 12345, data, ((int)mvpMapObjectMappoints.size() / 2)))
                 // Number of numbers, ??, input data, number of samples (here is half of data)
    {
        std::cerr << "Failed to build Isolation Forest.\n";
        return;
    }
    std::vector<double> anomaly_scores;

    // STEP 4 Calculate Anomaly_score, and mark the point greater than the threshold as outlier
    if (!forest.GetAnomalyScores(data, anomaly_scores))
    {
        std::cerr << "Failed to calculate anomaly scores.\n";
        return;
    }
    std::vector<int> outlier_ids;
    for (uint32_t i = 0; i < (int)mvpMapObjectMappoints.size(); i++)
    {
        // If Anomaly_score is greater than the threshold, it is considered outlier
        if (anomaly_scores[i] > th)
            outlier_ids.push_back(i);
    }

    if (outlier_ids.empty())
        return;

    // step 5. Remove outliers from mvpMapObjectMapppoints of object3d.
    int id_Mappoint = -1;
    int id_MOutpoint_out = 0;
    unique_lock<mutex> lock(mMutexMapPoints); // lock.
    vector<MapPoint *>::iterator pMP;
    for (pMP = mvpMapObjectMappoints.begin();
         pMP != mvpMapObjectMappoints.end();)
    {
        // pMP and numMappoint, increment from the beginning, review in turn
        id_Mappoint++;

        cv::Mat pos = Converter::toCvMat((*pMP)->GetWorldPos());

        if (id_Mappoint == outlier_ids[id_MOutpoint_out])
        {
            id_MOutpoint_out++;
            pMP = mvpMapObjectMappoints.erase(pMP);
            //std::cout<<"[iforest debug] mSumPointsPos size:"<< mSumPointsPos.size() <<", pos size:"<< pos.size() <<std::endl;
            mSumPointsPos -= pos;
        }
        else
        {
            ++pMP;
        }
    }
}

void Object_Map::Update_Twobj()      // Update the coordinates of the object in the world
{
    // Rotation matrix.
    float cp = cos(mCuboid3D.rotP);
    float sp = sin(mCuboid3D.rotP);
    float sr = sin(mCuboid3D.rotR);
    float cr = cos(mCuboid3D.rotR);
    float sy = sin(mCuboid3D.rotY);
    float cy = cos(mCuboid3D.rotY);
    Eigen::Matrix<double, 3, 3> REigen;
    REigen << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    cv::Mat Ryaw = Converter::toCvMat(REigen);

    // Transformation matrix.
    cv::Mat Twobj = cv::Mat::eye(4, 4, CV_32F);
    const cv::Mat Rcw = Twobj.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Twobj.rowRange(0, 3).col(3);

    cv::Mat R_result = Rcw * Ryaw;

    Twobj.at<float>(0, 0) = R_result.at<float>(0, 0);
    Twobj.at<float>(0, 1) = R_result.at<float>(0, 1);
    Twobj.at<float>(0, 2) = R_result.at<float>(0, 2);
    Twobj.at<float>(0, 3) = mCuboid3D.cuboidCenter[0];

    Twobj.at<float>(1, 0) = R_result.at<float>(1, 0);
    Twobj.at<float>(1, 1) = R_result.at<float>(1, 1);
    Twobj.at<float>(1, 2) = R_result.at<float>(1, 2);
    Twobj.at<float>(1, 3) = mCuboid3D.cuboidCenter[1];

    Twobj.at<float>(2, 0) = R_result.at<float>(2, 0);
    Twobj.at<float>(2, 1) = R_result.at<float>(2, 1);
    Twobj.at<float>(2, 2) = R_result.at<float>(2, 2);
    Twobj.at<float>(2, 3) = mCuboid3D.cuboidCenter[2];

    Twobj.at<float>(3, 0) = 0;
    Twobj.at<float>(3, 1) = 0;
    Twobj.at<float>(3, 2) = 0;
    Twobj.at<float>(3, 3) = 1;

    // note no yaw.
    cv::Mat Twobj_without_yaw = cv::Mat::eye(4, 4, CV_32F);
    Twobj_without_yaw.at<float>(0, 3) = mCuboid3D.cuboidCenter[0];//mAveCenter3D.at<float>(0);
    Twobj_without_yaw.at<float>(1, 3) = mCuboid3D.cuboidCenter[1];
    Twobj_without_yaw.at<float>(2, 3) = mCuboid3D.cuboidCenter[2];//mAveCenter3D.at<float>(2);

    // SE3.[origin]
    //g2o::SE3Quat obj_pose = Converter::toSE3Quat(Twobj);
    //g2o::SE3Quat obj_pose_without_yaw = Converter::toSE3Quat(Twobj_without_yaw);
    //this->mCuboid3D.pose = obj_pose;
    //this->mCuboid3D.pose_without_yaw = obj_pose_without_yaw;
    this->mCuboid3D.pose_mat = Twobj;
    //this->mCuboid3D.pose_noyaw_mat = Twobj_without_yaw;
}

void Object_Map::ComputeProjectRectFrameToCurrentFrame(Frame &Frame)
{
    cv::Mat Tcw_ = Converter::toCvMat(Frame.GetPose());
    const cv::Mat Rcw = Tcw_.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Tcw_.rowRange(0, 3).col(3);
    vector<float> x_pt;
    vector<float> y_pt;
    for (int j = 0; j < mvpMapObjectMappoints.size(); j++)
    {
        MapPoint *pMP = mvpMapObjectMappoints[j];
        cv::Mat PointPosWorld = Converter::toCvMat(pMP->GetWorldPos());

        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        const float xc = PointPosCamera.at<float>(0);
        const float yc = PointPosCamera.at<float>(1);
        const float invzc = 1.0 / PointPosCamera.at<float>(2);

        float u = Frame.fx * xc * invzc + Frame.cx;
        float v = Frame.fy * yc * invzc + Frame.cy;

        x_pt.push_back(u);
        y_pt.push_back(v);

    }

    if (x_pt.size() == 0)
        return;

    sort(x_pt.begin(), x_pt.end());
    sort(y_pt.begin(), y_pt.end());
    float x_min = x_pt[0];
    float x_max = x_pt[x_pt.size() - 1];
    float y_min = y_pt[0];
    float y_max = y_pt[y_pt.size() - 1];

    if (x_min < 0)
        x_min = 0;
    if (y_min < 0)
        y_min = 0;
    if (x_max > Frame.mColorImage.cols)
        x_max = Frame.mColorImage.cols;
    if (y_max > Frame.mColorImage.rows)
        y_max = Frame.mColorImage.rows;

    mRect_byProjectPoints = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}

cv::Rect Object_Map::ComputeProjectRectFrameToCurrentKeyFrame(KeyFrame &kF)
{
    cv::Mat Tcw_ = Converter::toCvMat(kF.GetPose());
    const cv::Mat Rcw = Tcw_.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Tcw_.rowRange(0, 3).col(3);
    vector<float> x_pt;
    vector<float> y_pt;

    for (int j = 0; j < mvpMapObjectMappoints.size(); j++)
    {
        MapPoint *pMP = mvpMapObjectMappoints[j];
        cv::Mat PointPosWorld = Converter::toCvMat(pMP->GetWorldPos());

        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        const float xc = PointPosCamera.at<float>(0);
        const float yc = PointPosCamera.at<float>(1);
        const float invzc = 1.0 / PointPosCamera.at<float>(2);

        float u = kF.fx * xc * invzc + kF.cx;
        float v = kF.fy * yc * invzc + kF.cy;

        x_pt.push_back(u);
        y_pt.push_back(v);

    }

    if (x_pt.size() == 0)
        return cv::Rect(0.0, 0.0, 0.0, 0.0);

    sort(x_pt.begin(), x_pt.end());
    sort(y_pt.begin(), y_pt.end());
    float x_min = x_pt[0];
    float x_max = x_pt[x_pt.size() - 1];
    float y_min = y_pt[0];
    float y_max = y_pt[y_pt.size() - 1];

    if (x_min < 0)
        x_min = 0;
    if (y_min < 0)
        y_min = 0;
    if (x_max > kF.raw_img.cols)
        x_max = kF.raw_img.cols;
    if (y_max > kF.raw_img.rows)
        y_max = kF.raw_img.rows;

    return cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}


void Object_Map::UpdateCoView(Object_Map *Obj_CoView)
{
    int nObjId = Obj_CoView->mnId;

    map<int, int>::iterator sit;
    sit = this->mmAppearSametime.find(nObjId);

    if (sit != this->mmAppearSametime.end())
    {
        int sit_sec = sit->second;
        this->mmAppearSametime.erase(nObjId);
        this->mmAppearSametime.insert(make_pair(nObjId, sit_sec + 1));
    }
    else
        this->mmAppearSametime.insert(make_pair(nObjId, 1));   // first co-view.
}

vector<MapPoint* > Object_Map::GetObjectMappoints(){
    //unique_lock<mutex> lock(mMutex); mvpMapObjectMappoints_NewForActive
    unique_lock<mutex> lock(mMutexMapPoints);
    return vector<MapPoint* >(mvpMapObjectMappoints.begin(), mvpMapObjectMappoints.end());
}

vector<MapPoint* > Object_Map::GetNewObjectMappoints(){
    //unique_lock<mutex> lock(mMutex); mvpMapObjectMappoints_NewForActive
    unique_lock<mutex> lock(mMutexMapPoints);
    return vector<MapPoint* >(mvpMapObjectMappoints_NewForActive.begin(), mvpMapObjectMappoints_NewForActive.end());
}

// ************************************
// object3d track
// ************************************

// MotionIou 1,  NoPara 2,  t_test 3,  ProIou 4
// return Reasons for false: class id does not match; IOU is too small; Frame id is not incremented;
bool Object_Map::UpdateToObject3D(Object_2D* Object_2d,
                                  Frame &mCurrentFrame,
                                  int Flag)
{

    if (Object_2d->mclass_id != mnClass)
        return false;

    cv::Mat Tcw_ = Converter::toCvMat(mCurrentFrame.GetPose());
    const cv::Mat Rcw = Tcw_.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Tcw_.rowRange(0, 3).col(3);

    // step 1. whether the box projected into the image changes greatly after the new point cloud is associated.
    if ((Flag != MotionIou) && (Flag != ProIou))
    // This step is only used for NoPara and t_test
    // Because of the following, it is similar to the matching part of IOU. 
    // NoPara and t_test thought that the IOU matching degree was not good before, 
    // so it is necessary to verify whether the IOU matches again.
    {
        // notes：
        // ProjectRect1 represents the target frame projection on the object map;
        // ProjectRect2 represents the target frame projection integrated into the current frame
        cv::Rect ProjectRect_3D;
        cv::Rect ProjectRect_3Dand2D;

        // projected bounding box1.
        this->ComputeProjectRectFrameToCurrentFrame(mCurrentFrame);
        ProjectRect_3D = this->mRect_byProjectPoints;

        // mixed points of frame object and map object.
        vector<float> x_pt;
        vector<float> y_pt;

        // notes:
        // Obj_c_MapPonits is the 3D point on the object,
        // mvpMapObjectMappoints is the point on the object map
        for (int i = 0; i < Object_2d->mvMapPonits.size(); ++i)
        {
            MapPoint *pMP = Object_2d->mvMapPonits[i];
            cv::Mat PointPosWorld = Converter::toCvMat(pMP->GetWorldPos());
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        for (int j = 0; j < mvpMapObjectMappoints.size(); ++j)
        {
            MapPoint *pMP = mvpMapObjectMappoints[j];
            cv::Mat PointPosWorld = Converter::toCvMat(pMP->GetWorldPos());
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        // rank.
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > mCurrentFrame.mColorImage.cols)
            x_max = mCurrentFrame.mColorImage.cols;
        if (y_max > mCurrentFrame.mColorImage.rows)
            y_max = mCurrentFrame.mColorImage.rows;

        // projected bounding box2.
        ProjectRect_3Dand2D = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

        // 4. Calculate Iou
        float fIou = Converter::bboxOverlapratio(ProjectRect_3D, ProjectRect_3Dand2D);
        float fIou2 = Converter::bboxOverlapratioFormer(ProjectRect_3Dand2D, Object_2d->mBox_cvRect);
        if ((fIou < 0.5) && (fIou2 < 0.8))  
            return false;
    }

    // step 2. update the ID of the last frame
    // Update the frame id and object detection frame of last and lastlast
    if (mnLastAddID != (int)mCurrentFrame.mnId)
    {
        mnLastLastAddID = mnLastAddID;
        mnLastAddID = mCurrentFrame.mnId;
        mLastLastRect = mLastRect;
        mLastRect = Object_2d->mBox_cvRect;
        mnConfidence_foractive++;
        AddObj2d(Object_2d);
    }
    else
        return false;

    Object_2d->mnId = mnId;

    // step 3. Add the point cloud of the frame object to the map object
    for (size_t j = 0; j < Object_2d->mvMapPonits.size(); ++j)
    {
        MapPoint *pMP = Object_2d->mvMapPonits[j];

        cv::Mat pointPos = Converter::toCvMat(pMP->GetWorldPos());
        cv::Mat mDis = mAveCenter3D - pointPos;

        float fDis = sqrt(mDis.at<float>(0) * mDis.at<float>(0)
                            + mDis.at<float>(1) * mDis.at<float>(1)
                            + mDis.at<float>(2) * mDis.at<float>(2));

        float th = 1.0;
        if (mvObject_2ds.size() > 5)
            th = 0.9;

        if (fDis > th * mCuboid3D.mfRMax)
            continue;

        pMP->object_mnId = mnId;
        pMP->object_class = mnClass;

        // Record the number of times this point is seen by this object3d
        map<int, int>::iterator sit;
        sit = pMP->viewdCount_forObjectId.find(this->mnId);
        if (sit != pMP->viewdCount_forObjectId.end())
        {
            int sit_sec = sit->second;
            pMP->viewdCount_forObjectId.erase(this->mnId);   // zhang reported an error
            pMP->viewdCount_forObjectId.insert(make_pair(this->mnId, sit_sec + 1));
        }
        else
        {
            pMP->viewdCount_forObjectId.insert(make_pair(this->mnId, 1));
        }

        // notes: Check for duplicate map points and add new map points
        {
            unique_lock<mutex> lock(mMutexMapPoints);
            bool new_point = true;
            // old points.
            for (size_t m = 0; m < mvpMapObjectMappoints.size(); ++m)
            {
                cv::Mat obj_curr_pos = Converter::toCvMat(pMP->GetWorldPos());
                cv::Mat obj_map_pos = Converter::toCvMat(mvpMapObjectMappoints[m]->GetWorldPos());

                if (cv::countNonZero(obj_curr_pos - obj_map_pos) == 0)
                {
                    mvpMapObjectMappoints[m]->feature_uvCoordinate = pMP->feature_uvCoordinate;
                    new_point = false;
                    break;
                }
            }
            // new point.
            if (new_point)
            {
                mvpMapObjectMappoints.push_back(pMP);
                mvpMapObjectMappoints_NewForActive.push_back(pMP);

                cv::Mat x3d = Converter::toCvMat(pMP->GetWorldPos());
                mSumPointsPos += x3d;
            }
        }
    }

    // step 4. the historical point cloud is projected into the image,
    // and the points not in the box(should not on the edge) are removed.
    // Project the historical point into the image, if not in the box, propose
    if ((Object_2d->mBox_cvRect.x > 25) && (Object_2d->mBox_cvRect.y > 25) &&
        (Object_2d->mBox_cvRect.x + Object_2d->mBox_cvRect.width < mCurrentFrame.mColorImage.cols - 25) &&
        (Object_2d->mBox_cvRect.y + Object_2d->mBox_cvRect.height < mCurrentFrame.mColorImage.rows - 25))
    {
        unique_lock<mutex> lock(mMutexMapPoints); // lock.
        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            // Find the object category on the current object point in the map
            int sit_sec = 0;
            map<int , int>::iterator sit;
            sit = (*pMP)->viewdCount_forObjectId.find(mnId);
            if (sit != (*pMP)->viewdCount_forObjectId.end())
            {
                sit_sec = sit->second;
            }
            if (sit_sec > 8)
            {
                ++pMP;
                continue;
            }

            cv::Mat PointPosWorld = Converter::toCvMat((*pMP)->GetWorldPos());
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            if ((u > 0 && u < mCurrentFrame.mColorImage.cols) && (v > 0 && v < mCurrentFrame.mColorImage.rows))
            {
                if (!Object_2d->mBox_cvRect.contains(cv::Point2f(u, v)))
                {
                    pMP = mvpMapObjectMappoints.erase(pMP);
                    mSumPointsPos -= PointPosWorld;
                }
                else
                {
                    ++pMP;
                }
            }
            else
            {
                ++pMP;
            }
        }
    }

    // step 5. update object mean.
    this->ComputeMeanAndDeviation_3D();

    // step 6. i-Forest.
    this->IsolationForestDeleteOutliers();

    mCurrentFrame.mvObject_2ds.push_back(Object_2d);
    std::cout   << "[Object] Successfully merged with the old object, cude h:" << this->mCuboid3D.height
                << ", cude w:" << this->mCuboid3D.width
                << ", cude l:" << this->mCuboid3D.lenth
                << std::endl;
    return true;
}


bool Object_Map::WhetherOverlap(Object_Map *CompareObj)
{
    // distance between two centers.
    float dis_x = abs(mCuboid3D.cuboidCenter(0) - CompareObj->mCuboid3D.cuboidCenter(0));
    float dis_y = abs(mCuboid3D.cuboidCenter(1) - CompareObj->mCuboid3D.cuboidCenter(1));
    float dis_z = abs(mCuboid3D.cuboidCenter(2) - CompareObj->mCuboid3D.cuboidCenter(2));

    float sum_lenth_half = mCuboid3D.lenth / 2 + CompareObj->mCuboid3D.lenth / 2;
    float sum_width_half = mCuboid3D.width / 2 + CompareObj->mCuboid3D.width / 2;
    float sum_height_half = mCuboid3D.height / 2 + CompareObj->mCuboid3D.height / 2;

    // whether overlap.
    if ((dis_x < sum_lenth_half) && (dis_y < sum_width_half) && (dis_z < sum_height_half))
        return true;
    else
        return false;
}

// ************************************
// object3d localmap
// ************************************
void Object_Map::SearchAndMergeMapObjs_fll(Map *mpMap)
{
    unique_lock<mutex> lock(mMutexObj);

    map<int, int>::iterator sit;
    std::vector<Object_Map*> obj_3ds = mpMap->GetObjects();
    for (sit = mReObj.end(); sit != mReObj.begin(); sit--)
    {
        int nObjId = sit->first;
        Object_Map* obj_ass = obj_3ds[nObjId];  // new bug: You can first check whether it is a fee control Check whether nObjId is greater than obj_3ds.size(). Or remove this line
        if (sit->second < 3)
            continue;

        if (obj_ass->bad_3d)
            continue;

        // Through the double-sample Ttest test, verify whether it is the same object
        bool bDoubelTtest = this->DoubleSampleTtest_fll(obj_ass);
        bool bSametime = true;

        // make sure they don't appear at the same time.
        // Query whether two objects appear at the same time.
        map<int, int>::iterator sit2;
        sit2 = mmAppearSametime.find(nObjId);
        if (sit2 != mmAppearSametime.end())
        {
            continue;
        }
        else
            bSametime = false;

        // If it is satisfied, it does not appear at the same time, and if it passes the double-sample Ttest, it will be integrated. The party with the most observed times (the party with more obj2d) will be retained.
        if((!bSametime || bDoubelTtest))
        {
            int nAppearTimes1 = mvObject_2ds.size();
            int nAppearTimes2 = obj_ass->mvObject_2ds.size();

            if (nAppearTimes1 > nAppearTimes2)
            {
                this->MergeTwoMapObjs_fll(obj_ass);
                this->ComputeMeanAndDeviation_3D();
                this->IsolationForestDeleteOutliers();
                obj_ass->bad_3d = true;
            }
            else
            {
                obj_ass->MergeTwoMapObjs_fll(this);
                obj_ass->ComputeMeanAndDeviation_3D();
                obj_ass->IsolationForestDeleteOutliers();
                this->bad_3d = true;
            }
        }
    }
}

bool Object_Map::DoubleSampleTtest_fll(semantic_slam::Object_Map *RepeatObj) {
    // Read t-distribution boundary value.
    float tTestData[122][9] = {0};
    ifstream infile;
    std::string filePath = WORK_SPACE_PATH + "/data/t_test.txt";
    infile.open(filePath);
    for (int i = 0; i < 122; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            infile >> tTestData[i][j];
        }
    }
    infile.close();

    int ndf1 = this->mvObject_2ds.size();
    float fMean1_x = this->mAveCenter3D.at<float>(0, 0);
    float fMean1_y = this->mAveCenter3D.at<float>(1, 0);
    float fMean1_z = this->mAveCenter3D.at<float>(2, 0);
    float fCenterStandar1_x = this->mCenterStandar_x;
    float fCenterStandar1_y = this->mCenterStandar_y;
    float fCenterStandar1_z = this->mCenterStandar_z;

    int ndf2 = RepeatObj->mvObject_2ds.size();
    float fMean2_x = RepeatObj->mAveCenter3D.at<float>(0, 0);
    float fMean2_y = RepeatObj->mAveCenter3D.at<float>(1, 0);
    float fMean2_z = RepeatObj->mAveCenter3D.at<float>(2, 0);
    float fCenterStandar2_x = RepeatObj->mCenterStandar_x;
    float fCenterStandar2_y = RepeatObj->mCenterStandar_y;
    float fCenterStandar2_z = RepeatObj->mCenterStandar_z;

    // Combined standard deviation.
    float d_x = sqrt( ( ( (ndf1-1)*fMean1_x*fMean1_x + (ndf2-1)*fMean2_x*fMean2_x ) / (ndf1 + ndf2 - 2) ) * (1/ndf1 + 1/ndf2) );
    float d_y = sqrt( ( ( (ndf1-1)*fMean1_y*fMean1_y + (ndf2-1)*fMean2_y*fMean2_y ) / (ndf1 + ndf2 - 2) ) * (1/ndf1 + 1/ndf2) );
    float d_z = sqrt( ( ( (ndf1-1)*fMean1_z*fMean1_z + (ndf2-1)*fMean2_z*fMean2_z ) / (ndf1 + ndf2 - 2) ) * (1/ndf1 + 1/ndf2) );

    // t-test
    float t_test_x = ( fMean1_x -fMean2_x ) / d_x;
    float t_test_y = ( fMean1_y -fMean2_y ) / d_y;
    float t_test_z = ( fMean1_z -fMean2_z ) / d_z;

    // Satisfy t test in 3 directions.
    if ((t_test_x < tTestData[min((ndf1 + ndf2 - 2), 121)][5]) &&
        (t_test_y < tTestData[min((ndf1 + ndf2 - 2), 121)][5]) &&
        (t_test_z < tTestData[min((ndf1 + ndf2 - 2), 121)][5]))
    {
        return true;
    }
    else
        return false;
}

void Object_Map::MergeTwoMapObjs_fll(Object_Map *RepeatObj)
{
    // step 1. Add RepeatObj to the current obj3d
    // update points.
    for (int i = 0; i < RepeatObj->mvpMapObjectMappoints.size(); i++)
    {
        MapPoint *pMP = RepeatObj->mvpMapObjectMappoints[i];

        if(pMP->isBad())
            continue;

        cv::Mat pointPos = Converter::toCvMat(pMP->GetWorldPos());
        Eigen::Vector3d scale = Converter::toSE3Quat(this->mCuboid3D.pose_mat).inverse() * Converter::toVector3d(pointPos);
        if ((abs(scale[0]) > 1.1 * this->mCuboid3D.lenth / 2) ||
            (abs(scale[1]) > 1.1 * this->mCuboid3D.width / 2) ||
            (abs(scale[2]) > 1.1 * this->mCuboid3D.height / 2))
        {
            continue;
        }

        pMP->object_mnId = mnId;
        pMP->object_class = mnClass;

        map<int, int>::iterator sit;
        sit = pMP->viewdCount_forObjectId.find(pMP->object_mnId);
        // The point is +1 by the number of times this obj3d
        if (sit != pMP->viewdCount_forObjectId.end())
        {
            int sit_sec = sit->second;
            pMP->viewdCount_forObjectId.erase(pMP->object_mnId);
            pMP->viewdCount_forObjectId.insert(make_pair(pMP->object_mnId, sit_sec + 1));
        }
        else
        {
            pMP->viewdCount_forObjectId.insert(make_pair(pMP->object_mnId, 1));
        }
        {
            unique_lock<mutex> lock(mMutexMapPoints);
            bool new_point = true;
            // old points.
            for (size_t m = 0; m < mvpMapObjectMappoints.size(); ++m)
            {
                cv::Mat obj_curr_pos = Converter::toCvMat(pMP->GetWorldPos());
                cv::Mat obj_map_pos = Converter::toCvMat(mvpMapObjectMappoints[m]->GetWorldPos());

                if (cv::countNonZero(obj_curr_pos - obj_map_pos) == 0)
                {
                    mvpMapObjectMappoints[m]->feature_uvCoordinate = pMP->feature_uvCoordinate;
                    new_point = false;

                    break;
                }
            }
            // new points.
            if (new_point)
            {
                mvpMapObjectMappoints.push_back(pMP);
                mvpMapObjectMappoints_NewForActive.push_back(pMP);
                cv::Mat x3d = Converter::toCvMat(pMP->GetWorldPos());
                mSumPointsPos += x3d;
            }
        }
    }

    // step 2. update frame objects.
    //int end;
    //if( RepeatObj->mvObject_2ds.size() > 10)
    //    end = RepeatObj->mvObject_2ds.size()-10;
    //else
    //    end = 0;
    for (int j=RepeatObj->mvObject_2ds.size()-1; j>=0; j--)
    {
        //Object_2D *ObjectFrame = RepeatObj->mvObject_2ds[j];
        //auto ObjectFrame = new Object_2D(RepeatObj->mvObject_2ds[j]);
        if(RepeatObj->mvObject_2ds[j]->bad)
            continue;
        //auto ObjectFrame = new Object_2D(RepeatObj->mvObject_2ds[j]);
        //Object_2D *ObjectFrame = RepeatObj->mvObject_2ds[j];
        //std::cout << "2  ";
        //ObjectFrame = RepeatObj->mvObject_2ds[j];std::cout<<"3  ";
        //ObjectFrame->mnId = mnId;std::cout<<"4  ";
        Object_2D *ObjectFrame = RepeatObj->mvObject_2ds[j];
        ObjectFrame->mnId = mnId;
        mnConfidence_foractive++;

        AddObj2d(ObjectFrame);//this->mvObject_2ds.push_back(ObjectFrame);
    }

    // step 3. Add the RepeatObj common view relationship to the current obj3d
    // the co-view relationship
    {
        map<int, int>::iterator sit;
        for (sit = RepeatObj->mmAppearSametime.begin(); sit != RepeatObj->mmAppearSametime.end(); sit++)
        {
            int nObjId = sit->first;
            int sit_sec = sit->second;

            map<int, int>::iterator sit2;
            sit2 = mmAppearSametime.find(nObjId);
            if (sit2 != mmAppearSametime.end())
            {
                int sit_sec2 = sit2->second;
                mmAppearSametime.erase(nObjId);
                mmAppearSametime.insert(make_pair(nObjId, sit_sec2 + sit_sec));
            }
            else
                mmAppearSametime.insert(make_pair(nObjId, 1));
        }
    }

    // step 4. Update the id of the last observed frame of the current obj3d
    // update the last observed frame.
    int nOriginLastAddID = mnLastAddID;
    int nOriginLastLatsAddID = mnLastLastAddID;
    cv::Rect OriginLastRect = mLastRect;
    // If this object appears later, last will not be updated. Check if lastlast needs to be updated
    // this object appeared recently.
    //TODO: Are the following numbers minus 1? Answer: No. Because mnLastAddID = mCurrentFrame.mnId, LastAddID is the latest observation frame
    if (mnLastAddID > RepeatObj->mnLastAddID)
    {
        if (nOriginLastLatsAddID < RepeatObj->mnLastAddID)
        {
            mnLastLastAddID = RepeatObj->mnLastAddID;
            mLastLastRect = RepeatObj->mvObject_2ds[  RepeatObj->mvObject_2ds.size()-1  ]->mBox_cvRect;
        }
    }
    // If RepeatObj appears later, update last
    // RepeatObj appeared recently.
    else
    {
        mnLastAddID = RepeatObj->mnLastAddID;
        mLastRect = RepeatObj->mvObject_2ds[RepeatObj->mvObject_2ds.size() - 1]->mBox_cvRect;

        // Check if lastlast needs to be updated:
        // If lastlast appears later, it will not be updated
        if (nOriginLastAddID > RepeatObj->mnLastLastAddID)
        {
            mnLastLastAddID = nOriginLastAddID;
            mLastLastRect = OriginLastRect;
        }
        // If the lastlast of RepeatObj appears later, update
        else if(RepeatObj->mvObject_2ds.size() >=  2 )
        {
            mnLastLastAddID = RepeatObj->mnLastLastAddID;
            std::cout << "[mergy debug]: size:" << RepeatObj->mvObject_2ds.size() << std::endl;
            std::cout << "[mergy debug]: width:" << RepeatObj->mvObject_2ds[RepeatObj->mvObject_2ds.size() - 2]->mBox_cvRect.width << std::endl;
            std::cout << "[mergy debug]: height:" << RepeatObj->mvObject_2ds[RepeatObj->mvObject_2ds.size() - 2]->mBox_cvRect.height << std::endl;
            mLastLastRect = RepeatObj->mvObject_2ds[RepeatObj->mvObject_2ds.size() - 2]->mBox_cvRect;
        }
    }

    // step 5. update direction.
    // TODO: Modification type number
    // 62, 58, 41, 77, 66, 75, 64, 45, 56, 60
    if (  (mnClass == 73) /*book*/ || (mnClass == 64) /*mouse*/ || (mnClass == 65)  /*remote*/
        || (mnClass == 66) /*keyboard*/ || (mnClass == 56) /*chair*/ || (mnClass == 62)
        || (mnClass == 58) || (mnClass == 41)
        || (mnClass == 77) || (mnClass == 45) || (mnClass == 60))
    {
        if(RepeatObj->mvAngleTimesAndScore.size() > 0)
        {
            for (auto &row_repeat : RepeatObj->mvAngleTimesAndScore)
            {
                bool new_measure = true;

                if(this->mvAngleTimesAndScore.size() > 0)
                {
                    for (auto &row_this : this->mvAngleTimesAndScore)
                    {
                        if(row_repeat[0] == row_this[0])
                        {
                            row_this[1] += row_repeat[1];

                            row_this[2] = row_this[2] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[2] * (row_repeat[1] / row_this[1]);

                            row_this[3] = row_this[3] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[3] * (row_repeat[1] / row_this[1]);

                            row_this[4] = row_this[4] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[4] * (row_repeat[1] / row_this[1]);

                            new_measure = false;
                            break;
                        }
                    }
                }

                if(new_measure == true)
                {
                    this->mvAngleTimesAndScore.push_back(row_repeat);
                }
            }
        }

        if(this->mvAngleTimesAndScore.size() > 0)
        {
            int best_num = 0;
            float best_score = 0.0;
            for(int i = 0; i < min(6, (int)this->mvAngleTimesAndScore.size()); i++)
            {
                float fScore = this->mvAngleTimesAndScore[i][2];
                if(fScore > best_score)
                {
                    best_score = fScore;
                    best_num = i;
                }
            }

            this->mCuboid3D.rotY = this->mvAngleTimesAndScore[best_num][0];
            this->mCuboid3D.mfErrorParallel = this->mvAngleTimesAndScore[best_num][3];
            this->mCuboid3D.mfErroeYaw = this->mvAngleTimesAndScore[best_num][4];
            this->Update_Twobj();
        }
    }
}


// Fusion of two overlapping objects. To go through iou, volume, AppearSametime, yolo class, four tests
void Object_Map::DealTwoOverlapObjs_fll(semantic_slam::Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z) {
    bool bIou = false;       // false: Iou is large.
    bool bVolume = false;    // false: small volume difference.
    bool bSame_time = false; // false: doesn't simultaneous appearance.
    bool bClass = false;     // false: different classes.

    // volume of two objects
    float fThis_obj_volume = (mCuboid3D.lenth * mCuboid3D.width) * mCuboid3D.height;
    float fOverlap_obj_volume = (OverlapObj->mCuboid3D.lenth * OverlapObj->mCuboid3D.width) * OverlapObj->mCuboid3D.height;

    // Whether the volume Iou is greater than 0.3
    float overlap_volume = (overlap_x * overlap_y) * overlap_z;
    if ((overlap_volume / (fThis_obj_volume + fOverlap_obj_volume - overlap_volume)) >= 0.3)
        bIou = true;
    else
        bIou = false;

    // Whether the volume difference is too large, the volume difference is more than two times,
    // compute the volume difference
    if ((fThis_obj_volume > 2 * fOverlap_obj_volume) || (fOverlap_obj_volume > 2 * fThis_obj_volume))
        bVolume = true;
    else
        bVolume = false;

    // If the number of times two objects appear at the same time is greater than 3,
    // whether simultaneous appearance.
    map<int, int>::iterator sit;
    sit = mmAppearSametime.find(OverlapObj->mnId);
    if (sit != mmAppearSametime.end())
    {
        if (sit->second > 3)
            bSame_time = true;
        else
            bSame_time = false;
    }
    else
        bSame_time = false;

    // class Is it the same
    if (mnClass == OverlapObj->mnClass)
        bClass = true;
    else
        bClass = false;


    // case 1: IOU is large, the volume difference is small, doesn't simultaneous appearance, same class --> the same object, merge them.
    if ((bIou == true) && (bVolume == false) && (bSame_time == false) && (bClass == true))
    {
        if (this->mvObject_2ds.size() >= OverlapObj->mvObject_2ds.size())
        {
            this->MergeTwoMapObjs_fll(OverlapObj);
            OverlapObj->bad_3d = true;
        }
        else
        {
            OverlapObj->MergeTwoMapObjs_fll(this);
            this->bad_3d = true;
        }
    }

    // case 2: may be a false detection.
    else if ((bVolume == true) && (bSame_time == false) && (bClass == true))
    {
        if ((this->mvObject_2ds.size() >= OverlapObj->mvObject_2ds.size()) && (fThis_obj_volume > fOverlap_obj_volume))
            OverlapObj->bad_3d = true;
        else if ((this->mvObject_2ds.size() < OverlapObj->mvObject_2ds.size()) && (fThis_obj_volume < fOverlap_obj_volume))
            this->bad_3d = true;
    }

    // case 3: divide the overlap area of two objects equally.  (No significant effect.)
    else if ((bIou == true) && (bVolume == false) && (bSame_time == true) && (bClass == true))
    {
        this->DivideEquallyTwoObjs_fll(OverlapObj, overlap_x, overlap_y, overlap_z);
        OverlapObj->DivideEquallyTwoObjs_fll(OverlapObj, overlap_x, overlap_y, overlap_z);

        this->ComputeMeanAndDeviation_3D();
        OverlapObj->ComputeMeanAndDeviation_3D();
    }

    // case 4: big one gets smaller, the smaller one stays the same. (No significant effect.)
    else if ((bIou == false) && (bVolume == true) && (bSame_time == true) && (bClass == false))
    {
        if (fThis_obj_volume > fOverlap_obj_volume)
            this->BigToSmall_fll(OverlapObj, overlap_x, overlap_y, overlap_z);
        else if (fThis_obj_volume < fOverlap_obj_volume)
            OverlapObj->BigToSmall_fll(this, overlap_x, overlap_y, overlap_z);
    }

    // case 5:
    else if((bIou == true) && (bSame_time == false) && (bClass == true))
    {
        if (this->mvObject_2ds.size()/2 >= OverlapObj->mvObject_2ds.size())
        {
            this->MergeTwoMapObjs_fll(OverlapObj);
            OverlapObj->bad_3d = true;
        }
        else if(OverlapObj->mvObject_2ds.size()/2 >= this->mvObject_2ds.size())
        {
            OverlapObj->MergeTwoMapObjs_fll(this);
            this->bad_3d = true;
        }
    }
    // TODO: case ...... and so on ......
}

// big one gets smaller, the smaller one stays the same. (No significant effect! Almost no use.)
void Object_Map::BigToSmall_fll(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z)
{
    // in which direction does the overlap occur.
    bool bxMin = false;
    bool bxMax = false;
    bool byMin = false;
    bool byMax = false;
    bool bzMin = false;
    bool bzMax = false;

    // x_min
    if ((SmallObj->mCuboid3D.x_min > this->mCuboid3D.x_min) && (SmallObj->mCuboid3D.x_min < this->mCuboid3D.x_max))
        bxMin = true;
    // x_max
    if ((SmallObj->mCuboid3D.x_max > this->mCuboid3D.x_min) && (SmallObj->mCuboid3D.x_max < this->mCuboid3D.x_max))
        bxMax = true;
    // y_min
    if ((SmallObj->mCuboid3D.y_min > this->mCuboid3D.y_min) && (SmallObj->mCuboid3D.y_min < this->mCuboid3D.y_max))
        byMin = true;
    // y_max
    if ((SmallObj->mCuboid3D.y_max > this->mCuboid3D.y_min) && (SmallObj->mCuboid3D.y_max < this->mCuboid3D.y_max))
        byMax = true;
    // z_min
    if ((SmallObj->mCuboid3D.z_min > this->mCuboid3D.z_min) && (SmallObj->mCuboid3D.z_min < this->mCuboid3D.z_max))
        bzMin = true;
    // z_max
    if ((SmallObj->mCuboid3D.z_max > this->mCuboid3D.z_min) && (SmallObj->mCuboid3D.z_max < this->mCuboid3D.z_max))
        bzMax = true;

    // false: one direction，ture: two directions.
    bool bx = false;
    bool by = false;
    bool bz = false;
    // x
    if ((bxMin = true) && (bxMax = true))
        bx = true;
    else
        bx = false;
    // y
    if ((byMin = true) && (byMax = true))
        by = true;
    else
        by = false;
    // z
    if ((bzMin = true) && (bzMax = true))
        bz = true;
    else
        bz = false;

    // Which direction to eliminate?
    int nFlag; // 0:x   1:y   2:z   3: surround

    // x
    if ((bx == false) && (by == true) && (bz == true))
        nFlag = 0;
    // y
    if ((bx == true) && (by == false) && (bz == true))
        nFlag = 1;
    // z
    if ((bx == true) && (by == true) && (bz == false))
        nFlag = 2;

    if ((bx == false) && (by == false) && (bz == true))
    {
        if (min(overlap_x, overlap_y) == overlap_x)
            nFlag = 0;
        else if (min(overlap_x, overlap_y) == overlap_y)
            nFlag = 1;
    }

    if ((bx == false) && (by == true) && (bz == false))
    {
        if (min(overlap_x, overlap_z) == overlap_x)
            nFlag = 0;
        else if (min(overlap_x, overlap_z) == overlap_z)
            nFlag = 2;
    }

    if ((bx == true) && (by == false) && (bz == false))
    {
        if (min(overlap_y, overlap_z) == overlap_y)
            nFlag = 1;
        else if (min(overlap_y, overlap_z) == overlap_z)
            nFlag = 2;
    }

    //     7------6
    //    /|     /|
    //   / |    / |
    //  4------5  |
    //  |  3---|--2
    //  | /    | /
    //  0------1
    {
        unique_lock<mutex> lock(mMutexMapPoints); // lock.

        // remove points in the overlap volume.
        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            cv::Mat PointPosWorld = Converter::toCvMat((*pMP)->GetWorldPos());

            // points in the smaller object.
            if ((PointPosWorld.at<float>(0) > SmallObj->mCuboid3D.x_min) && (PointPosWorld.at<float>(0) < SmallObj->mCuboid3D.x_max) &&
                (PointPosWorld.at<float>(1) > SmallObj->mCuboid3D.y_min) && (PointPosWorld.at<float>(1) < SmallObj->mCuboid3D.y_max) &&
                (PointPosWorld.at<float>(2) > SmallObj->mCuboid3D.z_min) && (PointPosWorld.at<float>(2) < SmallObj->mCuboid3D.z_max))
                pMP = mvpMapObjectMappoints.erase(pMP);
            else
            {
                ++pMP;
            }
        }
    }

    this->ComputeMeanAndDeviation_3D();
}


// BRIEF Divide the overlap area of two objects equally.  (No significant effect! Almost no use.)
void Object_Map::DivideEquallyTwoObjs_fll(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z)
{
    {
        unique_lock<mutex> lock(mMutexMapPoints);

        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            cv::Mat PointPosWorld = Converter::toCvMat((*pMP)->GetWorldPos());
            float cuboidCenter0_ano = (AnotherObj->mCuboid3D.corner_2[0] + AnotherObj->mCuboid3D.corner_8[0])/2.0;
            float cuboidCenter1_ano = (AnotherObj->mCuboid3D.corner_2[1] + AnotherObj->mCuboid3D.corner_8[1]) / 2.0;
            float cuboidCenter2_ano = (AnotherObj->mCuboid3D.corner_2[2] + AnotherObj->mCuboid3D.corner_8[2])/2.0 ;
            if (((PointPosWorld.at<float>(0) > cuboidCenter0_ano - (AnotherObj->mCuboid3D.lenth / 2 - overlap_x / 2)) &&
                 (PointPosWorld.at<float>(0) < cuboidCenter0_ano + (AnotherObj->mCuboid3D.lenth / 2 - overlap_x / 2))) &&
                ((PointPosWorld.at<float>(1) > cuboidCenter1_ano - (AnotherObj->mCuboid3D.width / 2 - overlap_y / 2)) &&
                 (PointPosWorld.at<float>(1) < cuboidCenter1_ano + (AnotherObj->mCuboid3D.width / 2 - overlap_y / 2))) &&
                ((PointPosWorld.at<float>(2) > cuboidCenter2_ano - (AnotherObj->mCuboid3D.height / 2 - overlap_z / 2)) &&
                 (PointPosWorld.at<float>(2) < cuboidCenter2_ano + (AnotherObj->mCuboid3D.height / 2 - overlap_z / 2))))
            {
                pMP = mvpMapObjectMappoints.erase(pMP);
            }

            else
            {
                ++pMP;
            }
        }
    }
}

// ************************************
// object3d Information entropy calculation part 
// ************************************

Object_Map::Object_Map() {
    cv::FileStorage fSettings( WORK_SPACE_PATH + "config/RGB-D/" + yamlfile_object, cv::FileStorage::READ);
    if(!fSettings.isOpened())
	{
		cout << "Failed to open settings file at: " << WORK_SPACE_PATH + "config/RGB-D/" + yamlfile_object << endl;
	}
//	else cout << "success to open file at: " << WORK_SPACE_PATH + "/config/RGB-D/" + yamlfile_object << endl;

    mIE_rows = fSettings["IE.rows"];
    mIE_cols = fSettings["IE.cols"];
//    std::cout << "IE_RecoverInit:2  "
//                << ", " << mIE_rows
//                << ", " << mIE_cols
//                << std::endl;
    mP_occ = fSettings["IE.P_occ"];
    mP_free = fSettings["IE.P_free"];
    mP_prior = fSettings["IE.P_prior"];
    mIEThreshold = fSettings["IE.Threshold"];
    IE_RecoverInit();
}

double Object_Map::IE(const double &p){
    return -1*p*log(p) - (1-p)*log(1-p);
}

void Object_Map::IE_RecoverInit() {
        //mCuboid3D = object->mCuboid3D;
        //mvpMapObjectMappoints = object->mvpMapObjectMappoints;

        //vector<int> PointNum_OneColu(mIE_rows, 0.0) ;
        //vector<vector<int> >  PointNum(mIE_cols, PointNum_OneColu);
        //mvPointNum = PointNum;
        mvPointNum_mat = cv::Mat::zeros(mIE_rows,mIE_cols,CV_32F);

        //vector<double> GridProb_OneColu(mIE_rows, mP_prior) ;
        //vector<vector<double> > GridProb(mIE_cols, GridProb_OneColu);
        //mvGridProb = GridProb;
        mvGridProb_mat = cv::Mat::ones(mIE_rows,mIE_cols,CV_32F);
        mvGridProb_mat *= mP_prior;

        //vector<double> InforEntroy_OneColu(mIE_rows, IE(mP_prior) ) ;
        //vector<vector<double> > InforEntroy(mIE_cols, InforEntroy_OneColu);
        //mvInforEntroy = InforEntroy;
        mvInforEntroy_mat = cv::Mat::ones(mIE_rows,mIE_cols,CV_32F);
        mvInforEntroy_mat *= IE(mP_prior);
    }


//
void Object_Map::compute_grid_xy(const Eigen::Vector3d &zero_vec, const Eigen::Vector3d &point_vec, int& x, int& y){

    Eigen::Vector3d v1(zero_vec(0),zero_vec(1),0.0), v2(point_vec(0),point_vec(1),0.0);
    double cosValNew = v1.dot(v2) / (v1.norm()*v2.norm()); 
    double angleNew = acos(cosValNew) * 180 / M_PI;     

    if(point_vec(1) >= 0)
        x = floor(angleNew/(360.0/mIE_cols));
    else
        x = floor((360.0-angleNew)/(360.0/mIE_cols));

    y = floor(
                (  (point_vec(2)  + mCuboid3D.height/2 ) /mCuboid3D.height) * mIE_rows
            );
}


cv::Mat Object_Map::compute_pointnum_eachgrid(){
    float cuboidCenter0 = (mCuboid3D.corner_2[0] + mCuboid3D.corner_8[0])/2.0;
    float cuboidCenter1 = (mCuboid3D.corner_2[1] + mCuboid3D.corner_8[1]) / 2.0;
    float cuboidCenter2 = (mCuboid3D.corner_2[2] + mCuboid3D.corner_8[2])/2.0 ;
    double center_x = cuboidCenter0;
    double center_y = cuboidCenter1;
    double center_z = cuboidCenter2;

    cv::Mat T_w2o_mat = mCuboid3D.pose_mat;
    Eigen::Isometry3d T_w2o = semantic_slam::Converter::toSE3Quat(T_w2o_mat);

    Eigen::Vector3d zero_vec( 1,0,0);
    zero_vec = T_w2o* zero_vec;

    for (int i = 0; i < mvpMapObjectMappoints.size(); ++i) {
        cv::Mat point_pose = Converter::toCvMat(mvpMapObjectMappoints[i]->GetWorldPos());
        Eigen::Vector3d point_vec( point_pose.at<float>(0)-center_x, point_pose.at<float>(1)-center_y, point_pose.at<float>(2)-center_z);
        int x = -1 , y = -1;
        compute_grid_xy(zero_vec, point_vec, x, y);
        if( x>=0 && x<=mIE_cols && y>=0 && y<=mIE_rows ) {
            int temp = mvPointNum_mat.at<float>(x,y);
            //std::cout<<"compute_pointnum_eachgrid1: " << temp <<std::endl;
            mvPointNum_mat.at<float>(x,y) = temp+1;
            //std::cout<<"compute_pointnum_eachgrid2: " << mvPointNum_mat.at<float>(x,y) <<std::endl;
        }
        else{
//            std::cout << "compute grid index: ERROR:i " << i << ", x " << x << ", y " << y << std::endl;
        }
    }

    return mvPointNum_mat;
}


void Object_Map::compute_occupied_prob_eachgrid(){

//    std::cout << "debug The number of points and occupation probability of each grid：";
    for(int x=0; x<mIE_rows; x++){

        int num_onecol = 0;
        for(int y=0; y<mIE_cols; y++){
            num_onecol +=  mvPointNum_mat.at<float>(x,y);
        }

        if(num_onecol > mIEThreshold){
            
            for(int y=0; y<mIE_rows; y++){
                double lnv_p ;
                int PointNum = mvPointNum_mat.at<float>(x,y);
                int ObserveNum = mvObject_2ds.size();
                //if(ObserveNum==0) ObserveNum=40;
                if(PointNum == 0){
                    //free
                    //lnv_p = log(mP_prior) + ObserveNum * (log(mP_free) -log(mP_prior));
                    lnv_p = ObserveNum * log( mP_free/ (1.0 - mP_free));

                }
                else{
                    //occupied
                    //lnv_p = log(mP_prior) + ObserveNum * (log(mP_occ) - log(mP_prior));
                    lnv_p = ObserveNum * log( mP_occ/ (1.0 - mP_occ));
                }
                //mvGridProb_mat.at<float>(x,y) = exp(lnv_p);
                double bel = 1.0 - 1.0 / (1.0 + exp(lnv_p));
                mvGridProb_mat.at<float>(x,y) = (float) bel;
//                std::cout << mvPointNum_mat.at<float>(x,y) << "(" << mvGridProb_mat.at<float>(x,y) << "," << ObserveNum << "," << lnv_p << ")， ";
            }
        }
        else{
             for(int y=0; y<mIE_rows; y++){
                 //unkonwn
                 mvGridProb_mat.at<float>(x,y) = mP_prior;
//                 std::cout << mvPointNum_mat.at<float>(x,y) << "(" << mvGridProb_mat.at<float>(x,y) << ")， ";
             }
        }
    }
//    std::cout << "   " << std::endl;
}

void Object_Map::ComputeIE(){
    IE_RecoverInit();
    compute_pointnum_eachgrid();
    compute_occupied_prob_eachgrid();

//    std::cout << "debug Occupancy probability of each grid：";
    for(int x=0; x<mIE_cols; x++)
        for(int y=0; y<mIE_rows; y++){
//            std::cout << mvGridProb_mat.at<float>(x,y) << "， ";
            mvInforEntroy_mat.at<float>(x,y) = IE(mvGridProb_mat.at<float>(x,y));
        }
//    std::cout << "" << std::endl;

    double entroy = 0;
    for(int x=0; x<mIE_cols; x++)
        for(int y=0; y<mIE_rows; y++){
            entroy += mvInforEntroy_mat.at<float>(x,y);
        }
    mIE = entroy/(mIE_cols*mIE_rows);

    double main_x, main_y, main_z;
    for (int i = 0; i < mvpMapObjectMappoints.size(); ++i) {
        cv::Mat point_pose = Converter::toCvMat(mvpMapObjectMappoints[i]->GetWorldPos());
        main_x +=  point_pose.at<float>(0) - this->mCuboid3D.cuboidCenter(0) ;
        main_y +=  point_pose.at<float>(1) - this->mCuboid3D.cuboidCenter(1) ;
        main_z +=  point_pose.at<float>(2) - this->mCuboid3D.cuboidCenter(2) ;
    }
    double normalize = sqrt( main_x*main_x + main_y*main_y + main_z*main_z );
    main_x = main_x/normalize;
    main_y = main_y/normalize;
    main_z = main_z/normalize;
    mMainDirection =  Eigen::Vector3d(main_x, main_y, main_z);

}


double Object_Map::get_information_entroy(){
    return mIE;
}

// ************************************
// object3d Filter Candidates
// ************************************
bool Object_Map::WheatherInRectFrameOf(const cv::Mat &Tcw, const float &fx, const float &fy, const float &cx, const float &cy, const float &ImageWidth, const float &ImageHeight)
{
    const cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    vector<float> x_pt;
    vector<float> y_pt;
    for (int j = 0; j < mvpMapObjectMappoints.size(); j++)
    {
        MapPoint *pMP = mvpMapObjectMappoints[j];
        cv::Mat PointPosWorld = Converter::toCvMat(pMP->GetWorldPos());

        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        const float xc = PointPosCamera.at<float>(0);
        const float yc = PointPosCamera.at<float>(1);
        const float invzc = 1.0 / PointPosCamera.at<float>(2);

        float u = fx * xc * invzc + cx;
        float v = fy * yc * invzc + cy;

        x_pt.push_back(u);
        y_pt.push_back(v);

    }

    if (x_pt.size() == 0)
        return false;

    sort(x_pt.begin(), x_pt.end());
    sort(y_pt.begin(), y_pt.end());
    float x_min = x_pt[0];
    float x_max = x_pt[x_pt.size() - 1];
    float y_min = y_pt[0];
    float y_max = y_pt[y_pt.size() - 1];

    if (x_min < 0)
        return false;
    if (y_min < 0)
        return false;
    if (x_max > ImageWidth)
        return false;
    if (y_max > ImageHeight)
        return false;
    //Camera.width: 640
    //Camera.height: 480
}

}