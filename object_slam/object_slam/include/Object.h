//
// Created by lacie on 11/02/23.
//

#ifndef OBJECT_H
#define OBJECT_H


#include "MapPoint.h"
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <algorithm>

#include <numeric>
#include <math.h>
#include <assert.h>
#include <iostream>

#include "YoloDetection.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "IsolationForest.h"


extern std::string WORK_SPACE_PATH;
extern std::string yamlfile_object;
extern bool MotionIou_flag;
extern bool NoPara_flag;
extern bool ProIou_flag;
extern bool Ttest_flag;
extern bool iforest_flag;
extern bool little_mass_flag;
extern bool ProIou_only30_flag;

namespace semantic_slam
{
class Frame;
class MapPoint;
class KeyFrame;
class Map;
class Object_Map;

class Object_2D;
enum eAssociateFlag{
        MotionIou=1,
        NoPara=2,
        t_test=3,
        ProIou=4 //Debug=4
    };


class Object_2D {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        Object_2D();

        //Object_2D(const Object_2D* obj2d){
        //    std::cout<<"Object_2D  construct 2   ";
        //    mclass_id = obj2d->mclass_id;
        //    mScore = obj2d->mScore;
        //    mleft = obj2d->mleft;
        //    mright = obj2d->mright;
        //    mtop = obj2d->mtop;
        //    mbottom = obj2d->mbottom;
        //    mWidth = obj2d->mWidth;
        //    mHeight = obj2d->mHeight;
        //    // 2d center.
        //    mBoxCenter_2d = obj2d->mBoxCenter_2d;
        //    // opencv Rect format.
        //    mBox_cvRect = obj2d->mBox_cvRect;
        //    mBox_cvRect_FeaturePoints = obj2d->mBox_cvRect_FeaturePoints;
        //    this->mpCurrentFrame = obj2d->mpCurrentFrame;
        //    this->mpMap = obj2d->mpMap;
        //
        //    
        //    this->sum_pos_3d = obj2d->sum_pos_3d;
        //    this->mPos_world = obj2d->mPos_world;
        //    this->mvMapPonits = obj2d->mvMapPonits;
        //    mReIdAndIou = obj2d->mReIdAndIou;
        //
        //    bad = false;
        //    std::cout<<">>>   End"<<std::endl;
        //};
        Object_2D(Map* mpMap, Frame *CurrentFrame, const BoxSE &box);
    public:
        //yolo
        int mclass_id;                                                  // class id.
        float mScore;                                                   // Probability.
        float mleft;                                                    // size.
        float mright;
        float mtop;
        float mbottom;
        float mWidth;
        float mHeight;
        cv::Rect mBox_cvRect;                   // cv::Rect format.
        cv::Rect mBox_cvRect_FeaturePoints;     // the bounding box constructed by object feature points.
                                                
        //BoxSE mBox_yoloSE;                      // BoxSE
        cv::Point2f mBoxCenter_2d;              // 2D center.

        // Frame and Map
        Frame* mpCurrentFrame;                  
        Map* mpMap;

        // coordinate
        cv::Mat sum_pos_3d;                         // Summation of points observed in the current frame.
        cv::Mat mPos_world;                         // current object center (3d, world). 
        //cv::Mat mCenter_ofAssMapObj;                // map object center.   
        //float mStandar_x, mStandar_y, mStandar_z; // standard deviation 

        //mappoint
        std::vector<MapPoint*>  mvMapPonits;             // object points in current frame. 

        int mnId;                                   // object ID.
        int confidence;
        bool bad = false;

        int nMayRepeat = 0;
        std::map<int, float> mReIdAndIou;           // potential objects.

        void AddYoloBoxes(const BoxSE &box);            // copy box to object_2d.
        void ComputeMeanAndDeviation();                 // compute the mean and standard deviation of object points in current frame.
        void RemoveOutlier_ByHeightorDepth();                  // remove outliers by boxplot.
        void MergeTwo_Obj2D(Object_2D *Old_Object2D);
        int Object2D_DataAssociationWith_Object3D();    // data association.
        int creatObject();
        int  NoParaDataAssociation(Object_Map* ObjectMapSingle); // NP.

        void AddObjectPoint(MapPoint *pMP);
        void AddPotentialAssociatedObjects( std::vector<Object_Map*> obj3ds, int AssoId, int beAssoId);

    protected:
        std::mutex mMutexObjMapPoints;  
        std::mutex mMutexPos;
        std::mutex mMutexObj;
    public:
        static std::mutex mGlobalMutex; 
    // line.
    public:
        Eigen::MatrixXd mObjLinesEigen;   

};


struct Cuboid3D{
    //     7------6
    //    /|     /|
    //   / |    / |
    //  4------5  |
    //  |  3---|--2
    //  | /    | /
    //  0------1
    // lenth ：corner_2[0] - corner_1[0]
    // width ：corner_2[1] - corner_3[1]
    // height：corner_2[2] - corner_6[2]

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    // 8 vertices.
    Eigen::Vector3d corner_1;
    Eigen::Vector3d corner_2;
    Eigen::Vector3d corner_3;
    Eigen::Vector3d corner_4;
    Eigen::Vector3d corner_5;
    Eigen::Vector3d corner_6;
    Eigen::Vector3d corner_7;
    Eigen::Vector3d corner_8;

    // 8 vertices (without rotation).
    Eigen::Vector3d corner_1_w;
    Eigen::Vector3d corner_2_w;
    Eigen::Vector3d corner_3_w;
    Eigen::Vector3d corner_4_w;
    Eigen::Vector3d corner_5_w;
    Eigen::Vector3d corner_6_w;
    Eigen::Vector3d corner_7_w;
    Eigen::Vector3d corner_8_w;

    Eigen::Vector3d cuboidCenter;       // the center of the Cube, not the center of mass of the object
    float x_min, x_max, y_min, y_max, z_min, z_max;     // the boundary in XYZ direction.

    float lenth;
    float width;
    float height;
    float mfRMax;      

    //g2o::SE3Quat pose ;                               // 6 dof pose.
    cv::Mat pose_mat = cv::Mat::eye(4, 4, CV_32F);      
    //g2o::SE3Quat pose_without_yaw;                    // 6 dof pose without rotation.
    cv::Mat pose_noyaw_mat = cv::Mat::eye(4, 4, CV_32F);
    // angle.
    float rotY = 0.0;
    float rotP = 0.0;
    float rotR = 0.0;

    // line.
    float mfErrorParallel;
    float mfErroeYaw;
};

class Object_Map{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    Object_Map();

public:
    std::vector<Object_2D* > mvObject_2ds;  

    cv::Rect mLastRect;
    cv::Rect mLastLastRect;
    cv::Rect mRect_byProjectPoints;    
    
    int mnAddedID_nouse;
    int mnLastAddID;
    int mnLastLastAddID;

public:
    
    int mnId;                           
    int mnClass;
    bool bad_3d = false;                
    int mnConfidence_foractive;
    Cuboid3D mCuboid3D;                 
    cv::Mat mSumPointsPos;
    cv::Mat mAveCenter3D;               
    float mStandar_x, mStandar_y, mStandar_z;
    float mCenterStandar_x, mCenterStandar_y, mCenterStandar_z;
    float mCenterStandar;

    std::vector<MapPoint*> mvpMapObjectMappoints;
    std::vector<MapPoint*> mvpMapObjectMappoints_NewForActive;

    std::map<int, int> mReObj;                                      
    std::map<int, int> mmAppearSametime;                            

    std::vector<Eigen::Matrix<float,5,1>, Eigen::aligned_allocator<Eigen::Matrix<float,5,1>>> mvAngleTimesAndScore;    // Score of sampling angle.

    void ComputeMeanAndDeviation_3D();
    void IsolationForestDeleteOutliers();
    void Update_Twobj();                                               
    void ComputeProjectRectFrameToCurrentFrame(Frame &Frame);
    cv::Rect ComputeProjectRectFrameToCurrentKeyFrame(KeyFrame &kF);
    bool WhetherOverlap(Object_Map *CompareObj);
    void UpdateCoView(Object_Map *Obj_CoView);

    void AddObj2d(Object_2D* Object_2d){
        std::unique_lock<std::mutex> lock(mMutexObj2d);
        this->mvObject_2ds.push_back(Object_2d);
    }

    bool operator ==(const int &x)
    {
        return(this->mnClass == x);
    }

    // ************************************
    // object3d track
    bool UpdateToObject3D(Object_2D* Object_2d, Frame &mCurrentFrame, int Flag);

protected:
    std::mutex mMutexMapPoints;

// ************************************
// object3d localmap
public:
    static std::mutex mMutex_front_back;                                
    void SearchAndMergeMapObjs_fll(Map *mpMap);
    void MergeTwoMapObjs_fll(Object_Map *RepeatObj);
    bool DoubleSampleTtest_fll(Object_Map *RepeatObj);
    void DealTwoOverlapObjs_fll(Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
    void BigToSmall_fll(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z);
    void DivideEquallyTwoObjs_fll(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z);


// ************************************
// object3d 
public:
    int mIE_rows, mIE_cols;

    void IE_RecoverInit();
    int mIEThreshold;                                       
    Eigen::Vector3d mMainDirection;                         
    double mStatistics = 1;                                 

    double mP_occ;
    double mP_free;
    double mP_prior;

    double mIE;
    //std::vector<std::vector<double> > mvInforEntroy;                   
    cv::Mat mvInforEntroy_mat;
    //std::vector<std::vector<double> > mvGridProb;                     
    cv::Mat mvGridProb_mat;
    //std::vector<std::vector<int> > mvPointNum;                
    cv::Mat mvPointNum_mat;

    void compute_grid_xy(const Eigen::Vector3d &zero_vec, const Eigen::Vector3d &point_vec, int& x, int& y);
    cv::Mat compute_pointnum_eachgrid();
    void compute_occupied_prob_eachgrid();
    double IE(const double &p);
    void ComputeIE();
    double get_information_entroy();
    std::vector<MapPoint* >  GetObjectMappoints();
    std::vector<MapPoint* >  GetNewObjectMappoints();
protected:
    std::mutex mMutexPose;
    std::mutex mMutexObj2d;
    std::mutex mMutexObj;

public:
    bool WheatherInRectFrameOf(const cv::Mat &Tcw, const float &fx, const float &fy, const float &cx, const float &cy, const float &ImageWidth, const float &ImageHeight);  //计算到任意帧的投影
};

}

#endif //OBJECT_H
