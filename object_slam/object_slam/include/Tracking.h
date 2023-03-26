#ifndef TRACKING_H
#define TRACKING_H

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

// #include "Detector.h"
#include "PointCloudMapping.h"
#include "Flow.h"
#include "Geometry.h"
#include "YoloDetection.h"
#include "Object.h"
#include "MapPublisher.h"

#include "GeometricCamera.h"

#include <boost/make_shared.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <unordered_set>
#include <unordered_map>

// cube
#include "detect_3d_cuboid/matrix_utils.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"

// line
#include <line_lbd/line_descriptor.hpp>
#include <line_lbd/line_lbd_allclass.h>

class PointCloudMapping;
//class Detector;
class detect_3d_cuboid;

namespace semantic_slam
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;
class Object_Map;
class MapPublisher;
class MapCuboidObject;

class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings,
             MapPublisher* pMapPublisher, const string &_nameSeq=std::string());

    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             boost::shared_ptr<PointCloudMapping> pPointCloud,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq=std::string()
             /*,std::shared_ptr<Detector> pDetector = nullptr*/
             );

    ~Tracking();

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);
    bool ParseCUBEParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();
    void SetDetector(YoloDetection* pDetector);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    Sophus::SE3f GetCamTwc();
    Sophus::SE3f GetImuTwb();
    Eigen::Vector3f GetImuVwb();
    bool isImuPreintegrated();

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    //DEBUG
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    float GetImageScale();

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;
    cv::Mat mImDepth; // added to realize pointcloud view
    cv::Mat mImRGB; // added for color point map
    cv::Mat mImMask;
    float mFlowThreshold;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;


    vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    Eigen::Quaternionf currentQ = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
    Eigen::Quaternionf currentP = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);

#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;
    vector<double> vdORBExtract_ms;
    vector<double> vdStereoMatch_ms;
    vector<double> vdIMUInteg_ms;
    vector<double> vdPosePred_ms;
    vector<double> vdLMTrack_ms;
    vector<double> vdNewKF_ms;
    vector<double> vdTrackTotal_ms;
#endif

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    void LightTrack();
    bool TrackHomo(cv::Mat& homo);

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    //void CreateNewMapPoints();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    bool LightTrackWithMotionModel(bool &bVO);

    bool Relocalization();
    bool Relocalization(bool save_change);

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    int NeedNewKeyFrame();
    void CreateNewKeyFrame(bool CreateByObjs);

    // For 3D cuboid
    // int NeedNewKeyFrame();
    // void CreateNewKeyFrame(bool CreateByObjs);

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    // std::vector<IMU::Point> mvImuFromLastFrame;
    // std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    MapPublisher*  mpMapPublisher;

    bool bStepByStep;

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    //Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    // For point cloud viewing
    boost::shared_ptr<PointCloudMapping> mpPointCloudMapping;
    // std::shared_ptr<Detector> mpDetector;
    YoloDetection* mpDetector;
    // Geometry mGeometry;
    Flow mFlow;

    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    int initID, lastID;

    Sophus::SE3f mTlr;

    void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

public:
    cv::Mat mImRight;

// 3d cuboid testing
public:
    void CreateObject_InTrackMotion();

private:
    std::string mStrSettingPath;
    bool mbObjectIni = false;          // initialize the object map.
    int mnObjectIniFrameID;

    cv::Mat DrawQuadricProject( cv::Mat &im,
                                const cv::Mat &P,
                                const cv::Mat &axe,
                                const cv::Mat &Twq,
                                int nClassid,
                                bool isGT=true,
                                int nLatitudeNum = 7,
                                int nLongitudeNum = 6);

    void SampleObjYaw(Object_Map* obj3d);
    cv::Point2f WorldToImg(cv::Mat &PointPosWorld);
    static bool VIC(const Eigen::Matrix<float,5,1>& lhs, const Eigen::Matrix<float,5,1>& rhs)
    {
        return lhs[1] > rhs[1];
    }

    // line
    line_lbd_detect* line_lbd_ptr;

// For 3D cuboid testing (optimize)
public:
    Eigen::Matrix3d Kalib;
    Eigen::Matrix3f Kalib_f;
    Eigen::Matrix3d invKalib;
    Eigen::Matrix3f invKalib_f;
    cv::Mat InitToGround, GroundToInit;                     // orb's init camera frame to my ground
    Eigen::Matrix4f InitToGround_eigen, GroundToInit_eigen; // ground to init frame

    void DetectCuboid(KeyFrame *pKF);
    void AssociateCuboids(KeyFrame *pKF); // compare with keypoint feature inside box
    // void MonoObjDepthInitialization();    // initilize mono SLAM based on depth map.

    detect_3d_cuboid *detect_cuboid_obj;
    double obj_det_2d_thre;

    int start_msg_seq_id = -1;

    Eigen::MatrixXd kitti_sequence_img_to_object_detect_ind;
    std::string kitti_raw_sequence_name;

    bool whether_save_online_detected_cuboids;
    bool whether_save_final_optimized_cuboids;
    std::ofstream save_online_detected_cuboids;
    std::ofstream save_final_optimized_cuboids;
    // void SaveOptimizedCuboidsToTxt();
    bool done_save_obj_to_txt = false;
    unsigned int final_object_record_frame_ind;

    // ground detection
    float nominal_ground_height;
    float filtered_ground_height;
    std::vector<float> height_esti_history;
    float ground_roi_middle; // 4 for middle 1/2  3 for middle 1/3
    float ground_roi_lower;
    int ground_inlier_pts;
    float ground_dist_ratio;
    int ground_everyKFs;
    unsigned int first_absolute_scale_frameid;
    unsigned int first_absolute_scale_framestamp;

    bool use_truth_trackid; // for (dynamic) object assocition, testing.
    unordered_map<int, MapCuboidObject *> trackletid_to_landmark;
    bool triangulate_dynamic_pts;
};

} //namespace semantic_slam

#endif // TRACKING_H
