#include "Tracking.h"

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "GeometricTools.h"

#include "YoloDetection.h"
#include "Parameter.h"
#include "g2o_Object.h"
#include "MapCuboidObject.h"

#include <Eigen/StdVector>
#include <iostream>

#include <mutex>
#include <chrono>

using namespace std;

namespace semantic_slam
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                   Atlas *pAtlas, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor,
                   Settings* settings, MapPublisher* pMapPublisher, const string &_nameSeq):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
    mbReadyToInitializate(false), mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr), mpLastKeyFrame(static_cast<KeyFrame*>(NULL)), mpMapPublisher(pMapPublisher)
{
    // Load camera parameters from settings file
    if(settings){
        newParameterLoader(settings);
    }
    else{

        std::cout << "Normal setting reading \n";

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool b_parse_cam = ParseCamParamFile(fSettings);
        if(!b_parse_cam)
        {
            std::cout << "*Error with the camera parameters in the config file*" << std::endl;
        }

        // Load ORB parameters
        bool b_parse_orb = ParseORBParamFile(fSettings);
        if(!b_parse_orb)
        {
            std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
        }

        bool b_parse_imu = true;
        if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO || sensor==System::IMU_RGBD /*|| sensor==System::STEREO*/)
        {
            b_parse_imu = ParseIMUParamFile(fSettings);
            if(!b_parse_imu)
            {
                std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
            }

            mnFramesToResetIMU = mMaxFrames;
        }

        bool b_parse_cube = true;

        if(mpSystem->isYoloDetection)
        {
//            std::cout << "?????? \n";

            b_parse_cube = ParseCUBEParamFile(fSettings);

            if(!b_parse_imu)
            {
                std::cout << "*Error with the CUBE parameters in the config file*" << std::endl;
            }
        }

        if(!b_parse_cam || !b_parse_orb || !b_parse_imu || (!b_parse_cube && mpSystem->isYoloDetection))
        {
            std::cout << "Tracking param \n";
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    initID = 0; lastID = 0;
    mbInitWith3KFs = false;
    mnNumDataset = 0;

    vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
    std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for(GeometricCamera* pCam : vpCams)
    {
        std::cout << "Camera " << pCam->GetId();
        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            std::cout << " is pinhole" << std::endl;
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            std::cout << " is fisheye" << std::endl;
        }
        else
        {
            std::cout << " is unknown" << std::endl;
        }
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif

    std::cout << "Tracking:  Initial finish" << std::endl;

}

Tracking::Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
            boost::shared_ptr<PointCloudMapping> pPointCloud,
            KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq
            /*, std::shared_ptr<Detector> pDetector*/):
        mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
        mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpPointCloudMapping(pPointCloud),
        mpKeyFrameDB(pKFDB),
        mbReadyToInitializate(false), mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
        mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
        mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr), mpLastKeyFrame(static_cast<KeyFrame*>(NULL))
        /*, mpDetector(pDetector)*/
{
    std::cout << "Here \n";
    // Load camera parameters from settings file
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool b_parse_cam = ParseCamParamFile(fSettings);
        if(!b_parse_cam)
        {
            std::cout << "*Error with the camera parameters in the config file*" << std::endl;
        }

        // Load ORB parameters
        bool b_parse_orb = ParseORBParamFile(fSettings);
        if(!b_parse_orb)
        {
            std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
        }

        bool b_parse_imu = true;
        if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO || sensor==System::IMU_RGBD)
        {
            b_parse_imu = ParseIMUParamFile(fSettings);
            if(!b_parse_imu)
            {
                std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
            }

            mnFramesToResetIMU = mMaxFrames;
        }

        if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
        {
            std::cout << "Tracking param \n";
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    initID = 0; lastID = 0;
    mbInitWith3KFs = false;
    mnNumDataset = 0;

    vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
    std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for(GeometricCamera* pCam : vpCams)
    {
        std::cout << "Camera " << pCam->GetId();
        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            std::cout << " is pinhole" << std::endl;
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            std::cout << " is fisheye" << std::endl;
        }
        else
        {
            std::cout << " is unknown" << std::endl;
        }
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif
}

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracking::LocalMapStats2File()
{
    ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << fixed << setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
    for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
    {
        f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
          << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBASync_ms[i] << ","
          << mpLocalMapper->vdKFCullingSync_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
    {
        f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
          << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
          << mpLocalMapper->vnLBA_edges[i] << endl;
    }


    f.close();
}

void Tracking::TrackStats2File()
{
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
    f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

    for(int i=0; i<vdTrackTotal_ms.size(); ++i)
    {
        double stereo_rect = 0.0;
        if(!vdRectStereo_ms.empty())
        {
            stereo_rect = vdRectStereo_ms[i];
        }

        double resize_image = 0.0;
        if(!vdResizeImage_ms.empty())
        {
            resize_image = vdResizeImage_ms[i];
        }

        double stereo_match = 0.0;
        if(!vdStereoMatch_ms.empty())
        {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if(!vdIMUInteg_ms.empty())
        {
            imu_preint = vdIMUInteg_ms[i];
        }

        f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
          << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
    }

    f.close();
}

void Tracking::PrintTimeStats()
{
    // Save data in files
    TrackStats2File();
    LocalMapStats2File();


    ofstream f;
    f.open("ExecMean.txt");
    f << fixed;
    //Report the mean and std of each one
    std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    cout << "OpenCV version: " << CV_VERSION << endl;
    f << "OpenCV version: " << CV_VERSION << endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if(!vdRectStereo_ms.empty())
    {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdResizeImage_ms.empty())
    {
        average = calcAverage(vdResizeImage_ms);
        deviation = calcDeviation(vdResizeImage_ms, average);
        std::cout << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
        f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if(!vdStereoMatch_ms.empty())
    {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdIMUInteg_ms.empty())
    {
        average = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLBA_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdKFCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_edges);
    deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFopt);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_MPs);
    deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
    std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

    // Map complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
    std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBestMap = vpMaps[0];
    for(int i=1; i<vpMaps.size(); ++i)
    {
        if(pBestMap->GetAllKeyFrames().size() < vpMaps[i]->GetAllKeyFrames().size())
        {
            pBestMap = vpMaps[i];
        }
    }

    f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

    f << "---------------------------" << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdDataQuery_ms);
    deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdEstSim3_ms);
    deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdPRTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
    f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
    f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
    f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
    average = calcAverage(mpLoopClosing->vnLoopKFs);
    deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
    f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
    f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
    f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
    f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeKFs);
    deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeMPs);
    deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdGBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
    f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
    deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
    f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
    f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAKFs);
    deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAMPs);
    deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f.close();

}

#endif

Tracking::~Tracking()
{
    //f_track_stats.close();
}

void Tracking::newParameterLoader(Settings *settings) {

    std::cout << "New Parameter Loader \n";

    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if(settings->needToUndistort()){
        mDistCoef = settings->camera1DistortionCoef();
    }
    else{
        mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    }

    //TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = cv::Mat::eye(3,3,CV_32F);
    mK.at<float>(0,0) = mpCamera->getParameter(0);
    mK.at<float>(1,1) = mpCamera->getParameter(1);
    mK.at<float>(0,2) = mpCamera->getParameter(2);
    mK.at<float>(1,2) = mpCamera->getParameter(3);

    mK_.setIdentity();
    mK_(0,0) = mpCamera->getParameter(0);
    mK_(1,1) = mpCamera->getParameter(1);
    mK_(0,2) = mpCamera->getParameter(2);
    mK_(1,2) = mpCamera->getParameter(3);

    if((mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD) &&
        settings->cameraType() == Settings::KannalaBrandt){
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();

        mpFrameDrawer->both = true;
    }

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD ){
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    if(mSensor==System::RGBD || mSensor==System::IMU_RGBD){
        mDepthMapFactor = settings->depthMapFactor();
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    //ORB parameters
    int nFeatures = settings->nFeatures();
    int nLevels = settings->nLevels();
    int fIniThFAST = settings->initThFAST();
    int fMinThFAST = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    //IMU parameters
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuPer = 0.001; //1.0 / (double) mImuFreq;
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);

    mFlowThreshold = settings->flowThreshold();
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];
    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        if(mImageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx = fx * mImageScale;
            fy = fy * mImageScale;
            cx = cx * mImageScale;
            cy = cy * mImageScale;
        }

        vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);

        mpCamera = mpAtlas->AddCamera(mpCamera);

        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- Image scale: " << mImageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

        mK_.setIdentity();
        mK_(0,0) = fx;
        mK_(1,1) = fy;
        mK_(0,2) = cx;
        mK_(1,2) = cy;

        // For 3D Cuboid testing (optimize)
        // TODO: Check usage Kalib param
        Kalib.setIdentity();
        Kalib(0, 0) = fx;
        Kalib(0, 2) = cx;
        Kalib(1, 1) = fy;
        Kalib(1, 2) = cy;
        Kalib_f = Kalib.cast<float>();
        invKalib = Kalib.inverse();
        invKalib_f = Kalib_f.inverse();
        mpAtlas->Kalib = Kalib;
        mpAtlas->Kalib_f = Kalib_f;
        mpAtlas->invKalib_f = invKalib_f;
    }
    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if(!b_miss_params)
        {
            if(mImageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);
            mpCamera = mpAtlas->AddCamera(mpCamera);
            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- Image scale: " << mImageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;

            mK_.setIdentity();
            mK_(0,0) = fx;
            mK_(1,1) = fy;
            mK_(0,2) = cx;
            mK_(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            cv::Mat cvTlr;
            if(!node.empty())
            {
                cvTlr = node.mat();
                if(cvTlr.rows != 3 || cvTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params)
            {
                if(mImageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;

                    leftLappingBegin = leftLappingBegin * mImageScale;
                    leftLappingEnd = leftLappingEnd * mImageScale;
                    rightLappingBegin = rightLappingBegin * mImageScale;
                    rightLappingEnd = rightLappingEnd * mImageScale;
                }

                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);
                mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                mTlr = Converter::toSophus(cvTlr);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << mImageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << cvTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD )
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
            if(mImageScale != 1.f)
            {
                mbf *= mImageScale;
            }
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


    }

    if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    if(b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat cvTbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty())
    {
        cvTbc = node.mat();
        if(cvTbc.rows != 4 || cvTbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }
    cout << endl;
    cout << "Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
    Eigen::Matrix<float,4,4,Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f Tbc(eigTbc);

    node = fSettings["InsertKFsWhenLost"];
    mInsertKFsLost = true;
    if(!node.empty() && node.isInt())
    {
        mInsertKFsLost = (bool) node.operator int();
    }

    if(!mInsertKFsLost)
        cout << "Do not insert keyframes when lost visual tracking " << endl;


    float Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt())
    {
        mImuFreq = node.operator int();
        mImuPer = 0.001; //1.0 / (double) mImuFreq;
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.fastInit"];
    mFastInit = false;
    if(!node.empty())
    {
        mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
    }

    if(mFastInit)
        cout << "Fast IMU initialization. Acceleration is not checked \n";

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(mImuFreq);
    cout << endl;
    cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


    return true;
}

bool Tracking::ParseCUBEParamFile(cv::FileStorage &fSettings)
{

    std::cout << "CUBE enabled \n";

    // 3D cuboid testing
    bool use_LSD_algorithm = false;
    bool save_to_imgs = false;
    bool save_to_txts = false;
    int numOfOctave_ = 1;
    float Octave_ratio = 2.0;
    line_lbd_ptr = new line_lbd_detect(numOfOctave_, Octave_ratio);
    line_lbd_ptr->use_LSD = use_LSD_algorithm;
    line_lbd_ptr->save_imgs = save_to_imgs;
    line_lbd_ptr->save_txts = save_to_txts;
    line_lbd_ptr->line_length_thres = 15; // the threshold of removing short line.
    // line detect ------------------------------------------------

    // For 3D Cuboid testing (optimize)
    // TODO: Check param
    InitToGround = cv::Mat::eye(4, 4, CV_32F);
    // set initial camera pose wrt ground. by default camera parallel to ground, height=1.7 (kitti)
    double init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw;
    init_x = 0.0;
    init_y = 0.0;
    init_z = 2.25/2.0;
    init_qx = -0.7071;
    init_qy = 0.0;
    init_qz = 0.0;
    init_qw = 0.7071;
    Eigen::Quaternionf pose_quat(init_qw, init_qx, init_qy, init_qz);
    Eigen::Matrix3f rot = pose_quat.toRotationMatrix(); // 	The quaternion is required to be normalized
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            InitToGround.at<float>(row, col) = rot(row, col);

    InitToGround.at<float>(0, 3) = init_x;
    InitToGround.at<float>(1, 3) = init_y;
    InitToGround.at<float>(2, 3) = init_z;
    nominal_ground_height = init_z;

    cv::Mat R = InitToGround.rowRange(0, 3).colRange(0, 3);
    cv::Mat t = InitToGround.rowRange(0, 3).col(3);
    cv::Mat Rinv = R.t();
    cv::Mat Ow = -Rinv * t;
    GroundToInit = cv::Mat::eye(4, 4, CV_32F);
    Rinv.copyTo(GroundToInit.rowRange(0, 3).colRange(0, 3));
    Ow.copyTo(GroundToInit.rowRange(0, 3).col(3));

    std::cout << "InitToGround_eigen: \n" << InitToGround_eigen << std::endl;
    InitToGround_eigen = Converter::toMatrix4f(InitToGround);
    GroundToInit_eigen = Converter::toMatrix4f(GroundToInit);
    std::cout << "InitToGround_eigen: \n" << InitToGround_eigen << std::endl;

    mpAtlas->InitToGround = InitToGround;
    mpAtlas->GroundToInit = GroundToInit.clone();
    mpAtlas->InitToGround_eigen = InitToGround_eigen;
    mpAtlas->InitToGround_eigen_d = InitToGround_eigen.cast<double>();
    mpAtlas->GroundToInit_eigen_d = GroundToInit_eigen.cast<double>();
    mpAtlas->GroundToInit_opti = GroundToInit.clone();
    mpAtlas->InitToGround_opti = InitToGround.clone();
    mpAtlas->RealGroundToMine_opti = cv::Mat::eye(4, 4, CV_32F);
    mpAtlas->MineGroundToReal_opti = cv::Mat::eye(4, 4, CV_32F);

    // For 3D cuboid testing (optimize)
    // TODO: Add to settings
    use_truth_trackid = false; // whether use ground truth tracklet ID.
    whether_read_offline_cuboidtxt = false;
    whether_save_online_detected_cuboids = true;
    whether_save_final_optimized_cuboids = false;

    if (mpSystem->isCuboidEnable)
    {
        if (whether_save_final_optimized_cuboids)
        {
            if (final_object_record_frame_ind == 1e5)
            {
                std::cout << "Please set final_object_record_frame_ind!!!" << std::endl;
                whether_save_final_optimized_cuboids = false;
            }
        }
    }

    filtered_ground_height = 0;
    first_absolute_scale_frameid = 0;
    first_absolute_scale_framestamp = 0;

    ground_everyKFs = 10;
    ground_roi_middle = 3.0;    //# 3(1/3) or 4(1/2)
    ground_roi_lower = 3.0;      //# 2 or 3
    ground_inlier_pts = 20;
    ground_dist_ratio = 0.08;

    return true;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

void Tracking::SetStepByStep(bool bSet)
{
    bStepByStep = bSet;
}

void Tracking::SetDetector(YoloDetection* pDetector)
{
    mpDetector = pDetector;
}

bool Tracking::GetStepByStep()
{
    return bStepByStep;
}



Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
{
    //cout << "GrabImageStereo" << endl;

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    mImRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGRA2GRAY);
        }
    }

    if (mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    else if(mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr);
    else if(mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    else if(mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr,&mLastFrame,*mpImuCalib);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

    //cout << "Tracking start" << endl;
    Track();
    //cout << "Tracking end" << endl;

    return mCurrentFrame.GetPose();
}

// TODO: Need refactor Detector
Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename)
{
    mImGray = imRGB;
    mImDepth = imD;
    mImRGB = imRGB;

    // TODO: REMEMBER HERE
    std::vector<BoxSE> objects;
    if (mpSystem->isYoloDetection)
    {
        // Yolo
        cv::Mat InputImage;
        InputImage = imRGB.clone();
        mpDetector->Detectv3(InputImage, objects);
    }

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || mImDepth.type()!=CV_32F)
        mImDepth.convertTo(mImDepth,CV_32F,mDepthMapFactor);

    if (mSensor == System::RGBD)
    {
        mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    }
    else if(mSensor == System::IMU_RGBD)
    {
        mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    }

    if(mpSystem->isYoloDetection)
    {
        mCurrentFrame.mvDynamicArea = mpDetector->mvDynamicArea;
        mpDetector->mmDetectMap.clear();
        mpDetector->mvDynamicArea.clear();
    }

    cv::Mat CurrFrameTcw = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(mCurrentFrame.GetPose()));
//    if(!CurrFrameTcw.empty())
//    {
//        mGeometry.GeometricModelCorrection(mCurrentFrame, mImDepth, mImMask);
//    }

    cv::Mat homo;
    bool flag = false;
    float BTh = mFlowThreshold;

   flag = TrackHomo(homo);

    if(flag && !homo.empty())
    {
        mFlow.ComputeMask(mImGray, homo, mImMask, BTh);
    }
    else
    {
        mFlow.ComputeMask(mImGray, mImMask, BTh);
    }

    if (mSensor == System::RGBD && !mpSystem->isYoloDetection)
    {
        mCurrentFrame = Frame(mImGray,mImDepth,mImMask,timestamp,
                              mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    }
    else if(mSensor == System::IMU_RGBD && !mpSystem->isYoloDetection)
    {
        mCurrentFrame = Frame(mImGray,mImDepth,mImMask,timestamp,
                              mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,
                              mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    }
    // For 3D cuboid
    // TODO: REMEMBER HERE
    else if (mSensor == System::RGBD && mpSystem->isYoloDetection)
    {
        mCurrentFrame = Frame(imRGB, mImGray, mImDepth, mImMask, timestamp,
                              mpORBextractorLeft, line_lbd_ptr, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth, objects, mpCamera);

        // mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    }
    else if (mSensor == System::IMU_RGBD && mpSystem->isYoloDetection)
    {
        mCurrentFrame = Frame(imRGB, mImGray, mImDepth, mImMask, timestamp,
                              mpORBextractorLeft, line_lbd_ptr, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth, objects, mpCamera, &mLastFrame,*mpImuCalib);

        // mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    }

    if(mCurrentFrame.mnId == 0)
    {
        mpAtlas->img_height = mImGray.rows;
        mpAtlas->img_width = mImGray.cols;
    }

    // For 3D cuboid testing (optimize)
    mCurrentFrame.raw_img = mImGray;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    Track();

    return mCurrentFrame.GetPose();
}


Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
{
    mImGray = im;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if (mSensor == System::MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    else if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    lastID = mCurrentFrame.mnId;
    Track();

    return mCurrentFrame.GetPose();
}


void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

void Tracking::PreintegrateIMU()
{

    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if(mlQueueImuData.size() == 0)
    {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    while(true)
    {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if(!mlQueueImuData.empty())
            {
                IMU::Point* m = &mlQueueImuData.front();
                cout.precision(17);
                if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-mImuPer)
                {
                    mlQueueImuData.pop_front();
                }
                else if(m->t<mCurrentFrame.mTimeStamp-mImuPer)
                {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                }
                else
                {
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }

    const int n = mvImuFromLastFrame.size()-1;
    if(n==0){
        cout << "Empty IMU measurements vector!!!\n";
        return;
    }

    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);

    for(int i=0; i<n; i++)
    {
        float tstep;
        Eigen::Vector3f acc, angVel;
        if((i==0) && (i<(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if(i<(n-1))
        {
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
        }
        else if((i>0) && (i==(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
        }
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    //Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}


bool Tracking::PredictStateIMU()
{
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if(mbMapUpdated && mpLastKeyFrame)
    {
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else if(!mbMapUpdated)
    {
        const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

        mCurrentFrame.mImuBias = mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}

void Tracking::ResetFrameIMU()
{
    // TODO To implement...
}


void Tracking::Track()
{

    if (bStepByStep)
    {
        std::cout << "Tracking: Waiting to the next step" << std::endl;
        while(!mbStep && bStepByStep)
            usleep(500);
        mbStep = false;
    }

    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    if(!pCurrentMap)
    {
        cout << "ERROR: There is not an active map in the atlas" << endl;
    }

    if(mState!=NO_IMAGES_YET)
    {
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        }
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            if(mpAtlas->isInertial())
            {
                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        mpSystem->ResetActiveMap();
                    }
                    else
                    {
                        CreateMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
                return;
            }
        }
    }

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mbCreatedMap)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU = std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU = std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreIMU - time_StartPreIMU).count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif

    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
        {
            StereoInitialization();
        }
        else
        {
            MonocularInitialization();
        }

        mpFrameDrawer->Update(this);

        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if(mpAtlas->GetAllMaps().size() == 1)
        {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if(mState==OK)
            {

                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if((!mbVelocity && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }


                if (!bOK)
                {
                    if ( mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
                    {
                        mState = LOST;
                    }
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        // cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                    }
                    else
                    {
                        mState = LOST;
                    }
                }
            }
            else
            {

                if (mState == RECENTLY_LOST)
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))
                    {
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else
                    {
                        // Relocalization
                        bOK = Relocalization();
                        //std::cout << "mCurrentFrame.mTimeStamp:" << to_string(mCurrentFrame.mTimeStamp) << std::endl;
                        //std::cout << "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
                        if(mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }
                else if (mState == LOST)
                {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        mpSystem->ResetActiveMap();
                        Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
                    }else
                        CreateMapInAtlas();

                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if(mbVelocity)
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    Sophus::SE3f TcwMM;
                    if(mbVelocity)
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.GetPose();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPosePred - time_StartPosePred).count();
        vdPosePred_ms.push_back(timePosePred);
#endif


#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack = std::chrono::steady_clock::now();
#endif
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                bOK = TrackLocalMap();

            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else if (mState == OK)
        {
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState=RECENTLY_LOST;
            }
            else
                mState=RECENTLY_LOST; // visual to lost

            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            //}
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) &&
           (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        if(pCurrentMap->isImuInitialized())
        {
            if(bOK)
            {
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    ResetFrameIMU();
                }
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();
        double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLMTrack - time_StartLMTrack).count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        mpFrameDrawer->Update(this);

        if(mCurrentFrame.isSet())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        mpMapPublisher->SetCurrentCameraPose(Converter::toCvMat(mCurrentFrame.GetPose()));

        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            if(mLastFrame.isSet() && mCurrentFrame.isSet())
            {
                Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                mVelocity = mCurrentFrame.GetPose() * LastTwc;
                mbVelocity = true;
            }
            else {
                mbVelocity = false;
            }

            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();
#endif
            // Different output with origin version
            int bNeedKF = NeedNewKeyFrame();

            // Check if we need to insert a new keyframe
            // if(bNeedKF && bOK)
            if((bNeedKF == 1) && (bOK || (mInsertKFsLost && mState==RECENTLY_LOST &&
                                   (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))))
                CreateNewKeyFrame(false);
            else if ((bNeedKF == 2) && (bOK || (mInsertKFsLost && mState==RECENTLY_LOST &&
                                    (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))))
                CreateNewKeyFrame(true);

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();
            double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(pCurrentMap->KeyFramesInMap()<=10)
            {
                mpSystem->ResetActiveMap();
                return;
            }
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }

            CreateMapInAtlas();
            return;
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if(mCurrentFrame.isSet())
        {
            Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }
    }

#ifdef REGISTER_LOOP
    if (Stop()) {
        // Safe area to stop
        while(isStopped())
        {
            usleep(3000);
        }
    }
#endif
}

void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return;
            }

            if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA).norm()<0.5)
            {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
        }
        else
            mCurrentFrame.SetPose(Sophus::SE3f());

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2){
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    Eigen::Vector3f x3D;
                    mCurrentFrame.UnprojectStereo(i, x3D);
                    MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }
        } else{
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }
        }

        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        //cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        //mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());
        mpMapPublisher->SetCurrentCameraPose(Converter::toCvMat(mCurrentFrame.GetPose()));

        mState = OK;
    }
}


void Tracking::MonocularInitialization()
{
    if(!mbReadyToInitializate)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size() > 100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if(mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            mbReadyToInitializate = true;

            return;
        }
    }
    else
    {
        if (((int)mCurrentFrame.mvKeys.size() <= 100) || ((mSensor == System::IMU_MONOCULAR) && (mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp > 1.0)))
        {
            mbReadyToInitializate = false;

            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            mbReadyToInitializate = false;
            return;
        }

        Sophus::SE3f Tcw;
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Tcw,mvIniP3D,vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(Sophus::SE3f());
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}


void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    if (mpSystem->isCuboidEnable && mpSystem->isg2oObjectOptimize)
    {
        DetectCuboid(pKFini);
        AssociateCuboids(pKFini);
        DetectCuboid(pKFcur);
        AssociateCuboids(pKFcur);
    }

    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        // Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        // Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        // Add to Map
        mpAtlas->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    //mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;

    initID = pKFcur->mnId;
}


void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();

    mbSetInit = false;

    mnInitialFrameId = mCurrentFrame.mnId + 1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;

    // The last relocation KF_id is the current id, because it is the new starting point for new map
    // mnLastRelocFrameId = mnLastInitFrameId;

    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId + 1), Verbose::VERBOSITY_NORMAL);

    // Init value for know if there are enough MapPoints in the last KF
    mbVO = false;

    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mbReadyToInitializate = false;
    }

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpImuPreintegratedFromLastKF)
    {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
    }

    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

    if(mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame*>(NULL);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i = 0; i < mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
    {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(/*Converter::toSophus(fusion(cv::Mat::eye(4, 4, CV_32F))) */ mLastFrame.GetPose());

    // mCurrentFrame.PrintPointDistribution();

    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i = 0; i < mCurrentFrame.N; i++)
    {
        // if(i >= mCurrentFrame.Nleft) break;

        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
    {
        return;
    }

    if(pRef->GetPose().log() == Sophus::SE3f().log())
    {
        return;
    }
    else
    {
        std::cout << "Are you running? \n";
        mLastFrame.SetPose(Tlr * pRef->GetPose());
    }

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor

    std::vector<std::pair<float, int>> vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1? mLastFrame.N : mLastFrame.Nleft;

    vDepthIdx.reserve(Nfeat);

    for(int i = 0; i < Nfeat; i++)
    {
        float z = mLastFrame.mvDepth[i];

        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth < mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.

    int nPoints = 0;
    for(size_t j = 0; j < vDepthIdx.size(); j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
            bCreateNew = true;

        if(bCreateNew)
        {
            Eigen::Vector3f x3D;

            if(mLastFrame.Nleft == -1){
                mLastFrame.UnprojectStereo(i, x3D);
            }
            else{
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    }
    else
    {
        mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    }

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;

    if(mSensor==System::STEREO)
        th=7;
    else
        th=15;

    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

    }

    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            return true;
        else
            return false;
    }

    // For 3D cuboid
    // TODO: REMEMBER HERE
     CreateObject_InTrackMotion();

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i = 0; i < mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    UpdateLocalMap();
    SearchLocalPoints();

    // TODO: check outliers before PO
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    int inliers;
    if (!mpAtlas->isImuInitialized())
        Optimizer::PoseOptimization(&mCurrentFrame);
    else
    {
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        }
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;


    if (mSensor == System::IMU_MONOCULAR)
    {
        if((mnMatchesInliers<15 && mpAtlas->isImuInitialized())||(mnMatchesInliers<50 && !mpAtlas->isImuInitialized()))
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else
    {
        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }
}

int Tracking::NeedNewKeyFrame()
{
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
        //Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
        thRefRatio = 0.9f;
    }*/

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle); //mpLocalMapper->KeyframesInQueue() < 2);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && mSensor!=System::IMU_RGBD && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);

    // std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;

    // TODO: REMEMBER HERE
    bool c1d = false;
    if(mCurrentFrame.AppearNewObject)
    {
        c1d = true;
    }
    
    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle || mpLocalMapper->IsInitializing())
        {
            return 1;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return 1;
                else
                    return false;
            }
            else
            {
                //std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
                return false;
            }
        }
    }
     else
         return false;

    // TODO: REMEMBER HERE
    if (c1d)
    {
        if (bLocalMappingIdle)
        {
            return 2;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR)
            {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return 2;
                else
                    return 0;
            }
            else
                return 0;
        }
    }

    return 0;
}

template <class BidiIter> //Fisher-Yates shuffle
BidiIter random_unique2(BidiIter begin, BidiIter end, int num_random)
{
    size_t left = std::distance(begin, end);
    while (num_random--)
    {
        BidiIter r = begin;
        std::advance(r, rand() % left);
        std::swap(*begin, *r);
        ++begin;
        --left;
    }
    return begin;
}

void Tracking::CreateNewKeyFrame(bool CreateByObjs)
{
    if(mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
        return;

    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF;
    if(mSensor == System::RGBD || mSensor == System::IMU_RGBD)
    {
        pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB,
                           this->mImRGB, this->mImDepth);
    }
    else
    {
        pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    }

    if(mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // keyframe created by objects.
    if (CreateByObjs)
        pKF->mbCreatedByObjs = true;

    // For 3D cuboid testing (optimize)
    if(mpSystem->isg2oObjectOptimize)
    {
        std::cout << "\033[33m Created new keyframe!  " << pKF->mnId << " local cuboid " << pKF->local_cuboids.size()
                  << "   total ID  " << pKF->mnFrameId << "\033[0m" << std::endl;
    }

    if (mpSystem->isCuboidEnable && mpSystem->isg2oObjectOptimize)
    {
        DetectCuboid(pKF);
        AssociateCuboids(pKF);
    }

    // TODO: REMEMBER HERE
    // For 3D cuboid
    if(CreateByObjs)
    {
        pKF->mbByNewObj = true;
    }

    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }

    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if include imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            maxPoint = 100;

        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    Eigen::Vector3f x3D;

                    if(mCurrentFrame.Nleft == -1){
                        mCurrentFrame.UnprojectStereo(i, x3D);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }
            //Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
        }
    }

    //copied from localMapping, only for dynamic object
    if (mono_allframe_Obj_depth_init && mpSystem->isDynamicObject && mpSystem->isg2oObjectOptimize)
    {
        KeyFrame *mpCurrentKeyFrame = pKF;

        const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        double total_feature_pts = vpMapPointMatches.size();
        double raw_depth_pts = 0; // point already have depth
        double plane_object_initialized_pts = 0;
        std::vector<int> raw_pixels_no_depth_inds;

        if (triangulate_dynamic_pts)
        {
            vector<MapPoint *> frameMapPointMatches;
            if (mpSystem->isUseDynamicKLTFeatures)
                frameMapPointMatches = mCurrentFrame.mvpMapPointsHarris;
            else
                frameMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

            cv::Mat Tcw_last = Converter::toCvMat(mpLastKeyFrame->GetPose());
            cv::Mat Tcw_now = Converter::toCvMat(mpCurrentKeyFrame->GetPose());
            const float &cx1 = mpCurrentKeyFrame->cx;
            const float &cy1 = mpCurrentKeyFrame->cy;
            const float &invfx1 = mpCurrentKeyFrame->invfx;
            const float &invfy1 = mpCurrentKeyFrame->invfy;
            for (size_t i = 0; i < frameMapPointMatches.size(); i++)
            {
                MapPoint *pMP = frameMapPointMatches[i];
                if (pMP && pMP->is_dynamic) // the point is matched to this frame, and also dynamic.
                {
                    // check if this point is created by last keyframe, if yes, triangulate it with this frame!   if created earlier, not need
                    if (!pMP->is_triangulated) // if not Triangulated
                    {
                        int pixelindLastKf = get<0>(pMP->GetIndexInKeyFrame(mpLastKeyFrame));
                        if (pixelindLastKf == -1)
                        {
                            std::cout << "Point frame observation not added yet" << std::endl;
                            continue;
                        }
                        MapCuboidObject *objectLastframe;
                        if (mpSystem->isUseDynamicKLTFeatures)
                            objectLastframe = mpLastKeyFrame->local_cuboids[mpLastKeyFrame->keypoint_associate_objectID_harris[pixelindLastKf]];
                        else
                            objectLastframe = mpLastKeyFrame->local_cuboids[mpLastKeyFrame->keypoint_associate_objectID[pixelindLastKf]];
                        g2o::cuboid cube_pose_lastkf;
                        if (objectLastframe->already_associated)
                        {
                            // TODO: Checking about compare function
                            cube_pose_lastkf = objectLastframe->associated_landmark->allDynamicPoses[mpLastKeyFrame].first;
                        }
                        else
                            cube_pose_lastkf = objectLastframe->GetWorldPos();
                        // get new cube pose in this frame??? based on keypoint object asscoiate id.
                        MapCuboidObject *objectThisframe;
                        if (mpSystem->isUseDynamicKLTFeatures)
                            objectThisframe = mpCurrentKeyFrame->local_cuboids[mCurrentFrame.keypoint_associate_objectID_harris[i]];
                        else
                            objectThisframe = mpCurrentKeyFrame->local_cuboids[mpCurrentKeyFrame->keypoint_associate_objectID[i]];
                        g2o::cuboid cube_pose_now = objectThisframe->GetWorldPos(); //current obj pose, not BA optimimized
                        // check truth tracklet id.
                        if (use_truth_trackid)
                            if (objectLastframe->truth_tracklet_id != objectThisframe->truth_tracklet_id)
                            {
                                std::cout << "Different object tracklet id, possibly due to wrong KLT point tracking" << std::endl;
                                continue;
                            }
                        g2o::SE3Quat objecttransform = cube_pose_now.pose * cube_pose_lastkf.pose.inverse();
                        cv::Mat Tcw_now_withdynamic = Tcw_now * Converter::toCvMat(objecttransform);

                        cv::KeyPoint kp1, kp2;
                        if (mpSystem->isUseDynamicKLTFeatures)
                        {
                            kp1 = mpLastKeyFrame->mvKeysHarris[pixelindLastKf];
                            kp2 = mCurrentFrame.mvKeysHarris[i];
                        }
                        else
                        {
                            kp1 = mpLastKeyFrame->mvKeysUn[pixelindLastKf];
                            kp2 = mpCurrentKeyFrame->mvKeysUn[i];
                        }
                        // Check parallax between rays
                        cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
                        cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx1) * invfx1, (kp2.pt.y - cy1) * invfy1, 1.0);

                        cv::Mat x3D;
                        {
                            // Linear Triangulation Method
                            cv::Mat A(4, 4, CV_32F);
                            A.row(0) = xn1.at<float>(0) * Tcw_last.row(2) - Tcw_last.row(0);
                            A.row(1) = xn1.at<float>(1) * Tcw_last.row(2) - Tcw_last.row(1);
                            A.row(2) = xn2.at<float>(0) * Tcw_now_withdynamic.row(2) - Tcw_now_withdynamic.row(0);
                            A.row(3) = xn2.at<float>(1) * Tcw_now_withdynamic.row(2) - Tcw_now_withdynamic.row(1);

                            cv::Mat w, u, vt;
                            cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

                            x3D = vt.row(3).t();

                            if (x3D.at<float>(3) == 0)
                                continue;

                            // Euclidean coordinates
                            x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
                        }

                        if ((Converter::toVector3f(x3D) - pMP->GetWorldPosVec()).norm() > 5) // if too far from object center, not good triangulation.
                        {
                            continue;
                        }
                        pMP->is_triangulated = true;
                        pMP->PosToObj = Converter::toCvMat(cube_pose_now.pose.inverse().map(Converter::toVector3d(x3D)));
                    }
                }
            }
        }

        if (1) // randomly select N points, don't initialize all of them
        {
            bool actually_use_obj_depth = false;
            if (mono_allframe_Obj_depth_init && mpSystem->isCuboidEnable && mpSystem->isAssociatePointWithObject)
                if (mpCurrentKeyFrame->keypoint_associate_objectID.size() > 0)
                    actually_use_obj_depth = true;

            if (actually_use_obj_depth)
            {
                cout << "Tracking just about to initialize object depth point" << endl;

                // detect new KLF feature points   away from existing featute points.
                if (mpSystem->isUseDynamicKLTFeatures)
                {
                    // loop over all mappoints, create circle mask
                    // 		    objmask_img:  0 background, >0 object areas   change to--->   255 object areas. 0 background.
                    int MIN_DIST = 10;															// or 10  20
                    cv::Mat good_mask;															//area to generate new features.
                    threshold(mCurrentFrame.objmask_img, good_mask, 0, 255, cv::THRESH_BINARY); // threshold to be 0,255

                    // final existing object mappoints.
                    vector<pair<int, cv::Point2f>> exist_obj_mappts; //point, obverse time

                    for (size_t i = 0; i < mCurrentFrame.mvpMapPointsHarris.size(); i++)
                    {
                        MapPoint *pMP = mCurrentFrame.mvpMapPointsHarris[i]; //TODO later should be separate mappoint for dynamic
                        if (pMP && pMP->is_dynamic)
                        {
                            exist_obj_mappts.push_back(make_pair(pMP->Observations(), mCurrentFrame.mvKeysHarris[i].pt));
                        }
                    }
                    // sort(exist_obj_mappts.begin(), exist_obj_mappts.end(), [](MapPoint *a, MapPoint *b) { return a->Observations() > b->Observations(); });
                    sort(exist_obj_mappts.begin(), exist_obj_mappts.end(), [](const pair<int, cv::Point2f> &a, const pair<int, cv::Point2f> &b) { return a.first > b.first; });

                    for (auto &it : exist_obj_mappts)
                    {
                        if (good_mask.at<uchar>(it.second) == 255)
                        {
                            cv::circle(good_mask, it.second, MIN_DIST, 0, -1);
                        }
                    }
                    cout << "mCurrentFrame.mvpMapPointsHarris size   " << mCurrentFrame.mvpMapPointsHarris.size() << "  " << exist_obj_mappts.size() << endl;

                    int max_new_pts = 200; //100
                    vector<cv::Point2f> corners;
                    cv::goodFeaturesToTrack(mpCurrentKeyFrame->raw_img, corners, max_new_pts, 0.1, MIN_DIST, good_mask);

                    int numTracked = mCurrentFrame.mvKeysHarris.size();
                    int numNewfeat = corners.size();
                    int totalfeat = numTracked + numNewfeat;

                    mpCurrentKeyFrame->mvKeysHarris = mCurrentFrame.mvKeysHarris;
                    mpCurrentKeyFrame->mvKeysHarris.resize(totalfeat);
                    mpCurrentKeyFrame->mvpMapPointsHarris = mCurrentFrame.mvpMapPointsHarris;
                    mpCurrentKeyFrame->mvpMapPointsHarris.resize(totalfeat);
                    mpCurrentKeyFrame->keypoint_associate_objectID_harris = mCurrentFrame.keypoint_associate_objectID_harris;
                    mpCurrentKeyFrame->keypoint_associate_objectID_harris.resize(totalfeat);

                    //create and append new detected features.
                    for (int new_fea_ind = 0; new_fea_ind < numNewfeat; new_fea_ind++)
                    {
                        int maskval = int(mCurrentFrame.objmask_img.at<uchar>(corners[new_fea_ind])); //0 background, >0 object id
                        int pixelcubeid = maskval - 1;
                        if (maskval == 0)
                        {
                            std::cout << "Get invalid pixel object index" << std::endl;
                            exit(0);
                        }
                        cv::KeyPoint keypt;
                        keypt.pt = corners[new_fea_ind];
                        mpCurrentKeyFrame->mvKeysHarris[new_fea_ind + numTracked] = keypt;
                        mpCurrentKeyFrame->keypoint_associate_objectID_harris[new_fea_ind + numTracked] = pixelcubeid;

                        float point_depth = mpCurrentKeyFrame->local_cuboids[pixelcubeid]->cube_meas.translation()[2]; // camera z
                        cv::Mat x3D = mpCurrentKeyFrame->UnprojectPixelDepth(corners[new_fea_ind], point_depth);

                        MapPoint *pNewMP = new MapPoint(Converter::toVector3f(x3D), mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
                        pNewMP->is_dynamic = true;
                        mpCurrentKeyFrame->SetupSimpleMapPoints(pNewMP, new_fea_ind + numTracked); // add to frame observation, add to map.
                        //also changed mvpMapPointsHarris
                        pNewMP->is_triangulated = false;
                        pNewMP->SetWorldPos(Converter::toVector3f(x3D)); // compute dynamic point to object pose
                    }
                    cout << "tracked/new_created features   " << numTracked << "  " << numNewfeat << endl;

                    //update total features in mCurrentFrame
                    mCurrentFrame.mvKeysHarris = mpCurrentKeyFrame->mvKeysHarris;
                    mCurrentFrame.mvpMapPointsHarris = mpCurrentKeyFrame->mvpMapPointsHarris;
                    mCurrentFrame.keypoint_associate_objectID_harris = mpCurrentKeyFrame->keypoint_associate_objectID_harris;
                }
                else
                {
                    std::vector<int> has_object_depth_pixel_inds; // points with no depth yet but with matching object and plane

                    bool gridsHasMappt[FRAME_GRID_COLS][FRAME_GRID_ROWS];
                    for (int i = 0; i < FRAME_GRID_COLS; i++)
                        for (int j = 0; j < FRAME_GRID_ROWS; j++)
                            gridsHasMappt[i][j] = false;
                    for (size_t i = 0; i < vpMapPointMatches.size(); i++)
                    {
                        MapPoint *pMP = vpMapPointMatches[i];
                        if (!pMP) //no map point yet. not associated yet
                        {
                            int gridx, gridy;
                            if (mpCurrentKeyFrame->PosInGrid(mpCurrentKeyFrame->mvKeys[i], gridx, gridy))
                            {
                                if (gridsHasMappt[gridx][gridy])
                                    continue;
                            }
                            else
                                continue;

                            if (mpCurrentKeyFrame->keypoint_associate_objectID[i] > -1) // have associated object
                                if (mpCurrentKeyFrame->mvKeys[i].octave < 3)			//HACK for KLT tracking, better just use first octave
                                {
                                    has_object_depth_pixel_inds.push_back(i);
                                    gridsHasMappt[gridx][gridy] = true;
                                }
                        }
                        else
                            raw_depth_pts++;
                    }
                    bool whether_actually_planeobj_init_pt = false;

                    double depth_point_ration_now = raw_depth_pts / total_feature_pts;
                    int max_initialize_pts = 0;
                    if (depth_point_ration_now < 0.30) //0.3
                        whether_actually_planeobj_init_pt = true;
                    max_initialize_pts = std::min(int(total_feature_pts * 0.30) - int(raw_depth_pts), int(has_object_depth_pixel_inds.size()));
                    max_initialize_pts = std::min(max_initialize_pts, 80);

                    cout << "all points to initilaze  " << has_object_depth_pixel_inds.size() << "  initialized " << max_initialize_pts << endl;
                    int nPoints = 0;

                    if (whether_actually_planeobj_init_pt)
                    {
                        srand(time(NULL));
                        // 		    random_shuffle ( has_object_depth_pixel_inds.begin(), has_object_depth_pixel_inds.end() );
                        random_unique2(has_object_depth_pixel_inds.begin(), has_object_depth_pixel_inds.end(), max_initialize_pts);

                        int vector_counter = 0;
                        while ((nPoints < max_initialize_pts) && (vector_counter < (int)has_object_depth_pixel_inds.size()))
                        {
                            int pixel_ind = has_object_depth_pixel_inds[vector_counter];
                            float point_depth = -1;
                            cv::Mat x3D;

                            if ((point_depth < 0))
                            {
                                if (mpCurrentKeyFrame->keypoint_associate_objectID[pixel_ind] > -1)
                                {
                                    point_depth = mpCurrentKeyFrame->local_cuboids[mpCurrentKeyFrame->keypoint_associate_objectID[pixel_ind]]->cube_meas.translation()[2]; // camera z
                                    x3D = mpCurrentKeyFrame->UnprojectDepth(pixel_ind, point_depth);
                                }
                            }
                            if (point_depth > 0)
                            {
                                MapPoint *pNewMP = new MapPoint(Converter::toVector3f(x3D), mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
                                mpCurrentKeyFrame->SetupSimpleMapPoints(pNewMP, pixel_ind); // add to frame observation, add to map.
                                pNewMP->is_triangulated = false;
                                nPoints++;
                                if (mpSystem->isDynamicObject)
                                {
                                    pNewMP->is_dynamic = true;
                                }
                            }
                            else
                            {
                                //NOTE projected point is negative. remove association? because this point is bad
                            }
                            vector_counter++;
                        }
                        plane_object_initialized_pts = nPoints;
                        std::cout << "Online depth create mappoints!!!!!!!!  " << nPoints << std::endl;
                    }
                }
            }
        }

        // 	std::cout<<"Finish create my map points!!!!"<<std::endl;
        mCurrentFrame.mvpMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
        // mCurrentFrame.mvpMapPoints indepent of new created mappoints in localmapping
    }

    if (enable_ground_height_scale && mpSystem->isg2oObjectOptimize)
    {
        float img_width = float(mpAtlas->img_width);
        float img_height = float(mpAtlas->img_height);

        // do it in every frame, otherwise may take longer time when do it together for many frames.
        for (size_t iMP = 0; iMP < mCurrentFrame.mvpMapPoints.size(); iMP++)
            if (pKF->mvKeysUn[iMP].pt.x > img_width / ground_roi_middle && pKF->mvKeysUn[iMP].pt.x < img_width / ground_roi_middle * (ground_roi_middle - 1))
                if (pKF->mvKeysUn[iMP].pt.y > img_height / ground_roi_lower * (ground_roi_lower - 1)) // lower 1/3, I checked kitti sequence, roughly true.
                {
                    bool not_in_object = true;
                    if (pKF->keypoint_inany_object.size() > 0)
                        if (pKF->keypoint_inany_object[iMP])
                            not_in_object = false;
                    if (not_in_object)
                        pKF->ground_region_potential_pts.push_back(iMP); // used for latter adjacent frame ground fitting
                }

        std::cout << "Tracking: estimate ground: potential_pts: " << pKF->ground_region_potential_pts.size()
                  << " pKF->mnId: " << pKF->mnId << " ground_everyKFs: " << ground_everyKFs << std::endl;
        if (pKF->mnId % ground_everyKFs == 0)
        {
            unsigned long anchor_frame_kfid = 0;
            if (int(pKF->mnId) > ground_everyKFs)
                anchor_frame_kfid = pKF->mnId - ground_everyKFs;
            KeyFrame *first_keyframe = nullptr;
            std::vector<KeyFrame *> ground_local_KFs;
            unsigned long minKFid = pKF->mnId;
            for (size_t ii = 0; ii < mvpLocalKeyFrames.size(); ii++)
            {
                KeyFrame *pKFi = mvpLocalKeyFrames[ii];
                if (pKFi->mnId >= anchor_frame_kfid)
                {
                    ground_local_KFs.push_back(pKFi);
                    pKFi->mnGroundFittingForKF = pKF->mnId;
                    if (pKFi->mnId < minKFid)
                    {
                        minKFid = pKFi->mnId; // the original anchor frame id might not exist due to culling.
                        first_keyframe = pKFi;
                    }
                }
            }
            if (first_keyframe == nullptr)
            {
                std::cout << "Not found first keyframe!!!  " << std::endl;
                exit(0);
            }
            ground_local_KFs.push_back(pKF);
            anchor_frame_kfid = pKF->mnId;
            int initializer_starting_frame_id = (*mlpReferences.begin())->mnFrameId; // a fixed value

            KeyFrame *median_keyframe = nullptr; // scale relative to the center frames instead of the begining frame? more accurate?
            // added benchun 20201210
            if (pKF->mnId == 20)
                height_esti_history.push_back(2.25);
            std::cout << "Tracking: height_esti_history.size() " << height_esti_history.size() << std::endl;
            if (height_esti_history.size() > 0)
            {
                vector<unsigned> range_kf_ids;
                for (size_t i = 0; i < ground_local_KFs.size(); i++)
                    range_kf_ids.push_back(ground_local_KFs[i]->mnId);
                sort(range_kf_ids.begin(), range_kf_ids.end());
                unsigned median_frameid = range_kf_ids[range_kf_ids.size() / 2];
                for (size_t i = 0; i < ground_local_KFs.size(); i++)
                    if (ground_local_KFs[i]->mnId == median_frameid)
                    {
                        median_keyframe = ground_local_KFs[i];
                        break;
                    }
                if (median_keyframe == nullptr)
                {
                    std::cout << "Not found median keyframe!!!  " << std::endl;
                    exit(0);
                }
            }
            else
                median_keyframe = first_keyframe; // still want to scale at the very first fram

            bool recently_have_object = false;
            if (recently_have_object)
                std::cout << "Found cuboid landmark in this range" << std::endl;
            if ((!recently_have_object) || (height_esti_history.size() < 1))
            {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::PointXYZ pt;
                cloud.points.reserve(mvpLocalMapPoints.size());
                vector<MapPoint *> potential_plane_points;
                for (size_t i = 0; i < ground_local_KFs.size(); i++)
                {
                    std::vector<MapPoint *> framepointmatches = ground_local_KFs[i]->GetMapPointMatches();
                    for (size_t j = 0; j < ground_local_KFs[i]->ground_region_potential_pts.size(); j++)
                    {
                        MapPoint *pMP = framepointmatches[ground_local_KFs[i]->ground_region_potential_pts[j]];
                        if (pMP)
                            if (!pMP->isBad())
                                if (pMP->mnGroundFittingForKF != pKF->mnId)
                                {
                                    // std::cout << "Check" << std::endl;
                                    cv::Mat point_position = Converter::toCvMat(pMP->GetWorldPos()); // fit plane in global frame, then tranform plane. saving time for point transformation
                                    pt.x = point_position.at<float>(0);
                                    pt.y = point_position.at<float>(1);
                                    pt.z = point_position.at<float>(2);
                                    cloud.points.push_back(pt);
                                    potential_plane_points.push_back(pMP);
                                    pMP->mnGroundFittingForKF = pKF->mnId;
                                }
                    }
                }
                std::cout << "Tracking: Potential plane pt size    " << potential_plane_points.size() << "   " << ground_local_KFs.size() << std::endl;

                if (potential_plane_points.size()>=4)
                {
                    // TODO can we directly search height plane to find points supporting it?? not using ransac. Song used it.
                    pcl::SACSegmentation<pcl::PointXYZ> *seg = new pcl::SACSegmentation<pcl::PointXYZ>();
                    seg->setOptimizeCoefficients(true);
                    seg->setModelType(pcl::SACMODEL_PLANE);
                    seg->setMethodType(pcl::SAC_RANSAC);
                    if (height_esti_history.size() > 0)
                        seg->setDistanceThreshold(ground_dist_ratio * height_esti_history.back());
                    else
                        seg->setDistanceThreshold(0.005); // the raw map is scaled to mean 1.
                    pcl::ModelCoefficients coefficients;
                    pcl::PointIndices inliers;
                    seg->setInputCloud(cloud.makeShared());
                    // ca::Profiler::tictoc("pcl plane fitting time");
                    seg->segment(inliers, coefficients);

                    Eigen::Vector4f global_fitted_plane(coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);
                    float cam_plane_dist, angle_diff_normal;

                    // transform to anchor frame
                    KeyFrame *anchor_frame = first_keyframe; // first_keyframe  median_keyframe  pKF;
                    cv::Mat anchor_Tcw = Converter::toCvMat(anchor_frame->GetPose());
                    cv::Mat anchor_Twc = Converter::toCvMat(anchor_frame->GetPoseInverse());

                    // take averge of all camera pose dist to plane,  not just wrt anchor frame
                    if (1)
                    {
                        float sum_cam_dist = 0;
                        float sum_angle_diff = 0;
                        vector<float> temp_dists;
                        for (size_t i = 0; i < ground_local_KFs.size(); i++)
                        {
                            KeyFrame *localkf = ground_local_KFs[i];
                            cv::Mat cam_Twc = Converter::toCvMat(localkf->GetPoseInverse());
                            Eigen::Matrix4f cam_Twc_eig = Converter::toMatrix4f(cam_Twc);
                            Eigen::Vector4f local_kf_plane = cam_Twc_eig.transpose() * global_fitted_plane;
                            local_kf_plane = local_kf_plane / local_kf_plane.head<3>().norm(); // normalize the plane.

                            float local_cam_plane_dist = fabs(local_kf_plane(3));
                            float local_angle_diff_normal = acos(local_kf_plane.head(3).dot(Vector3f(0, 1, 0))) * 180.0 / M_PI; // 0~pi
                            if (local_angle_diff_normal > 90)
                                local_angle_diff_normal = 180.0 - local_angle_diff_normal;
                            sum_cam_dist += local_cam_plane_dist;
                            sum_angle_diff += local_angle_diff_normal;
                            temp_dists.push_back(local_cam_plane_dist);
                        }
                        cam_plane_dist = sum_cam_dist / float(ground_local_KFs.size());
                        angle_diff_normal = sum_angle_diff / float(ground_local_KFs.size());
                    }

                    std::cout << "Tracking: find Potential plane pt size   " << potential_plane_points.size()
                              << " Find init plane  dist  " << cam_plane_dist << "  angle  " << angle_diff_normal << "   inliers  " << inliers.indices.size() << std::endl;

                    if (int(inliers.indices.size()) > ground_inlier_pts) // or ratio
                    {
                        if (angle_diff_normal < 10)
                        {
                            // for kitti 02, unstale initialization. needs more times
                            if ((fabs(cam_plane_dist - nominal_ground_height) < 0.6) || (height_esti_history.size() < 4)) // or compare with last time?
                            {
                                height_esti_history.push_back(cam_plane_dist);

                                if (height_esti_history.size() == 1)
                                {
                                    first_absolute_scale_frameid = first_keyframe->mnFrameId;
                                    first_absolute_scale_framestamp = first_keyframe->mTimeStamp;
                                }

                                for (size_t i = 0; i < inliers.indices.size(); i++)
                                    potential_plane_points[inliers.indices[i]]->ground_fitted_point = true;

                                float final_filter_height = cam_plane_dist;
                                // take average or recent tow/three frames. or median filter? is this correct if object scale???
                                if (height_esti_history.size() > 2)
                                {
                                    final_filter_height = 0.6 * height_esti_history.back() + 0.4 * filtered_ground_height;
                                }
                                filtered_ground_height = final_filter_height;

                                float scaling_ratio = nominal_ground_height / final_filter_height;
                                if (height_esti_history.size() > 1) // ignore the first time.
                                {
                                    // don't want too large scaling, which might be wrong...
                                    scaling_ratio = std::min(std::max(scaling_ratio, 0.7f), 1.3f);
                                }
                                std::cout << "Actually scale map and frames~~~~~~~~~~~~~~~~" << std::endl;

                                if (enable_ground_height_scale)
                                {
                                    for (size_t iMP = 0; iMP < mvpLocalMapPoints.size(); iMP++) // approximatedly. actually mvpLocalMapPoints has much more points
                                    {
                                        cv::Mat poseLocalMapPoint = Converter::toCvMat(mvpLocalMapPoints[iMP]->GetWorldPos());
                                        cv::Mat local_pt = anchor_Tcw.rowRange(0, 3).colRange(0, 3) * poseLocalMapPoint + anchor_Tcw.rowRange(0, 3).col(3);
                                        cv::Mat scaled_global_pt = anchor_Twc.rowRange(0, 3).colRange(0, 3) * (local_pt * scaling_ratio) + anchor_Twc.rowRange(0, 3).col(3);
                                        mvpLocalMapPoints[iMP]->SetWorldPos(Converter::toVector3f(scaled_global_pt));
                                    }
                                    for (size_t iKF = 0; iKF < ground_local_KFs.size(); iKF++)
                                    {
                                        cv::Mat anchor_to_pose = Converter::toCvMat(ground_local_KFs[iKF]->GetPose()) * anchor_Twc;
                                        anchor_to_pose.col(3).rowRange(0, 3) = anchor_to_pose.col(3).rowRange(0, 3) * scaling_ratio;
                                        ground_local_KFs[iKF]->SetPose(Converter::toSophus(anchor_to_pose * anchor_Tcw));
                                    }
                                    cv::Mat _Tcw = Converter::toCvMat(mLastFrame.GetPose());
                                    cv::Mat anchor_to_pose = _Tcw * anchor_Twc;
                                    anchor_to_pose.col(3).rowRange(0, 3) = anchor_to_pose.col(3).rowRange(0, 3) * scaling_ratio;
                                    mLastFrame.SetPose(Converter::toSophus(anchor_to_pose * anchor_Tcw));
                                    mCurrentFrame.SetPose(pKF->GetPose());

                                    cv::Mat vec = Converter::toCvMat(mVelocity);
                                    vec.col(3).rowRange(0, 3) = vec.col(3).rowRange(0, 3) * scaling_ratio;
                                    mVelocity = Converter::toSophus(vec);

                                    // loop over mlpReferences, if any frames' references frames lie in this range, scale the relative poses accordingly
                                    // mlpReferences doesn't include the initialization stage... // if it is bad...??

                                    for (size_t ind = first_keyframe->mnFrameId - initializer_starting_frame_id; ind < mlpReferences.size(); ind++)
                                    {
                                        list<semantic_slam::KeyFrame*>::iterator lRit = mlpReferences.begin();
                                        list<Sophus::SE3f>::iterator it = mlRelativeFramePoses.begin();
                                        for(int i = 0; i< ind; i++){
                                            ++lRit;
                                            ++it;
                                        }
                                        if ((*lRit)->mnGroundFittingForKF == pKF->mnId)
                                        {
                                            cv::Mat Tcr = Converter::toCvMat((*it)); // reference to current
                                            Tcr.col(3).rowRange(0, 3) = Tcr.col(3).rowRange(0, 3) * scaling_ratio;
                                            (*it) = Converter::toSophus(Tcr);
                                        }
                                    }
                                }
                            }
                            else
                                std::cout << "\033[31m Too large change compared to last time. \033[0m" << cam_plane_dist << "   last  " << filtered_ground_height << std::endl;
                        }
                        else
                            std::cout << "\033[31m Bad ground orientation. \033[0m" << std::endl;
                    }
                    else
                        std::cout << "\033[31m Not enough inliers. \033[0m" << std::endl;
                }
                else
                    std::cout << "\033[31m Not enough Potential plane. \033[0m" << std::endl;
            }
        }
    }

    pKF->mvDynamicArea = mCurrentFrame.mvDynamicArea;

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

//    double minT, maxT;
//    cv::minMaxIdx(mImDepth, &minT, &maxT);
//
//    cout << "Depth mat type " << mImDepth.type() << endl;
//    cout << "DepthMapFactor " << mDepthMapFactor << endl;
//    cout << "BEFORE +++ (" << minT << ", " << maxT << ") \n";
//
//    cv::minMaxIdx(this->mImRGB, &minT, &maxT);
//
//
//    cout << "Color mat type " << mImRGB.type() << endl;
//    cout << "Color mat size " << mImRGB.size() << endl;
//    cout << "Color mat channels " << mImRGB.channels() << endl;
//    cout << "Color +++ (" << minT << ", " << maxT << ") \n";

    // Update object
//    if(mpDetector)
//    {
//        mpDetector->insertKFColorImg(pKF, this->mImRGB);
//    }

    // insert Key Frame into point cloud viewer
    if (!mpSystem->isYoloDetection)
    {
        mpPointCloudMapping->insertKeyFrame(pKF, this->mImRGB, this->mImDepth);
    }

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=6;
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15

        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {

            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }
    else
    {
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }


    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80) // 80
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);


        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&mvpLocalKeyFrames.size()<80)
    {
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    if(vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(bTcw)
            {
                Sophus::SE3f Tcw(eigTcw);
                mCurrentFrame.SetPose(Tcw);
                // Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }

}

    bool Tracking::Relocalization(bool save_change)
    {
        Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

        if(vpCandidateKFs.empty()) {
            Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75,true);

        vector<MLPnPsolver*> vpMLPnPsolvers;
        vpMLPnPsolvers.resize(nKFs);

        vector<vector<MapPoint*> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates=0;

        for(int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
                if(nmatches<15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
                    vpMLPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);

        while(nCandidates>0 && !bMatch)
        {
            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                MLPnPsolver* pSolver = vpMLPnPsolvers[i];
                Eigen::Matrix4f eigTcw;
                bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(bTcw)
                {
                    Sophus::SE3f Tcw(eigTcw);
                    mCurrentFrame.SetPose(Tcw);
                    // Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint*> sFound;

                    const int np = vbInliers.size();

                    for(int j=0; j<np; j++)
                    {
                        if(vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j]=NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if(nGood<10)
                        continue;

                    for(int io =0; io<mCurrentFrame.N; io++)
                        if(mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if(nGood<50)
                    {
                        int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                        if(nadditional+nGood>=50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if(nGood>30 && nGood<50)
                            {
                                sFound.clear();
                                for(int ip =0; ip<mCurrentFrame.N; ip++)
                                    if(mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                                // Final optimization
                                if(nGood+nadditional>=50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for(int io =0; io<mCurrentFrame.N; io++)
                                        if(mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io]=NULL;
                                }
                            }
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if(nGood>=50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if(!bMatch)
        {
            return false;
        }
        else
        {
            if(save_change)
                mnLastRelocFrameId = mCurrentFrame.mnId;
            cout << "Relocalized!!" << endl;
            return true;
        }
    }

    bool Tracking::LightTrackWithMotionModel(bool &bVO)
    {
        ORBmatcher matcher(0.9,true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        Frame lastFrameBU = mLastFrame;
        list<MapPoint*> lpTemporalPointsBU = mlpTemporalPoints;


        UpdateLastFrame(); //TODO: check out!

        mCurrentFrame.SetPose(mVelocity*mLastFrame.GetPose());


        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL)); //TODO:Checkout

        // Project points seen in previous frame
        int th;
        if(mSensor!=System::STEREO)
            th=15;
        else
            th=7;

        int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);//TODO:Checkout

        // If few matches, uses a wider window search
        if(nmatches<20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));//TODO:Checkout

            nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);//TODO:Checkout
        }

        if(nmatches<20)
        {
            mLastFrame = lastFrameBU;
            mlpTemporalPoints = lpTemporalPointsBU;
            return false;
        }
        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        mLastFrame = lastFrameBU;
        mlpTemporalPoints = lpTemporalPointsBU;

        bVO = nmatchesMap<10;
        return nmatches>20;

    }

    void Tracking::LightTrack()
    {
        Map* pCurrentMap = mpAtlas->GetCurrentMap();

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);
        bool useMotionModel = true; //set true

        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            std::cout << "Light Track \n";
            cout << "Light Tracking not working because Tracking is not initialized..." << endl;
            return;
        }
        else
        {
            // System is initialized. Track Frame.
            bool bOK;
            {
                // Localization Mode:
                if(mState==LOST)
                {
                    bOK = Relocalization(1);// Lost: relocated
                }
                else
                {
                    if(!mbVO) // There are more points tracked in the previous frame
                    {
                        // In last frame we tracked enough MapPoints in the map
                        cv::Mat mVec = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(mVelocity));
                        if(!mVec.empty() && useMotionModel)
                        {
                            bool _bOK = false;
                            // Lightweight motion tracking mode
                            bOK = LightTrackWithMotionModel(_bOK);// TODO: check out!!!
                        }
                        else
                        {
                            bOK = TrackReferenceKeyFrame();// Tracking Reference Frame
                        }
                    }
                    else
                    {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint*> vpMPsMM;
                        vector<bool> vbOutMM;
                        Sophus::SE3f TcwMM;
                        bool lightTracking = false;
                        bool bVO = false;
                        // Use motion tracking and relocalization mode to calculate two poses,
                        // if relocalization is successful, use the pose obtained by relocalization
                        cv::Mat mVec = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(mVelocity));
                        if(!mVec.empty() && useMotionModel)
                        {
                            lightTracking = true;
                            bOKMM = LightTrackWithMotionModel(bVO); // TODO: check out!!
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.GetPose();
                        }
                        bOKReloc = Relocalization(1);// relocation mode

                        if(bOKMM && !bOKReloc)
                        {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if((lightTracking && bVO) || (!lightTracking && mbVO))
                            {
                                for(int i =0; i<mCurrentFrame.N; i++)
                                {
                                    if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                    {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            if(!bOK)
            {
                if(pCurrentMap->KeyFramesInMap()<=5)
                {
                    cout << "Light Tracking not working..." << endl;
                    return;
                }
                cout << "Light Tracking field..." << endl;
            }
            // else cout << "Light Tracking ok " << endl;

            if(!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;
            //

        }
    }


    bool Tracking::TrackHomo(cv::Mat& homo)
    {
        Map* pCurrentMap = mpAtlas->GetCurrentMap();

        unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);
        // bool useMotionModel = true; //set true

        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            cout << "Light Tracking homo not working because Tracking is not initialized..." << endl;
            return false;
        }
        cv::Mat mVec = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(mVelocity));
        if(mState==OK && !mVec.empty())// Status ok not lost and speed model
        {
            Frame lastFrameBU = mLastFrame; // Back up last frame
            list<MapPoint*> lpTemporalPointsBU = mlpTemporalPoints;// Backup temporary point
            ORBmatcher matcher(0.9,true);// Match point matcher Minimum distance < 0.9*times short distance match successfully
            // Update the previous frame map point
            UpdateLastFrame();//

            mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
            // The current frame pose mVelocity is the pose transformation between the current frame and the previous frame
            // initialize null pointer
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

            // Project points seen in previous frame
            int th;
            if(mSensor  != System::STEREO)
                th=15;// search window
            else
                th=7;
            vector<cv::Point2f> points_last;
            vector<cv::Point2f> points_current;
            int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR,
                                                      points_last,points_current);

            // If few matches, uses a wider window search
            if(nmatches<20)
            {
                points_last.clear();
                points_current.clear();
                fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
                nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR,
                                                      points_last,points_current);
            }

            if(nmatches<20)
            {
                mLastFrame = lastFrameBU;
                mlpTemporalPoints = lpTemporalPointsBU;
                cout << "Tracking homo faild..." << endl;
                return false;
            }
            if(points_current.size() > 50)
            {
                // https://github.com/Ewenwan/MVision/blob/master/CNN/Action_Recognition/IDT/IDT/src/DenseTrackStab.cpp
                // https://github.com/Ewenwan/MVision/blob/master/vSLAM/ch7/pose_estimation_2d2d.cpp
                homo = findHomography ( points_current, points_last, cv::RANSAC, 3 );// points_last = H * points_current

                mLastFrame = lastFrameBU;
                mlpTemporalPoints = lpTemporalPointsBU;

                return true;
            }
            // Revert to original frame, unchanged
            mLastFrame = lastFrameBU;
            mlpTemporalPoints = lpTemporalPointsBU;
        }
        cout << "Tracking homo faild..." << endl;
        return false;
    }

void Tracking::Reset(bool bLocMap)
{
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit=false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();


    //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    //Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    //mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET; //NOT_INITIALIZED;

    mbReadyToInitializate = false;

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(pMap->GetAllKeyFrames().size() > 0)
        {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    //cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    mbVelocity = false;

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    mK_.setIdentity();
    mK_(0,0) = fx;
    mK_(1,1) = fy;
    mK_(0,2) = cx;
    mK_(1,2) = cy;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
{
    Map * pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    list<semantic_slam::KeyFrame*>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    for(auto lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap)
        {
            (*lit).translation() *= s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    while(!mCurrentFrame.imuIsPreintegrated())
    {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated)
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}

// TODO: Adding here
void Tracking::DetectCuboid(KeyFrame *pKF)
{
    cv::Mat pop_pose_to_ground = InitToGround.clone();
    std::vector<double> all_box_confidence;
    vector<int> truth_tracklet_ids;

    // Get 3D object in this keyframe
    std::vector<Object_Map*> all_objs = pKF->obj_3ds;

    std::cout << "Number 3d cuboid in KF " << pKF->mnId << ": " << all_objs.size() << "\n";

    // Change to g2o cuboid
    pKF->local_cuboids.clear();

    g2o::SE3Quat frame_pose_to_init = Converter::toSE3Quat(pKF->GetPoseInverse());

    for (int ii = 0; ii < (int)all_objs.size(); ii++)
    {
//        std::cout << "Step 1 \n";

        if(all_objs[ii]->bad_3d)
        {
            continue;
        }

        all_box_confidence.push_back(all_objs[ii]->mnConfidence_foractive);

        Cuboid3D raw_cuboid  = all_objs[ii]->mCuboid3D;

        g2o::cuboid cube_ground_value;
        Eigen::Matrix<double, 9, 1> cube_pose;
        cube_pose << raw_cuboid.cuboidCenter.x(), raw_cuboid.cuboidCenter.y(), raw_cuboid.cuboidCenter.z(), 0, 0, raw_cuboid.rotY,
                raw_cuboid.lenth, raw_cuboid.width, raw_cuboid.height;
        cube_ground_value.fromMinimalVector(cube_pose);

//        std::cout << "Step 2 \n";

        // Measurement in local camera frame
        MapCuboidObject *newcuboid = new MapCuboidObject(mpAtlas->GetCurrentMap());

        g2o::cuboid cube_local_meas = cube_ground_value.transform_to(Converter::toSE3Quat(pop_pose_to_ground));
        newcuboid->cube_meas = cube_local_meas;

        // TODO: Checking here
//        newcuboid->bbox_2d = all_objs[ii]->ComputeProjectRectFrameToCurrentKeyFrame(*pKF);
        newcuboid->bbox_2d = all_objs[ii]->mRect_byProjectPoints;

//        std::cout << "BBox 2D of cuboid " << ii << ": " << newcuboid->bbox_2d << std::endl;

        newcuboid->bbox_vec = Vector4d((double)newcuboid->bbox_2d.x + (double)newcuboid->bbox_2d.width / 2, (double)newcuboid->bbox_2d.y + (double)newcuboid->bbox_2d.height / 2,
                                       (double)newcuboid->bbox_2d.width, (double)newcuboid->bbox_2d.height);

//        std::cout << "Step 3 \n";

//        newcuboid->box_corners_2d = raw_cuboid->box_corners_2d;

        cv::Rect tmp = all_objs[ii]->ComputeProjectRectFrameToCurrentKeyFrame(*pKF);
        newcuboid->bbox_2d_tight = cv::Rect(tmp.x + tmp.width / 10.0,
                                            tmp.y + tmp.height / 10.0,
                                            tmp.width * 0.8, tmp.height * 0.8);
        get_cuboid_draw_edge_markers(newcuboid->edge_markers, Vector2d(1, 1), false);

        newcuboid->SetReferenceKeyFrame(pKF);
        newcuboid->object_id_in_localKF = pKF->local_cuboids.size();

        // g2o::cuboid global_obj_pose_to_init = cube_local_meas.transform_from(Converter::toSE3Quat(pop_pose_to_ground));
        g2o::cuboid global_obj_pose_to_init = cube_local_meas.transform_from(frame_pose_to_init);

        newcuboid->SetWorldPos(global_obj_pose_to_init);
        newcuboid->pose_noopti = global_obj_pose_to_init;
        if (use_truth_trackid)
            newcuboid->truth_tracklet_id = truth_tracklet_ids[ii];

        if (scene_unique_id == kitti)
        {
            if (cube_local_meas.pose.translation()(0) > 1)
                newcuboid->left_right_to_car = 2; // right
            if (cube_local_meas.pose.translation()(0) < -1)
                newcuboid->left_right_to_car = 1; // left
            if ((cube_local_meas.pose.translation()(0) > -1) && (cube_local_meas.pose.translation()(0) < 1))
                newcuboid->left_right_to_car = 0;
        }
        if (1)
        {
            double obj_cam_dist = std::min(std::max(newcuboid->cube_meas.translation()(2), 10.0), 30.0); // cut into [a,b]
            double obj_meas_quality = (60.0 - obj_cam_dist) / 40.0;
            newcuboid->meas_quality = obj_meas_quality;
        }
        else
            newcuboid->meas_quality = 1.0;
        if (all_box_confidence[ii] > 0)
            newcuboid->meas_quality *= all_box_confidence[ii]; // or =

        if (newcuboid->meas_quality < 0.1)
            std::cout <<"Abnormal measure quality!!:   " << newcuboid->meas_quality << std::endl;
        pKF->local_cuboids.push_back(newcuboid);

    }

    // std::cout << "Tracking: created local object num   " << pKF->local_cuboids.size() << std::endl;
    std::cout << "Tracking: detect cuboid for pKF id: " << pKF->mnId << "  total id: " << pKF->mnFrameId << "  numObj: " << pKF->local_cuboids.size() << std::endl;

    if (whether_save_online_detected_cuboids)
    {
        for (int ii = 0; ii < (int)all_objs.size(); ii++)
        {

            Cuboid3D raw_cuboid  = all_objs[ii]->mCuboid3D;
            g2o::cuboid cube_ground_value;
            Eigen::Matrix<double, 9, 1> cube_pose;

            cube_pose << raw_cuboid.cuboidCenter.x(), raw_cuboid.cuboidCenter.y(), raw_cuboid.cuboidCenter.z(), 0, 0, raw_cuboid.rotY,
                    raw_cuboid.lenth, raw_cuboid.width, raw_cuboid.height;

            save_online_detected_cuboids << pKF->mnFrameId << "  " << cube_pose.transpose() << "\n";

        }
    }

    if (mpSystem->isAssociatePointWithObject)
    {
        std::cout << "Tracking: associate_point_with_object  " << mpSystem->isAssociatePointWithObject
                  << "  whether_dynamic_object  " << mpSystem->isDynamicObject << std::endl;
        if (!mpSystem->isDynamicObject) //for old non-dynamic object, associate based on 2d overlap... could also use instance segmentation
        {
            pKF->keypoint_associate_objectID = vector<int>(pKF->mvKeys.size(), -1);

            std::vector<bool> overlapped(pKF->local_cuboids.size(), false);

            if (1) {
                for (size_t i = 0; i < pKF->local_cuboids.size(); i++)
                {
                    if (!overlapped[i])
                    {
                        for (size_t j = i + 1; j < pKF->local_cuboids.size(); j++)
                        {
                            if (!overlapped[j]) {
                                float iou_ratio = bboxOverlapratio(pKF->local_cuboids[i]->bbox_2d,
                                                                   pKF->local_cuboids[j]->bbox_2d);

//                                std::cout << "Overlapped " << i << " " << j << " ratio: " << iou_ratio << std::endl;

                                if (iou_ratio > 0.15) {
                                    overlapped[i] = true;
                                    overlapped[j] = true;
                                }
                            }
                        }
                    }
                }
            }
            if (!enable_ground_height_scale) {                                       // slightly faster
                if (pKF->local_cuboids.size() > 0) // if there is object
                    for (size_t i = 0; i < pKF->mvKeys.size(); i++) {
                        int associated_times = 0;
                        for (size_t j = 0; j < pKF->local_cuboids.size(); j++)
                        {
                            if (!overlapped[j])
                            {
                                if (pKF->local_cuboids[j]->bbox_2d.contains(pKF->mvKeys[i].pt)) {
                                    associated_times++;
                                    if (associated_times == 1)
                                    {
                                        pKF->keypoint_associate_objectID[i] = j;
                                    }
                                    else
                                    {
                                        pKF->keypoint_associate_objectID[i] = -1;
                                    }
                                }
                            }
                        }
                    }
            } else
            {
                pKF->keypoint_inany_object = vector<bool>(pKF->mvKeys.size(), false);
                for (size_t i = 0; i < pKF->mvKeys.size(); i++) {
                    int associated_times = 0;
                    for (size_t j = 0; j < pKF->local_cuboids.size(); j++)
                    {
                        if (pKF->local_cuboids[j]->bbox_2d.contains(pKF->mvKeys[i].pt)) {
                            pKF->keypoint_inany_object[i] = true;
                            if (!overlapped[j]) {
                                associated_times++;
                                if (associated_times == 1)
                                {
                                    pKF->keypoint_associate_objectID[i] = j;
                                }
                                else
                                {
                                    pKF->keypoint_associate_objectID[i] = -1;
                                }
                            }
                        }
                    }

                }
                if (height_esti_history.size() == 0) {
                    std::cout << "\033[31mTracking: height_esti_history.size() = 0, clear object  \033[0m" << std::endl;
                    pKF->local_cuboids.clear(); // don't do object when in initial stage...
                    pKF->keypoint_associate_objectID.clear();
                }
            }
        }

        std::cout << "[DetectCuboid]: " << pKF->keypoint_associate_objectID.size() << std::endl;

        if (mpSystem->isDynamicObject) //  for dynamic object, I use instance segmentation
        {
            if (pKF->local_cuboids.size() > 0) // if there is object
            {
                std::vector<MapPoint *> framePointMatches = pKF->GetMapPointMatches();

                if (pKF->keypoint_associate_objectID.size() < pKF->mvKeys.size())
                    std::cout << "Tracking Bad keypoint associate ID size   " << pKF->keypoint_associate_objectID.size()
                              << "  " << pKF->mvKeys.size() << std::endl;

                for (size_t i = 0; i < pKF->mvKeys.size(); i++) {
                    if (pKF->keypoint_associate_objectID[i] >= 0 &&
                        pKF->keypoint_associate_objectID[i] >= pKF->local_cuboids.size()) {
                        std::cout << "Detect cuboid find bad pixel obj id  " << pKF->keypoint_associate_objectID[i]
                                  << "  " << pKF->local_cuboids.size() << std::endl;
                    }
                    if (pKF->keypoint_associate_objectID[i] > -1) {
                        MapPoint *pMP = framePointMatches[i];
                        if (pMP)
                            pMP->is_dynamic = true;
                    }
                }
            }
        }
    }

    std::vector<KeyFrame *> checkframes = mvpLocalKeyFrames; // only check recent to save time

    int object_own_point_threshold = 20;
    if (scene_unique_id == kitti)
    {
        if (mono_allframe_Obj_depth_init)
            object_own_point_threshold = 50; // 50 using 10 is too noisy.... many objects don't have enough points to match with others then will create as new...
        else
            object_own_point_threshold = 30; // 30 if don't initialize object point sepratedly, there won't be many points....  tried 20, not good...
    }

    if (use_truth_trackid) //very accurate, no need of object point for association
        object_own_point_threshold = -1;

    // points and object are related in local mapping, when creating mapPoints

    //dynamic object: didn't triangulate point in localmapping. but in tracking
    std::cout << "[Tracking] check the objects has enough points, keyframe num:" << checkframes.size() << std::endl;
    for (size_t i = 0; i < checkframes.size(); i++)
    {
        KeyFrame *kfs = checkframes[i];
        for (size_t j = 0; j < kfs->local_cuboids.size(); j++)
        {
            MapCuboidObject *mPO = kfs->local_cuboids[j];

            if (!mPO->become_candidate)
            {
                // points number maybe increased when later triangulated
                mPO->check_whether_valid_object(object_own_point_threshold);
            }
        }
    }
}

void Tracking::AssociateCuboids(KeyFrame *pKF)
{
    // loop over current KF's objects, check with all past objects (or local objects), compare the associated object map points.
    // (if a local object is not associated, could re-check here as frame-object-point might change overtime, especially due to triangulation.)
    std::cout << "[Tracking] AssociateCuboids  " << "mvpLocalKeyFrames.size()  " << mvpLocalKeyFrames.size() << std::endl;

    std::vector<MapCuboidObject *> LocalObjectsCandidates;
    std::vector<MapCuboidObject *> LocalObjectsLandmarks;
    // keypoint might not added to frame observation yet, so object might not have many associated points yet....
    // method 1: just check current frame's object, using existing map point associated objects.
    // same as plane association, don't just check current frame, but check all recent keyframe's unmatched objects...
    for (size_t i = 0; i < mvpLocalKeyFrames.size(); i++) // pKF is not in mvpLocalKeyFrames yet
    {
        KeyFrame *kfs = mvpLocalKeyFrames[i];
         std::cout << "[Tracking] AssociateCuboids: kfs->local_cuboids.size()  " << kfs->local_cuboids.size() << std::endl;
        for (size_t j = 0; j < kfs->local_cuboids.size(); j++)
        {
            MapCuboidObject *mPO = kfs->local_cuboids[j];
            if (mPO->become_candidate && (!mPO->already_associated))
            {
                LocalObjectsCandidates.push_back(kfs->local_cuboids[j]);
            }
        }
        for (size_t j = 0; j < kfs->cuboids_landmark.size(); j++)
        {
            if (kfs->cuboids_landmark[j]) // might be deleted due to badFlag()
            {
                if (!kfs->cuboids_landmark[j]->isBad())
                {
                    if (kfs->cuboids_landmark[j]->association_refid_in_tracking != pKF->mnId) // could also use set to avoid duplicates
                    {
//                        std::cout << "Step 2 \n";
                        LocalObjectsLandmarks.push_back(kfs->cuboids_landmark[j]);
                        kfs->cuboids_landmark[j]->association_refid_in_tracking = pKF->mnId;
                    }
                }
            }
        }
    }

    std::cout << "[Tracking] Associate cuboids #candidate: " << LocalObjectsCandidates.size() << " #landmarks " << LocalObjectsLandmarks.size()
              << " #localKFs " << mvpLocalKeyFrames.size() << std::endl;

    int largest_shared_num_points_thres = 10;

    if (mono_allframe_Obj_depth_init)
    {
        largest_shared_num_points_thres = 20;
    }
    if (scene_unique_id == kitti)
    {
        largest_shared_num_points_thres = 10; // kitti vehicle occupy large region
    }

    if (mpSystem->isCuboidEnable && mono_allframe_Obj_depth_init) // dynamic object is more difficult. especially reverse motion
    {
        largest_shared_num_points_thres = 5;
    }

    MapCuboidObject *last_new_created_object = nullptr;

    for (size_t i = 0; i < LocalObjectsCandidates.size(); i++)
    {
        // there might be some new created object!
        if (last_new_created_object)
        {
            LocalObjectsLandmarks.push_back(last_new_created_object);
        }
        last_new_created_object = nullptr;

        // find existing object landmarks which share most points with this object
        MapCuboidObject *candidateObject = LocalObjectsCandidates[i];
        std::vector<MapPoint *> object_owned_pts = candidateObject->GetPotentialMapPoints();

        MapCuboidObject *largest_shared_objectlandmark = nullptr;
        if (LocalObjectsLandmarks.size() > 0)
        {
            map<MapCuboidObject *, int> LandmarkObserveCounter;

            for (size_t j = 0; j < object_owned_pts.size(); j++)
            {
                for (map<MapCuboidObject *, int>::iterator mit = object_owned_pts[j]->MapObjObservations.begin(); mit != object_owned_pts[j]->MapObjObservations.end(); mit++)
                {
                    LandmarkObserveCounter[mit->first]++;
                }
            }

            int largest_shared_num_points = largest_shared_num_points_thres;
            for (size_t j = 0; j < LocalObjectsLandmarks.size(); j++)
            {
                MapCuboidObject *pMP = LocalObjectsLandmarks[j];
                if (!pMP->isBad())
                    if (LandmarkObserveCounter.count(pMP))
                    {
                        if (LandmarkObserveCounter[pMP] > largest_shared_num_points)
                        {
                            largest_shared_num_points = LandmarkObserveCounter[pMP];
                            largest_shared_objectlandmark = pMP;
                        }
                    }
            }
        }

        if (largest_shared_objectlandmark == nullptr) // if not found, create as new landmark.  either using original local pointer, or initialize as new
        {
            if (use_truth_trackid)
            {
                if (candidateObject->truth_tracklet_id > -1) // -1 means no ground truth tracking ID, don't use this object
                    trackletid_to_landmark[candidateObject->truth_tracklet_id] = candidateObject;
                else
                    continue;
            }
            candidateObject->already_associated = true; // must be put before SetAsLandmark();
            KeyFrame *refframe = candidateObject->GetReferenceKeyFrame();

            candidateObject->addObservation(refframe, candidateObject->object_id_in_localKF); // add to frame observation
            refframe->cuboids_landmark.push_back(candidateObject);
            candidateObject->mnId = MapCuboidObject::getIncrementedIndex(); //mpMap->MapObjectsInMap();  // needs to manually set
            candidateObject->associated_landmark = candidateObject;
            candidateObject->SetAsLandmark();

            if (scene_unique_id == kitti) // object scale change back and forth
            {
                g2o::cuboid cubeglobalpose = candidateObject->GetWorldPos();
                cubeglobalpose.setScale(Eigen::Vector3d(1.9420, 0.8143, 0.7631));
                candidateObject->SetWorldPos(cubeglobalpose);
                candidateObject->pose_Twc_latestKF = cubeglobalpose;
                candidateObject->pose_noopti = cubeglobalpose;
                candidateObject->allDynamicPoses[refframe] = make_pair(cubeglobalpose, false); //Vector6d::Zero()  false means not BAed
            }

            // TODO: CHECKING HERE
            std::cout << "Add Map object \n";

            (mpAtlas->GetCurrentMap())->AddMapObject(candidateObject);

            last_new_created_object = candidateObject;

            g2o::cuboid cubePose = candidateObject->GetWorldPos();

//            candidateObject->allDynamicPoses[refframe] = std::make_pair(cubePose, false);

            std::cout << "Step 1 \n";
        }
        else // if found, then update observation.
        {
            candidateObject->already_associated = true; // must be put before SetAsLandmark();
            KeyFrame *refframe = candidateObject->GetReferenceKeyFrame();
            largest_shared_objectlandmark->addObservation(refframe, candidateObject->object_id_in_localKF);
            refframe->cuboids_landmark.push_back(largest_shared_objectlandmark);
            candidateObject->associated_landmark = largest_shared_objectlandmark;

            //NOTE use current frame's object poes, but don't use current object if very close to boundary.... large error
            // I use this mainly for kitti, as further objects are inaccurate.  for indoor object, we may not need it
            if (scene_unique_id == kitti)
            {
                g2o::cuboid cubeglobalpose = candidateObject->GetWorldPos();
                cubeglobalpose.setScale(Eigen::Vector3d(1.9420, 0.8143, 0.7631));

                largest_shared_objectlandmark->allDynamicPoses[refframe] = make_pair(cubeglobalpose, false);
                largest_shared_objectlandmark->SetWorldPos(cubeglobalpose);
                largest_shared_objectlandmark->pose_Twc_latestKF = cubeglobalpose; //if want to test without BA
                largest_shared_objectlandmark->pose_noopti = cubeglobalpose;
            }

            std::cout << "Merge to landmark \n";
            largest_shared_objectlandmark->MergeIntoLandmark(candidateObject);
        }
    }

    // remove outlier objects....
    bool remove_object_outlier = true;

    int minimum_object_observation = 2;
    if (scene_unique_id == kitti)
    {
        remove_object_outlier = false;
        if (mpSystem->isCuboidEnable )
        {
            remove_object_outlier = true;
            minimum_object_observation = 3; // dynamic object has more outliers
        }
    }

    bool check_object_points = true;

    if (remove_object_outlier)
    {
        vector<MapCuboidObject *> all_objects = (mpAtlas->GetCurrentMap())->GetAllMapObjects();
        for (size_t i = 0; i < all_objects.size(); i++)
        {
            MapCuboidObject *pMObject = all_objects[i];
            if ((!pMObject->isBad()) && (!pMObject->isGood))						// if not determined good or bad yet.
            {
                if ((int) pMObject->GetLatestKeyFrame()->mnId < (int) pKF->mnId - 15) //20
                {
                    // if not recently observed, and not enough observations.  NOTE if point-object not used in BA, filtered size will be zero...
                    bool no_enough_inlier_pts = check_object_points && (pMObject->NumUniqueMapPoints() > 20) &&
                                                (pMObject->used_points_in_BA_filtered.size() < 10) &&
                                                (pMObject->point_object_BA_counter > -1);
                    if (pMObject->Observations() < minimum_object_observation) {
                        pMObject->SetBadFlag();
                        cout << "Found one bad object !!!!!!!!!!!!!!!!!!!!!!!!!  " << pMObject->mnId << "  "
                             << pMObject->Observations() << "  " << pMObject->used_points_in_BA_filtered.size() << endl;

                        if (use_truth_trackid)
                            trackletid_to_landmark.erase(pMObject->truth_tracklet_id); // remove from track id mapping
                    } else {
                        pMObject->isGood = true;
                    }
                }
            }
        }
    }
}

// TODO: This function should be placed in the constructor of object
// TODO: Improve Cuboid tracker
void Tracking::CreateObject_InTrackMotion(){

    // *****************************
    // STEP 1. construct 2D object *
    // *****************************
    vector<Object_2D *> obj_2ds;

    // 3D object to add in keyframe from this frame
    mCurrentFrame.mvObject_3ds.clear();

    cv::Mat ColorImage = mCurrentFrame.mColorImage.clone();
    Map* mpMap = mpAtlas->GetCurrentMap();
    for (auto &box : mCurrentFrame.boxes)
    {
        Object_2D *obj2d = new Object_2D(mpMap, &mCurrentFrame, box);  //(Map* Map, Frame* CurrentFrame, const BoxSE &box)
        obj_2ds.push_back(obj2d);
    }
    // ***************************************
    // STEP 2. associate objects with points * 
    // ***************************************
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (pMP->isBad())
                continue;

            for (size_t k = 0; k < obj_2ds.size(); ++k)
            {
                if (obj_2ds[k]->mBox_cvRect.contains(mCurrentFrame.mvKeysUn[i].pt))// in rect.
                {
                    //pMP->object_view = true;                  // the point is associated with an object.
                    // TODO: Store the uv coordinates of pMP in the current frame. 
                    // But will it match obj2d->mvMapPonits in other frames, but the uv coordinates have not been modified?
                    // Answer: No. Therefore, it is stored in obj2d->mvMapPonits according to the uv coordinates. 
                    // So it must be modified
                    pMP->feature_uvCoordinate = mCurrentFrame.mvKeysUn[i]; // coordinate in current frame.
                    obj_2ds[k]->AddObjectPoint(pMP);
                }
            }
        }
    }

    // ***************************************
    // STEP 3. associate objects with lines *  
    // ***************************************
    // all lines in current frame.
    Eigen::MatrixXd AllLinesEigen = mCurrentFrame.all_lines_eigen;

    // step 1 make sure edges start from left to right.
    align_left_right_edges(AllLinesEigen);

    for(int i = 0; i < obj_2ds.size(); i++)
    {
        Object_2D* obj = obj_2ds[i];

        // step 2. expand the bounding box.
        double dLeftExpand = max(0.0, obj->mBox_cvRect.x - 15.0);
        double dRightExpand = min(mCurrentFrame.mColorImage.cols, obj->mBox_cvRect.x + obj->mBox_cvRect.width + 15);
        double dTopExpand = max(0.0, obj->mBox_cvRect.y - 15.0);
        double dBottomExpand = min(mCurrentFrame.mColorImage.rows, obj->mBox_cvRect.y + obj->mBox_cvRect.height + 15);
        Eigen::Vector2d ExpanLeftTop = Eigen::Vector2d(dLeftExpand, dTopExpand);			// lefttop.         
		Eigen::Vector2d ExpanRightBottom = Eigen::Vector2d(dRightExpand, dBottomExpand);  // rightbottom.     

        // step 3. Associate objects and lines together
        // associate object with lines.
        Eigen::MatrixXd ObjectLines(AllLinesEigen.rows(),AllLinesEigen.cols());
		int nInsideLinesNum = 0;
		for (int line_id = 0; line_id < AllLinesEigen.rows(); line_id++)   /* In the current frame, the line_id line */
        {
            // check endpoints of the lines, whether inside the box.
            if (check_inside_box(   AllLinesEigen.row(line_id).head<2>(),
                                    ExpanLeftTop,
                                    ExpanRightBottom ))
            {
                if(check_inside_box(AllLinesEigen.row(line_id).tail<2>(),
                                    ExpanLeftTop,
                                    ExpanRightBottom ))
                {
                    ObjectLines.row(nInsideLinesNum) = AllLinesEigen.row(line_id); /* The line_id line belongs to the i-th bbox, so insert this line into the ObjectLines collection. */
                    nInsideLinesNum++;
                }
            }
        }

        // step 4. merge lines.
        double pre_merge_dist_thre = 20;
		double pre_merge_angle_thre = 5;
		double edge_length_threshold = 30;
	    Eigen::MatrixXd ObjectLinesAfterMerge;
		merge_break_lines(	ObjectLines.topRows(nInsideLinesNum),
							ObjectLinesAfterMerge, 		// output lines after merge.
							pre_merge_dist_thre,		// the distance threshold between two line, 20 pixels.
							pre_merge_angle_thre, 		// angle threshold between two line, 5°.
							edge_length_threshold);		// length threshold, 30 pixels.

        // step 5. save object lines.
        obj->mObjLinesEigen = ObjectLinesAfterMerge;  //linedebug

        /* The capacity of vObjsLines is equal to objs_2d.size(). If a bit of vObjsLines is empty, it means that this object is not associated with a line. */
        mCurrentFrame.vObjsLines.push_back(ObjectLinesAfterMerge);   
    }

    // ***************************************************
    // STEP 4.
    // (1)compute the mean and standard of points.*
    // (2)Erase outliers (camera frame) by boxplot.*
    // **************************************************
    for (auto &obj2d : obj_2ds)
    {
        // compute the mean and standard.
        obj2d->ComputeMeanAndDeviation();

        // If the object has too few points, ignore.
        if (obj2d->mvMapPonits.size() < 8)
            continue;

        // Erase outliers by boxplot.
        obj2d->RemoveOutlier_ByHeightorDepth();
        obj2d->ComputeMeanAndDeviation();
    }

    // **************************************************************************
    // STEP 5. construct the bounding box by object feature points in the image.*
    // **************************************************************************
    // bounding box detected by yolo |  bounding box constructed by object points.
    //  _______________                 //  _____________
    // |   *        *  |                // |   *        *|
    // |    *  *       |                // |    *  *     |
    // |*      *  *    |                // |*      *  *  |
    // | *   *    *    |                // | *   *    *  |
    // |   *       *   |                // |___*_______*_|
    // |_______________|

    cv::Mat Tcw_ = Converter::toCvMat(Converter::toSE3Quat(mCurrentFrame.GetPose()));

    const cv::Mat Rcw = Tcw_.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Tcw_.rowRange(0, 3).col(3);

    for (auto &obj2d : obj_2ds)
    {
        // record the coordinates of each point in the xy(uv) directions.
        vector<float> x_pt;
        vector<float> y_pt;
        for (auto &pMP : obj2d->mvMapPonits)
        {
            float u = pMP->feature_uvCoordinate.pt.x;
            float v = pMP->feature_uvCoordinate.pt.y;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        if (x_pt.size() < 4) // ignore.
            continue;

        // extremum in xy(uv) direction
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        // make insure in the image.
        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > ColorImage.cols)
            x_max = ColorImage.cols;
        if (y_max > ColorImage.rows)
            y_max = ColorImage.rows;

        // the bounding box constructed by object feature points.
        // notes: Feature points within the field of view
        // Used in data associate
        obj2d->mBox_cvRect_FeaturePoints = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
    }

    // **********************************************************************************************
    // STEP 6. remove 2d bad bounding boxes.
    // Due to the complex scene and Yolo error detection, some poor quality objects need to be removed.
    // The strategy can be adjusted and is not unique, such as:
    // 1. objects overlap with too many object;
    // 2. objects with too few points;
    // 3. objects with too few points and on the edge of the image;
    // 4. objects too large and take up more than half of the image;
    // and so on ......
    // **********************************************************************************************
    // overlap with too many objects.
    for (size_t f = 0; f < obj_2ds.size(); ++f)
    {
        int num = 0;
        for (size_t l = 0; l < obj_2ds.size(); ++l)
        {
            if (f == l)
                continue;

            if (Converter::bboxOverlapratioLatter(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.05)
                num++;
        }
        // overlap with more than 3 objects
        if (num > 4)
            obj_2ds[f]->bad = true;
    }
    for (size_t f = 0; f < obj_2ds.size(); ++f)
    {
        if (obj_2ds[f]->bad)
            continue;

        // ignore the error detect by yolo.
        // if ((objs_2d[f]->_class_id == 0) || (objs_2d[f]->_class_id == 63) || (objs_2d[f]->_class_id == 15))
        //     objs_2d[f]->bad = true;

        // too large in the image.
        if ((float)obj_2ds[f]->mBox_cvRect.area() / (float)(ColorImage.cols * ColorImage.rows) > 0.5)
            obj_2ds[f]->bad = true;

        // too few object points.
        if (obj_2ds[f]->mvMapPonits.size() < 5)
            obj_2ds[f]->bad = true;

        // object points too few and the object on the edge of the image.
        else if  (obj_2ds[f]->mvMapPonits.size() < 10)
        {
            if ((obj_2ds[f]->mBox_cvRect.x < 20) || (obj_2ds[f]->mBox_cvRect.y < 20) ||
                (obj_2ds[f]->mBox_cvRect.x + obj_2ds[f]->mBox_cvRect.width > ColorImage.cols - 20) ||
                (obj_2ds[f]->mBox_cvRect.y + obj_2ds[f]->mBox_cvRect.height > ColorImage.rows - 20))
            {
                obj_2ds[f]->bad = true;
            }
        }

        // when the overlap is large, only one object remains.
        for (size_t l = 0; l < obj_2ds.size(); ++l)
        {
            if (obj_2ds[l]->bad)
                continue;

            if (f == l)
                continue;

            // retain objects which with high probability.
            if (Converter::bboxOverlapratio(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.3)
            {
                if (obj_2ds[f]->mScore < obj_2ds[l]->mScore)
                    obj_2ds[f]->bad = true;
                else if (obj_2ds[f]->mScore >= obj_2ds[l]->mScore)
                    obj_2ds[l]->bad = true;
            }
            // if one object surrounds another, keep the larger one.
            if (Converter::bboxOverlapratio(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.05)
            {
                if (Converter::bboxOverlapratioFormer(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.85)
                    obj_2ds[f]->bad = true;
                if (Converter::bboxOverlapratioLatter(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.85)
                    obj_2ds[l]->bad = true;
            }
        }
    }

    // erase the bad object.
    vector<Object_2D *>::iterator it;
    for (it = obj_2ds.begin(); it != obj_2ds.end(); )
    {
        if ((*it)->bad == true)
            it = obj_2ds.erase(it); // erase.
        else
            ++it;
    }

    // *************************************************************
    // STEP 7. copy objects in the last frame after initialization.*
    // *************************************************************
    if ((mbObjectIni == true) && (mCurrentFrame.mnId > mnObjectIniFrameID))
    {
        // copy objects in the last frame.
        mCurrentFrame.mvLastObject_2ds = mLastFrame.mvObject_2ds;

        // copy objects in the penultimate frame.
        if (!mLastFrame.mvLastObject_2ds.empty())
            mCurrentFrame.mvLastLastObject_2ds = mLastFrame.mvLastObject_2ds;
    }

    // *******************************************************************************
    // STEP 8. Merges objects with 5-10 points  between two adjacent frames.
    // Advantage: Small objects with too few points, can be merged to keep them from being eliminated.
    // (The effect is not very significant.)
    // Copy the points in the object detection frame in the previous frame to the corresponding obj2d according to iou.
    // But I want to ask which object mvLastLastObject_2ds is???
    // *******************************************************************************
    bool bMergeTwoObj = true;
    if ((!mCurrentFrame.mvLastObject_2ds.empty()) && bMergeTwoObj)
    {
        // object in current frame.
        for (size_t k = 0; k < obj_2ds.size(); ++k)
        {
            // ignore objects with more than 10 points.
            if (obj_2ds[k]->mvMapPonits.size() >= 10)
                continue;

            // object in last frame.
            for (size_t l = 0; l < mCurrentFrame.mvLastObject_2ds.size(); ++l)
            {
                // ignore objects with more than 10 points.
                if (mCurrentFrame.mvLastObject_2ds[l]->mvMapPonits.size() >= 10)
                    continue;

                // merge two objects.  
                // If the overlap rate of two object_2d is greater than 0.5, the two are fused into one object
                if (Converter::bboxOverlapratio(obj_2ds[k]->mBox_cvRect, mCurrentFrame.mvLastObject_2ds[l]->mBox_cvRect) > 0.5)
                {
                    obj_2ds[k]->MergeTwo_Obj2D(mCurrentFrame.mvLastObject_2ds[l]);
                    obj_2ds[k]->ComputeMeanAndDeviation();
                    break;
                }
            }
        }
    }

    // ************************************
    // STEP 9. Initialize the object map  *
    // ************************************
    if ( mbObjectIni == false){
        int nGoodObjId = -1;        // object id.
        for (auto &obj2d : obj_2ds)
        {
            // Initialize the object map need enough points.
            if (obj2d->mvMapPonits.size() < 10)
            {
                continue;
            }

            nGoodObjId++;;

            // Create an object in the map.
            Object_Map *Object3D = new Object_Map;
            Object3D->mvObject_2ds.push_back(obj2d);   // 2D objects in each frame associated with this 3D map object.
            Object3D->mnId = nGoodObjId;             // 3d objects in the map.
            Object3D->mnClass = obj2d->mclass_id;      // object class.
            Object3D->mnConfidence_foractive = 1;              // object confidence = mObjectFrame.size().
            Object3D->mnAddedID_nouse = mCurrentFrame.mnId;        // added id.
            Object3D->mnLastAddID = mCurrentFrame.mnId;      // last added id.
            Object3D->mnLastLastAddID = mCurrentFrame.mnId;  // last last added id.
            Object3D->mLastRect = obj2d->mBox_cvRect;             // last rect.
            //Object3D->mPredictRect = obj->mBoxRect;       // for iou.
            Object3D->mSumPointsPos = 0; //cv::Mat::zeros(3,1,CV_32F);
            Object3D->mAveCenter3D = obj2d->mPos_world;  ; //cv::Mat::zeros(3,1,CV_32F);

            // add properties of the point and save it to the object.
            for (size_t i = 0; i < obj2d->mvMapPonits.size(); i++)
            {
                if(obj2d->mvMapPonits[i]->isBad())
                    continue;

                MapPoint *pMP = obj2d->mvMapPonits[i];
                pMP->object_mnId = Object3D->mnId;
                pMP->object_class = Object3D->mnClass;
                pMP->viewdCount_forObjectId.insert(make_pair(Object3D->mnId, 1)); // the point is first observed by the object.

                // save to the object.
                Object3D->mvpMapObjectMappoints.push_back(pMP);
                Object3D->mvpMapObjectMappoints_NewForActive.push_back(pMP);
            }

            // 2d object.
            obj2d->mnId = Object3D->mnId;

            // save this 2d object to current frame (associates with a 3d object in the map).
            mCurrentFrame.mvObject_2ds.push_back(obj2d);

            // updata map object.
            Object3D->ComputeMeanAndDeviation_3D();

            //mpMap->mvObjectMap.push_back(ObjectMapSingle);
            mpMap->AddObject(Object3D);

            // The object is initialized
            mbObjectIni = true;
            mnObjectIniFrameID = mCurrentFrame.mnId;
        }
    }

    // TODO: ADD OBJECT_MAP TO KEYFRAME
    // **************************************************************
    // STEP 10. Data association and create objects*
    // **************************************************************
    if ((mCurrentFrame.mnId > mnObjectIniFrameID) && (mbObjectIni == true))
    {
        // step 10.1 points of the object that appeared in the last 30 frames
        // are projected into the image to form a projection bounding box.
        // Objects that appeared in the map in the past 30 frames, obj3d ∈ obj_3ds
        // The point in the project is projected into the Currentframe, 
        // and the projection bounding box of obj3d in the Currentframe is calculated, 
        // and stored in mRectProject_forDataAssociate2D of obj3d
        // To view obj3d, you can associate objects in Currentframe

        const std::vector<Object_Map*> obj_3ds = mpMap->GetObjects();

        for (int i = 0; i < (int)obj_3ds.size(); i++)
        {
            Object_Map* obj3d = obj_3ds[i];
            if (obj3d->bad_3d)
                continue;

            // object appeared in the last 30 frames.
            if(ProIou_only30_flag) {
                if (obj3d->mnLastAddID > mCurrentFrame.mnId - 30)
                {
                    // Project the point in obj3d to the current frame and calculate the projected bounding box
                    obj3d->ComputeProjectRectFrameToCurrentFrame(mCurrentFrame);
                    mCurrentFrame.mvObject_3ds.push_back(obj_3ds[i]);
                }
                else {
                    obj3d->mRect_byProjectPoints = cv::Rect(0, 0, 0, 0);
                }
            }
            else{
                // Project the point in obj3d to the current frame and calculate the projected bounding box
                obj3d->ComputeProjectRectFrameToCurrentFrame(mCurrentFrame);
                mCurrentFrame.mvObject_3ds.push_back(obj3d);
            }
        }

        // step 10.2 data association and creat new object
        // Compare obj2d in the current frame with all obj3d in the map to see if the data association is the same object
        // If not, generate a new object and add it to the map.
        for (size_t k = 0; k < obj_2ds.size(); ++k)
        {
            // ignore object with less than 5 points.
            if (obj_2ds[k]->mvMapPonits.size() < 5)
                continue;

            // Incorporate old objects or generate new ones
            int result = obj_2ds[k]->creatObject();
            switch (result) {
                case -1:   cout << "[Tracking] The detection frame is close to the edge" << endl;   break;
                case 0:    cout << "[Tracking] Blend into old objects" << endl;   break;
                case 1:    cout << "[Tracking] Generate new objects" << endl;     break;
            }
        }

        // **************************************************************
        // STEP 11. After a round of object generation, manage the objects in the map*
        // **************************************************************
        // step 11.1 remove objects with too few observations.
        // After updating the objects in the map, reacquire the objects in the map, 
        // and remove objects with fewer observations (by setting obj3d's bad_3d to true)
        const std::vector<Object_Map*> obj_3ds_new = mpMap->GetObjects();

        for (int i = (int)obj_3ds_new.size() - 1; i >= 0; i--)
        {
            Object_Map* obj3d = obj_3ds_new[i];
            if (obj3d->bad_3d)
                continue;

            // not been observed in the last 30 frames.
            if (obj3d->mnLastAddID < (mCurrentFrame.mnId - 30))
            {
                int df = (int)obj3d->mvObject_2ds.size();
                if (df < 10)
                {
                    // If the object has not been seen in the past 30 frames, and the number of observed frames is less than 5, it is set to bad_3d
                    if (df < 5){
                        std::cout << "object->bad Observation frame<5, object id:" << obj3d->mnLastAddID
                                  << ", frame id: " << (mCurrentFrame.mnId - 30) << std::endl;
                        obj3d->bad_3d = true;
                    }
                    // If the object has not been seen in the past 30 frames, and the number of observed frames is less than 10, 
                    // and it overlaps too much with other objects in the map, set it to bad_3d
                    else
                    {
                        for (int j = (int)obj_3ds_new.size() - 1; j >= 0; j--)
                        {
                            if (obj_3ds_new[j]->bad_3d || (i == j))
                                continue;

                            bool overlap = obj_3ds_new[i]->WhetherOverlap(obj_3ds_new[j]) ;
                            if (overlap)
                            {
                                obj_3ds_new[i]->bad_3d = true;
                                std::cout << "object->bad Observation frame<10, and overlap" << std::endl;
                                break;
                            }
                        }
                    }
                }
            }
        }

        // step 11.2 Update the co-view relationship between objects. (appears in the same frame).
        // For objects created for the first time, build a common-view relationship.
        for (int i = (int)obj_3ds_new.size() - 1; i >= 0; i--)
        {
            if (obj_3ds_new[i]->mnLastAddID == mCurrentFrame.mnId)
            {
                for (int j = (int)obj_3ds_new.size() - 1; j >= 0; j--)
                {
                    if (i == j)
                        continue;

                    if (obj_3ds_new[j]->mnLastAddID == mCurrentFrame.mnId)
                    {
                        obj_3ds_new[i]->UpdateCoView(obj_3ds_new[j]);
                    }
                }
            }
        }

        // Step 11.3 Merge potential associate objects (see mapping thread).

        // Step 11.4 Estimate the orientation of objects.
        for (int i = (int)obj_3ds_new.size() - 1; i >= 0; i--)
        {
            Object_Map* obj3d = obj_3ds_new[i];

            if (obj3d->bad_3d)
                continue;
            // If more than 5 frames, this object is not observed, skip
            if (obj3d->mnLastAddID < mCurrentFrame.mnId - 5)
                continue;

            // estimate only regular objects. // if (((objMap->mnClass == 73) || (objMap->mnClass == 64) || (objMap->mnClass == 65//     || (objMap->mnClass == 66) || (objMap->mnClass == 56)))
            // objects appear in current frame.
            if(obj3d->mnLastAddID == mCurrentFrame.mnId)
            {
                SampleObjYaw(obj3d);
            }

            // step 11.5 project quadrics to the image (only for visualization).
            cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
            axe.at<float>(0) = obj3d->mCuboid3D.lenth / 2;
            axe.at<float>(1) = obj3d->mCuboid3D.width / 2;
            axe.at<float>(2) = obj3d->mCuboid3D.height / 2;
            // object pose (world).
            cv::Mat Twq = (obj3d->mCuboid3D.pose_mat);
            // Projection Matrix K[R|t].
            cv::Mat P(3, 4, CV_32F);
            Rcw.copyTo(P.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(P.rowRange(0, 3).col(3));
            P = mCurrentFrame.mK * P;

            DrawQuadricProject( this->mCurrentFrame.mQuadricImage,
                                        P,
                                        axe,
                                        Twq,
                                        obj3d->mnClass).copyTo(this->mCurrentFrame.mQuadricImage);
        }
    }
}

cv::Mat Tracking::DrawQuadricProject(cv::Mat &im,
                                     const cv::Mat &P,   // projection matrix.
                                     const cv::Mat &axe, // axis length.
                                     const cv::Mat &Twq, // object pose.
                                     int nClassid,
                                     bool isGT,
                                     int nLatitudeNum,
                                     int nLongitudeNum)
{
    // color.
    std::vector<cv::Scalar> colors = {  cv::Scalar(135,0,248),
                                        cv::Scalar(255,0,253),
                                        cv::Scalar(4,254,119),
                                        cv::Scalar(255,126,1),
                                        cv::Scalar(0,112,255),
                                        cv::Scalar(0,250,250),
                                        };

    // draw params
    cv::Scalar sc = colors[nClassid % 6];

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
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
        }
        // [180, 360]
        for (int j = 0; j < pointGrid.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
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
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
        }
        // [180, 360]
        for (int j = 0; j < pointGrid_t.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
        }
    }

    return im;
}

// Estimate object orientation.
void Tracking::SampleObjYaw(Object_Map* obj3d)
{
    int numMax = 0;
    float fError = 0.0;
    float fErrorYaw;
    float minErrorYaw = 360.0;
    float sampleYaw = 0.0;
    int nAllLineNum = obj3d->mvObject_2ds.back()->mObjLinesEigen.rows(); 

    for(int i = 0; i < 30; i++)
    {
        // initial angle.
        float roll, pitch, yaw;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        float error = 0.0;
        float errorYaw = 0.0;

        // 1 -> 15: -45° - 0°
        // 16 -> 30: 0° - 45°
        if(i < 15)
            yaw = (0.0 - i*3.0)/180.0 * M_PI;
        else
            yaw = (0.0 + (i-15)*3.0)/180.0 * M_PI;

        // object pose in object frame. (Ryaw)
        float cp = cos(pitch);
        float sp = sin(pitch);
        float sr = sin(roll);
        float cr = cos(roll);
        float sy = sin(yaw);
        float cy = cos(yaw);
        Eigen::Matrix<double,3,3> REigen;
        REigen<<   cp*cy, (sr*sp*cy)-(cr*sy), (cr*sp*cy)+(sr*sy),
                cp*sy, (sr*sp*sy)+(cr*cy), (cr*sp*sy)-(sr*cy),
                    -sp,    sr*cp,              cr * cp;
        cv::Mat Ryaw = Converter::toCvMat(REigen);

        // 8 vertices of the 3D box, world --> object frame.
        cv::Mat corner_1 = Converter::toCvMat(obj3d->mCuboid3D.corner_1_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_2 = Converter::toCvMat(obj3d->mCuboid3D.corner_2_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_3 = Converter::toCvMat(obj3d->mCuboid3D.corner_3_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_4 = Converter::toCvMat(obj3d->mCuboid3D.corner_4_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_5 = Converter::toCvMat(obj3d->mCuboid3D.corner_5_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_6 = Converter::toCvMat(obj3d->mCuboid3D.corner_6_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_7 = Converter::toCvMat(obj3d->mCuboid3D.corner_7_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_8 = Converter::toCvMat(obj3d->mCuboid3D.corner_8_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);

        // rotate in object frame  + object frame --> world frame.
        corner_1 = Ryaw * corner_1 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_2 = Ryaw * corner_2 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_3 = Ryaw * corner_3 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_4 = Ryaw * corner_4 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_5 = Ryaw * corner_5 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_6 = Ryaw * corner_6 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_7 = Ryaw * corner_7 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_8 = Ryaw * corner_8 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);

        // project 8 vertices to image.
        cv::Point2f point1, point2, point3, point4, point5, point6, point7, point8;
        point1 = WorldToImg(corner_1);
        point2 = WorldToImg(corner_2);
        point3 = WorldToImg(corner_3);
        point4 = WorldToImg(corner_4);
        point5 = WorldToImg(corner_5);
        point6 = WorldToImg(corner_6);
        point7 = WorldToImg(corner_7);
        point8 = WorldToImg(corner_8);

        // angle of 3 edges(lenth, width, height).
        float angle1;
        float angle2;
        float angle3;
        // left -> right.
        if(point6.x > point5.x)
            angle1 = atan2(point6.y - point5.y, point6.x - point5.x);
        else
            angle1 = atan2(point5.y - point6.y, point5.x - point6.x);
        float lenth1 = sqrt((point6.y - point5.y) * (point6.y - point5.y) + (point6.x - point5.x) * (point6.x - point5.x));

        if(point7.x > point6.x)
            angle2 = atan2(point7.y - point6.y, point7.x - point6.x);
        else
            angle2 = atan2(point6.y - point7.y, point6.x - point7.x);
        float lenth2 = sqrt((point7.y - point6.y) * (point7.y - point6.y) + (point7.x - point6.x) * (point7.x - point6.x));

        if(point6.x > point2.x)
            angle3 = atan2(point6.y - point2.y, point6.x - point2.x);
        else
            angle3 = atan2(point2.y - point6.y, point2.x - point6.x);
        float lenth3 = sqrt((point6.y - point2.y) * (point6.y - point2.y) + (point6.x - point2.x) * (point6.x - point2.x));

        // compute angle between detected lines and cube edges.
        int num = 0;  // number parallel lines.
        for(int line_id = 0; line_id < obj3d->mvObject_2ds.back()->mObjLinesEigen.rows(); line_id++)
        {
            // angle of detected lines.
            double x1 = obj3d->mvObject_2ds.back()->mObjLinesEigen(line_id, 0);
            double y1 = obj3d->mvObject_2ds.back()->mObjLinesEigen(line_id, 1);
            double x2 = obj3d->mvObject_2ds.back()->mObjLinesEigen(line_id, 2);
            double y2 = obj3d->mvObject_2ds.back()->mObjLinesEigen(line_id, 3);
            float angle = atan2(y2 - y1, x2 - x1);

            // lenth.
            float lenth = sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));

            // angle between line and 3 edges.
            float dis_angle1 = abs(angle * 180/M_PI - angle1 * 180/M_PI);
            float dis_angle2 = abs(angle * 180/M_PI - angle2 * 180/M_PI);
            float dis_angle3 = abs(angle * 180/M_PI - angle3 * 180/M_PI);

            float th = 5.0;             // threshold of the angle.
            if(obj3d->mnClass == 56)   // chair.
            {
                if((dis_angle2 < th) || (dis_angle3 < th))
                    num++;
                if(dis_angle1 < th)
                {
                    num+=3;
                }
            }
            else
            {
                // the shortest edge is lenth1.
                if( min(min(lenth1, lenth2), lenth3) == lenth1)
                {
                    // error with other two edges.
                    if((dis_angle2 < th) || (dis_angle3 < th))
                    {
                        num++;
                        if(dis_angle2 < th)
                            error += dis_angle2;
                        if(dis_angle3 < th)
                            error += dis_angle3;
                    }

                    // angle error.
                    errorYaw+=min(dis_angle2, dis_angle3);
                }
                // the shortest edge is lenth2.
                if( min(min(lenth1, lenth2), lenth3) == lenth2)
                {
                    if((dis_angle1 < th) || (dis_angle3 < th))
                    {
                        num++;
                        if(dis_angle1 < th)
                            error += dis_angle1;
                        if(dis_angle3 < th)
                            error += dis_angle3;
                    }
                    errorYaw+=min(dis_angle3, dis_angle1);
                }
                // the shortest edge is lenth3.
                if( min(min(lenth1, lenth2), lenth3) == lenth3)
                {
                    if((dis_angle1 < th) || (dis_angle2 < th))
                    {
                        num++;
                        if(dis_angle1 < th)
                            error += dis_angle1;
                        if(dis_angle2 < th)
                            error += dis_angle2;
                    }
                    errorYaw+=min(dis_angle2, dis_angle1);
                }
            }
        }
        if(num == 0)
        {
            num = 1;
            errorYaw = 10.0;
        }

        // record the angle with max number parallel lines.
        if(num > numMax)
        {
            numMax = num;
            sampleYaw = yaw;

            fError = error; // no used in this version.
            // average angle error.
            fErrorYaw = (errorYaw/(float)num)/10.0;
        }
    }

    // scoring.
    float fScore;
    fScore = ((float)numMax / (float)nAllLineNum) * (1.0 - 0.1 * fErrorYaw);
    if(isinf(fScore))
        fScore = 0.0;

    // measurement： yaw, times, score, angle, angle error.
    Eigen::Matrix<float,5,1> AngleTimesAndScore;
    AngleTimesAndScore[0] = sampleYaw;
    AngleTimesAndScore[1] = 1.0;
    AngleTimesAndScore[2] = fScore;
    AngleTimesAndScore[3] = fError;     // no used in this version.
    AngleTimesAndScore[4] = fErrorYaw;

    // update multi-frame measurement.
    bool bNewMeasure = true;
    for (auto &row : obj3d->mvAngleTimesAndScore)
    {
        if(row[0] == AngleTimesAndScore[0])
        {
            row[1] += 1.0;
            row[2] = AngleTimesAndScore[2] * (1/row[1]) + row[2] * (1 - 1/row[1]);
            row[3] = AngleTimesAndScore[3] * (1/row[1]) + row[3] * (1 - 1/row[1]);
            row[4] = AngleTimesAndScore[4] * (1/row[1]) + row[4] * (1 - 1/row[1]);

            bNewMeasure = false;
        }
    }
    if(bNewMeasure == true)
    {
        obj3d->mvAngleTimesAndScore.push_back(AngleTimesAndScore);
    }

    //index = 1;
    std::sort(obj3d->mvAngleTimesAndScore.begin(), obj3d->mvAngleTimesAndScore.end(), VIC);
    //for (auto &row : obj3d->mvAngleTimesAndScore)
    //{
    //    std::cout << row[0] * 180.0 / M_PI  << "\t" <<  row[1] << "\t" <<  row[2] << std::endl;
    //}

    int best_num = 0;
    float best_score = 0;
    for(int i = 0; i < std::min(3, (int)obj3d->mvAngleTimesAndScore.size()); i++)
    {
        float fScore = obj3d->mvAngleTimesAndScore[i][2];
        if(fScore >= best_score)
        {
            best_score = fScore;
            best_num = i;
        }
    }

    // step 6. update object yaw.
    obj3d->mCuboid3D.rotY = obj3d->mvAngleTimesAndScore[best_num][0];
    obj3d->mCuboid3D.mfErrorParallel = obj3d->mvAngleTimesAndScore[best_num][3];
    obj3d->mCuboid3D.mfErroeYaw = obj3d->mvAngleTimesAndScore[best_num][4];
}

// project points to image.
cv::Point2f Tracking::WorldToImg(cv::Mat &PointPosWorld)
{
    // world.

    cv::Mat Tcw_ = Converter::toCvMat(Converter::toSE3Quat(mCurrentFrame.GetPose()));

    const cv::Mat Rcw = Tcw_.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Tcw_.rowRange(0, 3).col(3);

    // camera.
    cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

    const float xc = PointPosCamera.at<float>(0);
    const float yc = PointPosCamera.at<float>(1);
    const float invzc = 1.0 / PointPosCamera.at<float>(2);

    // image.
    float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
    float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

    return cv::Point2f(u, v);
}

void Tracking::NewDataset()
{
    mnNumDataset++;
}

int Tracking::GetNumberDataset()
{
    return mnNumDataset;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder)
{
    mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
    //mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap)
{
    mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
    if(!strNameFile_kf.empty())
        mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::GetImageScale()
{
    return mImageScale;
}

Sophus::SE3f Tracking::GetCamTwc()
{
    return (mCurrentFrame.GetPose()).inverse();
}

Sophus::SE3f Tracking::GetImuTwb()
{
    return mCurrentFrame.GetImuPose();
}

Eigen::Vector3f Tracking::GetImuVwb()
{
    return mCurrentFrame.GetVelocity();
}

bool Tracking::isImuPreintegrated()
{
    return mCurrentFrame.mpImuPreintegrated;
}

#ifdef REGISTER_LOOP
void Tracking::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool Tracking::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Tracking STOP" << endl;
        return true;
    }

    return false;
}

bool Tracking::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool Tracking::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

void Tracking::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}

#endif

} //namespace semantic_slam
