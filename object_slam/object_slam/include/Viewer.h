
#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"
#include "Object.h"
#include "MapPublisher.h"

#include <mutex>

namespace semantic_slam
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Settings;
class Object_Map;
class MapPublisher;

class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking,
           const string &strSettingPath, Settings* settings, MapPublisher* mpMapPublisher);

    void newParameterLoader(Settings* settings);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    bool isStepByStep();

    void Release();

    void Finalize(void);

    //void SetTrackingPause();

    bool both;

    std::vector<cv::Rect2i> mvPersonArea;
    map<string, vector<cv::Rect2i>> mmDetectMap;
    std::mutex mMutexPAFinsh;
    
private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;
    MapPublisher* mpMapPublisher;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mImageViewerScale;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    bool mbStopTrack;

    // For 3D cuboid testing
    int run_pangolin = 1;
    int run_rviz;
    int read_local_object;
    int show_object3d_frame;

    void read_local_object_file();
    void compute_corner(Object_Map* object);
    // demo.
    string mflag;
    //nbv test
    std::vector<semantic_slam::Object_Map*> vObjects;
    float mfx, mfy, mcx, mcy;

};

}

#endif // VIEWER_H
	

