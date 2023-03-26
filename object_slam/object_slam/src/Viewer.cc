#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace semantic_slam
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
               const string &strSettingPath, Settings* settings, MapPublisher* mMapPublisher):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), mpMapPublisher(mMapPublisher)
{
    if(settings)
    {
        std::cout << "Here \n";
        newParameterLoader(settings);
    }
    else
    {
        std::cout << "Normal reading \n";
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cout << "Viewer param \n";
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

    mbStopTrack = false;
}

void Viewer::newParameterLoader(Settings *settings) {
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();
    mViewpointX = settings->viewPointX();
    mViewpointY = settings->viewPointY();
    mViewpointZ = settings->viewPointZ();
    mViewpointF = settings->viewPointF();

    // For 3D cuboid
    run_pangolin = settings->runPangolin();
    run_rviz = settings->runRviz();
    read_local_object = settings->readLocalObject();
    show_object3d_frame = settings->showObject3DFrame();

    std::cout << "Show object 3D frame mode: " << show_object3d_frame << "\n";

}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    // For 3D cuboid testing
    node = fSettings["Viewer.pangolin"];
    if(!node.empty())
    {
        run_pangolin = (int)node.real();
    }
    else
    {
        std::cerr << "*Viewer.pangolin parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.rviz"];
    if(!node.empty())
    {
        run_rviz = (int)node.real();
    }
    else
    {
        std::cerr << "*Viewer.rviz parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.readlocalobject"];
    if(!node.empty())
    {
        read_local_object = (int)node.real();
    }
    else
    {
        std::cerr << "*Viewer.readlocalobject parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.show_object3d_frame"];
    if(!node.empty())
    {
        show_object3d_frame = (int)node.real();
    }
    else
    {
        std::cerr << "*Viewer.show_object3d_frame parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    if(run_pangolin)
    {
        pangolin::CreateWindowAndBind("Object Map Viewer",1024,768);
    }
    else
    {
        pangolin::CreateWindowAndBind("Object Map Viewer", 12, 9);
    }
    
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
    pangolin::Var<bool> menuTopView("menu.Top View",false,false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);

    // For 3D cuboid
    pangolin::Var<bool> menuShowQuadricObj("menu.Show QuadricObj", true, true);

    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuStop("menu.Stop",false,false);
    pangolin::Var<bool> menuSave("menu.Save", false, false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
    pangolin::Var<bool> menuStep("menu.Step",false,false);

    pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();

//    cv::namedWindow("Point, Line and Object Detection");
//    cv::moveWindow("Point, Line and Object Detection", 40, 40);
//    cv::namedWindow("Quadric Projection");
//    cv::moveWindow("Quadric Projection", 40, 40+480*0.8);

//    cv::namedWindow("[MotionIou]");
//    cv::moveWindow("[MotionIou]", 40, 40+480*0.7+40+480*0.7);
//    cv::namedWindow("[ProIou]");
//    cv::moveWindow("[ProIou]", 40+640*0.55, 40+480*0.7+40+480*0.7);

    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
    cv::namedWindow("ORB-SLAM3: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bStepByStep = false;
    bool bCameraView = true;

    if(mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->RGBD)
    {
        menuShowGraph = true;
    }

    float trackedImageScale = mpTracker->GetImageScale();

    // For 3D cuboid
    if (read_local_object)
    {
        read_local_object_file();
    }

    cout << "Starting the Viewer" << endl;

    while(1)
    {
        if (run_rviz)
        {
            mpMapPublisher->Refresh();
        }

        if (read_local_object)
        {
            mpMapPublisher->PublishObject(vObjects);
            mpMapPublisher->PublishIE(vObjects);
        }

        if (run_pangolin)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow);

            if(mbStopTrack)
            {
                menuStepByStep = true;
                mbStopTrack = false;
            }   

            if(menuFollowCamera && bFollow)
            {   
                if(bCameraView)
                    s_cam.Follow(Twc);
                else
                    s_cam.Follow(Ow);
            }
            else if(menuFollowCamera && !bFollow)
            {
                if(bCameraView)
                {
                    s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
                    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                    s_cam.Follow(Twc);
                }
                else
                {
                    s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
                    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));
                    s_cam.Follow(Ow);
                }
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if(menuCamView)
            {
                menuCamView = false;
                bCameraView = true;
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
            }

            if(menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
            {
                menuTopView = false;
                bCameraView = false;
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,50, 0,0,0,0.0,0.0, 1.0));
                s_cam.Follow(Ow);
            }

            if(menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if(!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            if(menuStepByStep && !bStepByStep)
            {
                //cout << "Viewer: step by step" << endl;
                mpTracker->SetStepByStep(true);
                bStepByStep = true;
            }
            else if(!menuStepByStep && bStepByStep)
            {
                mpTracker->SetStepByStep(false);
                bStepByStep = false;
            }

            if(menuStep)
            {
                mpTracker->mbStep = true;
                menuStep = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);

            mpMapDrawer->DrawGrid();
            mpMapDrawer->DrawCurrentCamera(Twc);

            if(menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph, menuShowInertialGraph, menuShowOptLba);

            if(menuShowPoints)
            {
                mpMapDrawer->DrawMapPoints();
                if (mpSystem->isg2oObjectOptimize)
                {
                    mpMapDrawer->DrawMapCuboids2();
                }
                else{
                    mpMapDrawer->DrawMapCuboids();
                }
            }
            else
            {
                // TODO: Uncomment for OctoMap viewer
                mpMapDrawer->DrawOctoMap();
                mpFrameDrawer->generatePC();
                // mpMapDrawer->DrawObs();
                mpMapDrawer->DrawObject();
            }

            // Draw world frame
            pangolin::glDrawAxis(10.0);

            pangolin::FinishFrame();
        }

        cv::Mat toShow;
        cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

        if(both){
            cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
            cv::hconcat(im,imRight,toShow);
        }
        else{
            toShow = im;
        }

        // For 3D cuboid
        if(show_object3d_frame) {
//            std::cout << "Visual Quadric Projection \n";
            cv::Mat QuadricImage = mpFrameDrawer->GetQuadricImage();
            if (!QuadricImage.empty()) {
                cv::Mat resizeimg;
                cv::resize(QuadricImage, resizeimg, cv::Size(640 * 0.7, 480 * 0.7), 0, 0, cv::INTER_CUBIC);
                cv::imshow("Quadratic Projection", resizeimg);
            }
            else
            {
                std::cout << "QuadraticImage is empty \n";
            }
        }

//        if(mImageViewerScale != 1.f)
//        {
//            int width = toShow.cols * mImageViewerScale;
//            int height = toShow.rows * mImageViewerScale;
//            cv::resize(toShow, toShow, cv::Size(width, height));
//        }

        {
            std::unique_lock<std::mutex> lock(mMutexPAFinsh);
            for (auto vit = mmDetectMap.begin(); vit != mmDetectMap.end(); vit++)
            {
                if (vit->second.size() != 0)
                {
                    for (auto area : vit->second)
                    {
                        cv::rectangle(toShow, area, cv::Scalar(0, 0, 255), 1);
                        cv::putText(toShow,
                                    vit->first,
                                    cv::Point(area.x, area.y),
                                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                    }
                }

            }
        }

        cv::imshow("JunBotView: Current Frame",toShow);
        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->ResetActiveMap();
            menuReset = false;
        }

        if(menuStop)
        {
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();

            // Stop all threads
            mpSystem->Shutdown();

            // Save camera trajectory
            mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
            mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
            menuStop = false;
        }

        if(menuSave)
        {
//            mpSystem->SaveMap("map.bin");
            mpMapDrawer->SaveOctoMap("octomap.ot");
            menuSave = false;
            cout << "save done!" << endl;

        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::Finalize(void)
{
    pangolin::BindToContext("Object Map Viewer");
}

void Viewer::read_local_object_file(){
    std::string filePath = WORK_SPACE_PATH + "/eval/Objects_with_points_for_read.txt";
    ifstream infile(filePath, ios::in);
    if (!infile.is_open())
    {
        cout << "open fail: "<< filePath <<" " << endl;
        exit(233);
    }
    else
    {
        std::cout << "read Objects_with_points.txt" << std::endl;
    }

    vector<double> row;

    cv::Mat cam_pose_mat;
    int mnid_current = -1;
    //string s0;
    //getline(infile, s0); 
    vObjects.clear();
    string line;
    int object_num = -1;
    int type = 1;
    while (getline(infile, line))
    {   //std::cout<<line<<std::endl;
        istringstream istr(line);
        istr >> type;

        if( type == 1){
            Object_Map *obj = new Object_Map();
            object_num ++;

            double temp;
            istr >> temp;    obj->mnId = temp;
            istr >> temp;    obj->mnClass = temp;
            istr >> temp;    obj->mnConfidence_foractive = temp;
            istr >> temp ; 

            Eigen::MatrixXd object_poses(1, 8); ;
            istr >> temp;  object_poses(0) = temp;  //obj->mCuboid3D.cuboidCenter0 = temp;
            istr >> temp;  object_poses(1) = temp;  //obj->mCuboid3D.cuboidCenter1 = temp;
            istr >> temp;  object_poses(2) = temp;  //obj->mCuboid3D.cuboidCenter2 = temp;
            istr >> temp;  object_poses(3) = temp;
            istr >> temp;  object_poses(4) = temp;
            istr >> temp;  object_poses(5) = temp;
            istr >> temp;  object_poses(6) = temp;
            g2o::SE3Quat cam_pose_se3(object_poses.row(0).head(7));

            obj->mCuboid3D.pose_mat = Converter::toCvMat(cam_pose_se3);
            istr >> temp;   obj->mCuboid3D.lenth = temp;
            istr >> temp;   obj->mCuboid3D.width = temp;
            istr >> temp;   obj->mCuboid3D.height = temp;

            compute_corner(obj);

            vObjects.push_back( obj );

            std::cout<<  "mnId: "<<vObjects[ object_num ]->mnId
                    <<  ", Class: " << vObjects[ object_num ]->mnClass <<std::endl;

        }
        else if( type == 0)
        {
            // std::cout << "Objects number: " << object_num << std::endl;
            double temp;
            istr >> temp;
            istr >> temp;

            MapPoint* point = new MapPoint();
            float x_p, y_p, z_p;
            istr >> temp;  x_p = temp;
            istr >> temp;  y_p = temp;
            istr >> temp;  z_p = temp;
            std::vector<float> vec{x_p, y_p, z_p};
            cv::Mat WorldPos(vec);

            point->SetWorldPos(Converter::toVector3f(WorldPos));
            vObjects[ object_num ]-> mvpMapObjectMappoints.push_back( point );
            //mpMapPub -> mpMap->mvObjectMap[ object_num ]->mvpMapObjectMappoints.push_back( &point );
        }

        row.clear();
        type = -1;
        istr.clear();
        line.clear();
    }

    for(int i=0; i<vObjects.size(); i++){
        vObjects[i]->ComputeIE();
    }
}

void Viewer::compute_corner(Object_Map* object) {

        float x_min_obj = (-0.5)*object->mCuboid3D.lenth;
        float x_max_obj = (0.5)*object->mCuboid3D.lenth;
        float y_min_obj = (-0.5)*object->mCuboid3D.width;
        float y_max_obj = (0.5)*object->mCuboid3D.width;
        float z_min_obj = (-0.5)*object->mCuboid3D.height;
        float z_max_obj = (0.5)*object->mCuboid3D.height;

        g2o::SE3Quat pose =  Converter::toSE3Quat( object->mCuboid3D.pose_mat);
        object->mCuboid3D.corner_1 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj) ;
        object->mCuboid3D.corner_2 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj) ;
        object->mCuboid3D.corner_3 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj) ;
        object->mCuboid3D.corner_4 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj) ;
        object->mCuboid3D.corner_5 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj) ;
        object->mCuboid3D.corner_6 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj) ;
        object->mCuboid3D.corner_7 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj) ;
        object->mCuboid3D.corner_8 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj) ;
}

/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
