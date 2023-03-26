#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core.hpp>

#include "System.h"

using namespace std;

void LoadImages(const string &strAssociationFilenameDepth, const string &strAssociationFilenameRGB, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "simple_rgbd_node");
    ros::start();

    if(argc != 6)
    {
        cerr << endl << "Usage: rosrun object_slam simple_rgbd_node path_to_vocabulary path_to_settings path_to_sequence path_to_association_depth path_to_association_rgb" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilenameDepth = string(argv[4]);
    string strAssociationFilenameRGB = string(argv[5]);
    LoadImages(strAssociationFilenameDepth, strAssociationFilenameRGB,vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    semantic_slam::System SLAM(argv[1], argv[2], semantic_slam::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();


        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt"); 

    ros::shutdown();

    return 0;
}

void LoadImages(const string &strAssociationFilenameDepth, const string &strAssociationFilenameRGB, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociationD;
    ifstream fAssociationRGB;
    fAssociationD.open(strAssociationFilenameDepth.c_str());
    fAssociationRGB.open(strAssociationFilenameRGB.c_str());
    while(!fAssociationD.eof())
    {
        string s;
        getline(fAssociationD,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;

            ss >> t;
            vTimestamps.push_back(t);
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

            string _s;
            stringstream _ss;
            getline(fAssociationRGB, _s);
            _ss << _s;
            _ss >> t;
            _ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
        }
    }
}
