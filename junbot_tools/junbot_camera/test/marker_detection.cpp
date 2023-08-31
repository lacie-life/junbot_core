#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>

using namespace cv;
using namespace std;

enum class CameraSide { LEFT, RIGHT };

//
// Declare all the calibration matrices as Mat variables.
//
Mat lmapx, lmapy, rmapx, rmapy;

image_transport::Publisher pub_img_rect_left, pub_img_rect_right;

ros::Publisher t265_state_pub;

sensor_msgs::CameraInfo output_camera_info_left, output_camera_info_right;

ros::Publisher left_camera_info_output_pub, right_camera_info_output_pub;

int dictitonaryId = 5;
float marker_length_m = 0.2;

Mat Q, P1, P2;
Mat R1, R2, K1, K2, D1, D2, R;
Vec3d T;
Vec2d size_input, size_output;

ros::Publisher  cancel;

void marker_detection(Mat& dst, const CameraSide& side)
{
  cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictitonaryId));
  
  std::ostringstream vector_to_marker;

  if(side == CameraSide::LEFT)
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(dst, dictionary, corners, ids);

    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(dst, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                K1, D1, rvecs, tvecs);

        // Draw axis for each marker
        for(int i=0; i < ids.size(); i++)
        {
            std_msgs::String msg;
            std::stringstream tmp;
            tmp << "{";
            cv::aruco::drawAxis(dst, K1, D1,
              rvecs[i], tvecs[i], 0.1);

            // This section is going to print the data for all the detected
            // markers. If you have more than a single marker, it is
            // recommended to change the below section so that either you
            // only print the data for a specific marker, or you print the
            // data for each marker separately.
            tmp << "\"id\":" << std::to_string(ids.at(i)) << ",";
            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) << tvecs[0](0);
            cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, cv::LINE_AA);
            tmp << "\"x\":" << vector_to_marker.str() << ",";

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) << tvecs[0](1);
            cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, cv::LINE_AA);
            tmp << "\"y\":" << vector_to_marker.str() << ",";

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)  << tvecs[0](2);
            cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, cv::LINE_AA);
            tmp << "\"z\":" << vector_to_marker.str();
            int center[2];
            for (int i_corners = 0; i<corners.size(); i++){
                center[0] = center[0] + corners.at(i).at(i_corners).x;
                center[1] = center[1] + corners.at(i).at(i_corners).y;
            }
            center[0] = center[0]/4;
            center[1] = center[1]/4;
            tmp << "\"centerX\"" << std::to_string(center[0]) << "\"centerY\"" << std::to_string(center[1]);
            tmp << "}";
            msg.data = tmp.str();
            cancel.publish(msg);
            std::cout << tmp.str() << std::endl;
        }
    }
  }
  else 
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(dst, dictionary, corners, ids);

    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(dst, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                K2, D2, rvecs, tvecs);

        // Draw axis for each marker
        for(int i=0; i < ids.size(); i++)
        {
          cv::aruco::drawAxis(dst, K2, D2,
              rvecs[i], tvecs[i], 0.1);

          // This section is going to print the data for all the detected
          // markers. If you have more than a single marker, it is
          // recommended to change the below section so that either you
          // only print the data for a specific marker, or you print the
          // data for each marker separately.
          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "x: " << std::setw(8) << tvecs[0](0);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, cv::LINE_AA);

          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "y: " << std::setw(8) << tvecs[0](1);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, cv::LINE_AA);

          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "z: " << std::setw(8) << tvecs[0](2);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, cv::LINE_AA);
        }
    }
  }
} 

//
// This function computes all the projection matrices and the rectification transformations 
// using the stereoRectify and initUndistortRectifyMap functions respectively.
// See documentation for stereoRectify: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
//
void init_rectification_map(string param_file_path) 
{

  FileStorage param_file = FileStorage(param_file_path, FileStorage::READ);

  param_file["K1"] >> K1;
  param_file["D1"] >> D1;
  param_file["K2"] >> K2;
  param_file["D2"] >> D2;
  param_file["R"]  >> R;
  param_file["T"]  >> T;
  param_file["input"]  >> size_input;
  param_file["output"] >> size_output;

  // The resolution of the input images used for stereo calibration.
  Size input_img_size(size_input[0], size_input[1]);

  // The resolution of the output rectified images. Lower resolution images require less computation time.
  Size output_img_size(size_output[0], size_output[1]);
  double alpha = 0.0;

  stereoRectify(K1, D1, K2, D2, 
                input_img_size, 
                R, T, 
                R1, R2, P1, P2, 
                Q,
                cv::CALIB_ZERO_DISPARITY,
                alpha, 
                output_img_size);
 
  fisheye::initUndistortRectifyMap(K1, D1, R1, P1, output_img_size, CV_32FC1, lmapx, lmapy);
  fisheye::initUndistortRectifyMap(K2, D2, R2, P2, output_img_size, CV_32FC1, rmapx, rmapy);

  // Copy the parameters for rectified images to the camera_info messages
  output_camera_info_left.width = size_output[0];
  output_camera_info_left.height = size_output[1];
  output_camera_info_left.D = vector<double>(5, 0);

  output_camera_info_right.width = size_output[0];
  output_camera_info_right.height = size_output[1];
  output_camera_info_right.D = vector<double>(5, 0);

  for (int i = 0; i < 9; i++)
  {
    output_camera_info_left.K[i] = K1.at<double>(i);
    output_camera_info_right.K[i] = K2.at<double>(i);
    output_camera_info_left.R[i] = R1.at<double>(i);
    output_camera_info_right.R[i] = R2.at<double>(i);
  }  
  for (int i = 0; i < 12; i++)
  {
    output_camera_info_left.P[i] = P1.at<double>(i);
    output_camera_info_right.P[i] = P2.at<double>(i);
  }
  
  ROS_INFO("Initialization complete. Publishing rectified images and camera_info when raw images arrive...");
}

//
// This function undistorts and rectifies the src image into dst. 
// The homographic mappings lmapx, lmapy, rmapx, and rmapy are found from OpenCV’s initUndistortRectifyMap function.
//
void undistort_rectify_image(Mat& src, Mat& dst, const CameraSide& side)
{
  if (side == CameraSide::RIGHT) 
  {
  //  const auto undis_fish_right = "Undisort Right FishCamera";
    remap(src, dst, lmapx, lmapy, cv::INTER_LINEAR);
    marker_detection(dst, CameraSide::RIGHT);
  //  imshow(undis_fish_right, dst);
  } 
  else 
  {
  //  const auto undis_fish_left = "Undisort Left FishCamera";
    remap(src, dst, rmapx, rmapy, cv::INTER_LINEAR);
    marker_detection(dst, CameraSide::LEFT);
  //  imshow(undis_fish_left, dst);
  }
}

//
// This callback function takes a pair of raw stereo images as inputs, 
// then undistorts and rectifies the images using the undistort_rectify_image function 
// defined above and publishes on the rectified image topic using pub_img_left/right.
//
void synched_img_callback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
    Mat tmp_left = cv_bridge::toCvShare(msg_left, "bgr8")->image;
    Mat tmp_right = cv_bridge::toCvShare(msg_right, "bgr8")->image;

    // std::cout << tmp_left.size() << "\n";

    Mat dst_left, dst_right;

    undistort_rectify_image(tmp_left, dst_left, CameraSide::LEFT);
    undistort_rectify_image(tmp_right, dst_right, CameraSide::RIGHT);

    sensor_msgs::ImagePtr rect_img_left = cv_bridge::CvImage(msg_left->header, "bgr8", dst_left).toImageMsg();
    sensor_msgs::ImagePtr rect_img_right = cv_bridge::CvImage(msg_right->header, "bgr8", dst_right).toImageMsg();

    pub_img_rect_left.publish(rect_img_left);
    pub_img_rect_right.publish(rect_img_right);

    std_msgs::Header header = msg_left->header;
    output_camera_info_left.header = header;
    output_camera_info_right.header = header;

    left_camera_info_output_pub.publish(output_camera_info_left);
    right_camera_info_output_pub.publish(output_camera_info_right);

    std_msgs::String msg;
    std::stringstream tmp;

    if(!tmp_left.empty() && !tmp_right.empty())
    {
      tmp << "{\"t265_state \": " << 1 << "}";
      msg.data = tmp.str();
      t265_state_pub.publish(msg);
    }
    else
    {
      tmp << "{\"t265_state \": " << 0 << "}";
      msg.data = tmp.str();
      t265_state_pub.publish(msg);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_fisheye_undistort");

  ros::NodeHandle nh("~");

  std::string param_file_path;

  if(nh.getParam("t265_param_file_path", param_file_path))
  {
    ROS_WARN("Using parameter file: %s", param_file_path.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param file path. Please check and try again.");
    ros::shutdown();
    return 0;
  }

  cancel = nh.advertise<std_msgs::String>("/detectionAruco", 1000);

    // Read the input parameters and perform initialization
  init_rectification_map(param_file_path);

  // The raw stereo images should be published as type sensor_msgs/Image
  image_transport::ImageTransport it(nh);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, "/t265/fisheye1/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, "/t265/fisheye2/image_raw", 1);
  
  // Having time synced stereo images might be important for other purposes, say generating accurate disparity maps. 
  // To sync the left and right image messages by their header time stamps, ApproximateTime is used.
  // More info here: http://wiki.ros.org/message_filters#Time_Synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&synched_img_callback, _1, _2));

  // The output data include rectified images and their corresponding camera info
  pub_img_rect_left  = it.advertise("/t265/fisheye1/rect/image",  1);
  pub_img_rect_right = it.advertise("/t265/fisheye2/rect/image", 1);

  left_camera_info_output_pub = nh.advertise<sensor_msgs::CameraInfo>("/t265/fisheye1/rect/camera_info", 1);
  right_camera_info_output_pub = nh.advertise<sensor_msgs::CameraInfo>("/t265/fisheye2/rect/camera_info", 1);

  t265_state_pub = nh.advertise<std_msgs::String>("/t265_state", 1000);

  // Processing start
  ros::spin();
}