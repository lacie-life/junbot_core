//
// Created by lacie on 12/02/2023.
//

#include "MapPublisher.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Atlas.h"
#include "Converter.h"

#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>

namespace semantic_slam
{

    MapPublisher::MapPublisher(Atlas *mAtlas, const string &strSettingPath)
    :  mpAtlas(mAtlas), mbCameraUpdated(false)
    {
        //Configure MapPoints
        fPointSize=0.01;
        mPoints.header.frame_id = MAP_FRAME_ID;
        mPoints.ns = POINTS_NAMESPACE;
        mPoints.id=0;
        mPoints.type = visualization_msgs::Marker::POINTS;
        mPoints.scale.x=fPointSize;
        mPoints.scale.y=fPointSize;
        mPoints.pose.orientation.w=1.0;
        mPoints.action=visualization_msgs::Marker::ADD;
        mPoints.color.a = 1.0;

        //Configure KeyFrames
        fCameraSize=0.04;
        mKeyFrames.header.frame_id = MAP_FRAME_ID;
        mKeyFrames.ns = KEYFRAMES_NAMESPACE;
        mKeyFrames.id=1;
        mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
        mKeyFrames.scale.x=0.005;
        mKeyFrames.pose.orientation.w=1.0;
        mKeyFrames.action=visualization_msgs::Marker::ADD;

        mKeyFrames.color.b=1.0f;
        mKeyFrames.color.a = 1.0;

        //Configure Covisibility Graph
        mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
        mCovisibilityGraph.ns = GRAPH_NAMESPACE;
        mCovisibilityGraph.id=2;
        mCovisibilityGraph.type = visualization_msgs::Marker::LINE_LIST;
        mCovisibilityGraph.scale.x=0.002;
        mCovisibilityGraph.pose.orientation.w=1.0;
        mCovisibilityGraph.action=visualization_msgs::Marker::ADD;
        mCovisibilityGraph.color.b=0.7f;
        mCovisibilityGraph.color.g=0.7f;
        mCovisibilityGraph.color.a = 0.3;

        //Configure KeyFrames Spanning Tree
        mMST.header.frame_id = MAP_FRAME_ID;
        mMST.ns = GRAPH_NAMESPACE;
        mMST.id=3;
        mMST.type = visualization_msgs::Marker::LINE_LIST;
        mMST.scale.x=0.005;
        mMST.pose.orientation.w=1.0;
        mMST.action=visualization_msgs::Marker::ADD;
        mMST.color.b=0.0f;
        mMST.color.g=1.0f;
        mMST.color.a = 1.0;

        //Configure Current Camera
        mCurrentCamera.header.frame_id = MAP_FRAME_ID;
        mCurrentCamera.ns = CAMERA_NAMESPACE;
        mCurrentCamera.id=4;
        mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
        mCurrentCamera.scale.x=0.01;//0.2; 0.03
        mCurrentCamera.pose.orientation.w=1.0;
        mCurrentCamera.action=visualization_msgs::Marker::ADD;
        mCurrentCamera.color.g=1.0f;
        mCurrentCamera.color.a = 1.0;

        //Configure Reference MapPoints
        mReferencePoints.header.frame_id = MAP_FRAME_ID;
        mReferencePoints.ns = POINTS_NAMESPACE;
        mReferencePoints.id=6;
        mReferencePoints.type = visualization_msgs::Marker::POINTS;
        mReferencePoints.scale.x=fPointSize;
        mReferencePoints.scale.y=fPointSize;
        mReferencePoints.pose.orientation.w=1.0;
        mReferencePoints.action=visualization_msgs::Marker::ADD;
        mReferencePoints.color.r =1.0f;
        mReferencePoints.color.a = 1.0;

        //Configure IE grid
        IE_id = 7;

        //Configure MapObjectPoints

        //Configure MapObjects
        object_id_init = 46;

        //Configure Publisher
        publisher = nh.advertise<visualization_msgs::Marker>("Point", 1000);
        publisher_curframe = nh.advertise<visualization_msgs::Marker>("CurFrame", 1000);
        publisher_KF = nh.advertise<visualization_msgs::Marker>("KeyFrame", 1000);
        publisher_CoView = nh.advertise<visualization_msgs::Marker>("CoView", 1000);
        publisher_object = nh.advertise<visualization_msgs::Marker>("objectmap", 1000);
        publisher_object2map = nh.advertise<vision_msgs::Detection3DArray>("detect_array", 1000);
        publisher_object_points = nh.advertise<visualization_msgs::Marker>("objectPoints", 1000);
        publisher_IE = nh.advertise<visualization_msgs::Marker>("object_ie", 1000);
        publisher_robotpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float qx = fSettings["Trobot_camera.qx"];
        float qy = fSettings["Trobot_camera.qy"];
        float qz = fSettings["Trobot_camera.qz"];
        float qw = fSettings["Trobot_camera.qw"];
        float tx = fSettings["Trobot_camera.tx"];
        float ty = fSettings["Trobot_camera.ty"];
        float tz = fSettings["Trobot_camera.tz"];

        mT_body_cam = Converter::Quation2CvMat(qx, qy, qz, qw, tx, ty, tz );
        mQuaternion_robot_camera.x = qx;
        mQuaternion_robot_camera.y = qy;
        mQuaternion_robot_camera.z = qz;
        mQuaternion_robot_camera.w = qw;
        mTranslation_robot_camera.x = tx;
        mTranslation_robot_camera.y = ty;
        mTranslation_robot_camera.z = tz;
    }

    void MapPublisher::Refresh()
    {
        {
            PublishCurrentCamera(mCameraPose);
        }
        {
            Map* mpMap = mpAtlas->GetCurrentMap();

            vector<KeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
            vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
            vector<MapPoint*> vRefMapPoints = mpMap->GetReferenceMapPoints();
            vector<Object_Map*> vMapObjects = mpMap->GetObjects();

            PublishMapPoints(vMapPoints, vRefMapPoints);
            PublishKeyFrames(vKeyFrames);
            PublishObject(vMapObjects);
            PublishObject2Map(vMapObjects);
//            PublishIE(vMapObjects);
        }
    }

    void MapPublisher::PublishMapPoints(const vector<MapPoint*> &vpMPs, const vector<MapPoint*> &vpRefMPs)
    {
        mPoints.points.clear();
        mReferencePoints.points.clear();

        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            geometry_msgs::Point p;
            cv::Mat pos = Converter::toCvMat(vpMPs[i]->GetWorldPos());
            p.x=pos.at<float>(0);
            p.y=pos.at<float>(1);
            p.z=pos.at<float>(2);

            mPoints.points.push_back(p);
        }

        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
                continue;
            geometry_msgs::Point p;
            cv::Mat pos = Converter::toCvMat((*sit)->GetWorldPos());
            p.x=pos.at<float>(0);
            p.y=pos.at<float>(1);
            p.z=pos.at<float>(2);

            mReferencePoints.points.push_back(p);
        }

        mPoints.header.stamp = ros::Time::now();
        mReferencePoints.header.stamp = ros::Time::now();
        publisher.publish(mPoints);
        publisher.publish(mReferencePoints);
    }

    void MapPublisher::PublishKeyFrames(const vector<KeyFrame*> &vpKFs)
    {
        mKeyFrames.points.clear();
        mCovisibilityGraph.points.clear();
        mMST.points.clear();

        float d = fCameraSize;

        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

        for(size_t i=0, iend=vpKFs.size() ;i<iend; i++)
        {
            cv::Mat Tcw = Converter::toCvMat(vpKFs[i]->GetPose());
            // std::cout << "[rviz debug] frame id： " << vpKFs[i]->mnId << ", Tcw:"
            // << Tcw << std::endl;
            cv::Mat Twc = Tcw.inv();
            cv::Mat ow = Converter::toCvMat(vpKFs[i]->GetCameraCenter());
            cv::Mat p1w = Twc*p1;
            cv::Mat p2w = Twc*p2;
            cv::Mat p3w = Twc*p3;
            cv::Mat p4w = Twc*p4;

            geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
            msgs_o.x=ow.at<float>(0);
            msgs_o.y=ow.at<float>(1);
            msgs_o.z=ow.at<float>(2);
            msgs_p1.x=p1w.at<float>(0);
            msgs_p1.y=p1w.at<float>(1);
            msgs_p1.z=p1w.at<float>(2);
            msgs_p2.x=p2w.at<float>(0);
            msgs_p2.y=p2w.at<float>(1);
            msgs_p2.z=p2w.at<float>(2);
            msgs_p3.x=p3w.at<float>(0);
            msgs_p3.y=p3w.at<float>(1);
            msgs_p3.z=p3w.at<float>(2);
            msgs_p4.x=p4w.at<float>(0);
            msgs_p4.y=p4w.at<float>(1);
            msgs_p4.z=p4w.at<float>(2);

            mKeyFrames.points.push_back(msgs_o);
            mKeyFrames.points.push_back(msgs_p1);
            mKeyFrames.points.push_back(msgs_o);
            mKeyFrames.points.push_back(msgs_p2);
            mKeyFrames.points.push_back(msgs_o);
            mKeyFrames.points.push_back(msgs_p3);
            mKeyFrames.points.push_back(msgs_o);
            mKeyFrames.points.push_back(msgs_p4);
            mKeyFrames.points.push_back(msgs_p1);
            mKeyFrames.points.push_back(msgs_p2);
            mKeyFrames.points.push_back(msgs_p2);
            mKeyFrames.points.push_back(msgs_p3);
            mKeyFrames.points.push_back(msgs_p3);
            mKeyFrames.points.push_back(msgs_p4);
            mKeyFrames.points.push_back(msgs_p4);
            mKeyFrames.points.push_back(msgs_p1);

            // Covisibility Graph
            vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = Converter::toCvMat((*vit)->GetCameraCenter());
                    geometry_msgs::Point msgs_o2;
                    msgs_o2.x=Ow2.at<float>(0);
                    msgs_o2.y=Ow2.at<float>(1);
                    msgs_o2.z=Ow2.at<float>(2);
                    mCovisibilityGraph.points.push_back(msgs_o);
                    mCovisibilityGraph.points.push_back(msgs_o2);
                }
            }

            // MST
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = Converter::toCvMat(pParent->GetCameraCenter());
                geometry_msgs::Point msgs_op;
                msgs_op.x=Owp.at<float>(0);
                msgs_op.y=Owp.at<float>(1);
                msgs_op.z=Owp.at<float>(2);
                mMST.points.push_back(msgs_o);
                mMST.points.push_back(msgs_op);
            }
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = Converter::toCvMat((*sit)->GetCameraCenter());
                geometry_msgs::Point msgs_ol;
                msgs_ol.x=Owl.at<float>(0);
                msgs_ol.y=Owl.at<float>(1);
                msgs_ol.z=Owl.at<float>(2);
                mMST.points.push_back(msgs_o);
                mMST.points.push_back(msgs_ol);
            }
        }

        mKeyFrames.header.stamp = ros::Time::now();
        mCovisibilityGraph.header.stamp = ros::Time::now();
        mMST.header.stamp = ros::Time::now();

        publisher_KF.publish(mKeyFrames);
        publisher_CoView.publish(mCovisibilityGraph);
        publisher_KF.publish(mMST);
    }
    void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw) {
        if (Tcw.empty())
            return;

        mCurrentCamera.points.clear();

        float d = fCameraSize;

        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

        cv::Mat Twc = Tcw.inv();
        cv::Mat ow = Twc * o;
        cv::Mat p1w = Twc * p1;
        cv::Mat p2w = Twc * p2;
        cv::Mat p3w = Twc * p3;
        cv::Mat p4w = Twc * p4;

        geometry_msgs::Point msgs_o, msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x = ow.at<float>(0);
        msgs_o.y = ow.at<float>(1);
        msgs_o.z = ow.at<float>(2);
        msgs_p1.x = p1w.at<float>(0);
        msgs_p1.y = p1w.at<float>(1);
        msgs_p1.z = p1w.at<float>(2);
        msgs_p2.x = p2w.at<float>(0);
        msgs_p2.y = p2w.at<float>(1);
        msgs_p2.z = p2w.at<float>(2);
        msgs_p3.x = p3w.at<float>(0);
        msgs_p3.y = p3w.at<float>(1);
        msgs_p3.z = p3w.at<float>(2);
        msgs_p4.x = p4w.at<float>(0);
        msgs_p4.y = p4w.at<float>(1);
        msgs_p4.z = p4w.at<float>(2);

        mCurrentCamera.points.push_back(msgs_o);
        mCurrentCamera.points.push_back(msgs_p1);
        mCurrentCamera.points.push_back(msgs_o);
        mCurrentCamera.points.push_back(msgs_p2);
        mCurrentCamera.points.push_back(msgs_o);
        mCurrentCamera.points.push_back(msgs_p3);
        mCurrentCamera.points.push_back(msgs_o);
        mCurrentCamera.points.push_back(msgs_p4);
        mCurrentCamera.points.push_back(msgs_p1);
        mCurrentCamera.points.push_back(msgs_p2);
        mCurrentCamera.points.push_back(msgs_p2);
        mCurrentCamera.points.push_back(msgs_p3);
        mCurrentCamera.points.push_back(msgs_p3);
        mCurrentCamera.points.push_back(msgs_p4);
        mCurrentCamera.points.push_back(msgs_p4);
        mCurrentCamera.points.push_back(msgs_p1);

        mCurrentCamera.header.stamp = ros::Time::now();

        publisher_curframe.publish(mCurrentCamera);

        cv::Mat T_w_cam = Tcw.inv();
        cv::Mat T_w_body = cv::Mat::eye(4, 4, CV_32F);
        T_w_body = T_w_cam * mT_body_cam.inv();
        geometry_msgs::PoseWithCovarianceStamped robotpose;
        robotpose.pose.pose.position.x = T_w_body.at<float>(0, 3);
        robotpose.pose.pose.position.y = T_w_body.at<float>(1, 3);
        robotpose.pose.pose.position.z = 0.0;
        //（1）The quaternion in gazebo is different from that of rviz, and needs to be rotated 90 degrees around the z axis
        //Eigen::AngleAxisd rotation_vector (-M_PI/2.0, Eigen::Vector3d(0,0,1));
        //Eigen::Quaterniond q_y_x = Eigen::Quaterniond(rotation_vector);
        //Eigen::Quaterniond q_w_body = Converter::toQuaterniond(T_w_body);
        //Eigen::Quaterniond q = q_y_x * q_w_body;
        //（2）Do not turn the z-axis 90 degrees
        Eigen::Quaterniond q_w_body = Converter::ExtractQuaterniond(T_w_body);
        Eigen::Quaterniond q = q_w_body;

        robotpose.pose.pose.orientation.w = q.w();
        robotpose.pose.pose.orientation.x = q.x();
        robotpose.pose.pose.orientation.y = q.y();
        robotpose.pose.pose.orientation.z = q.z();
        robotpose.header.frame_id= "odom";
        robotpose.header.stamp=ros::Time::now();
        publisher_robotpose.publish(robotpose);

        // Publish tf tree
        // Publish the tf transformation of the robot chassis and odom
        geometry_msgs::TransformStamped odom_trans;
        ros::Time current_time = ros::Time::now();
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = robotpose.pose.pose.position.x;
        odom_trans.transform.translation.y = robotpose.pose.pose.position.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = robotpose.pose.pose.orientation; //Quaternion_odom_robot;
        odom_broadcaster.sendTransform(odom_trans);

        // 5.5 Publish the tf transformation of camera_depth_optical_frame and base_link
        geometry_msgs::TransformStamped camera_trans;
        camera_trans.header.stamp = current_time;
        camera_trans.header.frame_id = "base_link";
        camera_trans.child_frame_id = "camera_depth_optical_frame";
        camera_trans.transform.translation.x = mTranslation_robot_camera.x;
        camera_trans.transform.translation.y = mTranslation_robot_camera.y;
        camera_trans.transform.translation.z = mTranslation_robot_camera.z;
        camera_trans.transform.rotation = mQuaternion_robot_camera;
        camera_broadcaster.sendTransform(camera_trans);
    }

    geometry_msgs::Point MapPublisher::corner_to_marker(Eigen::Vector3d& v){
        geometry_msgs::Point point;
        point.x = v[0];
        point.y = v[1];
        point.z = v[2];
        return point;
    }

    geometry_msgs::Point MapPublisher::corner_to_marker(const std::vector<float>& v){
        geometry_msgs::Point point;
        point.x = v[0];
        point.y = v[1];
        point.z = v[2];
        return point;
    }

    geometry_msgs::Point MapPublisher::corner_to_marker(const std::vector<double>& v){
        geometry_msgs::Point point;
        point.x = v[0];
        point.y = v[1];
        point.z = v[2];
        return point;
    }

    void MapPublisher::PublishObject(const vector<Object_Map*> &vObjs ){

        //for(size_t i=vObjs.size(); i<object_num_last ; i++)
        //{
        //    visualization_msgs::Marker marker;
        //    marker.id = object_id_init + i;
        //    marker.header.frame_id= MAP_FRAME_ID;
        //    marker.header.stamp=ros::Time::now();
        //    marker.action = visualization_msgs::Marker::DELETE;
        //    publisher.publish(marker);
        //}
        //object_num_last = vObjs.size();

        for(size_t i=0; i< vObjs.size(); i++)
        {

            // color.
            std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
            vector<float> color = colors_bgr[vObjs[i]->mnClass % 6];


            // The color used for the object is random
            std::default_random_engine e;
            std::uniform_real_distribution<double>  random(0.5,1);
            float r = random(e); float g = random(e); float b = random(e);

            // object
            visualization_msgs::Marker marker;
            marker.id = ++object_id_init;//object_id_init + i;
            marker.lifetime = ros::Duration(0.1);
            marker.header.frame_id= MAP_FRAME_ID;
            marker.header.stamp=ros::Time::now();

            if((vObjs[i]->mvpMapObjectMappoints.size() < 10) || (vObjs[i]->bad_3d == true))
            {
                continue;
            }
            marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 1.0;
            marker.scale.x = 0.01;

            //     8------7
            //    /|     /|
            //   / |    / |
            //  5------6  |
            //  |  4---|--3
            //  | /    | /
            //  1------2
            // lenth ：corner_2[0] - corner_1[0]
            // width ：corner_2[1] - corner_3[1]
            // height：corner_2[2] - corner_6[2]

            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_1));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_2));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_2));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_3));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_3));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_4));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_4));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_1));

            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_5));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_1));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_6));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_2));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_7));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_3));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_8));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_4));

            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_5));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_6));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_6));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_7));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_7));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_8));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_8));
            marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_5));

            publisher_object.publish(marker);

            // point in object
            visualization_msgs::Marker marker1;
            marker1.header.frame_id = MAP_FRAME_ID;
            marker1.ns = "ObjectPoints";
            marker1.lifetime = ros::Duration(0.2);
            marker1.id = ++object_id_init;
            marker1.type = visualization_msgs::Marker::POINTS;
            marker1.scale.x = 0.02;
            marker1.scale.y = 0.02;
            marker1.pose.orientation.w = 1.0;  //????
            marker1.action = visualization_msgs::Marker::ADD;
            marker1.color.r = color[2]/255.0;
            marker1.color.g = color[1]/255.0;
            marker1.color.b = color[0]/255.0;
            marker1.color.a = 0.5;
            vector<MapPoint* > vpMPs = vObjs[i]->GetObjectMappoints();
            for(size_t i = 0, iend = vpMPs.size();  i<iend;  i++)
            {
                //if(vpMPs[i]->isBad() )
                //    continue;
                geometry_msgs::Point p;
                cv::Mat pos = Converter::toCvMat(vpMPs[i]->GetWorldPos());
                //std::cout<<"[nbv debug:1 ]"<<pos <<std::endl;
                p.x=pos.at<float>(0);
                p.y=pos.at<float>(1);
                p.z=pos.at<float>(2);
                marker1.points.push_back(p);
            }
            publisher_object_points.publish(marker1);

            // new point in the object
            vector<float> color_new = colors_bgr[(vObjs[i]->mnClass+1) % 6];
            visualization_msgs::Marker marker2;
            marker2.header.frame_id = MAP_FRAME_ID;
            marker2.ns = "NewObjectPoints";
            marker2.lifetime = ros::Duration(0.2);
            marker2.id= ++object_id_init;
            marker2.type = visualization_msgs::Marker::POINTS;
            marker2.scale.x=0.02;
            marker2.scale.y=0.02;
            marker2.pose.orientation.w=1.0;  //????
            marker2.action=visualization_msgs::Marker::ADD;
            // marker2.color.r = color_new[2]/255.0;
            // marker2.color.g = color_new[1]/255.0;
            // marker2.color.b = color_new[0]/255.0;
            // marker2.color.a = 1.0;
            marker2.color.r = 0.0; marker2.color.g = 0.0; marker2.color.b = 1.0;
            marker2.color.a = 1.0;
            vector<MapPoint* > vpMPs_new = vObjs[i]->GetNewObjectMappoints();
            for(size_t i=0, iend=vpMPs_new.size();  i<iend;  i++)
            {
                //if(vpMPs[i]->isBad() )
                //    continue;
                geometry_msgs::Point p;
                cv::Mat pos = Converter::toCvMat(vpMPs_new[i]->GetWorldPos());
                //std::cout << "[nbv debug:1 ]" << pos << std::endl;
                p.x=pos.at<float>(0);
                p.y=pos.at<float>(1);
                p.z=pos.at<float>(2);
                marker2.points.push_back(p);
            }
            // std::cout << "[rviz debug:2 ]" << marker2.points.size() << std::endl;
            publisher_object_points.publish(marker2);
        }
    }

    void MapPublisher::PublishObject2Map(const std::vector<Object_Map *> &vpObjs)
    {
        // TODO: Convert to Map Coordinate
        if(vpObjs.size() <= 0)
        {
            return;
        }

        vision_msgs::Detection3DArray objDB;

        for(size_t i = 0; i < vpObjs.size(); i++)
        {
            if((vpObjs[i]->mvpMapObjectMappoints.size() < 10) || (vpObjs[i]->bad_3d == true))
            {
                continue;
            }

            vision_msgs::Detection3D obj;
            Object_Map* temp = vpObjs[i];

            obj.bbox.center.position.x = temp->mCuboid3D.cuboidCenter[0];   // 1 + 4/2
            obj.bbox.center.position.y = temp->mCuboid3D.cuboidCenter[1]; // 2 + 5/2
            obj.bbox.center.position.z = temp->mCuboid3D.cuboidCenter[2];   // 3 + 6/2
            obj.bbox.center.orientation.x = 0;
            obj.bbox.center.orientation.y = 0;
            obj.bbox.center.orientation.z = 0;
            obj.bbox.center.orientation.w = 1;
            obj.bbox.size.x = temp->mCuboid3D.x_max - temp->mCuboid3D.x_min;
            obj.bbox.size.y = temp->mCuboid3D.y_max - temp->mCuboid3D.y_min;
            obj.bbox.size.z = temp->mCuboid3D.z_max - temp->mCuboid3D.z_min;

            vision_msgs::ObjectHypothesisWithPose hypo;
            hypo.id = temp->mnClass;
            hypo.score = 1;

            obj.results.push_back(hypo);

            objDB.detections.push_back(obj);
        }
        objDB.header = std_msgs::Header();

        publisher_object2map.publish(objDB);
    }


    void MapPublisher::PublishIE(const vector<Object_Map*> &vObjs ){
        // color.
        std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
        vector<float> color;

        // rviz marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = MAP_FRAME_ID;
        marker.ns = "InformationEntroy";
        //marker.lifetime = ros::Duration(5.0);
        marker.lifetime = ros::Duration(0.2);
        //marker.id= IE_id ;  //TODO: absolute number
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.03;
        marker.scale.y = 0.08;
        marker.pose.orientation.w = 1.0;  //????
        marker.action=visualization_msgs::Marker::ADD;

        for(size_t i=0; i< vObjs.size(); i++){
            Object_Map* obj = vObjs[i];

            if((obj->mvpMapObjectMappoints.size() < 10) || (obj->bad_3d == true))
            {
                continue;
            }

            color = colors_bgr[obj->mnClass % 6];
            double diameter = sqrt(obj->mCuboid3D.width * obj->mCuboid3D.width   +   obj->mCuboid3D.lenth * obj->mCuboid3D.lenth )/2.0;
            for(int x=0; x<obj->mIE_rows; x++){
                double angle_divide = 2*M_PI/obj->mIE_rows;
                double angle = angle_divide * ( x + 0.5 );
                double p_x = cos(angle) * diameter;
                double p_y = sin(angle) * diameter;

                double h_divide =  obj->mCuboid3D.height/obj->mIE_cols;
                for(int y=0; y<obj->mIE_cols; y++){

                    double p_z = h_divide * (y+0.5) - obj->mCuboid3D.height/2.0;

                    Eigen::Matrix4d T = Converter::cvMattoMatrix4d(obj->mCuboid3D.pose_mat);
                    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
                    Eigen::Vector3d p_world = R * Eigen::Vector3d(p_x, p_y, p_z);
                    geometry_msgs::Point p;
                    p.x= p_world[0] + T(0, 3);
                    p.y= p_world[1] + T(1, 3);
                    p.z= p_world[2] + T(2, 3);

                    if(obj->mvGridProb_mat.at<float>(x,y) > 0.5){
                        //marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
                        marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.7;
                    }
                    else if(obj->mvGridProb_mat.at<float>(x,y) < 0.5){
                        //marker.color.r =0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
                        marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.15;
                    }
                    else {
                        marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 0.2;
                    }
                    marker.id= ++IE_id;
                    marker.points.push_back(p);
                    publisher_IE.publish(marker);
                    //usleep(100);
                }
            }
        }
    }

    void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)
    {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
        mbCameraUpdated = true;
    }

    cv::Mat MapPublisher::GetCurrentCameraPose()
    {
        unique_lock<mutex> lock(mMutexCamera);
        return mCameraPose.clone();
    }

    bool MapPublisher::isCamUpdated()
    {
        unique_lock<mutex> lock(mMutexCamera);
        return mbCameraUpdated;
    }

    void MapPublisher::ResetCamFlag()
    {
        unique_lock<mutex> lock(mMutexCamera);
        mbCameraUpdated = false;
    }
}