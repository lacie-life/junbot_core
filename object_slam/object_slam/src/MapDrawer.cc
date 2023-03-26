
#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Parameter.h"
#include "MapCuboidObject.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace semantic_slam {

    MapDrawer::MapDrawer(Atlas *pAtlas, const string &strSettingPath, Settings *settings) :
            mpAtlas(pAtlas),
            m_octree(NULL),
            m_maxRange(-1.0),
            m_useHeightMap(true),
            m_res(0.05),
            m_colorFactor(0.8),
            m_treeDepth(0),
            m_maxTreeDepth(0) {
        if (settings) {
            newParameterLoader(settings);
        } else {
            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
            bool is_correct = ParseViewerParamFile(fSettings);

            if (!is_correct) {
                std::cout << "Map drawer param \n";
                std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
                try {
                    throw -1;
                }
                catch (exception &e) {

                }
            }
        }

        m_octree = new octomap::ColorOcTree(m_res);
        // initialize octomap
        m_octree->setClampingThresMin(0.12);
        m_octree->setClampingThresMax(0.97);
        m_octree->setProbHit(0.7);
        m_octree->setProbMiss(0.4);

        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;

        bIsLocalization = false;
        mpMerge2d3d = new(MergeSG);

        // For 3D cuboid testing
        box_colors.push_back(Vector3f(230, 0, 0) / 255.0);	 // red  0
        box_colors.push_back(Vector3f(60, 180, 75) / 255.0);   // green  1
        box_colors.push_back(Vector3f(0, 0, 255) / 255.0);	 // blue  2
        box_colors.push_back(Vector3f(255, 0, 255) / 255.0);   // Magenta  3
        box_colors.push_back(Vector3f(255, 165, 0) / 255.0);   // orange 4
        box_colors.push_back(Vector3f(128, 0, 128) / 255.0);   //purple 5
        box_colors.push_back(Vector3f(0, 255, 255) / 255.0);   //cyan 6
        box_colors.push_back(Vector3f(210, 245, 60) / 255.0);  //lime  7
        box_colors.push_back(Vector3f(250, 190, 190) / 255.0); //pink  8
        box_colors.push_back(Vector3f(0, 128, 128) / 255.0);   //Teal  9

        all_edge_pt_ids.resize(8, 2); // draw 8 edges except front face
        all_edge_pt_ids << 2, 3, 3, 4, 4, 1, 3, 7, 4, 8, 6, 7, 7, 8, 8, 5;
        all_edge_pt_ids.array() -= 1;
        front_edge_pt_ids.resize(4, 2);
        front_edge_pt_ids << 1, 2, 2, 6, 6, 5, 5, 1;
        front_edge_pt_ids.array() -= 1;

        std::cout << "MapDrawer Init \n";
    }

    MapDrawer::~MapDrawer() {
        delete mpMerge2d3d;
        delete m_octree;
    }

    void MapDrawer::newParameterLoader(Settings *settings) {
        mKeyFrameSize = settings->keyFrameSize();
        mKeyFrameLineWidth = settings->keyFrameLineWidth();
        mGraphLineWidth = settings->graphLineWidth();
        mPointSize = settings->pointSize();
        mCameraSize = settings->cameraSize();
        mCameraLineWidth = settings->cameraLineWidth();
        m_res = settings->octoMapRes();

    }

    bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings) {
        bool b_miss_params = false;

        cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
        if (!node.empty()) {
            mKeyFrameSize = node.real();
        } else {
            std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.KeyFrameLineWidth"];
        if (!node.empty()) {
            mKeyFrameLineWidth = node.real();
        } else {
            std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.GraphLineWidth"];
        if (!node.empty()) {
            mGraphLineWidth = node.real();
        } else {
            std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.PointSize"];
        if (!node.empty()) {
            mPointSize = node.real();
        } else {
            std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.CameraSize"];
        if (!node.empty()) {
            mCameraSize = node.real();
        } else {
            std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.CameraLineWidth"];
        if (!node.empty()) {
            mCameraLineWidth = node.real();
        } else {
            std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["octoMap.res"];
        if (!node.empty()) {
            m_res = node.real();
        } else {
            std::cerr << "*octoMap.res parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        return !b_miss_params;
    }

    void MapDrawer::DrawMapPoints() {
        Map *pActiveMap = mpAtlas->GetCurrentMap();
        if (!pActiveMap)
            return;

        const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if (vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 0.0);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));

        }

        glEnd();

        // world coordinate
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph,
                                  const bool bDrawOptLba) {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        Map *pActiveMap = mpAtlas->GetCurrentMap();
        // DEBUG LBA
        std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
        std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

        if (!pActiveMap)
            return;

        const vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();

        if (bDrawKF) {
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat *) Twc.data());

                if (!pKF->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth * 5);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                } else {
                    //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                    glLineWidth(mKeyFrameLineWidth);
                    if (bDrawOptLba) {
                        if (sOptKFs.find(pKF->mnId) != sOptKFs.end()) {
                            glColor3f(0.0f, 1.0f, 0.0f); // Green -> Opt KFs
                        } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
                            glColor3f(1.0f, 0.0f, 0.0f); // Red -> Fixed KFs
                        } else {
                            glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                        }
                    } else {
                        glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                    }
                    glBegin(GL_LINES);
                }

                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();

                glEnd();
            }
        }

        if (bDrawGraph) {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            // cout << "-----------------Draw graph-----------------" << endl;
            for (size_t i = 0; i < vpKFs.size(); i++) {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
                if (!vCovKFs.empty()) {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                         vit != vend; vit++) {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow(0), Ow(1), Ow(2));
                        glVertex3f(Ow2(0), Ow2(1), Ow2(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent) {
                    Eigen::Vector3f Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owp(0), Owp(1), Owp(2));
                }

                // Loops
                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                    if ((*sit)->mnId < vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owl(0), Owl(1), Owl(2));
                }
            }

            glEnd();
        }

        if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
            glLineWidth(mGraphLineWidth);
            glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            //Draw inertial links
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKFi = vpKFs[i];
                Eigen::Vector3f Ow = pKFi->GetCameraCenter();
                KeyFrame *pNext = pKFi->mNextKF;
                if (pNext) {
                    Eigen::Vector3f Owp = pNext->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owp(0), Owp(1), Owp(2));
                }
            }

            glEnd();
        }

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();

        if (bDrawKF) {
            for (Map *pMap: vpMaps) {
                if (pMap == pActiveMap)
                    continue;

                vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

                for (size_t i = 0; i < vpKFs.size(); i++) {
                    KeyFrame *pKF = vpKFs[i];
                    Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                    unsigned int index_color = pKF->mnOriginMapId;

                    glPushMatrix();

                    glMultMatrixf((GLfloat *) Twc.data());

                    if (!vpKFs[i]->GetParent()) // It is the first KF in the map
                    {
                        glLineWidth(mKeyFrameLineWidth * 5);
                        glColor3f(1.0f, 0.0f, 0.0f);
                        glBegin(GL_LINES);
                    } else {
                        glLineWidth(mKeyFrameLineWidth);
                        glColor3f(mfFrameColors[index_color][0], mfFrameColors[index_color][1],
                                  mfFrameColors[index_color][2]);
                        glBegin(GL_LINES);
                    }

                    glVertex3f(0, 0, 0);
                    glVertex3f(w, h, z);
                    glVertex3f(0, 0, 0);
                    glVertex3f(w, -h, z);
                    glVertex3f(0, 0, 0);
                    glVertex3f(-w, -h, z);
                    glVertex3f(0, 0, 0);
                    glVertex3f(-w, h, z);

                    glVertex3f(w, h, z);
                    glVertex3f(w, -h, z);

                    glVertex3f(-w, h, z);
                    glVertex3f(-w, -h, z);

                    glVertex3f(-w, h, z);
                    glVertex3f(w, h, z);

                    glVertex3f(-w, -h, z);
                    glVertex3f(w, -h, z);
                    glEnd();

                    glPopMatrix();
                }
            }
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();

        // world coordinate
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
        glEnd();
    }

    void MapDrawer::DrawMapCuboids()
    {
        Map* mpMap = mpAtlas->GetCurrentMap();

        std::vector<Object_Map* > object_3d = mpMap->GetObjects();

        // std::cout << "[MapDrawer] Object Number: " << object_3d.size() << std::endl;

        for (size_t i = 0; i < object_3d.size(); i++)
        {
            if ((object_3d[i]->mvpMapObjectMappoints.size() < 10) || (object_3d[i]->bad_3d == true))
            {
                continue;
            }

            Cuboid3D cube = object_3d[i]->mCuboid3D;

            glBegin(GL_LINES);
            glLineWidth(mGraphLineWidth * 4);
            glColor3f(230 /255.0, 0.0, 0.0);

            //     7------6
            //    /|     /|
            //   / |    / |
            //  4------5  |
            //  |  3---|--2
            //  | /    | /
            //  0------1
            glVertex3f(cube.corner_1[0], cube.corner_1[1], cube.corner_1[2]);//
            glVertex3f(cube.corner_2[0], cube.corner_2[1], cube.corner_2[2]);//

            glVertex3f(cube.corner_2[0], cube.corner_2[1], cube.corner_2[2]);//
            glVertex3f(cube.corner_3[0], cube.corner_3[1], cube.corner_3[2]);//

            glVertex3f(cube.corner_3[0], cube.corner_3[1], cube.corner_3[2]);//
            glVertex3f(cube.corner_4[0], cube.corner_4[1], cube.corner_4[2]);//

            glVertex3f(cube.corner_4[0], cube.corner_4[1], cube.corner_4[2]);//
            glVertex3f(cube.corner_1[0], cube.corner_1[1], cube.corner_1[2]);//

            glVertex3f(cube.corner_1[0], cube.corner_1[1], cube.corner_1[2]);//
            glVertex3f(cube.corner_5[0], cube.corner_5[1], cube.corner_5[2]);//

            glVertex3f(cube.corner_2[0], cube.corner_2[1], cube.corner_2[2]);//
            glVertex3f(cube.corner_6[0], cube.corner_6[1], cube.corner_6[2]);//

            glVertex3f(cube.corner_3[0], cube.corner_3[1], cube.corner_3[2]);//
            glVertex3f(cube.corner_7[0], cube.corner_7[1], cube.corner_7[2]);//

            glVertex3f(cube.corner_4[0], cube.corner_4[1], cube.corner_4[2]);//
            glVertex3f(cube.corner_8[0], cube.corner_8[1], cube.corner_8[2]);//

            glVertex3f(cube.corner_5[0], cube.corner_5[1], cube.corner_5[2]);//
            glVertex3f(cube.corner_6[0], cube.corner_6[1], cube.corner_6[2]);//

            glVertex3f(cube.corner_6[0], cube.corner_6[1], cube.corner_6[2]);//
            glVertex3f(cube.corner_7[0], cube.corner_7[1], cube.corner_7[2]);//

            glVertex3f(cube.corner_7[0], cube.corner_7[1], cube.corner_7[2]);//
            glVertex3f(cube.corner_8[0], cube.corner_8[1], cube.corner_8[2]);//

            glVertex3f(cube.corner_8[0], cube.corner_8[1], cube.corner_8[2]);//
            glVertex3f(cube.corner_5[0], cube.corner_5[1], cube.corner_5[2]);//

            glEnd();
        }
    }

    void MapDrawer::DrawMapCuboids2()
    {
        // make sure final cuboid is in init world frame.
        // draw all map objects
        const vector<MapCuboidObject *> all_Map_objs = (mpAtlas->GetCurrentMap())->GetAllMapObjects();
        Vector4d front_face_color(1.0, 0.0, 1.0, 1.0); // draw front face edges magenta
//        std::cout << "MapDrawer: DrawMapCuboids: "<< all_Map_objs.size() << std::endl;
        for (size_t object_id = 0; object_id < all_Map_objs.size(); object_id++) {
            MapCuboidObject *obj_landmark = all_Map_objs[object_id];

            if (obj_landmark->isBad()) // some good, some bad, some not determined
                continue;

            // show objects that being optimized! for kitti fix scale, this will make map visualization better.
            if (bundle_object_opti) {
                if (!obj_landmark->obj_been_optimized) {
                    continue;
                }
            }

            Eigen::MatrixXd cube_corners;
            if (bundle_object_opti && whether_dynamic_object)
                cube_corners = obj_landmark->pose_Twc_afterba.compute3D_BoxCorner(); // show pose after BA, will have some delay, but looks good
            else
                cube_corners = obj_landmark->GetWorldPos().compute3D_BoxCorner();

            if (obj_landmark->Observations() == 1) {
                glLineWidth(mGraphLineWidth * 2);
                glBegin(GL_LINES);
                front_face_color = Vector4d(0, 0, 128.0 / 255.0, 1.0);
            } else {
                glLineWidth(mGraphLineWidth * 4);
                glBegin(GL_LINES);
                front_face_color = Vector4d(1.0, 0.0, 1.0, 1.0);
            }
            // draw cuboid
            Vector3f box_color = box_colors[obj_landmark->mnId % box_colors.size()];
            glColor4f(box_color(0), box_color(1), box_color(2), 1.0f); // draw all edges  cyan
            for (int line_id = 0; line_id < all_edge_pt_ids.rows(); line_id++) {
                glVertex3f(cube_corners(0, all_edge_pt_ids(line_id, 0)), cube_corners(1, all_edge_pt_ids(line_id, 0)),
                           cube_corners(2, all_edge_pt_ids(line_id, 0)));
                glVertex3f(cube_corners(0, all_edge_pt_ids(line_id, 1)), cube_corners(1, all_edge_pt_ids(line_id, 1)),
                           cube_corners(2, all_edge_pt_ids(line_id, 1)));
            }
            for (int line_id = 0; line_id < front_edge_pt_ids.rows(); line_id++) {
                glVertex3f(cube_corners(0, front_edge_pt_ids(line_id, 0)),
                           cube_corners(1, front_edge_pt_ids(line_id, 0)),
                           cube_corners(2, front_edge_pt_ids(line_id, 0)));
                glVertex3f(cube_corners(0, front_edge_pt_ids(line_id, 1)),
                           cube_corners(1, front_edge_pt_ids(line_id, 1)),
                           cube_corners(2, front_edge_pt_ids(line_id, 1)));
            }
            glEnd();

            // // draw dynamic object history path
            // if (whether_dynamic_object && obj_landmark->is_dynamic && obj_landmark->allDynamicPoses.size() > 0)
            // {
            // 	glLineWidth(mGraphLineWidth * 2);
            // 	glBegin(GL_LINE_STRIP);									   // line strip connects adjacent points
            // 	glColor4f(box_color(0), box_color(1), box_color(2), 1.0f); // draw all edges  cyan
            // 	for (auto it = obj_landmark->allDynamicPoses.begin(); it != obj_landmark->allDynamicPoses.end(); it++)
            // 	{
            // 		if (bundle_object_opti && !it->second.second) //only show optimized frame object pose
            // 			continue;
            // 		g2o::cuboid cubepose = it->second.first;
            // 		glVertex3f(cubepose.pose.translation()(0), cubepose.pose.translation()(1), cubepose.pose.translation()(2));
            // 	}
            // 	glEnd();
            // }
        }
    }

    void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw) {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.inverse();
    }

    bool MapDrawer::GetCurrentCameraPos(cv::Mat &Rcw, cv::Mat &Ow) {
        bool flag = false;
        cv::Mat camPose = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(mCameraPose));

        if (!camPose.empty()) {
            unique_lock<mutex> lock(mMutexCamera);
            Rcw = camPose.rowRange(0, 3).colRange(0, 3);
            Ow = -Rcw.t() * camPose.rowRange(0, 3).col(3);
            flag = true;
        }
        return flag;
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw) {
        Eigen::Matrix4f Twc;
        {
            unique_lock<mutex> lock(mMutexCamera);
            Twc = mCameraPose.matrix();
        }

        for (int i = 0; i < 4; i++) {
            M.m[4 * i] = Twc(0, i);
            M.m[4 * i + 1] = Twc(1, i);
            M.m[4 * i + 2] = Twc(2, i);
            M.m[4 * i + 3] = Twc(3, i);
        }

        MOw.SetIdentity();
        MOw.m[12] = Twc(0, 3);
        MOw.m[13] = Twc(1, 3);
        MOw.m[14] = Twc(2, 3);
    }


    void MapDrawer::DrawGrid() {
        glBegin(GL_LINES);
        glLineWidth(1);

        glColor3f(0.5, 0.5, 0.5); //gray
        int size = 10;
        for (int i = -size; i <= size; i++) {
            // xz -10～10
            glVertex3f(i, 0.6, size);
            glVertex3f(i, 0.6, -size);

            // xz -10～10
            glVertex3f(size, 0.6, i);
            glVertex3f(-size, 0.6, i);
        }
        glEnd();
    }

    void MapDrawer::DrawOctoMap() {
        Map *pActiveMap = mpAtlas->GetCurrentMap();

        if (!pActiveMap)
            return;

        vector<KeyFrame *> vKFs = pActiveMap->GetAllKeyFrames();

        int N = vKFs.size();

//        std::cout << "OctoMap Draw with " << N << " Keyframe \n";

        if (N == 0) {
            m_octree->clear();
            lastKeyframeSize = 0;
            return;
        }
        if (bIsLocalization == false) {
            UpdateOctomap(vKFs);
        }

        octomap::ColorOcTree::tree_iterator it = m_octree->begin_tree();
        octomap::ColorOcTree::tree_iterator end = m_octree->end_tree();
        int counter = 0;
        double occ_thresh = 0.8;
        int level = 16;
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);

        ////DRAW OCTOMAP BEGIN//////
        // double stretch_factor = 128/(1 - occ_thresh); //1280.0
        // occupancy range in which the displayed cubes can be

        for (; it != end; ++counter, ++it) {
            if (level != it.getDepth()) {
                continue;
            }
            double occ = it->getOccupancy();
            if (occ < occ_thresh) {
                continue;
            }

            // std::cout<< occ << std::endl;

            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            float halfsize = it.getSize() / 2.0;
            float x = it.getX();
            float y = it.getY();
            float z = it.getZ();

            double h = (std::min(std::max((y - minY) / (maxY - minY), 0.0), 1.0)) * 0.8;

            double r, g, b;
            heightMapColor(h, r, g, b);

            glBegin(GL_TRIANGLES);
            //Front
            glColor3d(r, g, b);
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);// - - - 1
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);// - + - 2
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);// + + -3

            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - -
            glVertex3f(x + halfsize, y + halfsize, z - halfsize); // + + -
            glVertex3f(x + halfsize, y - halfsize, z - halfsize); // + - -4

            //Back
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 1
            glVertex3f(x + halfsize, y - halfsize, z + halfsize); // + - + 2
            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + + 3

            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - +
            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + +
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 4

            //Left
            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - - 1
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 2
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 3

            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - -
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + +
            glVertex3f(x - halfsize, y + halfsize, z - halfsize); // - + - 4

            //Right
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            //top
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y - halfsize, z + halfsize);

            //bottom
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glEnd();

            glBegin(GL_LINES);
            glColor3f(0, 0, 0);
            //
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);// - - - 1
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);

            glVertex3f(x - halfsize, y + halfsize, z - halfsize);// - + - 2
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);// + + -3

            glVertex3f(x + halfsize, y + halfsize, z - halfsize);// + + -3
            glVertex3f(x + halfsize, y - halfsize, z - halfsize); // + - -4

            glVertex3f(x + halfsize, y - halfsize, z - halfsize); // + - -4
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);// - - - 1


            // back
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 1
            glVertex3f(x + halfsize, y - halfsize, z + halfsize); // + - + 2

            glVertex3f(x + halfsize, y - halfsize, z + halfsize); // + - + 2
            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + + 3

            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + + 3
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 4

            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 4
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 1

            // top
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);

            // bottom
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glEnd();
        }
    }

    void MapDrawer::DrawObject()
    {
        std::vector<Cluster>& Clusters = mpMerge2d3d->mpOD->mClusters;
        int objnumber = Clusters.size();

        if( objnumber >0)
        {
            std::cout<< "OD size: " << objnumber << std::endl;

            for(int m=0; m<objnumber; m++)
            {
                Cluster & cluster = Clusters[m];
                Eigen::Vector3f size  = cluster.size;
                Eigen::Vector3f cent  = cluster.centroid;


                glBegin(GL_LINES);
                glLineWidth(5);
                cv::Scalar color =  mpMerge2d3d->mpOD->getObjectColor(cluster.object_name);
                glColor3f(color.val[0]/255.0, color.val[1]/255.0, color.val[2]/255.0);

                glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
                glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//

                glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
                glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//

                glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
                glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//

                glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
                glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
                // 4
                glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
                glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

                glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
                glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

                glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
                glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

                glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
                glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
                // 4
                glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//
                glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

                glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//
                glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

                glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
                glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

                glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
                glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

                glEnd();
            }
        }
        // mlast_obj_size = objnumber;
    }

    void MapDrawer::UpdateOctomap(vector<KeyFrame*> vKFs)
    {
        int N = vKFs.size();
        if (N>1)
        {
            for ( size_t i=lastKeyframeSize; i < (unsigned int)N-1 ; i++ )
            {
                std::cout<< " keyFrame: "<< i << std::endl;

                Eigen::Isometry3d pose = semantic_slam::Converter::toSE3Quat( vKFs[i]->GetPose());

                pcl::PointCloud<pcl::PointXYZRGB>  ground;
                pcl::PointCloud<pcl::PointXYZRGB>  nonground;

                if(vKFs[i]->mvObject.size()>0)
                    GeneratePointCloud( vKFs[i], ground, nonground, vKFs[i]->mvObject);
                else
                    GeneratePointCloud( vKFs[i], ground, nonground);

                octomap::point3d sensorOrigin =
                        octomap::point3d( pose(0,3), pose(1,3), pose(2,3));

                InsertScan(sensorOrigin, ground, nonground);
            }
            lastKeyframeSize = N-1;
        }
    }

    // Generate point cloud of current frame, simple filter and separate ground and non-ground
    void MapDrawer::GeneratePointCloud(KeyFrame *kf,
                                       pcl::PointCloud<pcl::PointXYZRGB> &ground,
                                       pcl::PointCloud<pcl::PointXYZRGB> &nonground)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for ( int m=0; m<(kf->mImDep.rows); m+=1 )
        {
            for ( int n=0; n<(kf->mImDep.cols); n+=1 )
            {
                // Depth m is the unit, keep the points within 0~2m
                float d = kf->mImDep.ptr<float>(m)[n];
                //if (d < 0.01 || d>2.0) // Camera measurement range 0.5 ~ 6m
                if (d < 0.50 || d>3.0) // Camera measurement range 0.5 ~ 6m
                    continue;
                pcl::PointXYZRGB p;
                p.z = d;
                p.x = ( n - kf->cx) * p.z / kf->fx;
                p.y = ( m - kf->cy) * p.z / kf->fy;
                if(p.y<-3.0 || p.y>3.0) continue;// Reserve the points in the vertical direction within the range of -3 ~ 3m
                p.b = kf->mImRGB.ptr<uchar>(m)[n*3+0];
                p.g = kf->mImRGB.ptr<uchar>(m)[n*3+1];
                p.r = kf->mImRGB.ptr<uchar>(m)[n*3+2];
                cloud->points.push_back( p );

            }
        }

        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01,0.01, 0.01);
        vg.filter(*cloud);

        Eigen::Isometry3d T = semantic_slam::Converter::toSE3Quat(kf->GetPose());
        pcl::PointCloud<pcl::PointXYZRGB> temp;
        pcl::transformPointCloud( *cloud, temp, T.inverse().matrix());

        if(temp.size()<50)
        {
            printf("pointcloud too small skip ground plane extraction\n;");
            ground = temp;
        }
        else
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            pcl::SACSegmentation<pcl::PointCloud<pcl::PointXYZRGB>::PointType> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(200);
            seg.setDistanceThreshold(0.04);
            seg.setAxis(Eigen::Vector3f(0, 1 ,0));
            seg.setEpsAngle(0.5);

            pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered(temp);
            pcl::ExtractIndices<pcl::PointCloud<pcl::PointXYZRGB>::PointType> extract;
            bool groundPlaneFound = false;
            while(cloud_filtered.size()>10 && !groundPlaneFound)
            {
                seg.setInputCloud(cloud_filtered.makeShared());
                seg.segment(*inliers, *coefficients);
                if(inliers->indices.size()==0)
                {
                    break;
                }
                extract.setInputCloud(cloud_filtered.makeShared());
                extract.setIndices(inliers);


                // a*X + b*Y + c*Z + d = 0;
                if (std::abs(coefficients->values.at(3)) >0.07)
                {

                    printf("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f \n",
                           inliers->indices.size(),
                           cloud_filtered.size(),
                           coefficients->values.at(0),
                           coefficients->values.at(1),
                           coefficients->values.at(2),
                           coefficients->values.at(3));

                    extract.setNegative (false);
                    extract.filter (ground);
                    // remove ground points from full pointcloud:
                    // workaround for PCL bug:
                    if(inliers->indices.size() != cloud_filtered.size())
                    {
                        extract.setNegative(true);
                        pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
                        extract.filter(cloud_out);
                        nonground += cloud_out;
                        cloud_filtered = cloud_out;
                    }

                    groundPlaneFound = true;
                }
                else
                {
                    printf("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f \n",
                           inliers->indices.size(),
                           cloud_filtered.size(),
                           coefficients->values.at(0),
                           coefficients->values.at(1),
                           coefficients->values.at(2),
                           coefficients->values.at(3));

                    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
                    extract.setNegative (false);
                    extract.filter(cloud_out);
                    nonground +=cloud_out;
                    if(inliers->indices.size() != cloud_filtered.size())
                    {
                        extract.setNegative(true);
                        cloud_out.points.clear();
                        extract.filter(cloud_out);
                        cloud_filtered = cloud_out;
                    }
                    else
                    {
                        cloud_filtered.points.clear();

                    }
                }

            }//while

            if(!groundPlaneFound)
            {
                nonground = temp;
            }
        }
    }

    // Generate the point cloud of the current frame, simply filter and separate the ground and non-ground
    void MapDrawer::GeneratePointCloud(KeyFrame *kf,
                                       pcl::PointCloud<pcl::PointXYZRGB> &ground,
                                       pcl::PointCloud<pcl::PointXYZRGB> &nonground,
                                       std::vector<Object>& objects)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->resize(kf->mImDep.rows * kf->mImDep.cols);
        cloud->width    =  kf->mImDep.cols;
        cloud->height   =  kf->mImDep.rows;
        cloud->is_dense =  false;
        for ( int m=0; m<(kf->mImDep.rows); m+=1 )
        {
            for ( int n=0; n<(kf->mImDep.cols); n+=1 )
            {
                float d = kf->mImDep.ptr<float>(m)[n];
                //if (d < 0.01 || d>2.0)
                if (d < 0.50 || d>4.0)
                    continue;
                //float z = d;
                float y = ( m - kf->cy) * d / kf->fy;
                if(y<-3.0 || y>3.0) continue;
                int ind = m * kf->mImDep.cols + n;
                cloud->points[ind].z = d;
                cloud->points[ind].x = ( n - kf->cx) * d / kf->fx;
                cloud->points[ind].y = y;
                cloud->points[ind].b = kf->mImRGB.ptr<uchar>(m)[n*3+0];
                cloud->points[ind].g = kf->mImRGB.ptr<uchar>(m)[n*3+1];
                cloud->points[ind].r = kf->mImRGB.ptr<uchar>(m)[n*3+2];

            }
        }

        Eigen::Isometry3d T = semantic_slam::Converter::toSE3Quat(kf->GetPose());
        //pcl::PointCloud<pcl::PointXYZRGB> temp;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud( *cloud, *temp, T.inverse().matrix());

        mpMerge2d3d->merge(objects, kf->mImDep, temp);

        std::vector<Cluster>& Clusters = mpMerge2d3d->mpOD->mClusters;

        int objNumber = Clusters.size();

        std::cout<< "OD size: " << objNumber << std::endl;

        for( int m=0; m < objNumber; m++)
        {
            Cluster & cluster = Clusters[m];
            Eigen::Vector3f size  = cluster.size;
            Eigen::Vector3f cent  = cluster.centroid;

            std::cout<< "obj: " << cluster.object_name << " " << cluster.prob << " "
                     << cent[0] << " " << cent[1] << " " << cent[2] << " "
                     << size[0] << " " << size[1] << " " << size[2] << " "
                     << std::endl;
        }

        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(temp);
        vg.setLeafSize(0.01,0.01, 0.01);
        vg.filter(*temp);

        if(temp->size()<50)
        {
            printf("pointcloud too small skip ground plane extraction\n;");
            ground = *temp;
        }
        else
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            pcl::SACSegmentation<pcl::PointCloud<pcl::PointXYZRGB>::PointType> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(200);
            seg.setDistanceThreshold(0.04);
            seg.setAxis(Eigen::Vector3f(0, 1 ,0));
            seg.setEpsAngle(0.5);

            pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered(*temp);
            pcl::ExtractIndices<pcl::PointCloud<pcl::PointXYZRGB>::PointType> extract;
            bool groundPlaneFound = false;
            while(cloud_filtered.size()>10 && !groundPlaneFound)
            {
                seg.setInputCloud(cloud_filtered.makeShared());
                seg.segment(*inliers, *coefficients);
                if(inliers->indices.size()==0)
                {
                    break;
                }
                extract.setInputCloud(cloud_filtered.makeShared());
                extract.setIndices(inliers);

                // a*X + b*Y + c*Z + d = 0;
                if (std::abs(coefficients->values.at(3)) >0.07)
                {

                    printf("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f \n",
                           inliers->indices.size(),
                           cloud_filtered.size(),
                           coefficients->values.at(0),
                           coefficients->values.at(1),
                           coefficients->values.at(2),
                           coefficients->values.at(3));

                    extract.setNegative (false);
                    extract.filter (ground);
                    // remove ground points from full pointcloud:
                    // workaround for PCL bug:
                    if(inliers->indices.size() != cloud_filtered.size())
                    {
                        extract.setNegative(true);
                        pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
                        extract.filter(cloud_out);
                        nonground += cloud_out;
                        cloud_filtered = cloud_out;
                    }

                    groundPlaneFound = true;
                }
                else
                {
                    printf("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f \n",
                           inliers->indices.size(),
                           cloud_filtered.size(),
                           coefficients->values.at(0),
                           coefficients->values.at(1),
                           coefficients->values.at(2),
                           coefficients->values.at(3));

                    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
                    extract.setNegative (false);
                    extract.filter(cloud_out);
                    nonground +=cloud_out;
                    if(inliers->indices.size() != cloud_filtered.size())
                    {
                        extract.setNegative(true);
                        cloud_out.points.clear();
                        extract.filter(cloud_out);
                        cloud_filtered = cloud_out;
                    }
                    else
                    {
                        cloud_filtered.points.clear();

                    }
                }

            }//while

            if(!groundPlaneFound)
            {
                nonground = *temp;
            }
        }
    }

    void MapDrawer::InsertScan(octomap::point3d sensorOrigin,
                               pcl::PointCloud<pcl::PointXYZRGB> &ground,
                               pcl::PointCloud<pcl::PointXYZRGB> &nonground)
    {

        if(!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)||
           !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
        {
            printf("coulde not generate key for origin\n");
        }

        octomap::KeySet free_cells, occupied_cells;

        for(auto p:ground.points)
        {
            octomap::point3d point(p.x, p.y, p.z);
            // only clear space (ground points)
            if(m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
            {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);
            }
            octomap::OcTreeKey endKey;
            if(m_octree->coordToKeyChecked(point, endKey))
            {
                updateMinKey(endKey, m_updateBBXMin);
                updateMaxKey(endKey, m_updateBBXMax);
            }
            else
            {
                printf("could not generator key for endpoint");
            }
        }

        // all other points : free on ray, occupied on endpoings:
        for(auto p:nonground.points)
        {
            octomap::point3d point(p.x, p.y, p.z);
            //free cell
            if(m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
            {
                // free_cells.insert(m_keyRay.begin(),m_keyRay.end());
            }
            //occupided endpoint
            octomap::OcTreeKey key;
            if(m_octree->coordToKeyChecked(point, key))
            {
                occupied_cells.insert(key);
                updateMinKey(key, m_updateBBXMin);
                updateMaxKey(key, m_updateBBXMax);
                m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);
            }

        }
        //  pcl::PointCloud<pcl::PointXYZRGB>observation;

        for(octomap::KeySet::iterator it = free_cells.begin(),
                    end= free_cells.end();
            it!=end; ++it)
        {
            if(occupied_cells.find(*it) == occupied_cells.end())
            {
                m_octree->updateNode(*it, false);
            }
        }
        for(octomap::KeySet::iterator it = occupied_cells.begin(),
                    end= occupied_cells.end();
            it!=end; ++it)
        {
            m_octree->updateNode(*it, true);
        }

        m_octree->prune();
    }

    void MapDrawer::heightMapColor(double h,
                                   double& r,
                                   double &g,
                                   double& b)
    {

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;

        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;

        if(!(i & 1))
        {
            f = 1 - f;
        }
        m = v * (1-s);
        n = v * (1- s*f);

        switch(i)
        {
            case 6:
            case 0:
                r = v; g = n; b = m;
                break;
            case 1:
                r = n; g = v; b = m;
                break;
            case 2:
                r = m; g = v; b = n;
                break;
            case 3:
                r = m; g = n; b = v;
                break;
            case 4:
                r = n; g = m; b = v;
                break;
            case 5:
                r = v; g = m; b = n;
                break;
            default:
                r = 1; g = 0.5; b = 0.5;
                break;

        }

    }

    void MapDrawer::RegisterObs(pcl::PointCloud<pcl::PointXYZRGB> mvObs)
    {
        observation = mvObs;
    }

    // Fat fruit forest shows pcl point cloud points
    void MapDrawer::DrawObs(void)
    {
        glPointSize(mPointSize*2);
        glBegin(GL_POINTS);

        //glColor3f(1.0,0.0,0.0);
        for(unsigned int i=0; i< observation.points.size(); i++)
        {
            glColor3f(observation.points[i].r/255.0,
                      observation.points[i].g/255.0,
                      observation.points[i].b/255.0);
            glVertex3f(observation.points[i].x,observation.points[i].y,observation.points[i].z);
        }
        glEnd();
    }


    void MapDrawer::SaveOctoMap(const char *filename)
    {
        std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
        if (outfile.is_open())
        {
            m_octree->write(outfile);
            outfile.close();
        }
    }

    void MapDrawer::LoadOctoMap()
    {
        octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read("octomap.ot");
        m_octree= dynamic_cast<octomap::ColorOcTree*> (tree);
    }


} //namespace semantic_slam
