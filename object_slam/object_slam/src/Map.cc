#include "Map.h"
#include "ObjectDatabase.h"
#include "Object.h"

#include <mutex>

namespace semantic_slam {

    long unsigned int Map::nNextId = 0;

    Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0),
                 mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
                 mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0),
                 mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false) {
        mnId = nNextId++;
        mThumbnail = static_cast<GLubyte *>(NULL);
        mpOD = new(ObjectDatabase);
    }

    Map::Map(int initKFid) : mnInitKFid(initKFid), mnMaxKFid(initKFid),/*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0),
                             mIsInUse(false),
                             mHasTumbnail(false), mbBad(false), mbImuInitialized(false),
                             mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
                             mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false),
                             mbIMU_BA1(false), mbIMU_BA2(false) {
        mnId = nNextId++;
        mThumbnail = static_cast<GLubyte *>(NULL);
        mpOD = new(ObjectDatabase);
    }

    Map::~Map() {
        //TODO: erase all points from memory
        mspMapPoints.clear();

        //TODO: erase all keyframes from memory
        mspKeyFrames.clear();

        if (mThumbnail)
            delete mThumbnail;
        mThumbnail = static_cast<GLubyte *>(NULL);

        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        if (mspKeyFrames.empty()) {
            cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
            mnInitKFid = pKF->mnId;
            mpKFinitial = pKF;
            mpKFlowerID = pKF;
        }
        mspKeyFrames.insert(pKF);
        if (pKF->mnId > mnMaxKFid) {
            mnMaxKFid = pKF->mnId;
        }
        if (pKF->mnId < mpKFlowerID->mnId) {
            mpKFlowerID = pKF;
        }
    }

    void Map::AddMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::SetImuInitialized() {
        unique_lock<mutex> lock(mMutexMap);
        mbImuInitialized = true;
    }

    bool Map::isImuInitialized() {
        unique_lock<mutex> lock(mMutexMap);
        return mbImuInitialized;
    }

    void Map::EraseMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);
        if (mspKeyFrames.size() > 0) {
            if (pKF->mnId == mpKFlowerID->mnId) {
                vector<KeyFrame *> vpKFs = vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
                sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
                mpKFlowerID = vpKFs[0];
            }
        } else {
            mpKFlowerID = 0;
        }

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs) {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange() {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx() {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    vector<KeyFrame *> Map::GetAllKeyFrames() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint *> Map::GetReferenceMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetId() {
        return mnId;
    }

    long unsigned int Map::GetInitKFid() {
        unique_lock<mutex> lock(mMutexMap);
        return mnInitKFid;
    }

    void Map::SetInitKFid(long unsigned int initKFif) {
        unique_lock<mutex> lock(mMutexMap);
        mnInitKFid = initKFif;
    }

    long unsigned int Map::GetMaxKFid() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    KeyFrame *Map::GetOriginKF() {
        return mpKFinitial;
    }

    void Map::SetCurrentMap() {
        mIsInUse = true;
    }

    void Map::SetStoredMap() {
        mIsInUse = false;
    }

    void Map::clear() {
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++) {
            KeyFrame *pKF = *sit;
            pKF->UpdateMap(static_cast<Map *>(NULL));
//        delete *sit;
        }

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = mnInitKFid;
        mbImuInitialized = false;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
        mbIMU_BA1 = false;
        mbIMU_BA2 = false;
    }

    bool Map::IsInUse() {
        return mIsInUse;
    }

    void Map::SetBad() {
        mbBad = true;
    }

    bool Map::IsBad() {
        return mbBad;
    }


    void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel) {
        unique_lock<mutex> lock(mMutexMap);

        // Body position (IMU) of first keyframe is fixed to (0,0,0)
        Sophus::SE3f Tyw = T;
        Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
        Eigen::Vector3f tyw = Tyw.translation();

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(); sit != mspKeyFrames.end(); sit++) {
            KeyFrame *pKF = *sit;
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Twc.translation() *= s;
            Sophus::SE3f Tyc = Tyw * Twc;
            Sophus::SE3f Tcy = Tyc.inverse();
            pKF->SetPose(Tcy);
            Eigen::Vector3f Vw = pKF->GetVelocity();
            if (!bScaledVel)
                pKF->SetVelocity(Ryw * Vw);
            else
                pKF->SetVelocity(Ryw * Vw * s);

        }
        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(); sit != mspMapPoints.end(); sit++) {
            MapPoint *pMP = *sit;
            pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
            pMP->UpdateNormalAndDepth();
        }
        mnMapChange++;
    }

    void Map::SetInertialSensor() {
        unique_lock<mutex> lock(mMutexMap);
        mbIsInertial = true;
    }

    bool Map::IsInertial() {
        unique_lock<mutex> lock(mMutexMap);
        return mbIsInertial;
    }

    void Map::SetIniertialBA1() {
        unique_lock<mutex> lock(mMutexMap);
        mbIMU_BA1 = true;
    }

    void Map::SetIniertialBA2() {
        unique_lock<mutex> lock(mMutexMap);
        mbIMU_BA2 = true;
    }

    bool Map::GetIniertialBA1() {
        unique_lock<mutex> lock(mMutexMap);
        return mbIMU_BA1;
    }

    bool Map::GetIniertialBA2() {
        unique_lock<mutex> lock(mMutexMap);
        return mbIMU_BA2;
    }

    void Map::ChangeId(long unsigned int nId) {
        mnId = nId;
    }

    unsigned int Map::GetLowerKFID() {
        unique_lock<mutex> lock(mMutexMap);
        if (mpKFlowerID) {
            return mpKFlowerID->mnId;
        }
        return 0;
    }

    int Map::GetMapChangeIndex() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChange;
    }

    void Map::IncreaseChangeIndex() {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChange++;
    }

    int Map::GetLastMapChange() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChangeNotified;
    }

    void Map::SetLastMapChange(int currentChangeId) {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChangeNotified = currentChangeId;
    }

    void Map::PreSave(std::set<GeometricCamera *> &spCams) {
        int nMPWithoutObs = 0;

        std::set<MapPoint *> tmp_mspMapPoints1;
        tmp_mspMapPoints1.insert(mspMapPoints.begin(), mspMapPoints.end());

        for (MapPoint *pMPi: tmp_mspMapPoints1) {
            if (!pMPi || pMPi->isBad())
                continue;

            if (pMPi->GetObservations().size() == 0) {
                nMPWithoutObs++;
            }
            map<KeyFrame *, std::tuple<int, int>> mpObs = pMPi->GetObservations();
            for (map<KeyFrame *, std::tuple<int, int>>::iterator it = mpObs.begin(), end = mpObs.end();
                 it != end; ++it) {
                if (it->first->GetMap() != this || it->first->isBad()) {
                    pMPi->EraseObservation(it->first);
                }

            }
        }

        // Saves the id of KF origins
        mvBackupKeyFrameOriginsId.clear();
        mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
        for (int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i) {
            mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
        }


        // Backup of MapPoints
        mvpBackupMapPoints.clear();

        std::set<MapPoint *> tmp_mspMapPoints2;
        tmp_mspMapPoints2.insert(mspMapPoints.begin(), mspMapPoints.end());

        for (MapPoint *pMPi: tmp_mspMapPoints2) {
            if (!pMPi || pMPi->isBad())
                continue;

            mvpBackupMapPoints.push_back(pMPi);
            pMPi->PreSave(mspKeyFrames, mspMapPoints);
        }

        // Backup of KeyFrames
        mvpBackupKeyFrames.clear();
        for (KeyFrame *pKFi: mspKeyFrames) {
            if (!pKFi || pKFi->isBad())
                continue;

            mvpBackupKeyFrames.push_back(pKFi);
            pKFi->PreSave(mspKeyFrames, mspMapPoints, spCams);
        }

        mnBackupKFinitialID = -1;
        if (mpKFinitial) {
            mnBackupKFinitialID = mpKFinitial->mnId;
        }

        mnBackupKFlowerID = -1;
        if (mpKFlowerID) {
            mnBackupKFlowerID = mpKFlowerID->mnId;
        }
    }

    void
    Map::PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc/*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/,
                  map<unsigned int, GeometricCamera *> &mpCams) {
        std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(),
                  std::inserter(mspMapPoints, mspMapPoints.begin()));
        std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(),
                  std::inserter(mspKeyFrames, mspKeyFrames.begin()));

        map<long unsigned int, MapPoint *> mpMapPointId;
        for (MapPoint *pMPi: mspMapPoints) {
            if (!pMPi || pMPi->isBad())
                continue;

            pMPi->UpdateMap(this);
            mpMapPointId[pMPi->mnId] = pMPi;
        }

        map<long unsigned int, KeyFrame *> mpKeyFrameId;
        for (KeyFrame *pKFi: mspKeyFrames) {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateMap(this);
            pKFi->SetORBVocabulary(pORBVoc);
            pKFi->SetKeyFrameDatabase(pKFDB);
            mpKeyFrameId[pKFi->mnId] = pKFi;
        }

        // References reconstruction between different instances
        for (MapPoint *pMPi: mspMapPoints) {
            if (!pMPi || pMPi->isBad())
                continue;

            pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
        }

        for (KeyFrame *pKFi: mspKeyFrames) {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
            pKFDB->add(pKFi);
        }


        if (mnBackupKFinitialID != -1) {
            mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
        }

        if (mnBackupKFlowerID != -1) {
            mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
        }

        mvpKeyFrameOrigins.clear();
        mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
        for (int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i) {
            mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
        }

        mvpBackupMapPoints.clear();
    }

    // For 3D cuboid testing
    void Map::AddObject(Object_Map *pObj)
    {
        unique_lock<mutex> lock(mMutexMap);
        // mvObjectMap.insert(pObj);
        mpOD->addObject(pObj);
    }

    vector<Object_Map*> Map::GetObjects()
    {
        unique_lock<mutex> lock(mMutexMap);
        // return vector<Object_Map*>(mvObjectMap.begin(), mvObjectMap.end());
        std::vector<Object_Map*> objs = mpOD->getAllObject();

        return objs;
    }

    void Map::AddMapObject(MapCuboidObject *pMO)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapObjects.insert(pMO);
    }

    void Map::EraseMapObject(MapCuboidObject *pMO)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapObjects.erase(pMO);
    }

    vector<MapCuboidObject *> Map::GetAllMapObjects()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapCuboidObject *>(mspMapObjects.begin(), mspMapObjects.end());
    }

    vector<MapCuboidObject *> Map::GetGoodMapObjects()
    {
        vector<MapCuboidObject *> res;
        for (set<MapCuboidObject *>::iterator sit = mspMapObjects.begin(), send = mspMapObjects.end(); sit != send; sit++)
            if ((*sit)->isGood)
                res.push_back(*sit);
        return res;
    }

    long unsigned int Map::MapObjectsInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapObjects.size();
    }

    // TODO: Testing

//    KeyFrame *Map::_ReadKeyFrame(ifstream &f,
//                                 ORBVocabulary &voc,
//                                 std::vector<MapPoint *> amp,
//                                 ORBextractor *orb_ext) {
//        Frame fr;
//        fr.mpORBvocabulary = &voc;
//        f.read((char *) &fr.mnId, sizeof(fr.mnId));
//        //cerr << " reading keyfrane id " << fr.mnId << endl;
//        f.read((char *) &fr.mTimeStamp, sizeof(fr.mTimeStamp));
//        cv::Mat Tcw(4, 4, CV_32F);
//        f.read((char *) &Tcw.at<float>(0, 3), sizeof(float));
//        f.read((char *) &Tcw.at<float>(1, 3), sizeof(float));
//        f.read((char *) &Tcw.at<float>(2, 3), sizeof(float));
//        Tcw.at<float>(3, 3) = 1.;
//        cv::Mat Qcw(1, 4, CV_32F);
//        f.read((char *) &Qcw.at<float>(0, 0), sizeof(float));
//        f.read((char *) &Qcw.at<float>(0, 1), sizeof(float));
//        f.read((char *) &Qcw.at<float>(0, 2), sizeof(float));
//        f.read((char *) &Qcw.at<float>(0, 3), sizeof(float));
//        convert.RmatOfQuat(Tcw, Qcw);
//        fr.SetPose(Tcw);
//        f.read((char *) &fr.N, sizeof(fr.N));
//        fr.mvKeys.reserve(fr.N);
//        fr.mDescriptors.create(fr.N, 32, CV_8UC1);
//        fr.mvpMapPoints = vector<MapPoint *>(fr.N, static_cast<MapPoint *>(NULL));
//        for (int i = 0; i < fr.N; i++) {
//
//            cv::KeyPoint kp;
//            f.read((char *) &kp.pt.x, sizeof(kp.pt.x));
//            f.read((char *) &kp.pt.y, sizeof(kp.pt.y));
//            f.read((char *) &kp.size, sizeof(kp.size));
//            f.read((char *) &kp.angle, sizeof(kp.angle));
//            f.read((char *) &kp.response, sizeof(kp.response));
//            f.read((char *) &kp.octave, sizeof(kp.octave));
//            fr.mvKeys.push_back(kp);
//
//            for (int j = 0; j < 32; j++)
//                f.read((char *) &fr.mDescriptors.at<unsigned char>(i, j), sizeof(char));
//
//            unsigned long int mpidx;
//            f.read((char *) &mpidx, sizeof(mpidx));
//            if (mpidx == ULONG_MAX) fr.mvpMapPoints[i] = NULL;
//            else fr.mvpMapPoints[i] = amp[mpidx];
//        }
//        // mono only for now
//        fr.mvuRight = vector<float>(fr.N, -1);
//        fr.mvDepth = vector<float>(fr.N, -1);
//        fr.mpORBextractorLeft = orb_ext;
//        fr.InitializeScaleLevels();
//        fr.UndistortKeyPoints();
//        fr.AssignFeaturesToGrid();
//        fr.ComputeBoW();
//
//        KeyFrame *kf = new KeyFrame(fr, this, NULL);
//        kf->mnId = fr.mnId; // bleeee why? do I store that?
//        for (int i = 0; i < fr.N; i++) {
//            if (fr.mvpMapPoints[i]) {
//                fr.mvpMapPoints[i]->AddObservation(kf, i);
//                if (!fr.mvpMapPoints[i]->GetReferenceKeyFrame())
//                    fr.mvpMapPoints[i]->SetReferenceKeyFrame(kf);
//            }
//        }
//
//        return kf;
//    }
//
//    MapPoint *Map::_ReadMapPoint(ifstream &f) {
//        long unsigned int id;
//        f.read((char *) &id, sizeof(id));
//        cv::Mat wp(3, 1, CV_32F);
//        f.read((char *) &wp.at<float>(0), sizeof(float));
//        f.read((char *) &wp.at<float>(1), sizeof(float));
//        f.read((char *) &wp.at<float>(2), sizeof(float));
//        long int mnFirstKFid = 0, mnFirstFrame = 0;
//        MapPoint *mp = new MapPoint(wp, mnFirstKFid, mnFirstFrame, this);
//        mp->mnId = id;
//        return mp;
//    }
//
//    bool Map::Load(const string &filename, ORBVocabulary &voc) {
//
////         if (!Camera::initialized) {
////             cerr << "Map: camera is not initialized. Cowardly refusing to load anything" << endl;
////             return false;
////         }
//
//        int nFeatures = 2000;
//        float scaleFactor = 1.2;
//        int nLevels = 8, fIniThFAST = 20, fMinThFAST = 7;
//        semantic_slam::ORBextractor orb_ext = semantic_slam::ORBextractor(nFeatures, scaleFactor, nLevels, fIniThFAST,
//                                                                  fMinThFAST);
//
//        cerr << "Map: reading from " << filename << endl;
//        ifstream f;
//        f.open(filename.c_str());
//
//
//        long unsigned int nb_mappoints, max_id = 0;
//        f.read((char *) &nb_mappoints, sizeof(nb_mappoints));
//        cerr << "reading " << nb_mappoints << " mappoints" << endl;
//        for (unsigned int i = 0; i < nb_mappoints; i++) {
//            semantic_slam::MapPoint *mp = _ReadMapPoint(f);
//            if (mp->mnId >= max_id) max_id = mp->mnId;
//            AddMapPoint(mp);
//        }
//        semantic_slam::MapPoint::nNextId = max_id + 1; // that is probably wrong if last mappoint is not here :(
//
//        std::vector<MapPoint *> amp = GetAllMapPoints();
//
//        long unsigned int nb_keyframes;
//        f.read((char *) &nb_keyframes, sizeof(nb_keyframes));
//        cerr << "reading " << nb_keyframes << " keyframe" << endl;
//        vector<KeyFrame *> kf_by_order;
//        for (unsigned int i = 0; i < nb_keyframes; i++) {
//            KeyFrame *kf = _ReadKeyFrame(f, voc, amp, &orb_ext);
//            AddKeyFrame(kf);
//            kf_by_order.push_back(kf);
//        }
//
//        map<unsigned long int, KeyFrame *> kf_by_id;
//        for (auto kf: mspKeyFrames)
//            kf_by_id[kf->mnId] = kf;
//
//        for (auto kf: kf_by_order) {
//            unsigned long int parent_id;
//            f.read((char *) &parent_id, sizeof(parent_id));
//            if (parent_id != ULONG_MAX)
//                kf->ChangeParent(kf_by_id[parent_id]);
//            unsigned long int nb_con;
//            f.read((char *) &nb_con, sizeof(nb_con));
//            for (unsigned long int i = 0; i < nb_con; i++) {
//                unsigned long int id;
//                int weight;
//                f.read((char *) &id, sizeof(id));
//                f.read((char *) &weight, sizeof(weight));
//                kf->AddConnection(kf_by_id[id], weight);
//            }
//        }
//        // MapPoints descriptors
//        for (auto mp: amp) {
//            mp->ComputeDistinctiveDescriptors();
//            mp->UpdateNormalAndDepth();
//        }
//
//#if 0
//        for(auto mp: mspMapPoints)
//            if (!(mp->mnId%100))
//                cerr << "mp " << mp->mnId << " " << mp->Observations() << " " << mp->isBad() << endl;
//#endif
//
//#if 0
//        for(auto kf: kf_by_order) {
//            cerr << "loaded keyframe id " << kf->mnId << " ts " << kf->mTimeStamp << " frameid " << kf->mnFrameId << " TrackReferenceForFrame " << kf->mnTrackReferenceForFrame << endl;
//            cerr << " parent " << kf->GetParent() << endl;
//            cerr << "children: ";
//            for(auto ch: kf->GetChilds())
//                cerr << " " << ch;
//            cerr <<endl;
//        }
//#endif
//        return true;
//    }
//
//    void Map::_WriteMapPoint(ofstream &f, MapPoint *mp) {
//        f.write((char *) &mp->mnId, sizeof(mp->mnId));               // id: long unsigned int
//        cv::Mat wp = semantic_slam::Converter::toCvMat(mp->GetWorldPos());// x,y,z
//        f.write((char *) &wp.at<float>(0), sizeof(float));           // pos x: float
//        f.write((char *) &wp.at<float>(1), sizeof(float));           // pos y: float
//        f.write((char *) &wp.at<float>(2), sizeof(float));           // pos z: float
//    }
//
//    void Map::_WriteKeyFrame(ofstream &f, KeyFrame *kf, map<MapPoint *, unsigned long int> &idx_of_mp) {
//        f.write((char *) &kf->mnId, sizeof(kf->mnId));                 // id: long unsigned int
//        f.write((char *) &kf->mTimeStamp, sizeof(kf->mTimeStamp));     // ts: TimeStamp, double
//
//#if 0
//        cerr << "writting keyframe id " << kf->mnId << " ts " << kf->mTimeStamp << " frameid " << kf->mnFrameId << " TrackReferenceForFrame " << kf->mnTrackReferenceForFrame << endl;
//        cerr << " parent " << kf->GetParent() << endl;
//        cerr << "children: ";
//        for(auto ch: kf->GetChilds())
//            cerr << " " << ch->mnId;
//        cerr <<endl;
//        cerr << kf->mnId << " connected: (" << kf->GetConnectedKeyFrames().size() << ") ";
//        for (auto ckf: kf->GetConnectedKeyFrames())
//            cerr << ckf->mnId << "," << kf->GetWeight(ckf) << " ";
//        cerr << endl;
//#endif
//
//        cv::Mat Tcw = semantic_slam::Converter::toCvMat(semantic_slam::Converter::toSE3Quat(kf->GetPose()));
//        f.write((char *) &Tcw.at<float>(0, 3), sizeof(float));          // px: float
//        f.write((char *) &Tcw.at<float>(1, 3), sizeof(float));          // py: float
//        f.write((char *) &Tcw.at<float>(2, 3), sizeof(float));          // pz: float
//        vector<float> Qcw = Converter::toQuaternion(Tcw.rowRange(0, 3).colRange(0, 3));
//        f.write((char *) &Qcw[0], sizeof(float));                      // qx: float
//        f.write((char *) &Qcw[1], sizeof(float));                      // qy: float
//        f.write((char *) &Qcw[2], sizeof(float));                      // qz: float
//        f.write((char *) &Qcw[3], sizeof(float));                      // qw: float
//
//        f.write((char *) &kf->N, sizeof(kf->N));                       //  nb_features: int
//        for (int i = 0; i < kf->N; i++) {
//            cv::KeyPoint kp = kf->mvKeys[i];
//            f.write((char *) &kp.pt.x, sizeof(kp.pt.x));               // float
//            f.write((char *) &kp.pt.y, sizeof(kp.pt.y));               // float
//            f.write((char *) &kp.size, sizeof(kp.size));               // float
//            f.write((char *) &kp.angle, sizeof(kp.angle));              // float
//            f.write((char *) &kp.response, sizeof(kp.response));           // float
//            f.write((char *) &kp.octave, sizeof(kp.octave));             // int
//            // 描述子
//            for (int j = 0; j < 32; j++)
//                f.write((char *) &kf->mDescriptors.at<unsigned char>(i, j), sizeof(char));
//
//            unsigned long int mpidx;
//            MapPoint *mp = kf->GetMapPoint(i);
//            if (mp == NULL) mpidx = ULONG_MAX;
//            else mpidx = idx_of_mp[mp];
//            f.write((char *) &mpidx, sizeof(mpidx));  // long int
//        }
//
//    }
//
//    bool Map::Save(const string &filename) {
//        cerr << "Map: Saving to " << filename << endl;
//        ofstream f;
//        f.open(filename.c_str(), ios_base::out | ios::binary);
//
//        cerr << "  writing " << mspMapPoints.size() << " mappoints" << endl;
//        unsigned long int nbMapPoints = mspMapPoints.size();
//        f.write((char *) &nbMapPoints, sizeof(nbMapPoints));
//
//        for (auto mp: mspMapPoints)
//            _WriteMapPoint(f, mp);
//
//        map<MapPoint *, unsigned long int> idx_of_mp;
//        unsigned long int i = 0;
//        for (auto mp: mspMapPoints) {
//            idx_of_mp[mp] = i;
//            i += 1;
//        }
//
//        cerr << "  writing " << mspKeyFrames.size() << " keyframes" << endl;
//        unsigned long int nbKeyFrames = mspKeyFrames.size();
//        f.write((char *) &nbKeyFrames, sizeof(nbKeyFrames));
//        for (auto kf: mspKeyFrames)
//            _WriteKeyFrame(f, kf, idx_of_mp);
//
//        // store tree and graph
//        for (auto kf: mspKeyFrames) {
//            KeyFrame *parent = kf->GetParent();
//            unsigned long int parent_id = ULONG_MAX;
//            if (parent) parent_id = parent->mnId;
//            f.write((char *) &parent_id, sizeof(parent_id));
//
//            unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
//            f.write((char *) &nb_con, sizeof(nb_con));
//            for (auto ckf: kf->GetConnectedKeyFrames()) {
//                int weight = kf->GetWeight(ckf);
//                f.write((char *) &ckf->mnId, sizeof(ckf->mnId));
//                f.write((char *) &weight, sizeof(weight));
//            }
//        }
//
//        f.close();
//        cerr << "Map: finished saving" << endl;
//        struct stat st;
//        stat(filename.c_str(), &st);
//        cerr << "Map: saved " << st.st_size << " bytes" << endl;
//
//#if 0
//        for(auto mp: mspMapPoints)
//    if (!(mp->mnId%100))
//      cerr << "mp " << mp->mnId << " " << mp->Observations() << " " << mp->isBad() << endl;
//#endif
//
//        return true;
//    }

} //namespace semantic_slam
