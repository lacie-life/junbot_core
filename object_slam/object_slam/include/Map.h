#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "Atlas.h"
#include "MapCuboidObject.h"
#include "Object.h"
#include "ObjectDatabase.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>

class ObjectDatabase;

namespace semantic_slam
{

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;
class ORBextractor;
class Converter;
class Frame;
class MapCuboidObject;
class Object_Map;

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        //ar & mspKeyFrames;
        //ar & mspMapPoints;
        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel=false);

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera*> &mpCams);

//    bool Save(const std::string &filename);
//    bool Load(const std::string &filename, ORBVocabulary &voc);

    void printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder);

    vector<KeyFrame*> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:

//    void _WriteMapPoint(ofstream &f, MapPoint* mp);
//    void _WriteKeyFrame(ofstream &f, KeyFrame* kf,
//                        map<MapPoint*, unsigned long int>& idx_of_mp);
//
//    MapPoint* _ReadMapPoint(ifstream &f);
//    KeyFrame* _ReadKeyFrame(ifstream &f,
//                            ORBVocabulary &voc,
//                            std::vector<MapPoint*> amp,
//                            ORBextractor* ex);

    long unsigned int mnId;

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    // Mutex
    std::mutex mMutexMap;

    Converter convert;

protected:
    // std::vector<Object_Map*> mvObjectMap;
    // std::set<Object_Map*> mvObjectMap;
    ObjectDatabase* mpOD;

public:
    void AddObject(Object_Map *pObj);
    std::vector<Object_Map*> GetObjects();

// For 3D Cuboid testing (optimize)
public:
    int img_width, img_height;
    Eigen::Matrix3f Kalib_f, invKalib_f;
    Eigen::Matrix3d Kalib, invKalib;

    void AddMapObject(MapCuboidObject *pMO);
    void EraseMapObject(MapCuboidObject *pMO);
    std::vector<MapCuboidObject *> GetAllMapObjects(); // not sequential....
    std::vector<MapCuboidObject *> GetGoodMapObjects();
    long unsigned int MapObjectsInMap(); // get number of objects.

protected:
    std::set<MapCuboidObject *> mspMapObjects;
};

} //namespace semantic_slam

#endif // MAP_H
