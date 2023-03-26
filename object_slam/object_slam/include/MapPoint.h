#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"

#include "SerializationUtils.h"

#include <opencv2/core/core.hpp>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace semantic_slam
{

class KeyFrame;
class Map;
class Frame;
class MapCuboidObject;

class MapPoint
{

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;
        // Variables used by the tracking
        //ar & mTrackProjX;
        //ar & mTrackProjY;
        //ar & mTrackDepth;
        //ar & mTrackDepthR;
        //ar & mTrackProjXR;
        //ar & mTrackProjYR;
        //ar & mbTrackInView;
        //ar & mbTrackInViewR;
        //ar & mnTrackScaleLevel;
        //ar & mnTrackScaleLevelR;
        //ar & mTrackViewCos;
        //ar & mTrackViewCosR;
        //ar & mnTrackReferenceForFrame;
        //ar & mnLastFrameSeen;

        // Variables used by local mapping
        //ar & mnBALocalForKF;
        //ar & mnFuseCandidateForKF;

        // Variables used by loop closing and merging
        //ar & mnLoopPointForKF;
        //ar & mnCorrectedByKF;
        //ar & mnCorrectedReference;
        //serializeMatrix(ar,mPosGBA,version);
        //ar & mnBAGlobalForKF;
        //ar & mnBALocalForMerge;
        //serializeMatrix(ar,mPosMerge,version);
        //serializeMatrix(ar,mNormalVectorMerge,version);

        // Protected variables
        ar & boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
        ar & boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
        //ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
        //ar & mObservations;
        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar,mDescriptor,version);
        ar & mBackupRefKFId;
        //ar & mnVisible;
        //ar & mnFound;

        ar & mbBad;
        ar & mBackupReplacedId;

        ar & mfMinDistance;
        ar & mfMaxDistance;

    }


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    MapPoint();

    MapPoint(const cv::Mat &Pos, int FirstKFid, int FirstFrame, Map* pMap);
    MapPoint(const Eigen::Vector3f &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap);
    MapPoint(const Eigen::Vector3f &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const Eigen::Vector3f &Pos);
    Eigen::Vector3f GetWorldPos();

    Eigen::Vector3f GetNormal();
    void SetNormalVector(const Eigen::Vector3f& normal);

    KeyFrame* GetReferenceKeyFrame();
    void SetReferenceKeyFrame(KeyFrame* pRefKF);

    std::map<KeyFrame*,std::tuple<int,int>> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,int idx);
    void EraseObservation(KeyFrame* pKF);

    std::tuple<int,int> GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);
    int PredictScale(const float &currentDist, const float &logScaleFactor);

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void PrintObservations();

    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP);
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    Eigen::Vector3f mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    Eigen::Vector3f mPosMerge;
    Eigen::Vector3f mNormalVectorMerge;


    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;
    KeyFrame* mpHostKF;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

protected:    

    // Position in absolute coordinates
    Eigen::Vector3f mWorldPos;

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame*,std::tuple<int,int> > mObservations;
    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupObservationsId1;
    std::map<long unsigned int, int> mBackupObservationsId2;

    // Mean viewing direction
    Eigen::Vector3f mNormalVector;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;

    // Reference KeyFrame
    KeyFrame* mpRefKF;
    long unsigned int mBackupRefKFId;

    // Tracking counters
    int mnVisible;
    int mnFound;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    MapPoint* mpReplaced;
    // For save relation without pointer, this is necessary for save/load function
    long long int mBackupReplacedId;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    Map* mpMap;

    // Mutex
    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;

public:
    // bool have_feature;  
    // cv::KeyPoint feature;
    // bool object_view = false;
    int object_mnId;
    int object_class;
    std::map<int, int> viewdCount_forObjectId;   
    cv::KeyPoint feature_uvCoordinate;    //record the coordinates of each point in the xy(uv) directions  
    // set<int> frame_id;
    // bool First_obj_view;

    // For 3D cuboid testing (optimize)
    Eigen::Vector3f GetWorldPosVec();
    cv::Mat GetWorldPosBA();
    void AddObjectObservation(MapCuboidObject *obj); //called by AddObservation
    void EraseObjectObservation(MapCuboidObject *obj);
    void FindBestObject(); //find which object observes this point most

    int GetBelongedObject(MapCuboidObject *&obj); // change obj, return observation times.
    MapCuboidObject *GetBelongedObject();         //return obj

    // for dynamic stuff
    bool is_dynamic = false;
    cv::Mat mWorldPos_latestKF;
    cv::Mat mWorldPos_latestFrame;
    cv::Mat PosToObj; // 3d point relative to object center. ideally will converge/fix after BA

    bool is_triangulated = false; //whether this point is triangulated or depth inited?
    bool is_optimized = false;

    MapCuboidObject *best_object;                        // one point can only belong to at most one object
    int max_object_vote;                           // sometimes point is wrongly associated to an object. need more frame observation
    std::set<MapCuboidObject *> LocalObjObservations;    // observed by local objects which hasn't become landmark at that time
    std::map<MapCuboidObject *, int> MapObjObservations; //object and observe times.
    std::mutex mMutexObject;

    bool already_bundled;

    bool ground_fitted_point = false;
    long unsigned int mnGroundFittingForKF;

    int record_txtrow_id = -1; // when finally record to txt, row id in txt

};

} //namespace semantic_slam

#endif // MAPPOINT_H
