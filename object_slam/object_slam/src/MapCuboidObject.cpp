//
// Created by lacie on 18/02/2023.
//

#include "MapCuboidObject.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include "Converter.h"
#include <mutex>

namespace semantic_slam
{
    using namespace std;
    long int MapCuboidObject::nNextId = 0;
    mutex MapCuboidObject::mGlobalMutex;

    MapCuboidObject::MapCuboidObject(Map *pMap, bool update_index) : mbBad(false), mpMap(pMap)
    {
        if (update_index)
            mnId = nNextId++;
        else
            mnId = -1;
        nObs = 0;
        moRefKF = nullptr;
        mLatestKF = nullptr;

        largest_point_observations = 0;
        pointOwnedThreshold = 0;
        already_associated = false;
        become_candidate = false;

        association_refid_in_tracking = -1;
        associated_landmark = nullptr;

        object_id_in_localKF = -1;
        left_right_to_car = -1;

        isGood = false;

        truth_tracklet_id = -1;

        velocityTwist.setZero();
        velocityPlanar.setZero();
    }

    long int MapCuboidObject::getIncrementedIndex()
    {
        nNextId++;
        return nNextId;
    }

    vector<MapPoint *> MapCuboidObject::GetUniqueMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return vector<MapPoint *>(mappoints_unique_own.begin(), mappoints_unique_own.end());
    }

    int MapCuboidObject::NumUniqueMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mappoints_unique_own.size();
    }

    void MapCuboidObject::AddUniqueMapPoint(MapPoint *pMP, int obs_num)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mappoints_unique_own.insert(pMP);
        if (obs_num > largest_point_observations) // exact way.
            largest_point_observations = obs_num;
    }

    void MapCuboidObject::EraseUniqueMapPoint(MapPoint *pMP, int obs_num)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mappoints_unique_own.erase(pMP);
        if (obs_num == largest_point_observations)
            largest_point_observations -= 1; // HACK only an approximate way. otherwise need to keep sorted the observations (heap). complicated, not worth it.
    }

    vector<MapPoint *> MapCuboidObject::GetPotentialMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return vector<MapPoint *>(mappoints_potential_own.begin(), mappoints_potential_own.end());
    }

    Map *MapCuboidObject::GetMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void MapCuboidObject::UpdateMap(Map *pMap) {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }

    void MapCuboidObject::AddPotentialMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mappoints_potential_own.insert(pMP);
    }

    bool MapCuboidObject::check_whether_valid_object(int own_point_thre)
    {
//        std::cout << (int)mappoints_potential_own.size() << std::endl;
        if ((int)mappoints_potential_own.size() > own_point_thre)
            become_candidate = true;
        else
            become_candidate = false;
        return become_candidate;
    }

    void MapCuboidObject::SetAsLandmark()
    {
        for (set<MapPoint *>::iterator mit = mappoints_potential_own.begin(); mit != mappoints_potential_own.end(); mit++)
        {
            (*mit)->AddObjectObservation(this); // add into actual object landmark observation. make sure this->already_associated=true;
        }
    }

    void MapCuboidObject::MergeIntoLandmark(MapCuboidObject *otherLocalObject)
    {
        //NOTE other object is just local. not add to actual point-object observation yet. therefore no need to delete object-point observation.
        for (set<MapPoint *>::iterator mit = otherLocalObject->mappoints_potential_own.begin(); mit != otherLocalObject->mappoints_potential_own.end(); mit++)
        {
            (*mit)->AddObjectObservation(this);
        }
    }

    void MapCuboidObject::addObservation(KeyFrame *pKF, size_t idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mLatestKF == nullptr)
            mLatestKF = pKF;
        if (pKF->mnId >= mLatestKF->mnId)
            mLatestKF = pKF;

        if (mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;
        nObs++;

        observed_frames.push_back(pKF);
    }

    void MapCuboidObject::EraseObservation(KeyFrame *pKF)
    {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mObservations.count(pKF))
            {
                mObservations.erase(pKF);

                if (moRefKF == pKF) //update reference keyframe if possible
                {
                    long unsigned int min_kf_id = mLatestKF->mnId + 1;
                    KeyFrame *smallest_kf = nullptr;
                    for (unordered_map<KeyFrame *, size_t>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
                        if (mit->first->mnId < min_kf_id)
                            smallest_kf = mit->first;
                    moRefKF = smallest_kf;
                }
                if (mLatestKF == pKF) //update latest KF if possible
                {
                    observed_frames.pop_back();
                    mLatestKF = observed_frames.back();
                }

                nObs--;
                if (nObs <= 0)
                    bBad = true;
            }
        }

        if (bBad)
            SetBadFlag();
    }

    unordered_map<KeyFrame *, size_t> MapCuboidObject::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int MapCuboidObject::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    std::vector<KeyFrame *> MapCuboidObject::GetObserveFrames()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        vector<KeyFrame *> res;
        for (unordered_map<KeyFrame *, size_t>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
        {
            res.push_back(mit->first);
        }
        return res;
    }

    std::vector<KeyFrame *> MapCuboidObject::GetObserveFramesSequential()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return observed_frames;
    }

    bool MapCuboidObject::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    int MapCuboidObject::GetIndexInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    void MapCuboidObject::SetBadFlag()
    {
        unordered_map<KeyFrame *, size_t> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;   // fully copy the contents, similar to vector
            mObservations.clear(); // remove observations from point side
            observed_frames.clear();
        }
        for (unordered_map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;			   // NOTE when use obj_landmark in association and BA, check if valid.
            pKF->EraseMapObjectMatch(mit->second); // remove observations from frame side
        }

        mpMap->EraseMapObject(this);
    }

    bool MapCuboidObject::isBad()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }

    KeyFrame *MapCuboidObject::GetReferenceKeyFrame()
    {
        return moRefKF;
    }

    KeyFrame *MapCuboidObject::GetLatestKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mLatestKF;
    }

    void MapCuboidObject::SetReferenceKeyFrame(KeyFrame *refkf)
    {
        moRefKF = refkf;
    }

    void MapCuboidObject::SetWorldPos(const g2o::cuboid &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        pose_Twc = Pos;
        pose_Tcw = pose_Twc;
        pose_Tcw.pose = pose_Twc.pose.inverse();
    }

    g2o::cuboid MapCuboidObject::GetWorldPos()
    {
        unique_lock<mutex> lock(mMutexPos);
        if (is_dynamic)
            return pose_Twc_latestKF;
        return pose_Twc;
    }

    g2o::SE3Quat MapCuboidObject::GetWorldPosInv()
    {
        unique_lock<mutex> lock(mMutexPos);
        return pose_Tcw.pose;
    }

    g2o::cuboid MapCuboidObject::GetWorldPosBA()
    {
        unique_lock<mutex> lock(mMutexPos);
        return pose_Twc_afterba;
    }
}