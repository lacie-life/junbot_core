#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Atlas.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <unordered_set>


namespace semantic_slam
{

class Tracking;
class Viewer;
class Atlas;
class MapDrawer;

class FrameDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameDrawer(Atlas* pAtlas);
    FrameDrawer(Atlas* pAtlas, MapDrawer* pMapDrawer, const string &strSettingPath);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame(float imageScale=1.f);
    cv::Mat DrawRightFrame(float imageScale=1.f);
    void generatePC(void);

    bool both;

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    void FillImage(cv::Mat &im, const cv::Mat &mask, cv::Scalar color);

    // Info of the frame to be drawn
    cv::Mat mIm, mImRight;
    cv::Mat mDynMask;
    cv::Mat mImDep;
    cv::Mat mImRGB;
    cv::Mat mK;
    cv::Mat mTcw;

    int N;
    vector<cv::KeyPoint> mvCurrentKeys,mvCurrentKeysRight;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;
    std::vector<float> mvCurrentDepth;
    float mThDepth;

    Atlas* mpAtlas;
    MapDrawer* mpMapDrawer;

    std::mutex mMutex;
    vector<pair<cv::Point2f, cv::Point2f> > mvTracks;

    Frame mCurrentFrame;
    vector<MapPoint*> mvpLocalMap;
    vector<cv::KeyPoint> mvMatchedKeys;
    vector<MapPoint*> mvpMatchedMPs;
    vector<cv::KeyPoint> mvOutlierKeys;
    vector<MapPoint*> mvpOutlierMPs;

    map<long unsigned int, cv::Point2f> mmProjectPoints;
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

private:
    std::vector<cv::Scalar> colors = {  cv::Scalar(135,0,248),
                                        cv::Scalar(255,0,253),
                                        cv::Scalar(4,254,119),
                                        cv::Scalar(255,126,1),
                                        cv::Scalar(0,112,255),
                                        cv::Scalar(0,250,250),    };
    std::vector<std::string> class_names = {
                                        "person",
                                        "bicycle",
                                        "car",
                                        "motorbike",
                                        "aeroplane",
                                        "bus",
                                        "train",
                                        "truck",
                                        "boat",
                                        "traffic light",
                                        "fire hydrant",
                                        "stop sign",
                                        "parking meter",
                                        "bench",
                                        "bird",
                                        "cat",
                                        "dog",
                                        "horse",
                                        "sheep",
                                        "cow",
                                        "elephant",
                                        "bear",
                                        "zebra",
                                        "giraffe",
                                        "backpack",
                                        "umbrella",
                                        "handbag",
                                        "tie",
                                        "suitcase",
                                        "frisbee",
                                        "skis",
                                        "snowboard",
                                        "sports ball",
                                        "kite",
                                        "baseball bat",
                                        "baseball glove",
                                        "skateboard",
                                        "surfboard",
                                        "tennis racket",
                                        "bottle",
                                        "wine glass",
                                        "cup",
                                        "fork",
                                        "knife",
                                        "spoon",
                                        "bowl",
                                        "banana",
                                        "apple",
                                        "sandwich",
                                        "orange",
                                        "broccoli",
                                        "carrot",
                                        "hot dog",
                                        "pizza",
                                        "donut",
                                        "cake",
                                        "chair",
                                        "sofa",
                                        "pottedplant",
                                        "bed",
                                        "diningtable",
                                        "toilet",
                                        "tvmonitor",
                                        "laptop",
                                        "mouse",
                                        "remote",
                                        "keyboard",
                                        "cell phone",
                                        "microwave",
                                        "oven",
                                        "toaster",
                                        "sink",
                                        "refrigerator",
                                        "book",
                                        "clock",
                                        "vase",
                                        "scissors",
                                        "teddy bear",
                                        "hair drier",
                                        "toothbrush"
                                        };

public:
    // color image
    cv::Mat mQuadricIm;
    bool mbshow_yolo_result;
    cv::Mat GetQuadricImage();
    cv::Mat DrawQuadricImage();

    // bounding box.
    std::vector<BoxSE> Dboxes;
    cv::Mat DrawYoloInfo(cv::Mat &im, bool bText); 
    int CurFrameId = -1;

    // lines.
    std::vector< KeyLine> Dkeylines_raw_nouse, Dkeylines_out_nouse;
    double DTimeStamp_nouse;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >  DObjsLines;    // object lines.  std::vector<Eigen::MatrixXd>
};

} //namespace semantic_slam

#endif // FRAMEDRAWER_H
