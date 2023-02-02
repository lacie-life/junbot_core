//
// Created by lacie on 18/01/2023.
//

#ifndef AG_MAPPING_YOLODETECTION_H
#define AG_MAPPING_YOLODETECTION_H

#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <utility>
#include <time.h>
#include "yolov5_detection.h"

typedef struct Object
{
    cv::Rect_<float> rect;  // frame
    float prob;             // Confidence
    std::string object_name;// object class name
    int class_id;           // category id
} Object;

using namespace std;

class YoloDetection {
public:
    YoloDetection(std::string modelPath, bool isTensorRT = true);
    ~YoloDetection();

    void GetImage(cv::Mat& RGB);
    void ClearImage();
    bool Detect();

    bool Detect(const cv::Mat& bgr_img, std::vector<Object>& objects);
    bool Detectv2(const cv::Mat& bgr_img, std::vector<Object>& objects);
    cv::Mat display(std::vector<Object>& objects);

    void ClearArea();

    vector<cv::Rect2i> mvPersonArea = {};
    vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh=0.5, float iou_thresh=0.5);

public:
    cv::Mat mRGB;
    torch::jit::script::Module mModule;
    std::vector<std::string> mClassnames;
    YoLoObjectDetection* mModel;

    // 6-28
    vector<string> mvDynamicNames;
    vector<cv::Rect2i> mvDynamicArea;
    map<string, vector<cv::Rect2i>> mmDetectMap;
    std::vector<Object> mvObject;
};


#endif //AG_MAPPING_YOLODETECTION_H

