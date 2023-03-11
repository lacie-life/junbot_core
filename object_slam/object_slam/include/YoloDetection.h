//
// Created by lacie on 18/01/2023.
//

#ifndef AG_MAPPING_YOLODETECTION_H
#define AG_MAPPING_YOLODETECTION_H

// #include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <utility>
#include <time.h>
#include "yolov5_detection.h"

class BoxSE : public cv::Rect
{
    public:
	int m_class = -1;			// class id.
	float m_score = 0.0F;		// probability.
	std::string m_class_name;	// class name.

	BoxSE()
	{
		m_class_name = "Unknown";
	}

	BoxSE(int c, float s, int _x, int _y, int _w, int _h, std::string name = "")
		:m_class(c), m_score(s)
	{
		this->x = _x;
		this->y = _y;
		this->width = _w;
		this->height = _h;
		char const *lb[5] = { "th","st","nd","rd","th" };

		if (name.length() == 0)
		{
			m_class_name = std::to_string(m_class) + lb[m_class < 4 ? m_class : 4] + " class";
		}
	}
};

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
    
    // For 3d Cuboid
    bool Detectv3(const cv::Mat& bgr_img, std::vector<BoxSE>& objects);

    cv::Mat display(std::vector<Object>& objects);

    void ClearArea();

    vector<cv::Rect2i> mvPersonArea = {};
    // vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh=0.5, float iou_thresh=0.5);

public:
    cv::Mat mRGB;
    // torch::jit::script::Module mModule;
    std::vector<std::string> mClassnames;
    YoLoObjectDetection* mModel;

    // 6-28
    vector<string> mvDynamicNames;
    vector<cv::Rect2i> mvDynamicArea;
    map<string, vector<cv::Rect2i>> mmDetectMap;
    std::vector<Object> mvObject;
};


#endif //AG_MAPPING_YOLODETECTION_H

