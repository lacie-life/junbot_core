#ifndef DRAWING_UTILS_HPP
#define DRAWING_UTILS_HPP

#include <math.h>

#include <opencv2/opencv.hpp>

float const id_colors[5][3] = {
    { 232.0f, 176.0f, 59.0f},
    { 175.0f, 208.0f, 25.0f},
    { 102.0f, 205.0f, 105.0f},
    { 185.0f, 0.0f, 255.0f},
    { 99.0f, 107.0f, 252.0f}
};

inline cv::Scalar generateColorID_u(int idx) {
    if (idx < 0) return cv::Scalar(236, 184, 36, 255);
    int color_idx = idx % 5;
    return cv::Scalar(id_colors[color_idx][0], id_colors[color_idx][1], id_colors[color_idx][2], 255);
}


float const class_colors[6][3] = {
    { 44.0f, 117.0f, 255.0f}, // PEOPLE
    { 255.0f, 0.0f, 255.0f}, // VEHICLE
    { 0.0f, 0.0f, 255.0f},
    { 0.0f, 255.0f, 255.0f},
    { 0.0f, 255.0f, 0.0f},
    { 255.0f, 255.0f, 255.0f}
};


template<typename T>
inline uchar _applyFading(T val, float current_alpha, double current_clr) {
    return static_cast<uchar> (current_alpha * current_clr + (1.0 - current_alpha) * val);
}

inline cv::Vec4b applyFading(cv::Scalar val, float current_alpha, cv::Scalar current_clr) {
    cv::Vec4b out;
    out[0] = _applyFading(val.val[0], current_alpha, current_clr.val[0]);
    out[1] = _applyFading(val.val[1], current_alpha, current_clr.val[1]);
    out[2] = _applyFading(val.val[2], current_alpha, current_clr.val[2]);
    out[3] = 255;
    return out;
}

inline void drawVerticalLine(
        cv::Mat &left_display,
        cv::Point2i start_pt,
        cv::Point2i end_pt,
        cv::Scalar clr,
        int thickness) {
    int n_steps = 7;
    cv::Point2i pt1, pt4;
    pt1.x = ((n_steps - 1) * start_pt.x + end_pt.x) / n_steps;
    pt1.y = ((n_steps - 1) * start_pt.y + end_pt.y) / n_steps;

    pt4.x = (start_pt.x + (n_steps - 1) * end_pt.x) / n_steps;
    pt4.y = (start_pt.y + (n_steps - 1) * end_pt.y) / n_steps;

    cv::line(left_display, start_pt, pt1, clr, thickness);
    cv::line(left_display, pt4, end_pt, clr, thickness);
}

#endif
