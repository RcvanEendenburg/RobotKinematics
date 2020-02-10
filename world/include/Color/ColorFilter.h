#pragma once
#ifndef COLORFILTER_H_
#define COLORFILTER_H_

#include <opencv2/opencv.hpp>

class ColorFilter {
public:
    enum color {
        ALL, YELLOW, RED, GREEN, BLUE, BLACK, WHITE, WOOD
    };

    static void filterOnColor(cv::Mat &input, cv::Mat &output, cv::Mat &mask_Output, color aColour);
};
#endif
