//
// Created by rene on 17-06-20.
//

#ifndef SRC_COLOURSLIDERHSV_H
#define SRC_COLOURSLIDERHSV_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class ColorCalibration
{
public:
    int slideColourHSV();

private:
    void createBars();
    void applyHsvValues(cv::Mat& input, cv::Mat& output, cv::Mat& mask_Output);

    std::string sliderWindowHSV = "HSV-Sliders";
    std::string colourWindowHSV = "LiveCamfeed";

    const int slidermax = 255;

    int high_hue =0;
    int high_saturation = 0;
    int high_value = 0;

    int low_hue =0;
    int low_saturation = 0;
    int low_value = 0;

};



#endif //SRC_COLOURSLIDERHSV_H
