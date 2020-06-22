
#include "ColorCalibration.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
int ColorCalibration::slideColourHSV(int camid){
    //create windows
    cv::namedWindow(sliderWindowHSV,cv::WINDOW_NORMAL);
    cv::namedWindow(colourWindowHSV,cv::WINDOW_AUTOSIZE);

    createBars();

    //open cam
    cv::VideoCapture cap(camid);

    if (!cap.isOpened()) {
        return -1;
    }


    char key = 0;
    // press ESC to close the loop
    while (key != 27) {
        cv::Mat camframe, output, mask;
        cap >> camframe;

        applyHsvValues(camframe, output, mask);
        cv::imshow(colourWindowHSV, output);

        key = cv::waitKey(30);
    }


    //update once to show windows
    cv::waitKey(30);
    return 0;
}

void ColorCalibration::createBars()
{
    cv::createTrackbar("High_HUE",sliderWindowHSV,&high_hue,179);
    cv::createTrackbar("High_SATURATION",sliderWindowHSV,&high_saturation,slidermax);
    cv::createTrackbar("High_VALUE",sliderWindowHSV,&high_value,slidermax);

    cv::createTrackbar("low_HUE",sliderWindowHSV,&low_hue,179);
    cv::createTrackbar("low_SATURATION",sliderWindowHSV,&low_saturation,slidermax);
    cv::createTrackbar("low_VALUE",sliderWindowHSV,&low_value,slidermax);
}

void ColorCalibration::applyHsvValues(cv::Mat& input, cv::Mat& output, cv::Mat& mask_Output)
{
    cv::Mat mask, desta, dstb, picture_hsv;
    cv::cvtColor(input, picture_hsv, cv::COLOR_BGR2HSV);

    cv::inRange(picture_hsv, cv::Scalar(low_hue,low_saturation,low_value), cv::Scalar(high_hue,high_saturation,high_value), mask);
    mask_Output = mask;
    desta = cv::Scalar::all(0);
    picture_hsv.copyTo(dstb, mask);
    cv::cvtColor(dstb, output, cv::COLOR_HSV2BGR);
}



