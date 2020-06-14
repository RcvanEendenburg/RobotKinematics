//
// Created by rene on 04-05-20.
//

#ifndef WORLD_ARUCOCALIBRATION_H
#define WORLD_ARUCOCALIBRATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class ArucoCalibration
{
public:
    void Calibrate(cv::Mat img, int markerSize, int arucoId, int ArucoDistanceToBase);

    cv::Point2d GetMarkerLocation()
    {
        return markerLocation;
    };
    float PixelToMM(float pixels)
    {
        return pixelToMMRatio * pixels;
    };

    bool IsCalibrated()
    {
        return isCalibrated;
    }

private:
    void setMarker(std::vector<cv::Point2f> c, int markerSize, int ArucoDistanceToBase);

    float pixelToMMRatio;
    bool isCalibrated = false;
    cv::Point2d markerLocation;
};
#endif //WORLD_ARUCOCALIBRATION_H
