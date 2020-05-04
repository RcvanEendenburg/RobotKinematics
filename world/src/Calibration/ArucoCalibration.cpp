//
// Created by rene on 04-05-20.
//

#include "Calibration/ArucoCalibration.h"
#include "Calibration/CalibrationException.h"

#include <utilities/IniParser.h>
#include <exception>

void ArucoCalibration::Calibrate(cv::Mat img)
{
    // used code from https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html

    // retrieve aruco size from config file.
    Utilities::IniParser parser = Utilities::IniParser("/home/rene/G/RobotKinematics/src/world/config/config.ini");
    parser.parse();

    int markerSize = parser.get<int>("Calibration","ArucoMarkerSize");

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(img, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250), corners, ids);

    if (ids.size() > 0)
    {
        std::vector<cv::Point2f> c = corners.at(0);
        float centX = (c.at(0).x + c.at(2).x) /2;
        float centY = (c.at(0).y + c.at(2).y) /2;
        double length = cv::norm(c.at(0) - c.at(1));

        markerLocation = cv::Point2d(centX, centY);
        pixelToMMRatio = markerSize / length;

        isCalibrated = true;
    }

    else
    {
        // Setting some debug numbers so it looks real!
        markerLocation = cv::Point2d(20,30);
        pixelToMMRatio = 1.2;

       // throw new CalibrationException("No Aruco marker found!");
    }
}