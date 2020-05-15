//
// Created by rene on 04-05-20.
//

#include "Calibration/ArucoCalibration.h"
#include "Calibration/CalibrationException.h"

#include <utilities/IniParser.h>
#include <utilities/Logger.h>
#include <utilities/LogToCout.h>

#include <algorithm>

void ArucoCalibration::Calibrate(cv::Mat img, int markerSize, int arucoId)
{
    // retrieve aruco size from config file.
    auto& logger = Utilities::Logger::instance();
    logger.setLogOutput(std::make_unique<Utilities::LogToCout>());
    logger.log(Utilities::LogLevel::Debug, "Calibrating coordinate system and camera with Aruco");

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(img, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250), corners, ids);

    std::vector<int>::iterator p;
    p = std::find(ids.begin(), ids.end(), arucoId);

    logger.log(Utilities::LogLevel::Debug, "Amount of markers found: %d",ids.size());

    if(p != ids.end())
    {
        logger.log(Utilities::LogLevel::Debug, "Found aruco marker: %d",arucoId);
        setMarker(corners.at(arucoId), markerSize);
    }
    else if (ids.size() > 0)
    {
        logger.log(Utilities::LogLevel::Warning, "Did not found aruco marker: %d "
                   "\n Falling back to first available marker: %d",
                   arucoId, ids.at(0));
        setMarker(corners.at(0), markerSize);
    }
    else
    {
        markerLocation = cv::Point2d(20,30);
        pixelToMMRatio = 1.2;
        logger.log(Utilities::LogLevel::Error, " No markers found!!");
        logger.log(Utilities::LogLevel::Error, " Continue with debug values");

        // throw new CalibrationException("No Aruco marker found!");
    }
}

void ArucoCalibration::setMarker(std::vector<cv::Point2f> c, int markerSize)
{
    float centX = (c.at(0).x + c.at(2).x) /2;
    float centY = (c.at(0).y + c.at(2).y) /2;
    double length = cv::norm(c.at(0) - c.at(1));

    markerLocation = cv::Point2d(centX, centY);
    pixelToMMRatio = markerSize / length;

    isCalibrated = true;
}
