//
// Created by rene on 04-05-20.
//

#include "Calibration/ArucoCalibration.h"
#include "Calibration/CalibrationException.h"

#include <utilities/IniParser.h>
#include <utilities/Logger.h>
#include <utilities/LogToCout.h>

#include <algorithm>

void ArucoCalibration::Calibrate(cv::Mat img, int markerSize, int arucoId, int ArucoDistanceToBase)
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
        setMarker(corners.at(std::distance(ids.begin(), p)), markerSize, ArucoDistanceToBase);
    }
    else if (!ids.empty())
    {
        logger.log(Utilities::LogLevel::Warning, "Did not found aruco marker: %d "
                   "\n Falling back to first available marker: %d",
                   arucoId, ids.at(0));

        //TODO: remove the magic
        switch(ids.at(0))
        {
            case 1:
                setMarker(corners.at(0), markerSize, ArucoDistanceToBase, 60);
            case 3:
                setMarker(corners.at(0), markerSize, ArucoDistanceToBase, -60);
        }
    }
    else
    {
        throw CalibrationException("No Aruco marker found!");
    }

    logger.log(Utilities::LogLevel::Debug, "pixel to MM ratio set at %f", pixelToMMRatio);

}

void ArucoCalibration::setMarker(std::vector<cv::Point2f> c, int markerSize, int ArucoDistanceToBase, int DistanceToCenter)
{
    float centX = (c.at(0).x + c.at(2).x) /2;
    float centY = (c.at(0).y + c.at(2).y) /2;
    double length = cv::norm(c.at(0) - c.at(1));

    markerLocation = cv::Point2d(centX + DistanceToCenter, centY - ArucoDistanceToBase);
    pixelToMMRatio = markerSize / length;
    isCalibrated = true;
}
