#pragma once
#ifndef SHAPEFINDER_HPP_
#define SHAPEFINDER_HPP_


#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <world/Shape.h>
#include "../Shape/AbstractShape.h"

#include "Sensor/Sensor.h"
#include "Color/ColorFilter.h"
#include "Color/ColorRange.h"
#include "Calibration/ArucoCalibration.h"

#include <ros/ros.h>
#include <world/ShapeFinderService.h>

#include <utilities/IniParser.h>

    class ShapeFinder{
    public:
        std::vector<world::Shape> ServiceCallback(Shape::ShapeTypes shape, ColorFilter::color color);

        ShapeFinder(std::unique_ptr<Sensor> sensor, Utilities::IniParser& anIniParser);

      bool handleRequest(world::ShapeFinderService::Request &req, world::ShapeFinderService::Response &res);

      template <typename T>
      std::shared_ptr<T> find(std::vector<cv::Point> &approx, std::vector<cv::Point> &contours, double mincos, double maxcos)
      {
        return T::isShape(approx, contours, mincos, maxcos);
      }

    private:
        template <typename T>
        std::vector<std::shared_ptr<T>> FindShapes(cv::Mat &input, cv::Mat &output, cv::Mat &Mask);
        void convertPixelToMM(std::vector<world::Shape>& foundShapes, ArucoCalibration* calibration);
        cv::Mat filterImage(cv::Mat input);
        std::unique_ptr<Sensor> mySensor;
        cv::Mat PrepareImgForContourMatching(cv::Mat &input, cv::Mat &Mask);
        double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
        void setLabel(cv::Mat& im, const std::string& label, std::vector<cv::Point>& contour);
        ros::NodeHandle n;
        ros::ServiceServer service;
        Utilities::IniParser& iniParser;
    };
#endif 
