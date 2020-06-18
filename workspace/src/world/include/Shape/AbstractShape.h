#ifndef SHAPE_H_
#define SHAPE_H_

#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <world/Shape.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <world/ShapeFinderService.h>
#include "Sensor/Sensor.h"
#include "Color/ColorFilter.h"
#include "Color/ColorRange.h"

class Shape
    {
    public:
    enum ShapeTypes{SQUARE, RECTANGLE, UNDEFINED, CIRCLE};

    Shape(int32_t id, ShapeTypes shape ,geometry_msgs::Point center, double width, float rotation) :
        id(id), shape(shape), center(center), width(width), rotation(rotation) {}


    world::Shape toShapeMessage() {
        world::Shape shapeMessage;
        shapeMessage.shape = shape;
        shapeMessage.points = center;
        shapeMessage.width = width;
        shapeMessage.rotation = rotation;
        return shapeMessage;
    }

    virtual ~Shape() = default;

    int32_t getId()
    {
        return id;
    }
    void setId(int32_t aId)
    {
        id = aId;
    }

    void translateCoordinate(cv::Point2d aOrigin)
    {
        center.x = center.x - aOrigin.x;
        center.z = center.z - aOrigin.y;
    }

    geometry_msgs::Point getCenter()
    {
        return center;
    }

protected:

    static cv::Point2d getCenter(std::vector<cv::Point> &contour) {
        cv::Moments mom = cv::moments(contour, false);
        cv::Point2d center(mom.m10/mom.m00, mom.m01/mom.m00);
        return center;
    }

private:
    int32_t id;
    ShapeTypes shape;
    geometry_msgs::Point center;
    double width;
    float rotation;
};



#endif