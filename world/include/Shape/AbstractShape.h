#ifndef SHAPE_H_
#define SHAPE_H_

#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include <world/Point2d.h>
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

    Shape(int32_t id, ShapeTypes shape ,world::Point2d center, float rotation) : id(id), shape(shape), center(center), rotation(rotation) {}

        world::Shape toShapeMessage()
        {
            world::Shape shapeMessage;
            shapeMessage.shape = shape;
            shapeMessage.points = center;
            shapeMessage.rotation = rotation;
            return shapeMessage;
        }

        virtual ~Shape() = default;

        static cv::Point2d getCenter(std::vector<cv::Point> &contour) {
            cv::Moments mom = cv::moments(contour, false);
            cv::Point2d center(mom.m10/mom.m00, mom.m01/mom.m00);
            return center;
        }

        int32_t getId()
        {
            return id;
        }
        void setId(int32_t aId)
        {
            id = aId;
        }

private:

        int32_t id;
        ShapeTypes shape;
        world::Point2d center;
        float rotation;
};



#endif