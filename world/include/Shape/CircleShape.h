//
// Created by rene on 6/2/19.
//

#ifndef WORLD_CIRCLESHAPE_H
#define WORLD_CIRCLEHAPE_H
#include "AbstractShape.h"
class CircleShape : public Shape
{
public:
    CircleShape(int32_t id, ShapeTypes shape, world::Point2d center, float rotation):
    Shape(id,shape,center,rotation)
    {

    };
private:
    friend class ShapeFinder;
    static std::shared_ptr<CircleShape> isShape(std::vector<cv::Point> &approx, std::vector<cv::Point> &contours, double mincos, double maxcos)
    {
        cv::Rect r = cv::boundingRect(contours[i]);
        //if the contour has almost the same height as width and the contour area is almost equal to a perfect circle than I assume it is a circle
        if (aShape == CIRCLE && std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&   std::abs(1 - (cv::contourArea(contours[i]) / (CV_PI * std::pow(r.width / 2, 2)))) <= 0.1)
        {
            auto cv = Shape::getCenter(contours);
            auto center = world::Point2d();
            center.x = cv.x;
            center.y = cv.y;
            return std::make_shared<RectShape>(0,ShapeTypes::CIRCLE,center,0);
        }

        return nullptr;
    };

    static std::string getName()
    {
        return "Circle";
    }
};

#endif //WORLD_CIRCLESHAPE_H
