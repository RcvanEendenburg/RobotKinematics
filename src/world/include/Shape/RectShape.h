//
// Created by rene on 6/2/19.
//

#ifndef WORLD_RECTSHAPE_H
#define WORLD_RECTSHAPE_H
#include "AbstractShape.h"
class RectShape : public Shape
{
public:
    RectShape(int32_t id, ShapeTypes shape, world::Point2d center, float rotation):
    Shape(id,shape,center,rotation)
    {

    };
private:
    friend class ShapeFinder;
    static std::shared_ptr<RectShape> isShape(std::vector<cv::Point> &approx, std::vector<cv::Point> &contours, double mincos, double maxcos)
    {
            cv::RotatedRect rotated = cv::minAreaRect(contours);

            cv::Point2f vertices[4];
            rotated.points(vertices);

            double length = cv::norm(vertices[0] - vertices[1]);//length of rotated box
            double width = cv::norm(vertices[1] - vertices[2]);//width of rotated box

        if (approx.size() == 4 && mincos >= -0.15 && maxcos <= 0.3 && std::abs(1 - ((length / width))) >= 0.15  )
        {
            auto cv = Shape::getCenter(contours);
            auto center = world::Point2d();
            center.x = cv.x;
            center.y = cv.y;
            return std::make_shared<RectShape>(0,ShapeTypes::RECTANGLE,center,rotated.angle);
        }

        return nullptr;
    };

    static std::string getName()
    {
        return "Rect";
    }


};

#endif //WORLD_SQUARESHAPE_H
