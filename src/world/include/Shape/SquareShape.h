//
// Created by rene on 6/2/19.
//

#ifndef WORLD_SQUARESHAPE_H
#define WORLD_SQUARESHAPE_H
#include "AbstractShape.h"
class Square : public Shape
{
public:
    Square(int32_t id, ShapeTypes shape, world::Point2d center, float rotation):
    Shape(id,shape,center,rotation)
    {

    };
private:
    friend class ShapeFinder;
    static std::shared_ptr<Square> isShape(std::vector<cv::Point> &approx, std::vector<cv::Point> &contours, double mincos, double maxcos)
    {
            cv::RotatedRect rotated = cv::minAreaRect(contours);

            cv::Point2f vertices[4];
            rotated.points(vertices);
            // TODO: Add the dimensions stuffs here!
            double length = cv::norm(vertices[0] - vertices[1]);//length of rotated box
            double width = cv::norm(vertices[1] - vertices[2]);//width of rotated box

        if (approx.size() == 4 && mincos >= -0.15 && maxcos <= 0.3 && std::abs(1 - ((length / width))) <= 0.15 )
        {
            auto cv = Shape::getCenter(contours);
            auto center = world::Point2d();
            center.x = cv.x;
            center.y = cv.y;
            return std::make_shared<Square>(0,ShapeTypes::SQUARE,center,rotated.angle);
        }

        return nullptr;
    };

    static std::string getName()
    {
        return "Square";
    }


};

#endif //WORLD_SQUARESHAPE_H
