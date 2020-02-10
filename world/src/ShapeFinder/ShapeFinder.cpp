

#include "Shape/ShapeFinder.h"
#include "Shape/SquareShape.h"
#include "Shape/RectShape.h"
#include "Shape/AbstractShape.h"

ShapeFinder::ShapeFinder(std::unique_ptr<Sensor> sensor) : mySensor(std::move(sensor)), n()
{
    service = n.advertiseService("ShapeFinderService" , &ShapeFinder::handleRequest, this);
}


bool ShapeFinder::handleRequest(world::ShapeFinderService::Request &req, world::ShapeFinderService::Response &res)
{
  try {
    res.foundShapes = ServiceCallback(static_cast<Shape::ShapeTypes>(req.shape),
        static_cast<ColorFilter::color>(req.color));
  } catch(std::exception& e)
  {
      ROS_ERROR_STREAM(e.what());
      return false;
  }
  return true;
}

std::vector<world::Shape> ShapeFinder::ServiceCallback(Shape::ShapeTypes shape, ColorFilter::color color) {

    std::vector<world::Shape> response;
    cv::Mat img, colorFiltered, ColorMask, markedImg;
    img = getFilteredImage();
    img.copyTo(markedImg);
    cv::cvtColor(markedImg, markedImg, cv::COLOR_HSV2BGR);
    ColorFilter::filterOnColor(img, colorFiltered, ColorMask, color);

    std::vector<std::shared_ptr<Shape>> shapes;


    switch(shape) {
        case Shape::ShapeTypes::SQUARE:
        {
            std::vector<std::shared_ptr<Square>> shapes;
            shapes = FindShapes<Square>(colorFiltered, markedImg, ColorMask);
            for(auto shape: shapes)
            {
                response.push_back(shape->toShapeMessage());
            }

            break;
        }
        case Shape::ShapeTypes::RECTANGLE:
        {
            std::vector<std::shared_ptr<RectShape>> shapes;
            shapes = FindShapes<RectShape>(colorFiltered, markedImg, ColorMask);
            for(auto shape: shapes)
            {
                response.push_back(shape->toShapeMessage());
            }
            break;
        }
    }
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    imshow( "Display window", markedImg );
    cv::waitKey(0);
    return response;
}



cv::Mat ShapeFinder::getFilteredImage()
{

    cv::Mat input_grey, dst, input, picture_hsv, picture_blur;
    input = mySensor->getFrame();

    cv::cvtColor(input, picture_hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(picture_hsv, picture_blur, cv::Size(3,3),0,0);

    return picture_blur;
}

template <typename T>
std::vector<std::shared_ptr<T>> ShapeFinder::FindShapes(cv::Mat &input, cv::Mat &markedImage, cv::Mat &Mask) {

    std::vector<std::shared_ptr<T>> shapes;
    cv::Mat filteredImage = PrepareImgForContourMatching(input, Mask);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    //finding all contours in the image

    cv::findContours( Mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    for( unsigned int i = 0; i< contours.size(); i=hierarchy[i][0] ) // iterate through each contour.
    {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.03, true);


        if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx)) // if area too small, skip.
        {
            continue;
        }

        std::vector<double> cos;
        for (unsigned int j = 2; j < approx.size()+1; j++)
        {
            cos.push_back(angle(approx[j%approx.size()], approx[j-2], approx[j-1])); // Calculate angles between found points
        }

        std::sort(cos.begin(), cos.end());

        double mincos = cos.front();
        double maxcos = cos.back();
        
        auto shape = find<T>(approx, contours[i],mincos,maxcos);
        if(shape != nullptr)
         {
            shape->setId(shapes.size());
            shapes.push_back(shape);
            setLabel(markedImage, "Id: "  + std::to_string(shape->getId()), contours[i]);
         }
    }
    return shapes;
}

cv::Mat ShapeFinder::PrepareImgForContourMatching(cv::Mat &input, cv::Mat &Mask) {

    cv::Mat input_grey, dst;
    cv::cvtColor(input, input_grey, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(input_grey,input_grey,cv::Size(7,5),0);
    adaptiveThreshold(input_grey, input_grey,255,CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,151,10);
    dst = input_grey & Mask;
    return dst;
}


double ShapeFinder::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void ShapeFinder::setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
    int fontface = cv::FONT_HERSHEY_COMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Rect r = cv::boundingRect(contour);

    cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
    cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
    cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}
