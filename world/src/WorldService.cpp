#include <exception>

#include <world/ShapeFinderService.h>
#include <world/Shape.h>


#include "Shape/ShapeFinder.h"

#include "Sensor/Sensor.h"
#include "Sensor/CameraSensor.h"
#include "Sensor/StaticImage.h"
#include "Color/ColorFilter.h"
#include <utilities/IniParser.h>



int main(int argc, char **argv)
{
    ros::init(argc,argv, "WorldService");

  //  Utilities::IniParser parser = Utilities::IniParser("/home/rene/RobotsKinematica/src/world/config/config.ini");
  //  parser.parse();
    ShapeFinder shapeFinder(std::move(std::make_unique<StaticImage>("/home/rene/RobotsKinematica/src/world/TestImage/Blocks01.jpg")));

/* TODO ENABLE WHEN GET IS IMPLEMENTED
    if(parser.get<bool>("ImageSource","CameraEnabled"))
    {
        ShapeFinder shapeFinder(std::move(std::make_unique<CameraSensor>()));
    } else {
        ShapeFinder shapeFinder(std::move(std::make_unique<StaticImage>(parser.get<std::string>("ImageSource","ImagePath"))));
    }

*/
    ros::spin();

    return 0;
}

