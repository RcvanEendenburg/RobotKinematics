#include <exception>

#include <world/ShapeFinderService.h>
#include <world/Shape.h>


#include "ShapeFinder/ShapeFinder.h"

#include "Sensor/Sensor.h"
#include "Sensor/CameraSensor.h"
#include "Sensor/StaticImage.h"
#include "Color/ColorFilter.h"
#include <utilities/IniParser.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "WorldService");

    ShapeFinder shapeFinder(std::move(std::make_unique<StaticImage>("/home/rene/G/RobotKinematics/src/world/TestImage/Blocks01.jpg")));

/*  Use this when parser can retrieve Stringformat
    Utilities::IniParser parser = Utilities::IniParser("/home/rene/G/RobotKinematics/src/world/config/config.ini");
    parser.parse();

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

