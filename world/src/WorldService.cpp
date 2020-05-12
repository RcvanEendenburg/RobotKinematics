#include <exception>

#include <world/ShapeFinderService.h>
#include <world/Shape.h>


#include "ShapeFinder/ShapeFinder.h"

#include "Sensor/Sensor.h"
#include "Sensor/CameraSensor.h"
#include "Sensor/StaticImage.h"
#include "Color/ColorFilter.h"
#include <utilities/IniParser.h>
#include <utilities/Logger.h>
#include <utilities/LogToCout.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "WorldService");

    //ShapeFinder shapeFinder(std::move(std::make_unique<StaticImage>("/home/rene/G/RobotKinematics/src/world/TestImage/Blocks01.jpg")));
    Utilities::IniParser parser = Utilities::IniParser("/home/rene/G/RobotKinematics/src/world/config/config.ini");
    parser.parse();

    auto& logger = Utilities::Logger::instance();
    logger.setLogOutput(std::make_unique<Utilities::LogToCout>());
    logger.log(Utilities::LogLevel::Debug, "Starting WorldService");

    std::unique_ptr<Sensor> aSensor;

    if(parser.get<bool>("ImageSource","CameraEnabled"))
    {
        aSensor = std::make_unique<CameraSensor>();
        logger.log(Utilities::LogLevel::Debug, "Using Camera");
    } else {
        aSensor = std::make_unique<StaticImage>(parser.get<std::string>("ImageSource","ImagePath"));
        logger.log(Utilities::LogLevel::Debug, "Using testimage");
    }

    ShapeFinder shapeFinder(std::move(aSensor));

    ros::spin();

    return 0;
}

