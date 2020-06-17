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
    auto &logger = Utilities::Logger::instance();
    logger.setLogOutput(std::make_unique<Utilities::LogToCout>());
    logger.log(Utilities::LogLevel::Debug, "Starting WorldService");

    if(argc > 1) {
        ros::init(argc, argv, "WorldService");

        Utilities::IniParser parser = Utilities::IniParser(argv[1]);
        parser.parse();

        std::unique_ptr<Sensor> aSensor;

        if (parser.get<bool>("ImageSource", "CameraEnabled")) {
            aSensor = std::make_unique<CameraSensor>();
            logger.log(Utilities::LogLevel::Debug, "Using Camera");
        }
        else {
            aSensor = std::make_unique<StaticImage>(parser.get<std::string>("ImageSource", "ImagePath"));
            logger.log(Utilities::LogLevel::Debug, "Using testimage");
        }

        ShapeFinder shapeFinder(std::move(aSensor), parser);

        ros::spin();
    }
    else {
        logger.log(Utilities::LogLevel::Error, "No config file supplied!");
    }
    return 0;
}

