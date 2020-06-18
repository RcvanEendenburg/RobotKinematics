#ifndef APPLICATION_H
#define APPLICATION_H

#include <al5d/Communicator.h>
#include <memory>
#include <string>
#include <utilities/IniParser.h>
#include <utilities/Logger.h>
#include <al5d/Controller.h>

class Application
{
public:
    Application(int argc, char **argv, const std::string &configFile);
    ~Application() = default;

    void
    move(double baseAngle, double shoulderAngle, double elbowAngle,
         double wristAngle, double wristRotateAngle, double gripperDistance, unsigned short time);

    void
    run() const;

private:
    friend class Communication::Communicator;
    Utilities::Logger &logger;

    Utilities::IniParser iniParser;

    Communication::Communicator communicator;

    Controller controller;
};

#endif //APPLICATION_H
