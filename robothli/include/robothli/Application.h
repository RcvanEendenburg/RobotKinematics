#ifndef APPLICATION_H
#define APPLICATION_H

#include <robothli/Communicator.h>
#include <memory>
#include <functional>
#include <string>
#include <utilities/IniParser.h>
#include <utilities/Logger.h>
#include <robothli/GradientDescent.h>
#include <robothli/RobotModel.h>

class Application
{
public:
    Application(int argc, char **argv, const std::string &configFile);
    ~Application() = default;

    void
    run() const;

private:
    friend class Communication::Communicator;
    Utilities::Logger &logger;

    Utilities::IniParser iniParser;

    void
    moveToGoal(double x, double y, double z);

    Communication::Communicator communicator;
    Kinematics::PosePoint start;
    std::unique_ptr<Robot::RobotModel<4>> robot;
};

#endif //APPLICATION_H
