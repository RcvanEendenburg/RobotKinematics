//
// Created by derk on 28-4-20.
//

#include <tui/Communicator.h>
#include <tui/Application.h>

namespace Communication
{
Communicator::Communicator(const std::string &highLevelDriver, Application &app) :
    actionClient(highLevelDriver, true),
    application(app)
{
    application.logger.log(Utilities::LogLevel::Debug, "Setting up communication module...");
//    actionClient.waitForServer();
}

void
Communicator::goToPosition(double x, double y, double z)
{
    application.logger.log(Utilities::LogLevel::Debug, "Sending goal to the high level driver...");
    tui::PickUpObjectGoal goal;
    goal.point.x = x;
    goal.point.y = y;
    goal.point.z = z;

    actionClient.sendGoal(goal);

    bool finished_before_timeout = actionClient.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
        application.logger.log(Utilities::LogLevel::Debug, "Successfully sent the goal to the high level driver!");
    else
        application.logger.log(Utilities::LogLevel::Error, "Oops action server timed out...");
}


}