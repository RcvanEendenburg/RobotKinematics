//
// Created by derk on 28-4-20.
//

#include <tui/Communicator.h>
#include <tui/Application.h>

namespace Communication
{
Communicator::Communicator(const std::string &highLevelDriver, Application &app) :
    actionClient(highLevelDriver, true), application(app), worldClient(node.serviceClient<tui::ShapeFinderService>(
    application.iniParser.get<std::string>("TUI", "world_service")))
{
    application.logger.log(Utilities::LogLevel::Debug, "Setting up communication module...");
    actionClient.waitForServer();
}

void
Communicator::goToPosition(double x, double y, double z, double rotation, double openingDistance)
{
    tui::PickUpObjectGoal goal;
    goal.point.x = x;
    goal.point.y = y;
    goal.point.z = z;
    goal.rotation = rotation;
    goal.opening_distance = openingDistance;

    actionClient.sendGoal(goal);

    bool finished_before_timeout = actionClient.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout)
        application.logger.log(Utilities::LogLevel::Error, "Oops action server timed out...");
}

std::vector<tui::Shape> Communicator::findShapes(unsigned int shape, unsigned int color)
{
    tui::ShapeFinderService srv;
    srv.request.shape = shape;
    srv.request.color = color;
    if(worldClient.call(srv))
    {
        return srv.response.foundShapes;
    }
    throw std::runtime_error("Failed to call world service");
}

}