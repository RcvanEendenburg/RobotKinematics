//
// Created by derk on 27-4-20.
//

#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <string>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tui/PickUpObjectAction.h>
#include <tui/ShapeFinderService.h>

class Application;

namespace Communication
{
class Communicator
{
public:
    Communicator(const std::string &highLevelDriverName, Application &app);

    void goToPosition(double x, double y, double z, double rotation = 0, double openingDistance = 0);

    std::vector<tui::Shape> findShapes(unsigned int shape, unsigned int color);

private:
    ros::NodeHandle node;
    actionlib::SimpleActionClient<tui::PickUpObjectAction> actionClient;
    Application &application;
    ros::ServiceClient worldClient;
};
}

#endif //COMMUNICATOR_H
