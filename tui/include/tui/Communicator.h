//
// Created by derk on 27-4-20.
//

#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <string>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tui/PickUpObjectAction.h>

class Application;

namespace Communication
{
class Communicator
{
public:
    Communicator(const std::string &highLevelDriverName, Application &app);

    void goToPosition(unsigned int x, unsigned int y, unsigned int z);

private:
    ros::NodeHandle node_handle;
    actionlib::SimpleActionClient<tui::PickUpObjectAction> actionClient;

    Application &application;
};
}

#endif //COMMUNICATOR_H
