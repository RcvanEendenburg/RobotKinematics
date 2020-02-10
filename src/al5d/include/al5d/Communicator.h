#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <al5d/MoveRobotAction.h>

class Application;

namespace Communication
{
class Communicator
{
public:
    Communicator(const std::string &name, Application &app);

    void
    execute(const al5d::MoveRobotGoalConstPtr &goal);

private:
    ros::NodeHandle node_handle;
    actionlib::SimpleActionServer<al5d::MoveRobotAction> action_server;

    Application &application;
};
}
#endif //COMMUNICATOR_H
