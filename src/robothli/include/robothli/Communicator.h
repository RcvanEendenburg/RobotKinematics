#ifndef COMUNICATOR_H
#define COMUNICATOR_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <robothli/PickUpObjectAction.h>
#include <robothli/MoveRobotAction.h>

class Application;

namespace Communication
{
class Communicator
{
public:
    Communicator(const std::string &name, const std::string &low_level_driver_name, Application &app);

    void
    execute(const robothli::PickUpObjectGoalConstPtr &goal);

    void
    move(double baseAngle, double shoulderAngle, double elbowAngle,
         double wristAngle, double wristRotateAngle, double gripperAngle, unsigned short time);

private:
    ros::NodeHandle node_handle;
    actionlib::SimpleActionServer<robothli::PickUpObjectAction> action_server;
    actionlib::SimpleActionClient<robothli::MoveRobotAction> action_client;

    Application &application;
};
}
#endif //COMUNICATOR_H
