#include <robothli/Communicator.h>
#include <robothli/Application.h>

namespace Communication
{
Communicator::Communicator(const std::string &node_name, const std::string &low_level_driver_name, Application &app) :
    action_server(node_handle, node_name,
                                                                                      boost::bind(&Communicator::execute,
                                                                                                  this,
                                                                                                  _1), false),
    action_client(low_level_driver_name, true),
    application(app)
{
    application.logger.log(Utilities::LogLevel::Debug, "Starting high level driver action server...");
    action_server.start();
    application.logger.log(Utilities::LogLevel::Debug, "Waiting for the low level driver to start...");
    //action_client.waitForServer();
    application.logger.log(Utilities::LogLevel::Debug, "Ready!");
}

void
Communicator::execute(const robothli::PickUpObjectGoalConstPtr &goal)
{
    auto point = goal->point;
    application.logger.log(Utilities::LogLevel::Debug, "Goal received X: %f Y: %f Z: %f",point.x,point.y,point.z);

    application.moveToGoal(point.x, point.y, point.z, goal->rotation, goal->opening_distance);
    action_server.setSucceeded();
}

void
Communicator::move(double baseAngle, double shoulderAngle, double elbowAngle,
                   double wristAngle, double wristRotateAngle, double gripperDistance, unsigned short time)
{
    application.logger.log(Utilities::LogLevel::Debug, "Sending goal to the low level driver...");
    robothli::MoveRobotGoal goal;
    goal.base_angle = baseAngle;
    goal.shoulder_angle = shoulderAngle;
    goal.elbow_angle = elbowAngle;
    goal.wrist_angle = wristAngle;
    goal.wrist_rotate_angle = wristRotateAngle;
    goal.gripper_distance = gripperDistance;
    goal.time = time;

    action_client.sendGoal(goal);

    bool finished_before_timeout = action_client.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
        application.logger.log(Utilities::LogLevel::Debug, "Successfully sent the goal to the low level driver!");
    else
        application.logger.log(Utilities::LogLevel::Error, "Oops action server timed out...");
}


}