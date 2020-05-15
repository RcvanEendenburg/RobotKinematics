#include <al5d/Communicator.h>
#include <al5d/Application.h>

namespace Communication
{
Communicator::Communicator(const std::string &name, Application &app) : action_server(node_handle, name,
                                                                                      boost::bind(&Communicator::execute,
                                                                                                  this,
                                                                                                  _1), false),
                                                                        application(app)
{
    action_server.start();
}

void
Communicator::execute(const al5d::MoveRobotGoalConstPtr &goal)
{
    application.move(goal->base_angle, goal->shoulder_angle, goal->elbow_angle,
                     goal->wrist_angle, goal->wrist_rotate_angle, goal->gripper_distance, goal->time);
    action_server.setSucceeded();
}

}