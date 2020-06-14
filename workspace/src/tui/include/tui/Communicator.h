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
/**
 * This class is created to separate the communication with ROS from the application.
 */
class Communicator
{
public:
    /**
     * Constructs the communicator. Waits for the high level driver to start up.
     * @param highLevelDriverName The name of the high level driver.
     * @param app A reference to the application.
     */
    Communicator(const std::string &highLevelDriverName, Application &app);

    /**
     * Sends the desired position to the high level driver.
     * @param x
     * @param y
     * @param z
     * @param rotation The rotation of the shape.
     * @param openingDistance How much should the gripper open?
     */
    void goToPosition(double x, double y, double z, double rotation = 0, double openingDistance = 0);

    /**
     * Retrieves the shapes from the world service based on it's shape type and color.
     * @param shape The shape type.
     * @param color The color type.
     * @return The shapes.
     */
    std::vector<tui::Shape> findShapes(unsigned int shape, unsigned int color);

private:
    ros::NodeHandle node;
    actionlib::SimpleActionClient<tui::PickUpObjectAction> actionClient;
    Application &application;
    ros::ServiceClient worldClient;
};
}

#endif //COMMUNICATOR_H
