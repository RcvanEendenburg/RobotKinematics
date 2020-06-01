//
// Created by derk on 29-5-20.
//

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <string>
#include <ros/ros.h>

class Publisher
{
public:
    Publisher(const std::string& topic);
    ~Publisher() = default;

    void sendCommand(const std::string& command);

private:
    ros::NodeHandle node;
    ros::Publisher publisher;
};


#endif //PUBLISHER_H
