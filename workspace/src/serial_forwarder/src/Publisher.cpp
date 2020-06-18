//
// Created by derk on 29-5-20.
//

#include <serial_forwarder/Publisher.h>
#include <std_msgs/String.h>

Publisher::Publisher(const std::string &topic) : node(), publisher(node.advertise<std_msgs::String>(topic,1000))
{
}

void Publisher::sendCommand(const std::string &command)
{
    std_msgs::String strMessage;
    strMessage.data = command;
    publisher.publish(strMessage);
}