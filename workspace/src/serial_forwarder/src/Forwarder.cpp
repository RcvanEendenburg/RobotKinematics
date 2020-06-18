//
// Created by derk on 1-6-20.
//

#include <serial_forwarder/Forwarder.h>


Forwarder::Forwarder(const std::string &topic, const std::string &serialPort) : publisher(topic),
serial(serialPort, std::bind(&Forwarder::handleSerialCommand, this, std::placeholders::_1)), running(true)
{
    try {
        serial.startReading();
    }
    catch(std::exception& e)
    {
        running = false;
        throw;
    }
}

void Forwarder::handleSerialCommand(std::string command)
{
    publisher.sendCommand(command);
}
