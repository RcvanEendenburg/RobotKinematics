//
// Created by derk on 1-6-20.
//

#ifndef FORWARDER_H
#define FORWARDER_H

#include <serial_forwarder/Publisher.h>
#include <serial_forwarder/SerialPort.h>

#include <atomic>

class Forwarder
{
public:
    Forwarder(const std::string& topic, const std::string& serialPort);
    ~Forwarder() = default;

private:
    void handleSerialCommand(std::string command);

    Publisher publisher;
    SerialPort serial;
    std::atomic_bool running;
};


#endif //FORWARDER_H
