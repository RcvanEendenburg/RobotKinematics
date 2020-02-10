#ifndef SERIALCOMMUNICATOR_H
#define SERIALCOMMUNICATOR_H

#include <string>

#include <boost/asio/serial_port.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/read.hpp>
#include <boost/bind.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/io_service.hpp>

class SerialCommunicator
{
public:
    explicit SerialCommunicator(std::string deviceName);
    ~SerialCommunicator() = default;

    void
    send(const std::string &message);

private:
    std::string serialDeviceName;
    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort;
};

#endif //SERIALCOMMUNICATOR_H
