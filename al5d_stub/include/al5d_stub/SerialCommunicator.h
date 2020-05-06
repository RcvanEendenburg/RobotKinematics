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
    SerialCommunicator(std::string deviceName, std::size_t aTimeOut);
    ~SerialCommunicator() = default;

    bool readChar(char& val);

private:
    void timeOut(const boost::system::error_code& error);
    void readComplete(const boost::system::error_code& error, size_t bytes_transferred);

    std::string serialDeviceName;
    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort;
    size_t timeout;
    char c;
    boost::asio::deadline_timer timer;
    bool readError;
};

#endif //SERIALCOMMUNICATOR_H
