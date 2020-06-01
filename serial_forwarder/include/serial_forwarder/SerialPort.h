//
// Created by derk on 29-5-20.
//

#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <functional>

class SerialPort
{
public:
    SerialPort(const std::string& portName, std::function<void(std::string)> handler);

    /**
     * This will start a read operation.
     * Every successful read will call the handler.
     */
    void startReading();
private:
    std::function<void(std::string)> readHandler;

    boost::asio::io_service io;
    boost::asio::serial_port port;
};


#endif //SERIALPORT_H
