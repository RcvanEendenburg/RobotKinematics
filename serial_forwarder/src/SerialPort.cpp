//
// Created by derk on 29-5-20.
//

#include <serial_forwarder/SerialPort.h>

SerialPort::SerialPort(const std::string& portName, std::function<void(std::string)> handler) : io(),
port(io, portName), readHandler(std::move(handler))
{
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
    port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port::stop_bits::type::one));
    port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port::parity::type::none));
    port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port::flow_control::type::none));
    port.set_option(boost::asio::serial_port_base::character_size(8));

    if(!port.is_open())
        throw std::runtime_error("Couldn't open serial port!");
}

void SerialPort::startReading()
{
    while(true) {
        if (!port.is_open())
            throw std::runtime_error("Serial port has closed!");

        std::string command;
        boost::asio::read_until(port, boost::asio::dynamic_buffer(command), '\r');
        std::thread([this, command]()
                    { readHandler(command); }).detach();
    }
}

