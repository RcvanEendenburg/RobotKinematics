//
// Created by derk on 29-5-20.
//

#include <serial_forwarder/SerialPort.h>
#include <thread>

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
    using boost::asio::buffers_begin;
    while(true) {
        if (!port.is_open())
            throw std::runtime_error("Serial port has closed!");

        boost::asio::streambuf buffer;
        boost::asio::read_until(port, buffer, '\r');
        auto bufs = buffer.data();
        std::string command(buffers_begin(bufs), buffers_begin(bufs) + buffer.size());
        std::thread([this, command]()
                    { readHandler(command); }).detach();
    }
}

