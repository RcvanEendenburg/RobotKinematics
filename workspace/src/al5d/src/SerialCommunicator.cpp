#include <al5d/SerialCommunicator.h>
#include <boost/asio/write.hpp>

SerialCommunicator::SerialCommunicator(std::string deviceName) : serialDeviceName(std::move(deviceName)),
                                                                 serialPort(ioService, serialDeviceName)
{
    serialPort.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port::stop_bits::type::one));
    serialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port::parity::type::none));
    serialPort.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port::flow_control::type::none));
    serialPort.set_option(boost::asio::serial_port_base::character_size(8));

    if (!serialPort.is_open())
        throw std::runtime_error("Couldn't open serial port");
}

void
SerialCommunicator::send(const std::string &message)
{
    if (!serialPort.is_open())
        throw std::runtime_error("Serial port is not open");

    boost::asio::write(serialPort, boost::asio::buffer(message.c_str(), message.size()));
}