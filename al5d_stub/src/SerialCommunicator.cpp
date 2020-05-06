#include <al5d_stub/SerialCommunicator.h>
#include <boost/asio/write.hpp>

SerialCommunicator::SerialCommunicator(std::string deviceName, std::size_t aTimeOut) :
serialDeviceName(std::move(deviceName)), serialPort(ioService, serialDeviceName), timeout(aTimeOut),
timer(serialPort.get_io_service()), readError(true)
{
    serialPort.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port::stop_bits::type::one));
    serialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port::parity::type::none));
    serialPort.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port::flow_control::type::none));
    serialPort.set_option(boost::asio::serial_port_base::character_size(8));

    if (!serialPort.is_open())
        throw std::runtime_error("Couldn't open serial port");
}

void SerialCommunicator::readComplete(const boost::system::error_code& error, size_t bytes_transferred)
{
    readError = (error || bytes_transferred == 0);
    timer.cancel();
}

// Called when the timer's deadline expires.
void SerialCommunicator::timeOut(const boost::system::error_code& error)
{
    if (error) return;
    serialPort.cancel();
}

bool SerialCommunicator::readChar(char& val)
{
    val = c = '\0';
    serialPort.get_io_service().reset();

    boost::asio::async_read(serialPort, boost::asio::buffer(&c, 1),
                            boost::bind(&SerialCommunicator::readComplete,
                                        this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));

    timer.expires_from_now(boost::posix_time::milliseconds(timeout));
    timer.async_wait(boost::bind(&SerialCommunicator::timeOut,
                                 this, boost::asio::placeholders::error));

    serialPort.get_io_service().run();

    if (!readError)
        val = c;

    return !readError;
}