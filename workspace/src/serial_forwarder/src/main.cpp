//
// Created by derk on 29-5-20.
//

#include <serial_forwarder/Forwarder.h>
#include <iostream>

int main(int argc, char** argv)
{
    try
    {
        if(argc > 1)
        {
            ros::init(argc, argv, "serial_forwarder");
            Forwarder forwarder("SSC32U_request_topic", argv[1]);
        }
        else
        {
            throw std::runtime_error("No serial port provided!");
        }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}
