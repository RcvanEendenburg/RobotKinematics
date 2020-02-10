//
// Created by rene on 6/1/19.
//

#ifndef SRC_SENSOREXCEPTION_H
#define SRC_SENSOREXCEPTION_H

#include <exception>

class SensorException : public std::exception
{
public:

    explicit SensorException(const char* aMessage):message(aMessage)
    {}


    virtual const char* what() const throw()
    {
        return message;
    }
private:
    const char* message;

};

#endif //SRC_SENSOREXCEPTION_H
