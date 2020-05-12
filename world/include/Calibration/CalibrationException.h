//
// Created by rene on 6/1/19.
//

#ifndef SRC_CALIBRATIONEXCEPTION_H
#define SRC_CALIBRATIONEXCEPTION_H

#include <exception>

class CalibrationException : public std::exception
{
public:

    explicit CalibrationException(const char* aMessage):message(aMessage)
    {}

    virtual const char* what() const throw()
    {
        return message;
    }
private:
    const char* message;

};

#endif //SRC_CALIBRATIONEXCEPTION_H
