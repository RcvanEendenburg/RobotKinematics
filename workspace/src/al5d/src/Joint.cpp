#include <al5d/Joint.h>

Joint::Joint() : channel(0), valueBounds({0,0}), pwmBounds({0,0}), speed(0)
{

}

Joint &
Joint::operator=(const Joint &rhs)
{
    if (this != &rhs)
    {
        channel = rhs.channel;
        currentValue = rhs.currentValue;
        valueBounds = rhs.valueBounds;
        pwmBounds = rhs.pwmBounds;
        speed = rhs.speed;
    }
    return *this;
}

void
Joint::setValue(double value)
{
    currentValue = value;
}

void
Joint::setChannel(unsigned short aChannel)
{
    channel = aChannel;
}

unsigned short
Joint::getChannel() const
{
    return channel;
}

unsigned short
Joint::getPwm() const
{
    return static_cast<unsigned short>(pwmBounds.first + ((pwmBounds.second - pwmBounds.first) /
    (valueBounds.second - valueBounds.first)) * (currentValue - valueBounds.first));
}

void
Joint::setLimits(MinMaxValue value, MinMaxPwm pwm)
{
    valueBounds = value;
    pwmBounds = pwm;
}

void
Joint::setSpeed(unsigned short aSpeed)
{
    speed = aSpeed;
}

unsigned short
Joint::getSpeed() const
{
    return speed;
}