#include <al5d/Joint.h>

Joint::Joint() : channel(0), angle(0), minAngle(0), maxAngle(0), minPwm(0), maxPwm(0), speed(0)
{

}

Joint::Joint(unsigned short aChannel, double anAngle) : channel(aChannel),
                                                        angle(anAngle),
                                                        minAngle(0),
                                                        maxAngle(0),
                                                        minPwm(0),
                                                        maxPwm(0),
                                                        speed(0)
{

}

Joint &
Joint::operator=(const Joint &rhs)
{
    if (this != &rhs)
    {
        channel = rhs.channel;
        angle = rhs.angle;
        minAngle = rhs.minAngle;
        maxAngle = rhs.maxAngle;
        minPwm = rhs.minPwm;
        maxPwm = rhs.maxPwm;
        speed = rhs.speed;
    }
    return *this;
}

void
Joint::setAngle(double anAngle)
{
    angle = anAngle;
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
    return static_cast<unsigned short>(minPwm + ((maxPwm - minPwm) / (maxAngle - minAngle)) * (angle - minAngle));
}

void
Joint::setLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm)
{
    minAngle = aMinAngle;
    maxAngle = aMaxAngle;
    minPwm = aMinPwm;
    maxPwm = aMaxPwm;
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