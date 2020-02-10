#include <robothli/Joint.h>
#include <utilities/Logger.h>
#include <cassert>

namespace Kinematics
{
Joint::Joint(double aLength,
             double anAngle,
             double anAngleOffset,
             double aLowerBound,
             double anUpperBound,
             bool aYRotate,
             bool aStaticLength)
    : length(aLength),
      angle(anAngle),
      angleOffset(anAngleOffset),
      lowerBound(aLowerBound),
      upperBound(anUpperBound),
      yRotate(aYRotate),
      staticLength(aStaticLength)
{
    assert(anAngle >= aLowerBound && anAngle <= anUpperBound);
    auto &logger = Utilities::Logger::instance();
    logger.log(Utilities::LogLevel::Debug, "New joint -> {length: %f, angle: %f, angle offset: %f, lower bound: %f, "
                                           "upper bound: %f, y rotate: %f, static length: %f}", length, angle,
                                           lowerBound, upperBound, yRotate, staticLength);
}

double
Joint::getAngle() const
{
    return angle;
}

void
Joint::setAngle(double anAngle)
{
    if (anAngle < lowerBound)
        anAngle = lowerBound;
    else if (anAngle > upperBound)
        anAngle = upperBound;
    angle = anAngle;
}

double
Joint::getLowerBound() const
{
    return lowerBound;
}

double
Joint::getUpperBound() const
{
    return upperBound;
}

double
Joint::getLength() const
{
    return length;
}

double
Joint::getAngleOffset() const
{
    return angleOffset;
}

bool
Joint::canYRotate() const
{
    return yRotate;
}

bool
Joint::isStatic() const
{
    return staticLength;
}

void
Joint::setNext(std::shared_ptr<Joint> aNext)
{
    next = std::move(aNext);
    next->previous = shared_from_this();
}

std::shared_ptr<Joint>
Joint::getPrevious() const
{
    return previous;
}

std::shared_ptr<Joint>
Joint::getNext() const
{
    return next;
}
}