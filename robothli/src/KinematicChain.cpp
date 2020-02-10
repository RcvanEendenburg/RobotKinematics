#include <iostream>
#include <cmath>
#include <robothli/KinematicChain.h>

namespace Kinematics
{
KinematicChain::KinematicChain(std::shared_ptr<Joint> first, std::shared_ptr<Joint> last)
    : firstJoint(std::move(first)), lastJoint(std::move(last))
{
}

KinematicChain::KinematicChain(std::initializer_list<std::shared_ptr<Joint>> joints)
{
    firstJoint = *joints.begin();
    lastJoint = *std::prev(joints.end());
    for (auto it = joints.begin(); it!=joints.end(); ++it)
    {
        auto next = std::next(it);
        if (next!=joints.end())
        {
            (*it)->setNext(*next);
        }
    }
}

std::shared_ptr<Joint>
KinematicChain::begin() const
{
    return firstJoint;
}

std::shared_ptr<Joint>
KinematicChain::end() const
{
    return lastJoint;
}

std::shared_ptr<Joint>
KinematicChain::at(std::size_t index) const
{
    std::shared_ptr<Joint> current = firstJoint;

    for (std::size_t i = 0; current; ++i)
    {
        if (i==index)
            return current;
        current = current->getNext();
    }
    throw std::runtime_error("Couldn't find joint at specified index");
}

double
KinematicChain::getAngleSumZ(std::shared_ptr<Joint> beginJoint, const std::shared_ptr<Joint> &endJoint) const
{
    std::shared_ptr<Joint> joint = std::move(beginJoint);
    std::shared_ptr<Joint> end = !endJoint ? nullptr : endJoint->getNext();

    if (!isValidJoint(joint))
        throw std::runtime_error("Begin joint is not valid");

    if (end && !isValidJoint(end))
        throw std::runtime_error("End joint is not valid");

    if (end && !areJointsInOrder(joint, end))
        throw std::runtime_error("Joints are not in order");

    double sum = 0;
    while (joint && joint!=end)
    {
        if (joint->canYRotate())
        {
            joint = joint->getNext();
            continue;
        }
        sum += degreesToRadians(joint->getAngle());
        joint = joint->getNext();
    }
    return sum;
}

double
KinematicChain::getAngleSumY(std::shared_ptr<Joint> beginJoint, const std::shared_ptr<Joint> &endJoint) const
{
    std::shared_ptr<Joint> joint = std::move(beginJoint);
    std::shared_ptr<Joint> end = !endJoint ? nullptr : endJoint->getNext();

    if (!isValidJoint(joint))
        throw std::runtime_error("Begin joint is not valid");

    if (end && !isValidJoint(end))
        throw std::runtime_error("End joint is not valid");

    if (end && !areJointsInOrder(joint, end))
        throw std::runtime_error("Joints are not in order");

    double sum = 0;
    while (joint && joint!=end)
    {
        if (!joint->canYRotate())
        {
            joint = joint->getNext();
            continue;
        }
        sum += degreesToRadians(joint->getAngle());
        joint = joint->getNext();
    }
    return sum;
}

double
KinematicChain::getStaticLength() const
{
    double sum = 0;
    std::shared_ptr<Joint> joint = firstJoint;
    while (joint)
    {
        if (joint->isStatic())
        {
            sum += joint->getLength();
        }
        joint = joint->getNext();
    }
    return sum;
}

std::shared_ptr<Joint>
KinematicChain::getNextZRotationJoint(std::shared_ptr<Joint> joint) const
{
    std::shared_ptr<Joint> begin = std::move(joint);
    while (begin)
    {
        if (begin->canYRotate())
        {
            begin = begin->getNext();
            continue;
        }
        return begin;
    }
    return nullptr;
}

std::shared_ptr<Joint>
KinematicChain::getFirstZRotationJoint() const
{
    return getNextZRotationJoint(firstJoint);
}

std::shared_ptr<Joint>
KinematicChain::getFirstYRotationJoint() const
{
    return getNextYRotationJoint(firstJoint);
}

std::shared_ptr<Joint>
KinematicChain::getNextYRotationJoint(std::shared_ptr<Joint> joint) const
{
    std::shared_ptr<Joint> begin = std::move(joint);
    while (begin)
    {
        if (!begin->canYRotate())
        {
            begin = begin->getNext();
            continue;
        }
        return begin;
    }
    return nullptr;
}

void
KinematicChain::set(std::shared_ptr<Joint> first, std::shared_ptr<Joint> last)
{
    firstJoint = std::move(first);
    lastJoint = std::move(last);

    if (!areJointsInOrder(firstJoint, lastJoint))
        throw std::runtime_error("First and last are not connected");
}

double
KinematicChain::degreesToRadians(double degrees) const
{
    return degrees*std::acos(-1)/180.0;
}

double
KinematicChain::radiansToDegrees(double radians) const
{
    return radians*180.0/std::acos(-1);
}

bool
KinematicChain::isValidJoint(const std::shared_ptr<Joint> &joint) const
{
    if (!joint)
        return false;
    std::shared_ptr<Joint> current = firstJoint;
    while (current)
    {
        if (joint==current)
            return true;
        current = current->getNext();
    }
    return false;
}

bool
KinematicChain::areJointsInOrder(const std::shared_ptr<Joint> &first, const std::shared_ptr<Joint> &second) const
{
    if (!first)
        return false;
    if (!second)
        return false;
    std::shared_ptr<Joint> current = firstJoint;

    bool foundFirst = false;

    while (current)
    {
        if (current==first)
            foundFirst = true;
        if (foundFirst && current==second)
            return true;
        current = current->getNext();
    }
    return false;
}
bool
KinematicChain::operator==(const KinematicChain &Rhs) const
{
    return firstJoint==Rhs.firstJoint &&
        lastJoint==Rhs.lastJoint;
}
bool
KinematicChain::operator!=(const KinematicChain &Rhs) const
{
    return !(Rhs==*this);
}

KinematicChain &
KinematicChain::operator=(const KinematicChain &rhs)
{
    if (this!=&rhs)
    {
        firstJoint = rhs.firstJoint;
        lastJoint = rhs.lastJoint;
    }
    return *this;
}
}