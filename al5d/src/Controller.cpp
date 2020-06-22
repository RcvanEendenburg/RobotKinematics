#include <al5d/Controller.h>
#include <al5d/SSC32UCommandBuilder.h>

Controller::Controller(const std::string &serialDeviceName) : logger(Utilities::Logger::instance()),
                                                              serialCom(serialDeviceName)
{
}

void
Controller::setBaseChannel(unsigned short channel)
{
    logger.log(Utilities::LogLevel::Debug, "Setting base channel to %d", channel);
    joints[Base].setChannel(channel);
}

void
Controller::setShoulderChannel(unsigned short channel)
{
    logger.log(Utilities::LogLevel::Debug, "Setting shoulder channel to %d", channel);
    joints[Shoulder].setChannel(channel);
}

void
Controller::setElbowChannel(unsigned short channel)
{
    logger.log(Utilities::LogLevel::Debug, "Setting elbow channel to %d", channel);
    joints[Elbow].setChannel(channel);
}

void
Controller::setWristChannel(unsigned short channel)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist channel to %d", channel);
    joints[Wrist].setChannel(channel);
}

void
Controller::setWristRotateChannel(unsigned short channel)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist rotate channel to %d", channel);
    joints[WristRotate].setChannel(channel);
}

void
Controller::setGripperChannel(unsigned short channel)
{
    logger.log(Utilities::LogLevel::Debug, "Setting gripper channel to %d", channel);
    joints[Gripper].setChannel(channel);
}

void
Controller::setBaseAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting base angle to %f", angle);
    joints[Base].setValue(angle);
}

void
Controller::setShoulderAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting shoulder angle to %f", angle);
    joints[Shoulder].setValue(angle);
}

void
Controller::setElbowAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting elbow angle to %f", angle);
    joints[Elbow].setValue(angle);
}

void
Controller::setWristAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist angle to %f", angle);
    joints[Wrist].setValue(angle);
}

void
Controller::setWristRotateAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist rotate angle to %f", angle);
    joints[WristRotate].setValue(angle);
}

void
Controller::setGripperDistance(double distance)
{
    logger.log(Utilities::LogLevel::Debug, "Setting gripper distance to %f", distance);
    joints[Gripper].setValue(distance);
}

void
Controller::setBaseLimits(Joint::MinMaxValue angleBounds, Joint::MinMaxPwm pwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting base limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", angleBounds.first, angleBounds.second,
                                           pwm.first, pwm.second);
    joints[Base].setLimits(angleBounds, pwm);
}

void
Controller::setShoulderLimits(Joint::MinMaxValue angleBounds, Joint::MinMaxPwm pwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting shoulder limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", angleBounds.first, angleBounds.second,
                                           pwm.first, pwm.second);
    joints[Shoulder].setLimits(angleBounds, pwm);
}

void
Controller::setElbowLimits(Joint::MinMaxValue angleBounds, Joint::MinMaxPwm pwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting elbow limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", angleBounds.first, angleBounds.second,
                                           pwm.first, pwm.second);
    joints[Elbow].setLimits(angleBounds, pwm);
}

void
Controller::setWristLimits(Joint::MinMaxValue angleBounds, Joint::MinMaxPwm pwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", angleBounds.first, angleBounds.second,
                                           pwm.first, pwm.second);
    joints[Wrist].setLimits(angleBounds, pwm);
}

void
Controller::setWristRotateLimits(Joint::MinMaxValue angleBounds, Joint::MinMaxPwm pwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist rotate limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", angleBounds.first, angleBounds.second,
                                           pwm.first, pwm.second);
    joints[WristRotate].setLimits(angleBounds, pwm);
}

void
Controller::setGripperLimits(Joint::MinMaxValue distanceBounds, Joint::MinMaxPwm pwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting gripper limits -> min_distance: %f, max_distance: %f, "
                                           "min_pwm: %d, max_pwm: %d", distanceBounds.first, distanceBounds.second,
                                           pwm.first, pwm.second);
    joints[Gripper].setLimits(distanceBounds, pwm);
}

void
Controller::setBaseSpeed(unsigned short speed)
{
    logger.log(Utilities::LogLevel::Debug, "Setting base speed to %d", speed);
    joints[Base].setSpeed(speed);
}

void
Controller::setShoulderSpeed(unsigned short speed)
{
    logger.log(Utilities::LogLevel::Debug, "Setting shoulder speed to %d", speed);
    joints[Shoulder].setSpeed(speed);
}

void
Controller::setElbowSpeed(unsigned short speed)
{
    logger.log(Utilities::LogLevel::Debug, "Setting elbow speed to %d", speed);
    joints[Elbow].setSpeed(speed);
}

void
Controller::setWristSpeed(unsigned short speed)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist speed to %d", speed);
    joints[Wrist].setSpeed(speed);
}

void
Controller::setWristRotateSpeed(unsigned short speed)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist rotate speed to %d", speed);
    joints[WristRotate].setSpeed(speed);
}

void
Controller::setGripperSpeed(unsigned short speed)
{
    logger.log(Utilities::LogLevel::Debug, "Setting gripper speed to %d", speed);
    joints[Gripper].setSpeed(speed);
}

void
Controller::setGlobalTime(unsigned short time)
{
    logger.log(Utilities::LogLevel::Debug, "Setting global time to %d", time);
    global_time = time;
}

void
Controller::move(unsigned short time)
{
    if(time < global_time)
        time = global_time;
    auto command = SSC32UCommandBuilder<6>::createCommand(joints, time);
    auto logStr = "SSC32U command: " + command;
    logger.log(Utilities::LogLevel::Debug, logStr.c_str());
    serialCom.send(command);
}