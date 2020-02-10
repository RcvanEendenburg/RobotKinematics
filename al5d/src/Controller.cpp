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
    joints[Base].setAngle(angle);
}

void
Controller::setShoulderAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting shoulder angle to %f", angle);
    joints[Shoulder].setAngle(angle);
}

void
Controller::setElbowAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting elbow angle to %f", angle);
    joints[Elbow].setAngle(angle);
}

void
Controller::setWristAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist angle to %f", angle);
    joints[Wrist].setAngle(angle);
}

void
Controller::setWristRotateAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist rotate angle to %f", angle);
    joints[WristRotate].setAngle(angle);
}

void
Controller::setGripperAngle(double angle)
{
    logger.log(Utilities::LogLevel::Debug, "Setting gripper angle to %f", angle);
    joints[Gripper].setAngle(angle);
}

void
Controller::setBaseLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting base limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
    joints[Base].setLimits(aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);

}

void
Controller::setShoulderLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting shoulder limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
    joints[Shoulder].setLimits(aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);

}

void
Controller::setElbowLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting elbow limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
    joints[Elbow].setLimits(aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
}

void
Controller::setWristLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
    joints[Wrist].setLimits(aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
}

void
Controller::setWristRotateLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting wrist rotate limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
    joints[WristRotate].setLimits(aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
}

void
Controller::setGripperLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm)
{
    logger.log(Utilities::LogLevel::Debug, "Setting gripper limits -> min_angle: %f, max_angle: %f, "
                                           "min_pwm: %d, max_pwm: %d", aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
    joints[Gripper].setLimits(aMinAngle, aMaxAngle, aMinPwm, aMaxPwm);
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
Controller::move(unsigned short time)
{
    auto command = SSC32UCommandBuilder<6>::createCommand(joints, time);
    auto logStr = "SSC32U command: " + command;
    logger.log(Utilities::LogLevel::Debug, logStr.c_str());
    serialCom.send(command);
}