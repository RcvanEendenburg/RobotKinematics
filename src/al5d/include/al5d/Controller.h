#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <al5d/Joint.h>
#include <array>
#include <Logger.h>
#include <al5d/SerialCommunicator.h>

class Controller
{
public:
    explicit Controller(const std::string &serialDeviceName);
    ~Controller() = default;

    enum
    {
        Base = 0,
        Shoulder,
        Elbow,
        Wrist,
        WristRotate,
        Gripper
    };

    void
    setBaseAngle(double angle);
    void
    setShoulderAngle(double angle);
    void
    setElbowAngle(double angle);
    void
    setWristAngle(double angle);
    void
    setWristRotateAngle(double angle);
    void
    setGripperAngle(double angle);

    void
    setBaseChannel(unsigned short channel);
    void
    setShoulderChannel(unsigned short channel);
    void
    setElbowChannel(unsigned short channel);
    void
    setWristChannel(unsigned short channel);
    void
    setWristRotateChannel(unsigned short channel);
    void
    setGripperChannel(unsigned short channel);

    void
    setBaseLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm);
    void
    setShoulderLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm);
    void
    setElbowLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm);
    void
    setWristLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm);
    void
    setWristRotateLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm);
    void
    setGripperLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm);

    void
    setBaseSpeed(unsigned short speed);
    void
    setShoulderSpeed(unsigned short speed);
    void
    setElbowSpeed(unsigned short speed);
    void
    setWristSpeed(unsigned short speed);
    void
    setWristRotateSpeed(unsigned short speed);
    void
    setGripperSpeed(unsigned short speed);

    void
    move(unsigned short time);

private:
    Utilities::Logger &logger;
    std::array<Joint, 6> joints;
    SerialCommunicator serialCom;
};

#endif //CONTROLLER_H
