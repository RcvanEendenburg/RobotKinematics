#ifndef JOINT_H
#define JOINT_H

class Joint
{
public:
    Joint(unsigned short aChannel, double anAngle);

    Joint();

    Joint(const Joint &joint) = default;

    ~Joint() = default;

    Joint &
    operator=(const Joint &rhs);

    void
    setAngle(double anAngle);

    void
    setChannel(unsigned short aChannel);

    unsigned short
    getChannel() const;

    unsigned short
    getPwm() const;

    void
    setLimits(double aMinAngle, double aMaxAngle, unsigned short aMinPwm, unsigned short aMaxPwm);

    void
    setSpeed(unsigned short aSpeed);

    unsigned short
    getSpeed() const;

private:
    unsigned short channel;
    double angle;
    double minAngle;
    double maxAngle;
    unsigned short minPwm;
    unsigned short maxPwm;
    unsigned short speed;
};

#endif //JOINT_H