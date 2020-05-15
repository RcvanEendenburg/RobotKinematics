#ifndef JOINT_H
#define JOINT_H

#include <utility>

class Joint
{
public:
    using MinMaxValue = std::pair<double, double>;
    using MinMaxPwm = std::pair<unsigned short, unsigned short>;

    Joint();

    Joint(const Joint &joint) = default;

    ~Joint() = default;

    Joint &
    operator=(const Joint &rhs);

    void
    setValue(double value);

    void
    setChannel(unsigned short aChannel);

    unsigned short
    getChannel() const;

    unsigned short
    getPwm() const;

    void
    setLimits(MinMaxValue value, MinMaxPwm pwm);

    void
    setSpeed(unsigned short aSpeed);

    unsigned short
    getSpeed() const;

private:
    unsigned short channel;
    double currentValue{};
    MinMaxValue valueBounds;
    MinMaxPwm pwmBounds;
    unsigned short speed;
};

#endif //JOINT_H