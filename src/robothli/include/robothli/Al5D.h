#ifndef AL5D_H
#define AL5D_H

#include <robothli/KinematicChain.h>
#include <robothli/RobotModel.h>

namespace Robot
{
class Al5D : public RobotModel<4>
{
public:
    explicit Al5D(const Kinematics::PosePoint &start, std::array<double, 4> currentAngles);
    ~Al5D() override = default;

private:
    void
    setupKinematicChain(std::array<double, 4> currentAngles);

};
}
#endif //AL5D_H
