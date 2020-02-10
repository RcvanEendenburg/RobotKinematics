#include <robothli/Al5D.h>
#include <Utilities/Logger.h>

namespace Robot
{

Al5D::Al5D(const Kinematics::PosePoint &start, std::array<double, 4> currentAngles) : RobotModel(start)
{
    setupKinematicChain(currentAngles);
    endEffector = pose.calculatePoint(start);
}

void
Al5D::setupKinematicChain(std::array<double, 4> currentAngles)
{
    auto &logger = Utilities::Logger::instance();
    logger.log(Utilities::LogLevel::Debug, "Setting up AL5D kinematic chain");

    auto base = std::make_shared<Kinematics::Joint>(0.5, currentAngles[0], -90, -90, 90, true, true);
    auto shoulder = std::make_shared<Kinematics::Joint>(1.8, currentAngles[1], 0, -30, 90, false, false);
    auto elbow = std::make_shared<Kinematics::Joint>(2, currentAngles[2], 0, 0, 135, false, false);
    auto wrist = std::make_shared<Kinematics::Joint>(0.3, currentAngles[3], 0, -90, 90, false, false);

    base->setNext(shoulder);
    shoulder->setNext(elbow);
    elbow->setNext(wrist);

    kinematicChain.set(base, wrist);
    pose.setKinematicChain(kinematicChain);
}
}