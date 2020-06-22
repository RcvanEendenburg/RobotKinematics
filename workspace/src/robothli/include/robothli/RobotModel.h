#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <robothli/Pose.h>

namespace Robot
{
template<unsigned char T>
class RobotModel
{
public:
    explicit RobotModel(const Kinematics::PosePoint &aStart);
    virtual ~RobotModel() = default;
    constexpr unsigned char
    dof() { return T; }
    double
    distance(const Kinematics::PosePoint &goal) const;

    typename Kinematics::Pose<T>::Angles
    angles() const;
    typename Kinematics::Pose<T>::Angles
    offsettedAngles() const;
    typename Kinematics::Pose<T>::JacobianMatrix
    jacobianMatrix() const;

    Kinematics::PosePoint
    delta(const Kinematics::PosePoint &goal) const;

    Kinematics::PosePoint
    position() const;

    void
    update(Kinematics::Matrix<double, 1, T> angles);
    void setVertical();
    void
    reset();
protected:
    Kinematics::PosePoint start;
    Kinematics::Pose<T> pose;
    Kinematics::KinematicChain kinematicChain;
    Kinematics::PosePoint endEffector;
};
}

#include "RobotModel.ipp"

#endif //ROBOTMODEL_H
