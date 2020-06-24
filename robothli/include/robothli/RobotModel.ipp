
namespace Robot
{

template<unsigned char T>

RobotModel<T>::RobotModel(const Kinematics::PosePoint &aStart) : start(aStart), kinematicChain(), pose(),
                                                                 endEffector()
{
}

template<unsigned char T>
double
RobotModel<T>::distance(const Kinematics::PosePoint &goal) const
{
    return endEffector.magnitude(goal);
}

template<unsigned char T>
typename Kinematics::Pose<T>::Angles
RobotModel<T>::angles() const
{
    return pose.getAngles();
}

template<unsigned char T>
typename Kinematics::Pose<T>::Angles
RobotModel<T>::offsettedAngles() const
{
    return pose.getOffsettedAngles();
}

template<unsigned char T>
typename Kinematics::Pose<T>::JacobianMatrix
RobotModel<T>::jacobianMatrix() const
{
    return pose.getJacobianMatrix();
}

template<unsigned char T>
Kinematics::PosePoint
RobotModel<T>::delta(const Kinematics::PosePoint &goal) const
{
    return goal - endEffector;
}

template<unsigned char T>
void
RobotModel<T>::update(Kinematics::Matrix<double, 1, T> angles)
{
    pose.setAngles(angles);
    endEffector = pose.calculatePoint(start);
}

template<unsigned char T>
void
RobotModel<T>::setVertical()
{
    pose.setVertical();
    endEffector = pose.calculatePoint(start);
}

template<unsigned char T>
void
RobotModel<T>::reset()
{
    pose.randomizeAngles();
    endEffector = pose.calculatePoint(start);
}

template<unsigned char T>
Kinematics::PosePoint
RobotModel<T>::position() const
{
    return endEffector;
}

}