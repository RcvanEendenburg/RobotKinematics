#include <random>

namespace Kinematics
{
template<std::size_t dof>
Pose<dof>::Pose(const KinematicChain &aKinematicChain) : kinematicChain(aKinematicChain)
{
}

template<std::size_t dof> typename Pose<dof>::Angles
Pose<dof>::getAngles() const
{
    Pose<dof>::Angles result;
    std::shared_ptr<Joint> joint = kinematicChain.begin();
    for (std::size_t i = 0; i < dof; ++i)
    {
        if (joint)
        {
            result[0][i] = joint->getAngle();
            joint = joint->getNext();
        }
    }
    return result;
}

template<std::size_t dof> typename Pose<dof>::Angles
Pose<dof>::getOffsettedAngles() const
{
    Pose<dof>::Angles result;
    std::shared_ptr<Joint> joint = kinematicChain.begin();
    for (std::size_t i = 0; i < dof; ++i)
    {
        if (joint)
        {
            result[0][i] = joint->getAngle() + joint->getAngleOffset();
            joint = joint->getNext();
        }
    }
    return result;
}

template<std::size_t dof> void
Pose<dof>::setAngles(Pose<dof>::Angles angles)
{
    std::shared_ptr<Joint> joint = kinematicChain.begin();
    for (std::size_t i = 0; i < dof; ++i)
    {
        if (joint)
        {
            joint->setAngle(angles[0][i]);
            joint = joint->getNext();
        }
    }
    keepVertical();
}

template <std::size_t dof> void Pose<dof>::keepVertical()
{
    double sum                   = 45;
    std::shared_ptr<Joint> joint = kinematicChain.end()->getPrevious();
    while(joint)
    {
        if(joint->canYRotate())
        {
            joint = joint->getPrevious();
            continue;
        }
        sum -= joint->getAngle();
        joint = joint->getPrevious();
    }
    kinematicChain.end()->setAngle(sum);
}

template<std::size_t dof> const KinematicChain &
Pose<dof>::getKinematicChain() const
{
    return kinematicChain;
}

template<std::size_t dof> double
Pose<dof>::getPartialDerivativeX(std::size_t theta) const
{
    std::shared_ptr<Joint> joint = kinematicChain.at(theta);
    double sum = 0;

    bool yRotationJoint = joint ? joint->canYRotate() : false;
    while (joint)
    {
        if (!yRotationJoint)
        {
            if (joint->canYRotate())
            {
                joint = joint->getNext();
                continue;
            }
            sum += joint->getLength()
                *std::cos(kinematicChain.getAngleSumZ(kinematicChain.getFirstZRotationJoint(), joint));
        }
        else
        {
            if (!joint->canYRotate())
            {
                joint = joint->getNext();
                continue;
            }
            sum += std::cos(kinematicChain.getAngleSumY(kinematicChain.getFirstYRotationJoint(), joint));
        }
        joint = joint->getNext();
    }
    return sum;
}

template<std::size_t dof> double
Pose<dof>::getPartialDerivativeY(std::size_t theta) const
{
    std::shared_ptr<Joint> joint = kinematicChain.at(theta);
    double sum = 0;
    if ((joint && !joint->canYRotate()) || !joint)
    {
        return 0;
    }
    while (joint)
    {
        if (!joint->canYRotate())
        {
            joint = joint->getNext();
            continue;
        }
        sum += -std::sin(kinematicChain.getAngleSumY(kinematicChain.getFirstYRotationJoint(), joint));
        joint = joint->getNext();
    }
    return sum;
}

template<std::size_t dof> double
Pose<dof>::getPartialDerivativeZ(std::size_t theta) const
{
    std::shared_ptr<Joint> joint = kinematicChain.at(theta);
    double sum = 0;
    if ((joint && joint->canYRotate()) || !joint)
    {
        return 0;
    }
    while (joint)
    {
        if (joint->canYRotate())
        {
            joint = joint->getNext();
            continue;
        }
        sum +=
            joint->getLength()*-std::sin(kinematicChain.getAngleSumZ(kinematicChain.getFirstZRotationJoint(), joint));

        joint = joint->getNext();
    }
    return sum;
}

template<std::size_t dof>
PosePoint
Pose<dof>::calculatePoint(const PosePoint &beginPoint, std::size_t jointNr) const
{
    return PosePoint(calculateX(beginPoint.x(), jointNr),
                     calculateY(beginPoint.y(), jointNr),
                     calculateZ(beginPoint.z(), jointNr));
}

template<std::size_t dof> PosePoint
Pose<dof>::calculatePoint(double x0, double y0, double z0) const
{
    return PosePoint(calculateX(x0), calculateY(y0), calculateZ(z0));
}

template<std::size_t dof> double
Pose<dof>::calculateX(double x0, std::size_t jointNr) const
{
    std::shared_ptr<Joint> joint = kinematicChain.begin();
    std::shared_ptr<Joint> end = kinematicChain.at(jointNr);
    double sum = 0;

    while (joint!=end)
    {
        if (joint->canYRotate())
        {
            joint = joint->getNext();
            continue;
        }
        sum += joint->getLength()*std::sin(kinematicChain.getAngleSumZ(kinematicChain.begin(), joint))*
            std::sin(kinematicChain.getAngleSumY(kinematicChain.begin(), joint));
        joint = joint->getNext();
    }
    return sum + x0;
}

template<std::size_t dof> double
Pose<dof>::calculateY(double y0, std::size_t jointNr) const
{
    std::shared_ptr<Joint> joint = kinematicChain.begin();
    std::shared_ptr<Joint> end = kinematicChain.at(jointNr);
    double sum = 0;
    while (joint!=end)
    {
        if (joint->canYRotate())
        {
            joint = joint->getNext();
            continue;
        }
        sum += joint->getLength()*std::sin(kinematicChain.getAngleSumZ(kinematicChain.begin(), joint))*
            std::cos(kinematicChain.getAngleSumY(kinematicChain.begin(), joint));
        joint = joint->getNext();
    }
    return sum + y0;
}

template<std::size_t dof> double
Pose<dof>::calculateZ(double z0, std::size_t jointNr) const
{
    std::shared_ptr<Joint> joint = kinematicChain.begin();
    std::shared_ptr<Joint> end = kinematicChain.at(jointNr);
    double sum = 0;
    while (joint!=end)
    {
        if (joint->canYRotate())
        {
            joint = joint->getNext();
            continue;
        }
        sum += joint->getLength()*std::cos(kinematicChain.getAngleSumZ(kinematicChain.begin(), joint));
        joint = joint->getNext();
    }
    return sum + kinematicChain.getStaticLength() + z0;
}

template<std::size_t dof> typename Pose<dof>::JacobianMatrix
Pose<dof>::getJacobianMatrix() const
{
    JacobianMatrix result;
    for (std::size_t i = 0; i < dof; ++i)
    {
        result[0][i] = getPartialDerivativeX(i);
        result[1][i] = getPartialDerivativeY(i);
        result[2][i] = getPartialDerivativeZ(i);
    }
    return result;
}

template<std::size_t dof> void
Pose<dof>::randomizeAngles()
{
    std::shared_ptr<Joint> joint = kinematicChain.end();

    static std::random_device rd;
    static std::mt19937 gen(rd());
    while (joint)
    {
        std::uniform_int_distribution<> dis(joint->getLowerBound(), joint->getUpperBound());
        joint->setAngle(dis(gen));
        joint = joint->getPrevious();
    }
}

template<std::size_t dof> void
Pose<dof>::setKinematicChain(const KinematicChain &aKinematicChain)
{
    kinematicChain = aKinematicChain;
}

}