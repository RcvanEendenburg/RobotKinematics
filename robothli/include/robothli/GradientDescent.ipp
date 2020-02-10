#include <utilities/Logger.h>

namespace Kinematics
{
template<std::size_t dof>
GradientDescent<dof>::GradientDescent(const Beta &aBeta, std::unique_ptr<Robot::RobotModel<dof>> aRobot,
                                      double maxMagnitude, double maxIterations)
    : beta(aBeta), robot(std::move(aRobot)), maximumMagnitude(maxMagnitude), maximumIterations(maxIterations)
{
    auto &logger = Utilities::Logger::instance();
    logger.log(Utilities::LogLevel::Debug, "Gradient descent: beta begin = %f", beta.begin);
    logger.log(Utilities::LogLevel::Debug, "Gradient descent: beta big factor = %f", beta.bigFactor);
    logger.log(Utilities::LogLevel::Debug, "Gradient descent: beta small factor = %f", beta.smallFactor);
    logger.log(Utilities::LogLevel::Debug, "Gradient descent: max magnitude = %f", maxMagnitude);
    logger.log(Utilities::LogLevel::Debug, "Gradient descent: max iterations = %f", maxIterations);
}

template<std::size_t dof>
void
GradientDescent<dof>::startMoving(const Kinematics::PosePoint &goal)
{
    double factor = beta.begin;
    double previousMagnitude = std::numeric_limits<double>::max();

    unsigned long its = 0;

    auto &logger = Utilities::Logger::instance();

    logger.log(Utilities::LogLevel::Debug, "Starting to move to point: (%f, %f, %f)", goal.x(), goal.y(), goal.z());

    //If we save the current position, we always have a point to fallback to when the robot cannot reach the goal!
    auto start = robot->position();

    while (robot->distance(goal) > maximumMagnitude)
    {
        if (its > maximumIterations)
        {
            logger.log(Utilities::LogLevel::Warning, "Now trying to go back to previous position");
            startMoving(start);
            throw std::runtime_error("Gradient descent exceeded maximum amount of iterations!");
        }

        auto thetaOld = robot->angles();
        auto inverseJacobianMatrix = robot->jacobianMatrix().moorePenroseInverse();
        auto deltaE = robot->delta(goal)*factor;
        auto deltaTheta =
            inverseJacobianMatrix.transpose()*
                Matrix<double, 3, 1>({deltaE.x(), deltaE.y(), deltaE.z()});
        auto thetaNew = thetaOld + deltaTheta.transpose();

        robot->update(thetaNew);
        double currentMagnitude = robot->distance(goal);
        Step step = Step::Stuck;
        try
        {
            step = getStep(previousMagnitude, currentMagnitude);
        }
        catch (std::exception &e)
        {
            logger.log(Utilities::LogLevel::Error, e.what());
        }

        switch (step)
        {
            case Step::Big:factor *= beta.bigFactor;
                previousMagnitude = currentMagnitude;
                break;
            case Step::Small:factor *= beta.smallFactor;
                previousMagnitude = currentMagnitude;
                break;
            case Step::TooFar:factor *= beta.bigFactor;
                robot->update(thetaOld);
                break;
            case Step::Stuck:factor = beta.begin;
                robot->reset();
                break;
            default:throw std::runtime_error("Step not handled");
        }
        ++its;
    }

    auto endEffector = robot->position();
    logger.log(Utilities::LogLevel::Debug, "Current position of the robot is (%f, %f, %f)",
               endEffector.x(), endEffector.y(), endEffector.z());

}

template<std::size_t dof>
typename GradientDescent<dof>::Step
GradientDescent<dof>::getStep(double previousMagnitude, double currentMagnitude) const
{
    if (previousMagnitude > currentMagnitude && currentMagnitude > maximumMagnitude)
        return Step::Big;
    else if (previousMagnitude > currentMagnitude && currentMagnitude < maximumMagnitude)
        return Step::Small;
    else if (previousMagnitude - currentMagnitude > maximumMagnitude)
        return Step::TooFar;
    else if (currentMagnitude > beta.smallFactor)
        return Step::Stuck;
    auto errorStr =
        "Gradient descent couldn't calculate beta step -> previous magnitude: " + std::to_string(previousMagnitude) +
            " current magnitude: " + std::to_string(currentMagnitude);
    throw std::runtime_error(errorStr);
}

template<std::size_t dof>
std::array<double, dof>
GradientDescent<dof>::getCurrentAngles() const
{
    std::array<double, dof> result;
    auto currentAngles = robot->offsettedAngles();
    for (std::size_t i = 0; i < dof; ++i)
        result[i] = currentAngles[0][i];
    return result;
}


}
