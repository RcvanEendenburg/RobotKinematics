#include <utilities/Logger.h>
#include <sstream>

namespace Kinematics
{
template<std::size_t dof>
GradientDescent<dof>::GradientDescent(const Beta &aBeta, std::unique_ptr<Robot::RobotModel<dof>> aRobot,
                                      double maxMagnitude, double maxIterations)
    : beta(aBeta), robot(std::move(aRobot)), maximumMagnitude(maxMagnitude), maximumIterations(maxIterations), previousAngles()
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

    bool running = true;
    unsigned long its = 0;

    auto &logger = Utilities::Logger::instance();

    logger.log(Utilities::LogLevel::Debug, "Starting to move to point: (%f, %f, %f)", goal.x(), goal.y(), goal.z());

    BetaProgress betaProgress;

    const auto start = std::chrono::high_resolution_clock::now();
    while (running)
    {
        if (its > maximumIterations)
        {
            throw UnableToMove();
        }

        const auto thetaOld = robot->angles();
        const auto inverseJacobianMatrix = robot->jacobianMatrix().moorePenroseInverse();
        const auto deltaE = robot->delta(goal)*factor;
        const auto deltaTheta =
            inverseJacobianMatrix.transpose()*
                Matrix<double, 3, 1>({deltaE.x(), deltaE.y(), deltaE.z()});
        const auto thetaNew = thetaOld + deltaTheta.transpose();

        robot->update(thetaNew);
        const double currentMagnitude = robot->distance(goal);
        const Step step = getStep(previousMagnitude, currentMagnitude);
        const auto newAngles = robot->angles();

        switch (step)
        {
            case Step::Big:factor = beta.bigFactor;
                previousMagnitude = currentMagnitude;
                break;
            case Step::Small:factor = beta.smallFactor;
                previousMagnitude = currentMagnitude;
                break;
            case Step::TooFar:factor *= beta.smallFactor;
                robot->update(thetaOld);
                break;
            case Step::Stuck:factor = beta.begin;
                robot->reset();
                break;
            case Step::Done:running = false;
                break;
            default:break;
        }
        betaProgress.addStep(step);
        ++its;
    }
    
    try
    {
        robot->setVertical();
    }
    catch(const std::exception& e)
    {
        throw UnableToMove();
    }

    const auto stop = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    betaProgress.print();
    const auto endEffector = robot->position();
    logger.log(Utilities::LogLevel::Debug, "Current position of the robot is (%f, %f, %f)! Distance to the goal is %f",
               endEffector.x(), endEffector.y(), endEffector.z(), robot->distance(goal));
    logger.log(Utilities::LogLevel::Debug,
               "It took %d milliseconds to calculate the angles of the robot arm.",
               duration.count());

}

template<std::size_t dof>
Step
GradientDescent<dof>::getStep(double previousMagnitude, double currentMagnitude) const
{
    static Movement movement = Movement::Initial;
    static double lowestMagnitudeWhenIncreasing = std::numeric_limits<double>::max();
    static double lowestMagnitudeWhenDecreasing = std::numeric_limits<double>::max();

    static int numberOfIneffectiveOvershots = 0;

    Step step = Step::None;
    if (currentMagnitude < previousMagnitude)
    {
        //Making zig-zag movements
        if (movement==Movement::Decreasing)
            step = Step::Small;

        //Approaching goal from one side
        if (movement==Movement::Increasing || movement==Movement::Initial)
            step = Step::Big;

        movement = Movement::Increasing;
        numberOfIneffectiveOvershots = 0;
    }
    else
    {
        if (movement==Movement::Increasing)
        {
            step = Step::TooFar;
            if (lowestMagnitudeWhenIncreasing > previousMagnitude)
            {
                lowestMagnitudeWhenIncreasing = previousMagnitude;
                numberOfIneffectiveOvershots = 0;
            }
            else
                numberOfIneffectiveOvershots++;
        }

        else if (movement==Movement::Decreasing)
        {
            //Overshot to the other side and not making zig-zag movements
            step = Step::TooFar;
            if (lowestMagnitudeWhenDecreasing > previousMagnitude)
            {
                lowestMagnitudeWhenDecreasing = previousMagnitude;
                numberOfIneffectiveOvershots = 0;
            }
            else
                numberOfIneffectiveOvershots++;
        }
        movement = Movement::Decreasing;
    }

    if (numberOfIneffectiveOvershots==10)
    {
        if (currentMagnitude < maximumMagnitude)
            step = Step::Done;
        else
            step = Step::Stuck;
        numberOfIneffectiveOvershots = 0;
    }
    return step;
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

template <std::size_t dof>
void GradientDescent<dof>::restoreAngles()
{
    robot->update(previousAngles);
}

template <std::size_t dof>
void GradientDescent<dof>::saveAngles()
{
    previousAngles = robot->angles();
}

}
