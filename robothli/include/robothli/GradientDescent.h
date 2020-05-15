#ifndef GRADIENT_DESCENT_H
#define GRADIENT_DESCENT_H

#include <array>
#include <iostream>
#include <map>

#include <robothli/Point.h>
#include <robothli/RobotModel.h>
#include <robothli/Matrix.h>
#include <robothli/Pose.h>

namespace Kinematics
{

struct UnableToMove : public std::exception
{
    const char* what () const throw()
    {
        return "Unable to move to position";
    }
};

struct Beta
{
    const double bigFactor;
    const double smallFactor;
    const double begin;
};

/**
 * @brief Calculates the angles of a robot arm given a position.
 * @tparam dof The degrees of freedom.
 */
template<std::size_t dof> class GradientDescent
{
public:
    /**
     * @brief Setup the kinematics algorithm.
     * @param robot The robot.
     */
    GradientDescent(const Beta &aBeta, std::unique_ptr<Robot::RobotModel<dof>> aRobot,
                    double maxMagnitude, double maxIterations);

    ~GradientDescent() = default;

    /**
     * @brief Calculate the angles and end effector.
     * @param goal The point the end effector has to move to.
     */
    void
    startMoving(const PosePoint &goal);

    void restoreAngles();
    void saveAngles();

    std::array<double, dof>
    getCurrentAngles() const;

private:
    enum class Step
    {
        TooFar,
        Small,
        Big,
        Stuck
    };

    Step
    getStep(double previousMagnitude, double currentMagnitude) const;

    const Beta beta;
    std::unique_ptr<Robot::RobotModel<dof>> robot;

    const double maximumMagnitude;
    const double maximumIterations;
    typename Pose<dof>::Angles previousAngles;
};
}

#include "GradientDescent.ipp"
#endif// GRADIENT_DESCENT_H
