#ifndef POSE_H
#define POSE_H

#include <array>
#include <tuple>

#include <robothli/Point.h>
#include <robothli/KinematicChain.h>
#include <robothli/Matrix.h>

namespace Kinematics
{
typedef Point3D<double> PosePoint;

/**
 * @brief This class offers the functionality to calculate the angles/position of the robotarm.
 * @tparam dof The degrees of freedom.
 */
template<std::size_t dof> class Pose
{
public:
    typedef Matrix<double, 1, dof> Angles;
    typedef Matrix<double, 3, dof> JacobianMatrix;

    Pose() = default;
    /**
     * @brief Construct the pose with the configuration.
     * @param aConfiguration The configuration.
     */
    explicit Pose(const KinematicChain &aKinematicChain);

    ~Pose() = default;

    /**
     * @brief Get the angles in degrees.
     * @return Angles The angles.
     */
    Angles
    getAngles() const;

    Angles
    getOffsettedAngles() const;

    /**
     * @brief Set the angles in degrees.
     * @param angles The angles.
     */
    void
    setAngles(Angles angles);

    void
    randomizeAngles();

    void setVertical();

    /**
     * @brief Get the jacobian matrix.
     * @return JacobianMatrix The jacobian matrix.
     */
    JacobianMatrix
    getJacobianMatrix() const;

    /**
     * @brief Calculate the point with forward kinematics.
     * @param beginPoint The start point (x0, y0, z0).
     * @param jointNr (optional) The joint number
     * @return PosePoint The calculated point.
     */
    PosePoint
    calculatePoint(const PosePoint &beginPoint) const;

    /**
     * @brief Calculate the point with forward kinematics.
     * @param x0 The start x.
     * @param y0 The start y.
     * @param z0 The start z.
     * @return PosePoint The calculated point.
     */
    PosePoint
    calculatePoint(double x0, double y0, double z0) const;

    /**
     * @brief Get the kinematic chain.
     * @return const KinematicChain&
     */
    const KinematicChain &
    getKinematicChain() const;

    /**
     * @brief Get the partial derivative of x based on the current theta (angle) index.
     * @param theta The theta index.
     * @return double The partial derivative of x.
     */
    double
    getPartialDerivativeX(std::size_t theta) const;

    /**
     * @brief Get the partial derivative of y based on the current theta (angle) index.
     * @param theta The theta index.
     * @return double The partial derivative of y.
     */
    double
    getPartialDerivativeY(std::size_t theta) const;

    /**
     * @brief Get the partial derivative of z based on the current theta (angle) index.
     * @param theta The theta index.
     * @return double The partial derivative of z.
     */
    double
    getPartialDerivativeZ(std::size_t theta) const;

    void
    setKinematicChain(const KinematicChain &aKinematicChain);

private:
    /**
     * @brief Calculate the x with forward kinematics.
     * @param x0 The start x.
     * @return double The calculated x.
     */
    double
    calculateX(double x0) const;

    /**
     * @brief Calculate the y with forward kinematics.
     * @param y0 The start y.
     * @return double The calculated y.
     */
    double
    calculateY(double y0) const;

    /**
     * @brief Calculate the z with forward kinematics.
     * @param z0 The start z.
     * @return double The calculated z.
     */
    double
    calculateZ(double z0) const;

    KinematicChain kinematicChain;
};
}

#include "Pose.ipp"

#endif// POSE_H