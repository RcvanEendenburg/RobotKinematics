#ifndef JOINT_H
#define JOINT_H

#include <memory>
#include <tuple>

namespace Kinematics
{
/**
 * @brief This class contains the joint with its properties.
 */
class Joint : public std::enable_shared_from_this<Joint>
{
public:
    /**
     * @brief Construct a joint.
     * @param aLength The length of the part of the arm which the joint is connected to.
     * @param anAngle The current angle of the joint in degrees.
     * @param aYRotate If set to true, the joint can rotate over the y-axis. If false, it can only move over the z-axis.
     * @param aStaticLength If set to true, the length of the part of the arm which the joint is connected to doesn't
     * effect the rotation of this joint.
     */
    Joint(double aLength,
          double anAngle,
          double anAngleOffset,
          double aLowerBound,
          double anUpperBound,
          bool aYRotate,
          bool aStaticLength);
    ~Joint() = default;

    /**
     * @brief Get the current angle of the joint.
     * @return double The current angle of the joint.
     */
    double
    getAngle() const;

    /**
     * @brief Set the angle of the joint.
     * @details If the angle is lower than the lower bound, the angle will be set to the lower bound.
     * If the angle is higher than the upper bound, the angle will be set to the upper bound.
     * @param anAngle An angle in degrees.
     */
    void
    setAngle(double anAngle);

    /**
     * @brief Check if this joint can rotate over the y-axis or z-axis.
     * @return true The joint can move over the y-axis.
     * @return false The joint can move over the z-axis.
     */
    bool
    canYRotate() const;

    /**
     * @brief Check if this joint has a static length. This means that the length of the joint doesn't influence the
     * rotation of the joint.
     * @return true If this joint has a static length.
     * @return false If this joint doesn't have a static length.
     */
    bool
    isStatic() const;

    /**
     * @brief Get the length of the part of the arm which this joint is connected to.
     * @return double The length.
     */
    double
    getLength() const;

    double
    getLowerBound() const;

    double
    getUpperBound() const;

    double getAngleOffset() const;

    /**
     * @brief Set the next joint of this joint.
     * This will also set this joint as the previous joint of the next joint.
     * @param aNext The next joint relative to this joint.
     */
    void
    setNext(std::shared_ptr<Joint> aNext);

    /**
     * @brief Get the previous joint relative to this joint.
     * @return std::shared_ptr<Joint> The previous joint.
     * @return nullptr If the joint has no previous joint.
     */
    std::shared_ptr<Joint>
    getPrevious() const;

    /**
     * @brief Get the next joint relative to this joint.
     * @return std::shared_ptr<Joint> The next joint.
     * @return nullptr If the joint has no next joint.
     */
    std::shared_ptr<Joint>
    getNext() const;

private:
    double length;
    double angle;
    double angleOffset;
    double lowerBound;
    double upperBound;
    bool yRotate;
    bool staticLength;
    std::shared_ptr<Joint> next;
    std::shared_ptr<Joint> previous;
};
}

#endif// JOINT_HPP