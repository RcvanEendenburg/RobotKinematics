#ifndef KINEMATIC_CHAIN_H
#define KINEMATIC_CHAIN_H

#include <map>
#include <vector>

#include <robothli/Joint.h>

namespace Kinematics
{
/**
 * @brief The kinematic chain represents the joint structure of the robot
 */
class KinematicChain
{
public:

    KinematicChain() = default;

    /**
     * @brief Construct a kinematic chain.
     * @param first A pointer to the first joint of the chain.
     * @param last A pointer to the last joint of the chain.
     */
    KinematicChain(std::shared_ptr<Joint> first, std::shared_ptr<Joint> last);

    /**
     * @brief Construct a kinematic chain with an initializer list. The joints have to be in order.
     * @param joints The joints.
     */
    KinematicChain(std::initializer_list<std::shared_ptr<Joint>> joints);

    ~KinematicChain() = default;

    /**
     * @brief Get the first joint of the chain.
     * @return std::shared_ptr<Joint> The first joint of the chain.
     */
    std::shared_ptr<Joint>
    begin() const;

    /**
     * @brief Get the last joint of the chain.
     * @return std::shared_ptr<Joint> The last joint of the chain.
     */
    std::shared_ptr<Joint>
    end() const;

    /**
     * @brief Get a specific joint based on an index.
     * @param index An index.
     * @return std::shared_ptr<Joint> The joint at the index.
     * @return nullptr If the index is out of bound.
     */
    std::shared_ptr<Joint>
    at(std::size_t index) const;

    /**
     * @brief Get the sum of the angles that rotate over the z-axis within a range.
     * @param beginJoint The first joint of the range.
     * @param endJoint The last joint of the range. If this is a nullptr, the sum will be calculated with the last joint
     * of the chain.
     * @return double The sum of the angles in radians.
     */
    double
    getAngleSumZ(std::shared_ptr<Joint> beginJoint, const std::shared_ptr<Joint> &endJoint = nullptr) const;

    /**
     * @brief Get the sum of the angles that rotate over the y-axis within a range.
     * @param beginJoint The first joint of the range.
     * @param endJoint The last joint of the range. If this is a nullptr, the sum will be calculated with the last joint
     * of the chain.
     * @return double The sum of the angles in radians.
     */
    double
    getAngleSumY(std::shared_ptr<Joint> beginJoint, const std::shared_ptr<Joint> &endJoint = nullptr) const;

    /**
     * @brief Get the sum of the lengths that don't move. For example the base height.
     * @return double The sum of the lengths.
     */
    double
    getStaticLength() const;

    /**
     * @brief Get the first joint in the chain that rotates over the z-axis.
     * @return std::shared_ptr<Joint> The joint.
     */
    std::shared_ptr<Joint>
    getFirstZRotationJoint() const;

    /**
     * @brief Get the first joint in the chain that rotates over the y-axis.
     * @return std::shared_ptr<Joint> The joint.
     */
    std::shared_ptr<Joint>
    getFirstYRotationJoint() const;

    /**
     * @brief Get the next joint relative to the passed joint that rotates over the z-axis.
     * @param joint The passed joint.
     * @return std::shared_ptr<Joint> The joint.
     */
    std::shared_ptr<Joint>
    getNextZRotationJoint(std::shared_ptr<Joint> joint) const;

    /**
     * @brief Get the next joint relative to the passed joint that rotates over the y-axis.
     * @param joint The passed joint.
     * @return std::shared_ptr<Joint> The joint.
     */
    std::shared_ptr<Joint>
    getNextYRotationJoint(std::shared_ptr<Joint> joint) const;

    void
    set(std::shared_ptr<Joint> first, std::shared_ptr<Joint> last);

    double
    degreesToRadians(double degrees) const;
    double
    radiansToDegrees(double radians) const;

    bool
    operator==(const KinematicChain &Rhs) const;
    bool
    operator!=(const KinematicChain &Rhs) const;

    KinematicChain &
    operator=(const KinematicChain &rhs);

private:
    bool
    isValidJoint(const std::shared_ptr<Joint> &joint) const;
    bool
    areJointsInOrder(const std::shared_ptr<Joint> &first, const std::shared_ptr<Joint> &second) const;

    std::shared_ptr<Joint> firstJoint;
    std::shared_ptr<Joint> lastJoint;
};
}

#endif// KINEMATIC_CHAIN_H