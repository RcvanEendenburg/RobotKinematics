#include <gtest/gtest.h>
#include <robothli/Joint.h>

TEST(JointSuite, normalInitialization)
{
    double length = 0.8;
    double angle = 40;
    double lowerBound = 10;
    double upperBound = 90;
    bool canYRotate = false;
    bool isStaticLength = false;

    Kinematics::Joint joint(length, angle, 0, lowerBound, upperBound, canYRotate, isStaticLength);
    ASSERT_EQ(joint.getLength(), length);
    ASSERT_EQ(joint.getAngle(), angle);
    ASSERT_EQ(joint.getLowerBound(), lowerBound);
    ASSERT_EQ(joint.getUpperBound(), upperBound);
    ASSERT_EQ(joint.canYRotate(), canYRotate);
    ASSERT_EQ(joint.isStatic(), isStaticLength);
}

TEST(JointSuite, testAngleBounds)
{
    double lowerBound = 10;
    double upperBound = 90;

    Kinematics::Joint joint(0.2, 50, 0, lowerBound, upperBound, true, true);
    joint.setAngle(100);
    ASSERT_EQ(joint.getAngle(), joint.getUpperBound());
    joint.setAngle(0);
    ASSERT_EQ(joint.getAngle(), joint.getLowerBound());
}

TEST(JointSuite, testNextAndPrevious)
{
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    j1->setNext(j2);

    ASSERT_EQ(j1, j2->getPrevious());
    ASSERT_EQ(j2, j1->getNext());
}

int
main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}