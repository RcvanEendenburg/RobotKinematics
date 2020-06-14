#include <gtest/gtest.h>
#include <robothli/KinematicChain.h>

TEST(KinematicChainSuite, initializationList)
{
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);

    Kinematics::KinematicChain chain({j1, j2, j3});

    //Testing begin and end
    ASSERT_EQ(chain.begin(), j1);
    ASSERT_EQ(chain.end(), j3);

    //Manually testing the joint structure
    ASSERT_EQ(j1->getNext(), j2);
    ASSERT_EQ(j2->getPrevious(), j1);

    ASSERT_EQ(j2->getNext(), j3);
    ASSERT_EQ(j3->getPrevious(), j2);
}

TEST(KinematicChainSuite, setJointStructureManually)
{
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);

    j1->setNext(j2);
    j2->setNext(j3);

    Kinematics::KinematicChain chain(j1, j3);

    //Testing begin and end
    ASSERT_EQ(chain.begin(), j1);
    ASSERT_EQ(chain.end(), j3);

    //Manually testing the joint structure
    ASSERT_EQ(j1->getNext(), j2);
    ASSERT_EQ(j2->getPrevious(), j1);

    ASSERT_EQ(j2->getNext(), j3);
    ASSERT_EQ(j3->getPrevious(), j2);
}

TEST(KinematicChainSuite, setJointStructureManuallyDefaultConstructor)
{
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);

    j1->setNext(j2);
    j2->setNext(j3);

    Kinematics::KinematicChain chain;
    chain.set(j1, j3);

    //Testing begin and end
    ASSERT_EQ(chain.begin(), j1);
    ASSERT_EQ(chain.end(), j3);

    //Manually testing the joint structure
    ASSERT_EQ(j1->getNext(), j2);
    ASSERT_EQ(j2->getPrevious(), j1);

    ASSERT_EQ(j2->getNext(), j3);
    ASSERT_EQ(j3->getPrevious(), j2);
}

TEST(KinematicChainSuite, retrieveJointByIndex)
{
    Kinematics::KinematicChain chain;
    ASSERT_THROW(chain.at(0), std::runtime_error);

    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, 50, 0, 10, 100, true, true);

    j1->setNext(j2);
    j2->setNext(j3);

    chain.set(j1, j3);
    ASSERT_THROW(chain.at(3), std::runtime_error);

    //Testing by index
    ASSERT_EQ(j1, chain.at(0));
    ASSERT_EQ(j2, chain.at(1));
    ASSERT_EQ(j3, chain.at(2));
}

TEST(KinematicChainSuite, radianDegreesConversion)
{
    double beginDegrees = 90;
    Kinematics::KinematicChain chain;

    double radians = chain.degreesToRadians(beginDegrees);
    double degrees = chain.radiansToDegrees(radians);
    ASSERT_EQ(beginDegrees, degrees);
}

TEST(KinematicChainSuite, getAngleSumZ)
{
    //The sum of the angles when not rotating over the y-axis
    double a1 = 20;
    double a2 = 50;
    double a3 = 70;
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, a1, 0, 10, 100, false, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, a2, 0, 10, 100, false, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, a3, 0, 10, 100, false, true);
    auto j4 = std::make_shared<Kinematics::Joint>(0.2, 20, 0, 10, 100, true, true);

    Kinematics::KinematicChain chain({j1, j2, j3});

    a1 = chain.degreesToRadians(a1);
    a2 = chain.degreesToRadians(a2);
    a3 = chain.degreesToRadians(a3);

    ASSERT_EQ(chain.getAngleSumZ(j1), a1 + a2 + a3);
    ASSERT_EQ(chain.getAngleSumZ(j1, j3), a1 + a2 + a3);
    ASSERT_EQ(chain.getAngleSumZ(j2), a2 + a3);
    ASSERT_EQ(chain.getAngleSumZ(j2, j3), a2 + a3);
    ASSERT_EQ(chain.getAngleSumZ(j3), a3);
    ASSERT_EQ(chain.getAngleSumZ(j3, j3), a3);
    ASSERT_EQ(chain.getAngleSumZ(j2, j2), a2);
    ASSERT_EQ(chain.getAngleSumZ(j1, j1), a1);

    //Test invalid joint
    ASSERT_THROW(chain.getAngleSumZ(j4), std::runtime_error);

    //Test if joints are not in the correct order
    ASSERT_THROW(chain.getAngleSumZ(j4, j2), std::runtime_error);

    //Test if joints that rotate over the y-axis will be ignored
    Kinematics::KinematicChain chain2({j1, j2, j3, j4});
    ASSERT_EQ(chain2.getAngleSumZ(j1), a1 + a2 + a3);
}

TEST(KinematicChainSuite, getAngleSumY)
{
    //The sum of the angles when not rotating over the z-axis
    double a1 = 20;
    double a2 = 50;
    double a3 = 70;
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, a1, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, a2, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, a3, 0, 10, 100, true, true);
    auto j4 = std::make_shared<Kinematics::Joint>(0.2, 20, 0, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3});

    a1 = chain.degreesToRadians(a1);
    a2 = chain.degreesToRadians(a2);
    a3 = chain.degreesToRadians(a3);

    ASSERT_EQ(chain.getAngleSumY(j1), a1 + a2 + a3);
    ASSERT_EQ(chain.getAngleSumY(j1, j3), a1 + a2 + a3);
    ASSERT_EQ(chain.getAngleSumY(j2), a2 + a3);
    ASSERT_EQ(chain.getAngleSumY(j2, j3), a2 + a3);
    ASSERT_EQ(chain.getAngleSumY(j3), a3);
    ASSERT_EQ(chain.getAngleSumY(j3, j3), a3);
    ASSERT_EQ(chain.getAngleSumY(j2, j2), a2);
    ASSERT_EQ(chain.getAngleSumY(j1, j1), a1);

    //Test invalid joint
    ASSERT_THROW(chain.getAngleSumY(j4), std::runtime_error);

    //Test if joints are not in the correct order
    ASSERT_THROW(chain.getAngleSumY(j4, j2), std::runtime_error);

    //Test if joints that rotate over the z-axis will be ignored
    Kinematics::KinematicChain chain2({j1, j2, j3, j4});
    ASSERT_EQ(chain2.getAngleSumY(j1), a1 + a2 + a3);
}

TEST(KinematicChainSuite, getStaticLength)
{
    //The sum of the angles when not rotating over the z-axis
    double l1 = 0.5;
    double l2 = 2.2;
    double l3 = 1.6;
    auto j1 = std::make_shared<Kinematics::Joint>(l1, 20, 0, 10, 100, false, true);
    auto j2 = std::make_shared<Kinematics::Joint>(l2, 30, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(l3, 40, 0, 10, 100, true, true);

    Kinematics::KinematicChain chain({j1, j2, j3});

    ASSERT_EQ(chain.getStaticLength(), l1 + l2 + l3);
}

TEST(KinematicChainSuite, getNextZRotationJoint)
{
    auto j1 = std::make_shared<Kinematics::Joint>(1.3, 20, 0, 10, 100, false, true);
    auto j2 = std::make_shared<Kinematics::Joint>(2.3, 30, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(1, 40, 0, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3});

    ASSERT_EQ(chain.getNextZRotationJoint(j1), j1);
    ASSERT_EQ(chain.getNextZRotationJoint(j2), j3);
    ASSERT_EQ(chain.getNextZRotationJoint(j3), j3);
    ASSERT_EQ(chain.getNextZRotationJoint(nullptr), nullptr);
}

TEST(KinematicChainSuite, getNextYRotationJoint)
{
    auto j1 = std::make_shared<Kinematics::Joint>(1.3, 20, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(2.3, 30, 0, 10, 100, false, true);
    auto j3 = std::make_shared<Kinematics::Joint>(1, 40, 0, 10, 100, true, true);

    Kinematics::KinematicChain chain({j1, j2, j3});

    ASSERT_EQ(chain.getNextYRotationJoint(j1), j1);
    ASSERT_EQ(chain.getNextYRotationJoint(j2), j3);
    ASSERT_EQ(chain.getNextYRotationJoint(j3), j3);
    ASSERT_EQ(chain.getNextYRotationJoint(nullptr), nullptr);
}

TEST(KinematicChainSuite, getFirstZRotationJoint)
{
    auto j1 = std::make_shared<Kinematics::Joint>(1.3, 20, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(2.3, 30, 0, 10, 100, false, true);
    auto j3 = std::make_shared<Kinematics::Joint>(1, 40, 0, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3});

    ASSERT_EQ(chain.getFirstZRotationJoint(), j2);
}

TEST(KinematicChainSuite, getFirstYRotationJoint)
{
    auto j1 = std::make_shared<Kinematics::Joint>(1.3, 20, 0, 10, 100, false, true);
    auto j2 = std::make_shared<Kinematics::Joint>(2.3, 30, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(1, 40, 0, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3});

    ASSERT_EQ(chain.getFirstYRotationJoint(), j2);
}

TEST(KinematicChainSuite, testBadSet)
{
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 19, 0, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, 10, 0, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, 10, 0, 10, 100, true, true);
    auto j4 = std::make_shared<Kinematics::Joint>(0.2, 20, 0, 10, 100, false, true);

    Kinematics::KinematicChain chain;

    ASSERT_THROW(chain.set(j1, j4), std::runtime_error);
}

int
main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}