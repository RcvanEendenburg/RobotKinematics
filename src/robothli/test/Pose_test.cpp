#include <gtest/gtest.h>

#include <robothli/Pose.h>

TEST(PoseSuite, initialization)
{
    Kinematics::KinematicChain chain;
    Kinematics::Pose<3> pose(chain);

    ASSERT_EQ(pose.getKinematicChain(), chain);
}

TEST(PoseSuite, getAngles)
{
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 50, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, 70, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, 80, 10, 100, true, true);
    auto j4 = std::make_shared<Kinematics::Joint>(0.2, 20, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3, j4});

    const int dof = 4;

    Kinematics::Pose<dof> pose(chain);

    Kinematics::Matrix<double, 1, dof> matrix;
    for (std::size_t i = 0; i < dof; ++i)
        matrix[0][i] = chain.at(i)->getAngle();

    ASSERT_EQ(matrix, pose.getAngles());
}

TEST(PoseSuite, setAngles)
{
    const std::size_t dof = 4;
    std::array<double, dof> angles = {20, 30, 10, 20};
    Kinematics::Matrix<double, 1, dof> matrix;
    for (std::size_t i = 0; i < dof; ++i)
        matrix[0][i] = angles[i];

    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 50, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.2, 70, 10, 100, true, true);
    auto j3 = std::make_shared<Kinematics::Joint>(0.2, 80, 10, 100, true, true);
    auto j4 = std::make_shared<Kinematics::Joint>(0.2, 20, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3, j4});

    Kinematics::Pose<dof> pose(chain);
    pose.setAngles(matrix);

    ASSERT_EQ(pose.getAngles(), matrix);
}

TEST(PoseSuite, getPartialDerivativeX)
{
    double a1 = 50;
    double a2 = 20;
    double a3 = 10;
    double a4 = 20;

    const double l1 = 1.2;
    const double l2 = 2.3;
    const double l3 = 1;
    const double l4 = 0.2;

    const std::size_t dof = 4;

    auto j1 = std::make_shared<Kinematics::Joint>(l1, a1, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(l2, a2, 10, 100, false, true);
    auto j3 = std::make_shared<Kinematics::Joint>(l3, a3, 10, 100, false, true);
    auto j4 = std::make_shared<Kinematics::Joint>(l4, a4, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3, j4});

    a1 = chain.degreesToRadians(a1);
    a2 = chain.degreesToRadians(a2);
    a3 = chain.degreesToRadians(a3);
    a4 = chain.degreesToRadians(a4);

    Kinematics::Pose<dof> pose(chain);

    ASSERT_EQ(pose.getPartialDerivativeX(0), std::cos(a1));
    ASSERT_EQ(pose.getPartialDerivativeX(1), l2*std::cos(a2) + l3*std::cos(a2 + a3) + l4*std::cos(a2 + a3 + a4));
    ASSERT_EQ(pose.getPartialDerivativeX(2), l3*std::cos(a2 + a3) + l4*std::cos(a2 + a3 + a4));
    ASSERT_EQ(pose.getPartialDerivativeX(3), l4*std::cos(a2 + a3 + a4));
}

TEST(PoseSuite, getPartialDerivativeY)
{
    double a1 = 50;
    double a2 = 20;
    double a3 = 10;
    double a4 = 20;

    const double l1 = 1.2;
    const double l2 = 2.3;
    const double l3 = 1;
    const double l4 = 0.2;

    const std::size_t dof = 4;

    auto j1 = std::make_shared<Kinematics::Joint>(l1, a1, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(l2, a2, 10, 100, false, true);
    auto j3 = std::make_shared<Kinematics::Joint>(l3, a3, 10, 100, false, true);
    auto j4 = std::make_shared<Kinematics::Joint>(l4, a4, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3, j4});

    a1 = chain.degreesToRadians(a1);
    a2 = chain.degreesToRadians(a2);
    a3 = chain.degreesToRadians(a3);
    a4 = chain.degreesToRadians(a4);

    Kinematics::Pose<dof> pose(chain);

    ASSERT_EQ(pose.getPartialDerivativeY(0), -std::sin(a1));
    ASSERT_EQ(pose.getPartialDerivativeY(1), 0);
    ASSERT_EQ(pose.getPartialDerivativeY(2), 0);
    ASSERT_EQ(pose.getPartialDerivativeY(3), 0);
}

TEST(PoseSuite, getPartialDerivativeZ)
{
    double a1 = 50;
    double a2 = 20;
    double a3 = 10;
    double a4 = 20;

    const double l1 = 1.2;
    const double l2 = 2.3;
    const double l3 = 1;
    const double l4 = 0.2;

    const std::size_t dof = 4;

    auto j1 = std::make_shared<Kinematics::Joint>(l1, a1, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(l2, a2, 10, 100, false, true);
    auto j3 = std::make_shared<Kinematics::Joint>(l3, a3, 10, 100, false, true);
    auto j4 = std::make_shared<Kinematics::Joint>(l4, a4, 10, 100, false, true);

    Kinematics::KinematicChain chain({j1, j2, j3, j4});

    a1 = chain.degreesToRadians(a1);
    a2 = chain.degreesToRadians(a2);
    a3 = chain.degreesToRadians(a3);
    a4 = chain.degreesToRadians(a4);

    Kinematics::Pose<dof> pose(chain);

    ASSERT_EQ(pose.getPartialDerivativeZ(0), 0);
    ASSERT_EQ(pose.getPartialDerivativeZ(1), l2*-std::sin(a2) + l3*-std::sin(a2 + a3) + l4*-std::sin(a2 + a3 + a4));
    ASSERT_EQ(pose.getPartialDerivativeZ(2), l3*-std::sin(a2 + a3) + l4*-std::sin(a2 + a3 + a4));
    ASSERT_EQ(pose.getPartialDerivativeZ(3), l4*-std::sin(a2 + a3 + a4));
}

TEST(PoseSuite, getJacobianMatrix)
{
    auto j1 = std::make_shared<Kinematics::Joint>(0.2, 20, 10, 100, true, true);
    auto j2 = std::make_shared<Kinematics::Joint>(0.1, 80, 10, 100, false, true);
    auto j3 = std::make_shared<Kinematics::Joint>(2.1, 90, 10, 100, false, true);
    auto j4 = std::make_shared<Kinematics::Joint>(2.2, 110, 10, 120, false, true);

    const std::size_t dof = 4;

    Kinematics::KinematicChain chain({j1, j2, j3, j4});
    Kinematics::Pose<dof> pose(chain);

    Kinematics::Matrix<double, 3, dof> matrix;

    for (std::size_t i = 0; i < dof; ++i)
    {
        matrix[0][i] = pose.getPartialDerivativeX(i);
        matrix[1][i] = pose.getPartialDerivativeY(i);
        matrix[2][i] = pose.getPartialDerivativeZ(i);
    }

    ASSERT_EQ(matrix, pose.getJacobianMatrix());
}

int
main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}