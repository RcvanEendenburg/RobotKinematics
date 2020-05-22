#include <robothli/Application.h>
#include <utilities/LogToCout.h>
#include <robothli/Al5D.h>

Application::Application(int argc, char **argv, const std::string &configFile) :
    logger(Utilities::Logger::instance()),
    iniParser((logger.setLogOutput(std::make_unique<Utilities::LogToCout>()), configFile)),
    communicator(((iniParser.parse()), ros::init(argc, argv, iniParser.get<std::string>("RobotHLI", "node_name")),
        Communication::Communicator(iniParser.get<std::string>("RobotHLI", "node_name"),
                                    iniParser.get<std::string>("RobotHLI", "low_level_driver_name"), *this))),
    start(iniParser.get<double>("Kinematics", "start_x"), iniParser.get<double>("Kinematics", "start_y"),
          iniParser.get<double>("Kinematics", "start_z"))
{
    auto robotModel = iniParser.get<std::string>("RobotHLI", "robot_model");
    std::transform(robotModel.begin(), robotModel.end(), robotModel.begin(), ::tolower);
    if (robotModel!="al5d")
        throw std::runtime_error("Robot not supported!");
    std::array<double, 4> currentAngles =
        {
            iniParser.get<double>("Kinematics", "start_angle_base"),
            iniParser.get<double>("Kinematics", "start_angle_shoulder"),
            iniParser.get<double>("Kinematics", "start_angle_elbow"),
            iniParser.get<double>("Kinematics", "start_angle_wrist")};

    robot = std::move(std::make_unique<Robot::Al5D>(start, currentAngles));

    logger.log(Utilities::LogLevel::Debug, "Starting application...");
}

void
Application::moveToGoal(double x, double y, double z, double rotation, double openingDistance)
{
    static Kinematics::Beta beta
        {
            .bigFactor = iniParser.get<double>("Kinematics", "beta_big_factor"),
            .smallFactor = iniParser.get<double>("Kinematics", "beta_small_factor"),
            .begin = iniParser.get<double>("Kinematics", "beta_begin"),
        };
    static Kinematics::GradientDescent<4> algorithm(beta, std::move(robot),
                                                    iniParser.get<double>("Kinematics", "maximum_magnitude"),
                                                    iniParser.get<double>("Kinematics", "maximum_iterations"));
    algorithm.saveAngles();
    try
    {
        algorithm.startMoving(Kinematics::PosePoint(x, y, z));
        auto angles = algorithm.getCurrentAngles();
        communicator.move(angles[0], angles[1], angles[2], angles[3], rotation, openingDistance,
                          static_cast<unsigned short>(iniParser.get<int>("Robot", "move_time")));
    }
    catch (Kinematics::UnableToMove& e)
    {
        logger.log(Utilities::LogLevel::Error, e.what());
        logger.log(Utilities::LogLevel::Warning, "Changing angles to previous angles...");
        algorithm.restoreAngles();
    }
    catch (std::exception &e)
    {
        logger.log(Utilities::LogLevel::Error, e.what());
    }
}

void
Application::run() const
{
    ros::spin();
}