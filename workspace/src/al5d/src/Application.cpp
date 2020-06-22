#include <al5d/Application.h>
#include <utilities/LogToCout.h>

Application::Application(int argc, char **argv, const std::string &configFile) :
    logger(Utilities::Logger::instance()),
    iniParser((logger.setLogOutput(std::make_unique<Utilities::LogToCout>()), configFile)),
    communicator(((iniParser.parse()), ros::init(argc, argv, iniParser.get<std::string>("AL5D", "node_name")),
        Communication::Communicator(iniParser.get<std::string>("AL5D", "node_name"), *this))),
    controller(iniParser.get<std::string>("AL5D", "serial_port"))
{
    logger.log(Utilities::LogLevel::Debug, "Starting application...");
    controller.setBaseChannel(static_cast<unsigned short>(iniParser.get<int>("Joints", "base_channel")));
    controller.setShoulderChannel(static_cast<unsigned short>(iniParser.get<int>("Joints", "shoulder_channel")));
    controller.setElbowChannel(static_cast<unsigned short>(iniParser.get<int>("Joints", "elbow_channel")));
    controller.setWristChannel(static_cast<unsigned short>(iniParser.get<int>("Joints", "wrist_channel")));
    controller.setWristRotateChannel(static_cast<unsigned short>(iniParser.get<int>("Joints", "wrist_rotate_channel")));
    controller.setGripperChannel(static_cast<unsigned short>(iniParser.get<int>("Joints", "gripper_channel")));

    controller.setBaseLimits({iniParser.get<double>("Joints", "base_min_angle"),
                             iniParser.get<double>("Joints", "base_max_angle")},
                             {static_cast<unsigned short>(iniParser.get<int>("Joints", "base_min_pwm")),
                             static_cast<unsigned short>(iniParser.get<int>("Joints", "base_max_pwm"))});

    controller.setShoulderLimits({iniParser.get<double>("Joints", "shoulder_min_angle"),
                                 iniParser.get<double>("Joints", "shoulder_max_angle")},
                                 {static_cast<unsigned short>(iniParser.get<int>("Joints", "shoulder_min_pwm")),
                                 static_cast<unsigned short>(iniParser.get<int>("Joints", "shoulder_max_pwm"))});

    controller.setElbowLimits({iniParser.get<double>("Joints", "elbow_min_angle"),
                              iniParser.get<double>("Joints", "elbow_max_angle")},
                              {static_cast<unsigned short>(iniParser.get<int>("Joints", "elbow_min_pwm")),
                              static_cast<unsigned short>(iniParser.get<int>("Joints", "elbow_max_pwm"))});

    controller.setWristLimits({iniParser.get<double>("Joints", "wrist_min_angle"),
                              iniParser.get<double>("Joints", "wrist_max_angle")},
                              {static_cast<unsigned short>(iniParser.get<int>("Joints", "wrist_min_pwm")),
                              static_cast<unsigned short>(iniParser.get<int>("Joints", "wrist_max_pwm"))});

    controller.setWristRotateLimits({iniParser.get<double>("Joints", "wrist_rotate_min_angle"),
                                    iniParser.get<double>("Joints", "wrist_rotate_max_angle")},
                                    {static_cast<unsigned short>(iniParser.get<int>("Joints", "wrist_rotate_min_pwm")),
                                    static_cast<unsigned short>(iniParser.get<int>("Joints", "wrist_rotate_max_pwm"))});

    controller.setGripperLimits({iniParser.get<double>("Joints", "gripper_min_distance"),
                                iniParser.get<double>("Joints", "gripper_max_distance")},
                                {static_cast<unsigned short>(iniParser.get<int>("Joints", "gripper_min_pwm")),
                                static_cast<unsigned short>(iniParser.get<int>("Joints", "gripper_max_pwm"))});

    controller.setBaseSpeed(static_cast<unsigned short>(iniParser.get<int>("Joints", "base_speed")));
    controller.setShoulderSpeed(static_cast<unsigned short>(iniParser.get<int>("Joints", "shoulder_speed")));
    controller.setElbowSpeed(static_cast<unsigned short>(iniParser.get<int>("Joints", "elbow_speed")));
    controller.setWristSpeed(static_cast<unsigned short>(iniParser.get<int>("Joints", "wrist_speed")));
    controller.setWristRotateSpeed(static_cast<unsigned short>(iniParser.get<int>("Joints", "wrist_rotate_speed")));
    controller.setGripperSpeed(static_cast<unsigned short>(iniParser.get<int>("Joints", "gripper_speed")));

    controller.setGlobalTime(static_cast<unsigned short>(iniParser.get<int>("AL5D", "global_time")));
}

void
Application::move(double baseAngle, double shoulderAngle, double elbowAngle,
                  double wristAngle, double wristRotateAngle, double gripperDistance, unsigned short time)
{
    controller.setBaseAngle(baseAngle);
    controller.setShoulderAngle(shoulderAngle);
    controller.setElbowAngle(elbowAngle);
    controller.setWristAngle(wristAngle);
    controller.setWristRotateAngle(wristRotateAngle);
    controller.setGripperDistance(gripperDistance);
    controller.move(time);
}

void
Application::run() const
{
    ros::spin();
}