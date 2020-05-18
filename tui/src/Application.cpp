//
// Created by derk on 27-4-20.
//

#include <tui/Application.h>
#include <tui/MainMenu.h>
#include <utilities/LogToCout.h>

Application::Application(int argc, char **argv, const std::string &configFile) :
                             logger(Utilities::Logger::instance()),
                             iniParser((Utilities::Logger::instance().setLogOutput(std::make_unique<Utilities::LogToCout>()), configFile)),
                             communicator(((iniParser.parse()), ros::init(argc, argv, iniParser.get<std::string>("TUI", "node_name")),
                             Communication::Communicator(iniParser.get<std::string>("TUI", "high_level_driver_name"), *this)))
{
}

void Application::run()
{
    MainMenu menu(iniParser, logger, communicator);
    menu.start();
}