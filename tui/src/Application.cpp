//
// Created by derk on 27-4-20.
//

#include <tui/Application.h>
#include <tui/DevelopMode.h>
#include <utilities/LogToCout.h>

#include <iostream>

Application::Application(int argc, char **argv, const std::string &configFile) :
                             logger(Utilities::Logger::instance()),
                             iniParser((Utilities::Logger::instance().setLogOutput(std::make_unique<Utilities::LogToCout>()), configFile)),
                             currentMode(nullptr),
                             communicator(((iniParser.parse()), ros::init(argc, argv, iniParser.get<std::string>("TUI", "node_name")),
                             Communication::Communicator(iniParser.get<std::string>("TUI", "high_level_driver_name"), *this)))
{
}

void Application::run()
{
  std::string operation;
  while(true)
  {
    if(!currentMode || !currentMode->isStarted())
    {
        logger.log(Utilities::LogLevel::Raw, "Type 'develop' to access the developer mode.");
        logger.log(Utilities::LogLevel::Raw, "'exit' will exit the application.");
        std::cin >> operation;
        if (operation == "develop")
        {
            currentMode = std::move(std::make_unique<DevelopMode>(communicator, iniParser.get<int>("TUI", "standard_z")));
            currentMode->start();
        }
        else if (operation == "exit")
            break;
    }
  }
}