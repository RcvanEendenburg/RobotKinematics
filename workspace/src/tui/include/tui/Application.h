//
// Created by derk on 27-4-20.
//

#ifndef TUI_INCLUDE_TUI_APPLICATION_H
#define TUI_INCLUDE_TUI_APPLICATION_H

#include <tui/Mode.h>
#include <memory>
#include <utilities/IniParser.h>
#include <utilities/Logger.h>
#include <tui/Communicator.h>

class Application
{
public:
    /**
     * Sets up the application.
     * @param configFile The config file of the TUI.
     */
    Application(int argc, char **argv, const std::string &configFile);
    ~Application() = default;

    /**
     * Initiates the application
     */
    void run();

private:
    friend class Communication::Communicator;

    Utilities::Logger &logger;
    Utilities::IniParser iniParser;
    Communication::Communicator communicator;
};

#endif //TUI_INCLUDE_TUI_APPLICATION_H
