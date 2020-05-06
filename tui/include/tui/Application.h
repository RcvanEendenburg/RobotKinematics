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
    Application(int argc, char **argv, const std::string &configFile);
    ~Application() = default;

    void run();

private:
    friend class Communication::Communicator;

    Utilities::IniParser iniParser;

    Utilities::Logger &logger;
    std::unique_ptr<Mode> currentMode;

    Communication::Communicator communicator;
};

#endif //TUI_INCLUDE_TUI_APPLICATION_H
