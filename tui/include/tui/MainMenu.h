//
// Created by derk on 15-5-20.
//

#ifndef MAINMENU_H
#define MAINMENU_H

#include <tui/Mode.h>
#include <utilities/IniParser.h>
#include <utilities/Logger.h>
#include <tui/Communicator.h>

class MainMenu : public Mode
{
public:
    MainMenu(Utilities::IniParser& anIniParser, Utilities::Logger& aLogger, Communication::Communicator& communicator);
    ~MainMenu() override = default;
    virtual void start() override;

private:
    void startSinglePositionMode();
    void startSequenceMode();
    void startDevelopMode();

    Utilities::IniParser& iniParser;
    Utilities::Logger &logger;
    std::unique_ptr<Mode> currentMode;
};


#endif //MAINMENU_H
