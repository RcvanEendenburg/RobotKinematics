//
// Created by derk on 15-5-20.
//

#include <tui/MainMenu.h>
#include <tui/DevelopMode.h>
#include <tui/SinglePositionMode.h>
#include <tui/SequenceMode.h>

MainMenu::MainMenu(Utilities::IniParser& anIniParser, Utilities::Logger& aLogger, Communication::Communicator& communicator) :
Mode(communicator), iniParser(anIniParser), logger(aLogger)
{
    addSingleOperation(keywordToString(Keyword::Develop), [this](){startDevelopMode();});
    addSingleOperation(keywordToString(Keyword::Single), [this](){startSinglePositionMode();});
    addSingleOperation(keywordToString(Keyword::Sequence), [this](){startSequenceMode();});
}

void MainMenu::startDevelopMode()
{
    currentMode = std::move(std::make_unique<DevelopMode>(communicator));
    currentMode->start();
}

void MainMenu::startSinglePositionMode()
{
    currentMode = std::move(std::make_unique<SinglePositionMode>(communicator));
    currentMode->start();
}

void MainMenu::startSequenceMode()
{
    currentMode = std::move(std::make_unique<SequenceMode>(communicator));
    currentMode->start();
}

void MainMenu::start()
{
    std::stringstream ss;
    ss << "Type 'develop' to access the developer mode." << std::endl;
    ss << "Type 'interactive' to access the interactive mode." << std::endl;
    ss << "'exit' will exit the application." << std::endl;
    setWelcomeMessage(ss.str());
    setExitMessage("Exiting...");
    Mode::start();
}