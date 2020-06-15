//
// Created by derk on 15-5-20.
//

#include <tui/MainMenu.h>
#include <tui/DevelopMode.h>
#include <tui/SinglePositionMode.h>
#include <tui/SequenceMode.h>

MainMenu::MainMenu(Utilities::IniParser& anIniParser, Communication::Communicator& communicator) :
Mode(communicator), iniParser(anIniParser)
{
    addSingleOperation(keywordToString(Keyword::Develop), [this](){startDevelopMode();});
    addSingleOperation(keywordToString(Keyword::Single), [this](){startSinglePositionMode();});
    addSingleOperation(keywordToString(Keyword::Sequence), [this](){startSequenceMode();});
}

void MainMenu::startDevelopMode()
{
    auto developMode = std::move(std::make_unique<DevelopMode>(communicator));
    developMode->start();
}

void MainMenu::startSinglePositionMode()
{
    auto singlePositionMode = std::move(std::make_unique<SinglePositionMode>(communicator));
    singlePositionMode->start();
}

void MainMenu::startSequenceMode()
{
    auto sequenceMode = std::move(std::make_unique<SequenceMode>(communicator, iniParser.get<double>("TUI", "gripper_max_open_distance"),
        iniParser.get<double>("TUI","gripper_opening_height")));
    sequenceMode->start();
}

void MainMenu::start()
{
    std::stringstream ss;
    ss << "Type 'develop' to access the developer mode." << std::endl;
    ss << "Type 'single' to send the robot arm to single positions." << std::endl;
    ss << "Type 'sequence' to pick up a block and put it in the goal." << std::endl;
    ss << "'exit' will exit the application." << std::endl;
    setWelcomeMessage(ss.str());
    setExitMessage("Exiting...");
    Mode::start();
}