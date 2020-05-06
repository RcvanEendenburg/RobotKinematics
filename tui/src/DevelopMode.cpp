//
// Created by derk on 27-4-20.
//

#include <tui/DevelopMode.h>

DevelopMode::DevelopMode(Communication::Communicator& communicator) : Mode(communicator)
{
    addOperationWithArgument(keywordToString(Keyword::Goto), [this](const std::string& arg){return goToPosition(arg);});
}

void DevelopMode::goToPosition(const std::string& positionStr)
{
    std::string str = positionStr;
    auto trimLeftBracketPos = str.find_first_not_of('(');
    str = str.substr(trimLeftBracketPos != std::string::npos ? trimLeftBracketPos : 0);
    auto trimRightBracketPos = str.find_last_not_of(')');
    str = str.substr(0, trimRightBracketPos != std::string::npos ? trimRightBracketPos + 1 : str.size() - 1);
    std::string token;
    std::vector<unsigned int> coordinates;
    while(true)
    {
        std::size_t pos = str.find(',');
        token = str.substr(0, pos);
        str.erase(0, pos + 1);
        coordinates.push_back(static_cast<unsigned int>(std::stoi(token)));
        if(pos == std::string::npos) break;
    }

    if(coordinates.size() != 3)
        throw std::runtime_error("This is not a valid coordinate!");

    communicator.goToPosition(coordinates.at(0), coordinates.at(1), coordinates.at(2));
}

void DevelopMode::start()
{
    setWelcomeMessage("Use goto (x,y,z) to go to a specific position.");
    setExitMessage("Exiting developer mode...");
    Mode::start();
}