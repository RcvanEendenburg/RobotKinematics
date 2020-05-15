//
// Created by derk on 27-4-20.
//

#include <tui/Mode.h>

#include <iostream>
#include <thread>
#include <sstream>

const std::map<Mode::Keyword, std::string> Mode::keywordTable
{
    {Keyword::Quit, "exit"},
    {Keyword::Goto, "goto"},
    {Keyword::Black, "black"},
    {Keyword::Blue, "blue"},
    {Keyword::Green, "green"},
    {Keyword::Red, "red"},
    {Keyword::Yellow, "yellow"},
    {Keyword::White, "white"},
    {Keyword::Rectangle, "rectangle"},
    {Keyword::Square, "square"},
    {Keyword::Circle, "circle"},
    {Keyword::All, "all"},
    {Keyword::Wood, "wood"}
};

const std::string& Mode::keywordToString(Keyword keyword)
{
    auto result = std::find_if(keywordTable.begin(), keywordTable.end(),
                               [keyword](const auto& keywordBinding){return keywordBinding.first == keyword;});
    if(result != keywordTable.end())
        return result->second;
    throw std::runtime_error("Couldn't find the string to keyword");
}

Mode::Keyword Mode::stringToKeyword(const std::string& str)
{
    auto result = std::find_if(keywordTable.begin(), keywordTable.end(),
                               [str](const auto& keywordBinding){return keywordBinding.second == str;});
    if(result != keywordTable.end())
        return result->first;
    throw std::runtime_error("Couldn't find the keyword");
}

Mode::Mode(Communication::Communicator& aCommunicator) : started(true), communicator(aCommunicator), welcomeMessage(), exitMessage(),
logger(Utilities::Logger::instance())
{
    addSingleOperation(keywordToString(Keyword::Quit), std::bind(&Mode::exitAction, this));
}

bool Mode::isStarted() const
{
    return started;
}

void Mode::setWelcomeMessage(const std::string& message)
{
    welcomeMessage = message;
}

void Mode::setExitMessage(const std::string& message)
{
    exitMessage = message;
}

void Mode::addSingleOperation(const std::string& word, const std::function<void()>& action)
{
    singleOperations.insert(std::make_pair(word, action));
}

void Mode::addOperationWithArgument(const std::string& word, const std::function<void(std::string)>& action)
{
    operationsWithArgument.insert(std::make_pair(word, action));
}

void Mode::exitAction()
{
    started = false;
}

void Mode::handleOperationsWithArgument(const std::string& line)
{
    std::string lineWithoutComment = line.substr(0, line.find('#'));
    auto trimLeftPos = lineWithoutComment.find_first_not_of(' ');
    lineWithoutComment = lineWithoutComment.substr(trimLeftPos != std::string::npos ? trimLeftPos : 0);
    auto trimRightPos = lineWithoutComment.find_last_not_of(' ');
    lineWithoutComment = lineWithoutComment.substr(0, trimRightPos != std::string::npos ? trimRightPos + 1 : lineWithoutComment.size() - 1);
    std::string token;
    std::vector<std::string> tokens;
    while(true)
    {
        std::size_t pos = lineWithoutComment.find(' ');
        token = lineWithoutComment.substr(0, pos);
        lineWithoutComment.erase(0, pos + 1);
        tokens.push_back(token);
        if(pos == std::string::npos) break;
    }
    if(tokens.size() ==  2)
    {
        if (operationsWithArgument.count(tokens.at(0)) > 0)
        {
            try
            {
                auto operationWithArgument = operationsWithArgument.at(tokens.at(0));
                std::unique_lock<std::mutex> lock(mutex);
                operationWithArgument(tokens.at(1));
                lock.unlock();
            }
            catch(std::exception& e)
            {
                throw;
            }
        }
    }
    else if(tokens.size() == 3)
    {
        std::string operationsWithSpace = tokens.at(0) + ' ' + tokens.at(1);
        if (operationsWithArgument.count(operationsWithSpace) > 0)
        {
            try
            {
                auto operationWithArgument = operationsWithArgument.at(operationsWithSpace);
                std::unique_lock<std::mutex> lock(mutex);
                operationWithArgument(tokens.at(2));
                lock.unlock();
            }
            catch(std::exception& e)
            {
                throw;
            }
        }
    }
}

void Mode::setupUserInputThread()
{
    std::thread([this]()
    {
        started = true;
        std::string operation, argument;
        while(started)
        {
            std::string line;
            while (started && std::getline(std::cin, line))
            {
                if (singleOperations.count(line) > 0)
                    singleOperations.at(line)();
                else if(!line.empty())
                {
                    try
                    {
                        handleOperationsWithArgument(line);
                    }
                    catch(std::exception& e)
                    {
                        logger.log(Utilities::LogLevel::Error, e.what());
                    }
                }
            }
        }
    }).detach();
}

void Mode::scheduleAction(const std::function<void()>& action)
{
  scheduledActions.emplace_back(action);
}

void Mode::setupCommandThread()
{
    while(true)
    {
        std::unique_lock<std::mutex> lock(mutex);
        for (const auto &scheduledAction : scheduledActions)
            scheduledAction();
        scheduledActions.clear();
        lock.unlock();

        if (!started)
            break;
    }
}

void Mode::start()
{
    logger.log(Utilities::LogLevel::Raw, welcomeMessage.data());
    setupUserInputThread();
    setupCommandThread();
    logger.log(Utilities::LogLevel::Raw, exitMessage.data());
}