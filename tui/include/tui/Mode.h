//
// Created by derk on 27-4-20.
//

#ifndef TUI_INCLUDE_TUI_MODE_H
#define TUI_INCLUDE_TUI_MODE_H

#include <string>
#include <functional>
#include <map>
#include <mutex>
#include <atomic>
#include <utilities/Logger.h>
#include <tui/Communicator.h>

class Mode
{
public:
    explicit Mode(Communication::Communicator& aCommunicator);
    virtual ~Mode() = default;

    /**
     * Starts the mode.
     */
    virtual void start();

    /**
     * Checks whether the mode has been started or not.
     * @return True if the mode is started.
     */
    bool isStarted() const;

protected:

    /**
     * Sets welcome message which will be shown at startup of the mode.
     * @param message The message to display.
     */
    void setWelcomeMessage(const std::string& message);

    /**
     * Sets exit message which will be shown when exiting the mode.
     * @param message The message to display.
     */
    void setExitMessage(const std::string& message);

    /**
     * Adds a single operation which means that there is no argument.
     * @param word The word to detect.
     * @param action The action to perform when the word has been detected.
     */
    void addSingleOperation(const std::string& word, const std::function<void()>& action);

    /**
     * Adds a operation with argument.
     * @param word The word to detect.
     * @param action The action to perform when the word has been detected.
     */
    void addOperationWithArgument(const std::string& word, const std::function<void(std::string)>& action);

    /**
     * All possible keywords to use
     */
    enum class Keyword
    {
        Quit,
        Goto,
        Black,
        Blue,
        Green,
        Red,
        Yellow,
        White,
        Rectangle,
        Square,
        Circle,
        All,
        Wood,
        Develop,
        Interactive
    };

    ///@{
    /** Helper functions for keyword to string and other way around. **/
    const std::string& keywordToString(Keyword keyword);
    Keyword stringToKeyword(const std::string& str);
    ///@}

    /**
    * Schedule an action for the next cycle.
    * @param action
    */
    void scheduleAction(const std::function<void()>& action);

    std::atomic_bool started;
    Communication::Communicator& communicator;
private:
    /**
     * Action which defines what to do on the exit operation.
     */
    void exitAction();

    /**
     * This will execute the user input thread and handles the operations which the user can type.
     */
    void setupUserInputThread();

    /**
    * This thread sends all commands to other packages.
    */
    void setupCommandThread();

    void handleOperationsWithArgument(const std::string& line);

    std::string welcomeMessage;
    std::string exitMessage;

    std::map<std::string, std::function<void()>> singleOperations;
    std::map<std::string, std::function<void(std::string)>> operationsWithArgument;
    std::vector<std::function<void()>> scheduledActions;

    static const std::map<Keyword, std::string> keywordTable;
    std::mutex mutex;

protected:
    Utilities::Logger &logger;
};

#endif //TUI_INCLUDE_TUI_MODE_H
