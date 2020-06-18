//
// Created by derk on 15-5-20.
//

#ifndef MAINMENU_H
#define MAINMENU_H

#include <tui/Mode.h>
#include <utilities/IniParser.h>
#include <utilities/Logger.h>
#include <tui/Communicator.h>

/**
 * The main menu mode is used to have an overview about the different sub modes available and to select a sub mode.
 */
class MainMenu : public Mode
{
public:
    /**
     * Constructs the main menu.
     * @param anIniParser A reference to the ini parser.
     * @param communicator A reference to the communicator.
     */
    MainMenu(Utilities::IniParser& anIniParser, Communication::Communicator& communicator);
    ~MainMenu() override = default;

    /**
     * @see Mode::start
     */
    void start() override;

private:
    ///@{
    /** Starts the different modes. **/
    void startSinglePositionMode();
    void startSequenceMode();
    void startDevelopMode();
    ///@}

    Utilities::IniParser& iniParser;
};


#endif //MAINMENU_H
