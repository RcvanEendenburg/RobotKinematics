//
// Created by derk on 27-4-20.
//

#ifndef TUI_INCLUDE_TUI_DEVELOPMODE_H
#define TUI_INCLUDE_TUI_DEVELOPMODE_H

#include <tui/Mode.h>

/**
 * This mode contains the operations which are convenient when developing the application.
 *
 * Currently, this contains the "goto (x,y,z)" operation used to go to a point.
 */
class DevelopMode : public Mode
{
public:
  explicit DevelopMode(Communication::Communicator& communicator);
  ~DevelopMode() override = default;

  /**
   * @see Mode::start
   */
  void start() override;

private:
    /**
     * Sends a position to the robot arm.
     * @param positionStr The string should be in (x,y,z) format.
     */
    void goToPosition(const std::string& positionStr);
};

#endif //TUI_INCLUDE_TUI_DEVELOPMODE_H
