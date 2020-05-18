//
// Created by derk on 27-4-20.
//

#ifndef TUI_INCLUDE_TUI_DEVELOPMODE_H
#define TUI_INCLUDE_TUI_DEVELOPMODE_H

#include <tui/Mode.h>

class DevelopMode : public Mode
{
public:
  DevelopMode(Communication::Communicator& communicator, unsigned int zCoordinate);
  ~DevelopMode() override = default;
  virtual void start() override;

private:
    void goToPosition(const std::string& positionStr);

    unsigned int standardZ;
};

#endif //TUI_INCLUDE_TUI_DEVELOPMODE_H
