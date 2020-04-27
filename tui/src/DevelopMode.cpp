//
// Created by derk on 27-4-20.
//

#include <tui/DevelopMode.h>

void DevelopMode::start()
{
  setWelcomeMessage("Write some help text about which commands you can type");
  setExitMessage("Exiting developer mode...");
  Mode::start();
}