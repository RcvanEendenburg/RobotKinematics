//
// Created by derk on 27-4-20.
//

#include <tui/Application.h>
#include <tui/DevelopMode.h>

#include <iostream>

void Application::run()
{
  std::string operation;
  while(true)
  {
    if(!currentMode || !currentMode->isStarted())
    {
      std::cout << "Type 'develop' to access the developer mode." << std::endl;
      std::cout << "'exit' will exit the application." << std::endl;
      std::cin >> operation;
      if (operation == "develop")
      {
        currentMode = std::move(std::make_unique<DevelopMode>());
        currentMode->start();
      }
      else if (operation == "exit")
        break;
    }
  }
}