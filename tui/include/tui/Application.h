//
// Created by derk on 27-4-20.
//

#ifndef TUI_INCLUDE_TUI_APPLICATION_H
#define TUI_INCLUDE_TUI_APPLICATION_H

#include <tui/Mode.h>
#include <memory>

class Application
{
public:
    Application() = default;
    ~Application() = default;

    void run();

private:
    std::unique_ptr<Mode> currentMode;
};

#endif //TUI_INCLUDE_TUI_APPLICATION_H
