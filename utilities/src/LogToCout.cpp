#include <utilities/LogToCout.h>

#include <iostream>

namespace Utilities
{

void
LogToCout::handleMessage(LogLevel level, const std::string &message)
{
    switch (level)
    {
        case LogLevel::Debug:std::cout << "[DEBUG] " << message;
            break;
        case LogLevel::Warning:std::cout << "[WARNING] " << message;
            break;
        case LogLevel::Error:std::cout << "[ERROR] " << message;
            break;
        case LogLevel::Fatal:std::cout << "[FATAL] " << message;
            break;
        default:break;
    }
}
}
