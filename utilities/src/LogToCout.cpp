#include <utilities/LogToCout.h>
#include <iostream>

#define RST  "\x1B[0m"
#define RED  "\x1B[31m"
#define YEL  "\x1B[33m"
#define BLU  "\x1B[34m"

#define BOLD "\x1B[1m"
#define UNDL "\x1B[4m"

namespace Utilities
{

void
LogToCout::handleMessage(LogLevel level, const std::string &message)
{
    switch (level)
    {
    case LogLevel::Debug:std::cout << BOLD << BLU << "[DEBUG] " << RST << BLU << message << RST << std::flush;
            break;
        case LogLevel::Warning:std::cout << BOLD << YEL << "[WARNING] " << RST << YEL << message << RST << std::flush;
            break;
        case LogLevel::Error:std::cout << BOLD << RED << "[ERROR] " << RST << RED << message << RST << std::flush;
            break;
        case LogLevel::Fatal:std::cout << BOLD << UNDL << RED << "[FATAL] " << message << RST << std::flush;
            break;
        case LogLevel::Raw:std::cout << message;
            break;
        default:break;
    }
}
}
