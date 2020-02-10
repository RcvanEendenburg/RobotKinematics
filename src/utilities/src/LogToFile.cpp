#include <utilities/LogToFile.h>

namespace Utilities
{

LogToFile::LogToFile(std::string file) : file_name(std::move(file)) {}

void
LogToFile::open()
{
    stream.open(file_name, std::fstream::out | std::fstream::app);
    opened = true;
}

void
LogToFile::close()
{
    stream.close();
    opened = false;
}

void
LogToFile::handleMessage(LogLevel level, const std::string &message)
{
    switch (level)
    {
        case LogLevel::Debug:stream << "[DEBUG] " << message;
            break;
        case LogLevel::Warning:stream << "[WARNING] " << message;
            break;
        case LogLevel::Error:stream << "[ERROR] " << message;
            break;
        case LogLevel::Fatal:stream << "[FATAL] " << message;
            break;
        default:break;
    }
}
}