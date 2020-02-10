#ifndef UTILITIES_LOGOUTPUT_H
#define UTILITIES_LOGOUTPUT_H

#include <utilities/LogLevel.h>
#include <string>
#include <atomic>

namespace Utilities
{
class LogOutput
{
public:
    virtual void
    open() = 0;
    virtual void
    handleMessage(LogLevel level, const std::string &message) = 0;
    virtual void
    close() = 0;

    bool
    isOpened() { return opened; }

protected:
    std::atomic_bool opened;
};
}

#endif //UTILITIES_LOGOUTPUT_H
