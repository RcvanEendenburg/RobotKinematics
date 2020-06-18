#ifndef UTILITIES_LOGTOCOUT_H
#define UTILITIES_LOGTOCOUT_H

#include <utilities/LogOutput.h>
#include <string>

namespace Utilities
{
class LogToCout : public LogOutput
{
public:
    LogToCout() = default;
    ~LogToCout() = default;
    void
    open() override { opened = true; }
    void
    handleMessage(LogLevel level, const std::string &message) override;
    void
    close() override { opened = false; }
};
}
#endif //UTILITIES_INCLUDE_LOGTOCOUT_H
