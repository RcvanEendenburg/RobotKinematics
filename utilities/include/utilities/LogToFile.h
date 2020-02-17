#ifndef UTILITIES_LOGTOFILE_H
#define UTILITIES_LOGTOFILE_H

#include <utilities/LogOutput.h>
#include <fstream>

namespace Utilities
{
class LogToFile : public LogOutput
{
public:
    explicit LogToFile(std::string file);
    ~LogToFile() = default;
    void
    open() override;
    void
    handleMessage(LogLevel level, const std::string &message) override;
    void
    close() override;

private:
    std::string file_name;
    std::fstream stream;
};
}

#endif //UTILITIES_LOGTOFILE_H
