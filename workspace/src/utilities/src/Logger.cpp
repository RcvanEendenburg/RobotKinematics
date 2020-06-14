#include <utilities/Logger.h>
#include <cstdarg>
#include <iostream>
#include <sstream>

namespace Utilities {

Logger &Logger::instance() {
    static Logger logger;
    return logger;
}

Logger::Logger() : printing(true), switching_output(false) {
    thread = std::thread([this]() { logThread(); });
}

Logger::~Logger() {
    printing = false;
    queue_cv.notify_all();
    if (thread.joinable())
        thread.join();
}

void Logger::setLogOutput(std::unique_ptr<LogOutput> logOutput) {
    switching_output = true;
    std::unique_lock<std::mutex> lock(mutex);
    if (output && output->isOpened())
        output->close();
    output = std::move(logOutput);
    lock.unlock();
    switching_output = false;
}

void Logger::log(LogLevel logLevel, const char *fmt, ...) {
    std::ostringstream os;
    va_list args;
    va_start(args, fmt);

    while (*fmt != '\0')
    {
        if (*fmt == '%')
        {
            ++fmt;
            if (*fmt == 'd')
            {
                int i = va_arg(args, int);
                os << i;
            }
            else if (*fmt == 'c')
            {
                // note automatic conversion to integral type
                int c = va_arg(args, int);
                os << static_cast<char>(c) << '\n';
            }
            else if (*fmt == 'f')
            {
                double d = va_arg(args, double);
                os << d;
            }
        }
        else
        {
            os << *fmt;
        }
        ++fmt;
    }

    va_end(args);
    os << '\n';
    push(std::move(std::make_unique<LogMessage>(os.str(), logLevel)));
}

void Logger::push(std::unique_ptr<LogMessage> message) {
    std::unique_lock<std::mutex> lock(mutex);
    queue.push(std::move(message));
    lock.unlock();
    queue_cv.notify_all();
}

void Logger::logThread() {
    std::unique_lock<std::mutex> lock(mutex);

    while (true)
    {
        queue_cv.wait(lock, [this]()
        {
            return !queue.empty() || (queue.empty() && !printing);
        });
        if (queue.empty())
            break;
        auto message = std::move(queue.front());
        queue.pop();
        lock.unlock();
        while (switching_output);
        lock.lock();
        if (!output->isOpened())
        {
            output->open();
            if (!output->isOpened())
                throw std::runtime_error("Couldn't open log output!");
        }
        output->handleMessage(message->level, message->message);
    }
    output->close();
    lock.unlock();
}

} // namespace Utilities
