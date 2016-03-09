#ifndef _MATH_SCRAPS_LOGGER_H_
#define _MATH_SCRAPS_LOGGER_H_

#include <string>
#include <functional>

class LogCallback {
public:
    LogCallback () {}

    LogCallback (std::function<void(const std::string &log)> f) : m_callback(f) {}

    void execute(const std::string &log);

private:
    std::function<void(const std::string &log)> m_callback;

};


class logger {
public:
    static void SetLogCallback(std::function<void(const std::string &log)> callback);

    static void i(const char *fmt, ...);

    static void w(const char *fmt, ...);

    static void e(const char *fmt, ...);

};

#endif
