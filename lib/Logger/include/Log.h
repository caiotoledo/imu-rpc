#pragma once

#include <mutex>
#include <sstream>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

namespace logger
{

  typedef enum LogLevel
  {
    DEBUG,
    WARNING,
    ERROR
  } eLogLevel;

  class Log
  {
  private:
    std::mutex mtxLog;

    void PrintHeader(eLogLevel level, std::stringstream &sstr);

  public:
    Log() = default;
    ~Log() = default;

    void Print(eLogLevel level, const char *filename, const char *func, int linenum, const char *fmt, ...);
  };

} // namespace logger
