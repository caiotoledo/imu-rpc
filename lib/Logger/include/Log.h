#pragma once

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define SET_COUT_RED        std::cout << ANSI_COLOR_RED
#define SET_COUT_GREEN      std::cout << ANSI_COLOR_GREEN
#define SET_COUT_YELLOW     std::cout << ANSI_COLOR_YELLOW
#define RESET_COUT_COLOR    std::cout << ANSI_COLOR_RESET

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
    void PrintHeader(eLogLevel level);

  public:
    Log() {}
    ~Log() {}

    void Print(eLogLevel level, const char *fmt, ...);
  };

} // namespace logger
