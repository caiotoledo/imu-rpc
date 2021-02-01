#pragma once

#include <mutex>
#include <string>
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

  typedef enum LogType
  {
    STD_OUT_STREAM, /* Standard Output Stream */
    SYSLOG,         /* Syslog */
  } eLogType;

  typedef enum LogLevel
  {
    DEBUG,
    WARNING,
    ERROR,
    ALWAYS,
  } eLogLevel;

  class Log
  {
  private:
    std::mutex mtxLog;

    eLogType eType;

    void PrintHeader(eLogLevel level, std::stringstream &sstr);
    void PrintColorLevel(eLogLevel level, std::stringstream &sstr);

  public:
    Log(eLogType type=eLogType::STD_OUT_STREAM);

    void Init(eLogType type);
    void Print(eLogLevel level, const char *filename, const char *func, int linenum, const char *fmt, ...);

    ~Log();
  };

} // namespace logger
