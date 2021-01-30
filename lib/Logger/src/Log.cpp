#include <cstdarg>

#include <iostream>

#include "Log.h"

#define LOGHEADER(filename, func, line) "[" << filename << ":" << func << "@" << line << "]"

using namespace logger;

void Log::PrintHeader(eLogLevel level, std::stringstream &sstr)
{
  switch (level)
  {
  case DEBUG:
    sstr << "[DEBUG] ";
    break;
  case WARNING:
    sstr << "[WARNING] ";
    break;
  case ERROR:
    sstr << "[ERROR] ";
    break;
  default:
    return;
  }
}

void Log::Print(
  eLogLevel level,
  const char *filename,
  const char *func,
  int linenum,
  const char *fmt, ...
)
{
  std::lock_guard<std::mutex> lck(mtxLog);

  std::stringstream sLog;
  char buffer[256];

  switch (level)
  {
  case DEBUG:
    sLog << ANSI_COLOR_GREEN;
    break;
  case WARNING:
    sLog << ANSI_COLOR_YELLOW;
    break;
  case ERROR:
    sLog << ANSI_COLOR_RED;
    break;
  default:
    return;
  }

  sLog << LOGHEADER(filename, func, linenum);

  /* Show Log Header */
  this->PrintHeader(level, sLog);

  /* Print Log Data */
  va_list args;
  va_start(args, fmt);
  vsprintf(buffer, fmt, args);
  sLog << buffer;

  sLog << ANSI_COLOR_RESET;

  std::cout << sLog.str() << std::endl;
}
