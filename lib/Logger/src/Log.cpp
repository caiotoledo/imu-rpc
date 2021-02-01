#include <cstdarg>

#include <iostream>

#include "Log.h"

#define LOGHEADER(filename, func, line) "[" << filename << ":" << func << "@" << line << "]"

using namespace logger;

void Log::PrintHeader(eLogLevel level, std::stringstream &sstr)
{
  switch (level)
  {
  case eLogLevel::DEBUG:
    sstr << "[DEBUG] ";
    break;
  case eLogLevel::WARNING:
    sstr << "[WARNING] ";
    break;
  case eLogLevel::ERROR:
    sstr << "[ERROR] ";
    break;
  case eLogLevel::ALWAYS:
    sstr << " ";
    break;
  default:
    break;
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
  case eLogLevel::DEBUG:
    sLog << ANSI_COLOR_GREEN;
    break;
  case eLogLevel::WARNING:
    sLog << ANSI_COLOR_YELLOW;
    break;
  case eLogLevel::ERROR:
    sLog << ANSI_COLOR_RED;
    break;
  case eLogLevel::ALWAYS:
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
