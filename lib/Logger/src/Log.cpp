#include <cstdarg>

#include <iostream>

#include "Log.h"

#define LOGHEADER(filename, func, line) std::cout << "[" << filename << ":" << func << "@" << line << "]"

using namespace logger;

void Log::PrintHeader(eLogLevel level)
{
  switch (level)
  {
  case DEBUG:
    std::cout << "[DEBUG] ";
    break;
  case WARNING:
    std::cout << "[WARNING] ";
    break;
  case ERROR:
    std::cout << "[ERROR] ";
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

  char buffer[256];

  switch (level)
  {
  case DEBUG:
    SET_COUT_GREEN;
    break;
  case WARNING:
    SET_COUT_YELLOW;
    break;
  case ERROR:
    SET_COUT_RED;
    break;
  default:
    return;
  }

  LOGHEADER(filename, func, linenum);

  /* Show Log Header */
  this->PrintHeader(level);

  /* Print Log Data */
  va_list args;
  va_start(args, fmt);
  vsprintf(buffer, fmt, args);
  std::cout << buffer << std::endl;

  RESET_COUT_COLOR;
}
