#include <iostream>
#include <cstdarg>

#include "../include/Log.h"

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

void Log::Print(eLogLevel level, const char *fmt, ...)
{
  char buffer[256];

  /* Show Log Header */
  this->PrintHeader(level);

  /* Print Log Data */
  va_list args;
  va_start(args, fmt);
  vsprintf(buffer, fmt, args);
  std::cout << buffer << std::endl;
}
