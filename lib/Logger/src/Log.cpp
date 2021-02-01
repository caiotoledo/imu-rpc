#include <syslog.h>

#include <cstdarg>

#include <iostream>
#include <map>

#include "Log.h"

#define LOGHEADER(filename, func, line) "[" << filename << ":" << func << "@" << line << "]"

using namespace logger;

const std::map<eLogLevel, int> mapSyslogLevel =
{
  { eLogLevel::ERROR, LOG_ERR },
  { eLogLevel::WARNING, LOG_WARNING },
  { eLogLevel::DEBUG, LOG_DEBUG },
  { eLogLevel::ALWAYS, LOG_INFO },
};

Log::Log(eLogType type)
{
  Init(type);
}

void Log::Init(eLogType type)
{
  /* Close previous syslog if needed */
  if ((type == eLogType::SYSLOG) && (eType == eLogType::STD_OUT_STREAM))
  {
    closelog();
  }

  if ((eType != type) && (eType == eLogType::SYSLOG))
  {
    openlog(NULL, (LOG_CONS | LOG_PID | LOG_NDELAY), LOG_LOCAL0);
  }

  eType = type;
}

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

void Log::PrintColorLevel(eLogLevel level, std::stringstream &sstr)
{
  switch (level)
  {
  case eLogLevel::DEBUG:
    sstr << ANSI_COLOR_GREEN;
    break;
  case eLogLevel::WARNING:
    sstr << ANSI_COLOR_YELLOW;
    break;
  case eLogLevel::ERROR:
    sstr << ANSI_COLOR_RED;
    break;
  case eLogLevel::ALWAYS:
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

  if (eType == eLogType::STD_OUT_STREAM)
  {
    this->PrintColorLevel(level, sLog);
  }

  sLog << LOGHEADER(filename, func, linenum);

  /* Show Log Header */
  this->PrintHeader(level, sLog);

  /* Print Log Data */
  va_list args;
  va_start(args, fmt);
  vsprintf(buffer, fmt, args);
  sLog << buffer;

  if (eType == eLogType::STD_OUT_STREAM)
  {
    sLog << ANSI_COLOR_RESET;
  }

  switch (eType)
  {
  case eLogType::STD_OUT_STREAM:
    std::cout << sLog.str() << std::endl;
    break;
  case eLogType::SYSLOG:
    if (mapSyslogLevel.find(level) != mapSyslogLevel.end())
    {
      syslog(mapSyslogLevel.at(level), "%s", sLog.str().c_str());
    }
    break;
  default:
    break;
  }
}

Log::~Log()
{
  if (eType == eLogType::SYSLOG)
  {
    closelog();
  }
}
