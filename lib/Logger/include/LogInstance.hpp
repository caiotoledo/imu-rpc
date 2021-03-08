#pragma once

#include <iostream>
#include <string.h>

#include "SingletonInstancer.h"
#include "Log.h"

#ifdef LEVEL_DEBUG
  #define LEVEL_WARNING
#endif

#ifdef LEVEL_WARNING
  #define LEVEL_ERROR
#endif

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

/* Singleton class for Scheduler class */
using LogSingleton = singleton::SingletonInstancer<logger::Log>;

/* Macro for access the scheduler instance */
#define LOG_INSTANCE    (*LogSingleton::GetInstance(logger::eLogType::STD_OUT_STREAM))

/* Standard Log macro */
#define LOGMACRO(LEVEL, ...)   do {\
  LOG_INSTANCE.Print(logger::LEVEL, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__);\
} while(0)

#ifdef LEVEL_DEBUG
#define LOGDEBUG(...)   LOGMACRO(DEBUG, __VA_ARGS__)
#else
#define LOGDEBUG(...)
#endif

#ifdef LEVEL_WARNING
#define LOGWARN(...)    LOGMACRO(WARNING, __VA_ARGS__)
#else
#define LOGWARN(...)
#endif

#ifdef LEVEL_ERROR
#define LOGERROR(...)   LOGMACRO(ERROR, __VA_ARGS__)
#else
#define LOGERROR(...)
#endif

#define LOG(...)        LOGMACRO(ALWAYS, __VA_ARGS__)
