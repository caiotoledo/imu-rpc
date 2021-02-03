#pragma once

#define LOGDEBUG(...)
#define LOGWARN(...)
#define LOGERROR(...)
#define LOG(...)        LOGMACRO(ALWAYS, __VA_ARGS__)
