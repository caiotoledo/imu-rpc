#pragma once

#include <gmock/gmock.h>

#include <SingletonInstancer.hpp>

#include "MockTimer.hpp"

/* Singleton class for Timer class */
using TimerMockSingleton = singleton::SingletonInstancer<MockTimer>;

/* Macro for access the timer instance */
#define TIMER_INSTANCE    (*TimerMockSingleton::GetInstance())
