#pragma once

#include <gmock/gmock.h>

#include <SingletonInstancer.hpp>

#include "MockTimer.hpp"

/* Singleton class for Scheduler class */
using TimerMockSingleton = singleton::SingletonInstancer<MockTimer>;

/* Macro for access the scheduler instance */
#define TIMER_INSTANCE    (*TimerMockSingleton::GetInstance())
