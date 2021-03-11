#pragma once

#include <SingletonInstancer.hpp>
#include "Timer.hpp"

/* Singleton class for Scheduler class */
using TimerSingleton = singleton::SingletonInstancer<Timer::Timer>;

/* Macro for access the scheduler instance */
#define TIMER_INSTANCE    (*TimerSingleton::GetInstance())
