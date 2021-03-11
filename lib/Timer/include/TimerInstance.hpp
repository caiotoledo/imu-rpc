#pragma once

#include <SingletonInstancer.hpp>
#include "Timer.hpp"

/* Singleton class for Timer class */
using TimerSingleton = singleton::SingletonInstancer<Timer::Timer>;

/* Macro for access the timer instance */
#define TIMER_INSTANCE    (*TimerSingleton::GetInstance())
