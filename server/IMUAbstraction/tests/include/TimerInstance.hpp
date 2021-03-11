#pragma once

#include <gmock/gmock.h>

#include <SingletonInstancer.hpp>

class MockTimer {
public:
  MOCK_METHOD0(GetSeconds, double(void));
};

/* Singleton class for Scheduler class */
using TimerMockSingleton = singleton::SingletonInstancer<MockTimer>;

/* Macro for access the scheduler instance */
#define TIMER_INSTANCE    (*TimerMockSingleton::GetInstance())
