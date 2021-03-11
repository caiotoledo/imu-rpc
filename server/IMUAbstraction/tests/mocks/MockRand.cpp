#include <SingletonInstancer.hpp>

#include "MockRand.hpp"

using RandSingleton = singleton::SingletonInstancer<MockRand>;

extern "C"
{
  int rand(void)
  {
    return (*RandSingleton::GetInstance()).rand();
  }
}
