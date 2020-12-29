#include <thread>
#include <chrono>

#include <cstdlib>
#include <ctime>

#include <IMUIndustrialIO.hpp>

#include <LogInstance.h>

using namespace IMUAbstraction;

IMUIndustrialIO::IMUIndustrialIO(){}

void IMUIndustrialIO::AddUpdateDataCallback(std::function<void()> &&cb)
{
  vecCallback.push_back(cb);
}

eIMUAbstractionError IMUIndustrialIO::GetRawAccel(eAxis axis, double &val)
{
  (void)axis;
  val = 0;

  return eIMUAbstractionError::eRET_OK;
}
eIMUAbstractionError IMUIndustrialIO::GetRawGyro(eAxis axis, double &val)
{
  (void)axis;
  val = 0;

  return eIMUAbstractionError::eRET_OK;
}

IMUIndustrialIO::~IMUIndustrialIO(){}
