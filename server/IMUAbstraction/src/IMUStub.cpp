#include <thread>
#include <chrono>

#include <cstdlib>
#include <ctime>

#include <IMUStub.hpp>

#include <LogInstance.h>

using namespace IMUAbstraction;

constexpr int NOTIFICATION_DELAY = 1000;
constexpr int MAX_AXIS = 3;

double accel[MAX_AXIS] = {0};
double gyro[MAX_AXIS] = {0};

IMUStub::IMUStub()
{
  srand((unsigned) time(0));

  bThreadNotification = true;
  auto func = [this]()
  {
    while (bThreadNotification)
    {
      for (size_t i = 0; i < (sizeof(accel)/sizeof(accel[0])); i++)
      {
        accel[i] = this->GetRandomAccel();
        gyro[i] = this->GetRandomGyro();
      }

      for(auto &cb : vecCallback)
      {
        cb();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(NOTIFICATION_DELAY));
    }
  };
  thNotification = std::thread(func);
}

double IMUStub::GetRandomAccel(void)
{
  bool signal = ((bool)(rand() % 2));

  double r = (rand() % 200000);
  double ret = (r/100);

  ret = signal ? ret : (-ret);

  return ret;
}
double IMUStub::GetRandomGyro(void)
{
  bool signal = ((bool)(rand() % 2));

  double r = (rand() % 25000);
  double ret = (r/100);

  ret = signal ? ret : (-ret);

  return ret;
}

void IMUStub::AddUpdateDataCallback(std::function<void()> &&cb)
{
  vecCallback.push_back(cb);
}

eIMUAbstractionError IMUStub::GetRawAccel(eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  int axisVal = (int)axis;

  if ( axisVal >= 0 && axisVal <= (sizeof(accel)/sizeof(accel[0])))
  {
    val = accel[axisVal];
  }
  else
  {
    val = 0;
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  return ret;
}
eIMUAbstractionError IMUStub::GetRawGyro(eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  int axisVal = (int)axis;

  if ( axisVal >= 0 && axisVal <= (sizeof(accel)/sizeof(accel[0])))
  {
    val = gyro[axisVal];
  }
  else
  {
    val = 0;
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  return ret;
}

IMUStub::~IMUStub()
{
  if (thNotification.joinable())
  {
    bThreadNotification = false;
    thNotification.join();
  }
}
