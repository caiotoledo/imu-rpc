#include <map>
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

const std::map<eAccelScale, int> mapAccelScale =
{
  {eAccelScale::Accel_2g,  2000*100},
  {eAccelScale::Accel_4g,  4000*100},
  {eAccelScale::Accel_8g,  8000*100},
  {eAccelScale::Accel_16g, 16000*100},
};

const std::map<eGyroScale, int> mapGyroScale =
{
  {eGyroScale::Gyro_250,  250*100},
  {eGyroScale::Gyro_500,  500*100},
  {eGyroScale::Gyro_1000, 1000*100},
  {eGyroScale::Gyro_2000, 2000*100},
};

IMUStub::IMUStub(eAccelScale accelScale, eGyroScale gyroScale)
{
  srand((unsigned) time(0));

  this->SetAccelScale(accelScale);
  this->SetGyroScale(gyroScale);

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

  auto scale = GetAccelScale<int>();
  double r = (rand() % scale);
  double ret = (r/100);

  ret = signal ? ret : (-ret);

  return ret;
}
double IMUStub::GetRandomGyro(void)
{
  bool signal = ((bool)(rand() % 2));

  auto scale = GetGyroScale<int>();
  double r = (rand() % scale);
  double ret = (r/100);

  ret = signal ? ret : (-ret);

  return ret;
}

void IMUStub::AddUpdateDataCallback(std::function<void()> &&cb)
{
  vecCallback.push_back(cb);
}

eIMUAbstractionError IMUStub::SetSampleFrequency(eSampleFreq freq)
{
  return eIMUAbstractionError::eRET_ERROR;
}

eIMUAbstractionError IMUStub::SetAccelScale(eAccelScale scale)
{
  this->accelScale = scale;
  LOGDEBUG("Set Accel Scale [%d]->[%0.2f]", scale, static_cast<double>(mapAccelScale.at(this->accelScale)));

  return eIMUAbstractionError::eRET_OK;
}
eIMUAbstractionError IMUStub::SetGyroScale(eGyroScale scale)
{
  this->gyroScale = scale;
  LOGDEBUG("Set Gyro Scale [%d]->[%0.2f]", scale, static_cast<double>(mapGyroScale.at(this->gyroScale)));

  return eIMUAbstractionError::eRET_OK;
}
template <typename T>
T IMUStub::GetAccelScale()
{
  return static_cast<T>(mapAccelScale.at(this->accelScale));
}
template <typename T>
T IMUStub::GetGyroScale()
{
  return static_cast<T>(mapGyroScale.at(this->gyroScale));
}

eIMUAbstractionError IMUStub::GetRawAccel(eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  int axisVal = (int)axis;

  auto sizeAccelArray = (int)(sizeof(accel)/sizeof(accel[0]));
  if ( axisVal >= 0 && axisVal <= sizeAccelArray)
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

  auto sizeGyroArray = (int)(sizeof(gyro)/sizeof(gyro[0]));
  if ( axisVal >= 0 && axisVal <= sizeGyroArray)
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
