#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <thread>
#include <chrono>

#include <cstdlib>
#include <ctime>

#include <IMUIndustrialIO.hpp>

#include <LogInstance.h>

using namespace IMUAbstraction;

IMUIndustrialIO::IMUIndustrialIO(const char *path, int device_index) :
  DevicePath(path)
{
  DevicePath.append("/iio:device" + std::to_string(device_index) + "/");
  this->InitializePaths(DevicePath);

  bThreadSampleValues = true;
  auto sampleValues = [this]()
  {
    LOGDEBUG("Starting Sample Values Thread");
    while (bThreadSampleValues)
    {
      for(auto &val : imu_data)
      {
        val.accel = this->GetValueInFile<double>(val.DeviceAccelPath.c_str());
        val.gyro = this->GetValueInFile<double>(val.DeviceGyroPath.c_str());
      }

      this->NotifyUpdateData();
      /* FIXME: Use the sampling frequency or file interrupt update instead of fixed time */
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    LOGDEBUG("Finishing Sample Values Thread");
  };
  thSampleValues = std::thread(sampleValues);
}

void IMUIndustrialIO::InitializePaths(std::string const &sDevicePath)
{
  const std::map<int, std::string> mapAxis =
  {
    {0, "x"},
    {1, "y"},
    {2, "z"},
  };

  for (auto &val : mapAxis)
  {
    auto index = val.first;
    auto sAxis = val.second;

    auto sAccelPath = sDevicePath;
    sAccelPath.append("in_accel_" + sAxis + "_raw");
    imu_data[index].DeviceAccelPath = sAccelPath;

    auto sGyroPath = sDevicePath;
    sGyroPath.append("in_anglvel_" + sAxis + "_raw");
    imu_data[index].DeviceGyroPath = sGyroPath;
  }
}

void IMUIndustrialIO::AddUpdateDataCallback(std::function<void()> &&cb)
{
  vecCallback.push_back(cb);
}
void IMUIndustrialIO::NotifyUpdateData(void)
{
  for(auto &cb : vecCallback)
  {
    cb();
  }
}

template <typename T>
T IMUIndustrialIO::GetValueInFile(const char *path)
{
  T value = {0};

  std::ifstream ifFile(path);
  if (ifFile.is_open())
  {
    std::string strValue;
    while (std::getline(ifFile, strValue))
    {
      std::stringstream sstrValue;
      sstrValue << strValue;
      sstrValue >> value;
    }
  }

  return value;
}

eIMUAbstractionError IMUIndustrialIO::GetRawAccel(eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  if ((int)axis >= ((int)eAxis::X) && (int)axis <= ((int)eAxis::Z))
  {
    val = imu_data[(int)axis].accel;
  }
  else
  {
    val = 0;
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  return ret;
}
eIMUAbstractionError IMUIndustrialIO::GetRawGyro(eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  if ((int)axis >= ((int)eAxis::X) && (int)axis <= ((int)eAxis::Z))
  {
    val = imu_data[(int)axis].gyro;
  }
  else
  {
    val = 0;
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  return ret;
}

IMUIndustrialIO::~IMUIndustrialIO()
{
  if (thSampleValues.joinable())
  {
    bThreadSampleValues = false;
    thSampleValues.join();
  }
}
