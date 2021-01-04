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

const std::map<eAccelScale, std::string> mapAccelScale =
{
  {eAccelScale::Accel_2g,  "0.004785"},
  {eAccelScale::Accel_4g,  "0.002392"},
  {eAccelScale::Accel_8g,  "0.001196"},
  {eAccelScale::Accel_16g, "0.000598"},
};

const std::map<eGyroScale, std::string> mapGyroScale =
{
  {eGyroScale::Gyro_250,  "0.001064724"},
  {eGyroScale::Gyro_500,  "0.000532362"},
  {eGyroScale::Gyro_1000, "0.000266181"},
  {eGyroScale::Gyro_2000, "0.000133090"},
};

IMUIndustrialIO::IMUIndustrialIO(const char *path, int device_index, eAccelScale accelScale, eGyroScale gyroScale) :
  DevicePath(path)
{
  DevicePath.append("/iio:device" + std::to_string(device_index) + "/");
  this->InitializePaths(DevicePath);

  /* Set default precision */
  this->SetAccelScale(accelScale);
  this->SetGyroScale(gyroScale);

  bThreadSampleValues = true;
  auto sampleValues = [this]()
  {
    LOGDEBUG("Starting Sample Values Thread");
    while (bThreadSampleValues)
    {
      for(auto &val : imu_data.axisdata)
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
    imu_data.axisdata[index].DeviceAccelPath = sAccelPath;

    auto sGyroPath = sDevicePath;
    sGyroPath.append("in_anglvel_" + sAxis + "_raw");
    imu_data.axisdata[index].DeviceGyroPath = sGyroPath;
  }

  auto sAccelScalePath = sDevicePath;
  sAccelScalePath.append("in_accel_scale");
  imu_data.DeviceAccelScalePath = sAccelScalePath;

  auto sGyroScalePath = sDevicePath;
  sGyroScalePath.append("in_anglvel_scale");
  imu_data.DeviceGyroScalePath = sGyroScalePath;
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
    if (std::getline(ifFile, strValue))
    {
      std::stringstream sstrValue;
      sstrValue << strValue;
      sstrValue >> value;
    }
  }
  else
  {
    LOGERROR("Invalid Path!");
  }

  return value;
}

template <typename T>
eIMUAbstractionError IMUIndustrialIO::SetValueInFile(const char *path, T val)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  std::ofstream ofFile(path);
  if (ofFile.is_open())
  {
    ofFile << val;
  }
  else
  {
    LOGERROR("Invalid Path!");
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  return ret;
}

eIMUAbstractionError IMUIndustrialIO::SetAccelScale(eAccelScale scale)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  if (mapAccelScale.find(scale) != mapAccelScale.end())
  {
    auto val = mapAccelScale.at(scale);
    LOGDEBUG("Set Accel Scale [%d]->[%s]", scale, val.c_str());
    ret = this->SetValueInFile(imu_data.DeviceAccelScalePath.c_str(), val);
  }
  else
  {
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  if (ret != eIMUAbstractionError::eRET_OK)
  {
    LOGERROR("SetAccelScale failed [%d]", ret);
  }

  return ret;
}
eIMUAbstractionError IMUIndustrialIO::SetGyroScale(eGyroScale scale)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  if (mapGyroScale.find(scale) != mapGyroScale.end())
  {
    auto val = mapGyroScale.at(scale);
    LOGDEBUG("Set Gyro Scale [%d]->[%s]", scale, val.c_str());
    ret = this->SetValueInFile(imu_data.DeviceGyroScalePath.c_str(), val);
  }
  else
  {
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  if (ret != eIMUAbstractionError::eRET_OK)
  {
    LOGERROR("SetGyroScale failed [%d]", ret);
  }

  return ret;
}

eIMUAbstractionError IMUIndustrialIO::GetRawAccel(eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  if (axis >= eAxis::X && axis <= eAxis::Z)
  {
    val = imu_data.axisdata[(int)axis].accel;
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
  if (axis >= eAxis::X && axis <= eAxis::Z)
  {
    val = imu_data.axisdata[(int)axis].gyro;
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
