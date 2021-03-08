#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <chrono>

#include <cstdlib>
#include <ctime>
#include <cmath>

#include <IMUIndustrialIO.hpp>

#include <LogInstance.hpp>

#define _USE_MATH_DEFINES

/**
 * @brief Gravity acceleration value
 */
#define M_Gl (9.80665L)

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
  {eGyroScale::Gyro_250dps,  "0.001064724"},
  {eGyroScale::Gyro_500dps,  "0.000532362"},
  {eGyroScale::Gyro_1000dps, "0.000266181"},
  {eGyroScale::Gyro_2000dps, "0.000133090"},
};

const std::map<eSampleFreq, int> mapSampleFreq =
{
  {eSampleFreq::Freq_10ms,  10},
  {eSampleFreq::Freq_20ms,  20},
  {eSampleFreq::Freq_50ms,  50},
  {eSampleFreq::Freq_100ms, 100},
  {eSampleFreq::Freq_200ms, 200},
  {eSampleFreq::Freq_500ms, 500},
};

IMUIndustrialIO::IMUIndustrialIO(
  const char *path,
  int device_index,
  eAccelScale accelScale,
  eGyroScale gyroScale,
  eSampleFreq sampleFreq) :
  DevicePath(path)
{
  this->DevicePath.append("/iio:device" + std::to_string(device_index) + "/");

  this->imu_data.paths.DeviceBufferPath = std::string("/dev/iio:device" + std::to_string(device_index));

  this->imu_data.accelScale = accelScale;
  this->imu_data.gyroScale = gyroScale;
  this->imu_data.sampleFreq = sampleFreq;
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

    auto sAccelEnPath = sDevicePath;
    sAccelEnPath.append("scan_elements/in_accel_" + sAxis + "_en");
    imu_data.axisdata[index].DeviceAccelBufferEnPath = sAccelEnPath;

    auto sGyroEnPath = sDevicePath;
    sGyroEnPath.append("scan_elements/in_anglvel_" + sAxis + "_en");
    imu_data.axisdata[index].DeviceGyroBufferEnPath = sGyroEnPath;
  }

  auto sAccelScalePath = sDevicePath;
  sAccelScalePath.append("in_accel_scale");
  imu_data.paths.DeviceAccelScalePath = sAccelScalePath;

  auto sGyroScalePath = sDevicePath;
  sGyroScalePath.append("in_anglvel_scale");
  imu_data.paths.DeviceGyroScalePath = sGyroScalePath;

  auto sSampleFreqPath = sDevicePath;
  sSampleFreqPath.append("sampling_frequency");
  imu_data.paths.DeviceSampleFreqPath = sSampleFreqPath;

  auto sBuferEnablePath = sDevicePath;
  sBuferEnablePath.append("buffer/enable");
  imu_data.paths.DeviceBufferEnablePath = sBuferEnablePath;
}

eIMUAbstractionError IMUIndustrialIO::Init(void)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  this->InitializePaths(this->DevicePath);

  /* Set default precision */
  auto retAccel = static_cast<int>(this->SetAccelScale(imu_data.accelScale));
  auto retGyro = static_cast<int>(this->SetGyroScale(imu_data.gyroScale));
  auto retSample = static_cast<int>(this->SetSampleFrequency(imu_data.sampleFreq));
  ret = static_cast<eIMUAbstractionError>(retAccel + retGyro + retSample);

  /* Initialize Sampling IMU Thread if needed */
  if ((!thSampleValues.joinable()) && (ret == eIMUAbstractionError::eRET_OK))
  {
    bThreadSampleValues = true;
    auto sampleValues = [this](int sample_freq)
    {
      LOGDEBUG("Starting Sample Values Thread [Sample Freq %d]", sample_freq);
      while (bThreadSampleValues)
      {
        auto start = std::chrono::high_resolution_clock::now();

        for(auto &val : imu_data.axisdata)
        {
          val.accel = this->GetValueInFile<double>(val.DeviceAccelPath.c_str());
          val.gyro = this->GetValueInFile<double>(val.DeviceGyroPath.c_str());
        }
        this->NotifyUpdateData();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        if (duration.count() < sample_freq)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(sample_freq-duration.count()));
        }
      }
      LOGDEBUG("Finishing Sample Values Thread");
    };
    auto freq = this->GetValueInFile<int>(imu_data.paths.DeviceSampleFreqPath.c_str());
    thSampleValues = std::thread(sampleValues, freq);
    if (!thSampleValues.joinable())
    {
      /* Thread not initialized */
      ret = eIMUAbstractionError::eRET_ERROR;
    }
  }

  return ret;
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

int IMUIndustrialIO::GetSampleFrequency(void)
{
  return mapSampleFreq.at(sampleFreq_ms);
}

eIMUAbstractionError IMUIndustrialIO::SetSampleFrequency(eSampleFreq freq)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  if (mapSampleFreq.find(freq) != mapSampleFreq.end())
  {
    auto value = mapSampleFreq.at(freq);
    auto iScaleFileBefore = this->GetValueInFile<int>(imu_data.paths.DeviceSampleFreqPath.c_str());
    if (iScaleFileBefore != value)
    {
      ret = this->SetValueInFile<int>(imu_data.paths.DeviceSampleFreqPath.c_str(), value);
      LOGDEBUG("Set Sample Frequency [%d]->[%d ms]", freq, value);

      auto freqInFile = this->GetValueInFile<int>(imu_data.paths.DeviceSampleFreqPath.c_str());
      if (freqInFile != value)
      {
        LOGERROR("Failed Sampling Frequency [Desired %d ms] [Current %d ms]", value, freqInFile);
        ret = eIMUAbstractionError::eRET_ERROR;
      }
      else
      {
        sampleFreq_ms = freq;
      }
    }
  }
  else
  {
    ret = eIMUAbstractionError::eRET_INVALID_PARAMETER;
  }

  return ret;
}

eIMUAbstractionError IMUIndustrialIO::SetAccelScale(eAccelScale scale)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  if (mapAccelScale.find(scale) != mapAccelScale.end())
  {
    auto val = mapAccelScale.at(scale);
    auto sScaleFileBefore = this->GetValueInFile<std::string>(imu_data.paths.DeviceAccelScalePath.c_str());
    if (sScaleFileBefore.compare(val) != 0)
    {
      LOGDEBUG("Set Accel Scale [%d]->[%s]", scale, val.c_str());
      ret = this->SetValueInFile(imu_data.paths.DeviceAccelScalePath.c_str(), val);

      /* Verify if the value was written into the file */
      auto sScale = this->GetValueInFile<std::string>(imu_data.paths.DeviceAccelScalePath.c_str());
      if (sScale.compare(val) != 0)
      {
        ret = eIMUAbstractionError::eRET_ERROR;
        LOGERROR("Accel Scale not accepted [%s]", val.c_str());
      }
    }
  }
  else
  {
    ret = eIMUAbstractionError::eRET_INVALID_PARAMETER;
  }

  if (ret == eIMUAbstractionError::eRET_OK)
  {
    imu_data.accelScale = scale;
  }
  else
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
    auto sScaleFileBefore = this->GetValueInFile<std::string>(imu_data.paths.DeviceGyroScalePath.c_str());
    if (sScaleFileBefore.compare(val) != 0)
    {
      LOGDEBUG("Set Gyro Scale [%d]->[%s]", scale, val.c_str());
      ret = this->SetValueInFile(imu_data.paths.DeviceGyroScalePath.c_str(), val);

      /* Verify if the value was written into the file */
      auto sScale = this->GetValueInFile<std::string>(imu_data.paths.DeviceGyroScalePath.c_str());
      if (sScale.compare(val) != 0)
      {
        ret = eIMUAbstractionError::eRET_ERROR;
        LOGERROR("Gyro Scale not accepted [%s]", val.c_str());
      }
    }
  }
  else
  {
    ret = eIMUAbstractionError::eRET_INVALID_PARAMETER;
  }

  if (ret == eIMUAbstractionError::eRET_OK)
  {
    imu_data.gyroScale = scale;
  }
  else
  {
    LOGERROR("SetGyroScale failed [%d]", ret);
  }

  return ret;
}

eIMUAbstractionError IMUIndustrialIO::GetRawAccel(DBusTypes::eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  if (axis >= DBusTypes::eAxis::X && axis <= DBusTypes::eAxis::Z)
  {
    auto val_scale = std::stod(mapAccelScale.at(imu_data.accelScale));
    val = ((imu_data.axisdata[(int)axis].accel * val_scale)/M_Gl)*1000L;
  }
  else
  {
    val = 0;
    ret = eIMUAbstractionError::eRET_INVALID_PARAMETER;
  }

  return ret;
}
eIMUAbstractionError IMUIndustrialIO::GetRawGyro(DBusTypes::eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  if (axis >= DBusTypes::eAxis::X && axis <= DBusTypes::eAxis::Z)
  {
    auto val_scale = std::stod(mapGyroScale.at(imu_data.gyroScale));
    val = ((imu_data.axisdata[(int)axis].gyro * val_scale)/M_PIl)*180L;
  }
  else
  {
    val = 0;
    ret = eIMUAbstractionError::eRET_INVALID_PARAMETER;
  }

  return ret;
}

void IMUIndustrialIO::DeInit(void)
{
  /* Avoid deadlock by calling callbacks during DeInitialization */
  vecCallback.clear();

  if (thSampleValues.joinable())
  {
    bThreadSampleValues = false;
    thSampleValues.join();
  }
}

IMUIndustrialIO::~IMUIndustrialIO()
{
  this->DeInit();
}
