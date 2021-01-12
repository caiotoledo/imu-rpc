#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <fstream>

#include <IMUBufferIIO.hpp>

#include <LogInstance.h>

#define ST_ENABLE (1U)

using namespace IMUAbstraction;

IMUBufferIIO::IMUBufferIIO(
  const char *path,
  int device_index,
  eAccelScale accelScale,
  eGyroScale gyroScale,
  eSampleFreq sampleFreq
) :
  IMUIndustrialIO(path, device_index, accelScale, gyroScale, sampleFreq)
{
}

eIMUAbstractionError IMUBufferIIO::Init(void)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  this->InitializePaths(this->DevicePath);

  /* Set default precision */
  auto retAccel = static_cast<int>(this->SetAccelScale(imu_data.accelScale));
  auto retGyro = static_cast<int>(this->SetGyroScale(imu_data.gyroScale));
  auto retSample = static_cast<int>(this->SetSampleFrequency(imu_data.sampleFreq));
  auto retBuffering = static_cast<int>(this->ConfigureBuffering());
  ret = static_cast<eIMUAbstractionError>(retAccel + retGyro + retSample + retBuffering);

  /* Initialize Sampling IMU Thread if needed */
  if ((!thSampleValues.joinable()) && (ret == eIMUAbstractionError::eRET_OK))
  {
    bThreadSampleValues = true;
    auto sampleValues = [this](int sample_freq)
    {
      this->SamplingIMUValuesThread(sample_freq);
    };

    auto freq = this->GetValueInFile<int>(imu_data.paths.DeviceSampleFreqPath.c_str());
    thSampleValues = std::thread(sampleValues, freq);
    if (!thSampleValues.joinable())
    {
      /* Thread not initialized */
      bThreadSampleValues = false;
      ret = eIMUAbstractionError::eRET_ERROR;
    }
  }

  return ret;
}

void IMUBufferIIO::SamplingIMUValuesThread(uint16_t sample_freq)
{
  LOGDEBUG("Starting Sample Values Thread [Sample Freq %d]", sample_freq);

  auto bufferDevice = this->imu_data.paths.DeviceBufferPath.c_str();
  auto fdBufDevice = open(bufferDevice, O_RDONLY | O_NONBLOCK);

  while (bThreadSampleValues)
  {
    auto start = std::chrono::high_resolution_clock::now();

    if (fdBufDevice >= 0)
    {
      fd_set rfds;
      struct timeval tv;

      FD_ZERO(&rfds);
      FD_SET(fdBufDevice, &rfds);
      tv.tv_sec = 1;
      tv.tv_usec = 0;

      auto retselect = select(fdBufDevice+1, &rfds, NULL, NULL, &tv);
      if (retselect == 1)
      {
        uint8_t val[NUM_AXIS*2*4];

        auto size_read = read(fdBufDevice, val, sizeof(val));
        if (size_read >= 0)
        {
          for (size_t i = 0; i < (NUM_AXIS*2); i+=2)
          {
            /* Accelerometer data is available in the first 6 bytes */
            uint16_t accel_raw = (uint16_t)((val[i] << 8) | val[i+1]);
            auto accel_val = this->ConvertTwosComplementToNum(accel_raw);
            imu_data.axisdata[i/2].accel = accel_val;

            /* Gyroscope data is available after the 6 first bytes */
            auto index_offset = (NUM_AXIS*2);
            uint16_t gyro_raw = (uint16_t)((val[i+index_offset] << 8) | val[i+1+index_offset]);
            auto gyro_val = this->ConvertTwosComplementToNum(gyro_raw);
            imu_data.axisdata[i/2].gyro = gyro_val;
          }

          this->NotifyUpdateData();
        }
        else if (size_read == -1)
        {
          LOGWARN("read error [%d]-[%s]", errno, strerror(errno));
        }
      }
      else
      {
        LOGWARN("Device not ready [%d]", retselect);
      }
    }
    else
    {
      bThreadSampleValues = false;
      LOGERROR("Invalid File Descriptor [%d]-[%s]", errno, strerror(errno));
      break;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    if (duration.count() < sample_freq)
    {
      auto delay = sample_freq-duration.count();
      std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
  }

  if (fdBufDevice >= 0)
  {
    close(fdBufDevice);
  }

  LOGDEBUG("Finishing Sample Values Thread");
}

eIMUAbstractionError IMUBufferIIO::ConfigureBuffering(void)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  for(auto &val : imu_data.axisdata)
  {
    auto accel_buffer_en = this->GetValueInFile<int>(val.DeviceAccelBufferEnPath.c_str());
    if (accel_buffer_en != ST_ENABLE)
    {
      ret = this->SetValueInFile<int>(val.DeviceAccelBufferEnPath.c_str(), ST_ENABLE);
      if (ret != eIMUAbstractionError::eRET_OK)
      {
        LOGWARN("Failed to set value in [%s]", val.DeviceAccelBufferEnPath.c_str());
        break;
      }
    }

    auto gyro_buffer_en = this->GetValueInFile<int>(val.DeviceGyroBufferEnPath.c_str());
    if (gyro_buffer_en != ST_ENABLE)
    {
      ret = this->SetValueInFile<int>(val.DeviceGyroBufferEnPath.c_str(), ST_ENABLE);
      if (ret != eIMUAbstractionError::eRET_OK)
      {
        LOGWARN("Failed to set value in [%s]", val.DeviceGyroBufferEnPath.c_str());
        break;
      }
    }
  }

  auto buffer_en = this->GetValueInFile<int>(imu_data.paths.DeviceBufferEnablePath.c_str());
  if ((ret == eIMUAbstractionError::eRET_OK) && (buffer_en != ST_ENABLE))
  {
    ret = this->SetValueInFile<int>(imu_data.paths.DeviceBufferEnablePath.c_str(), ST_ENABLE);
    if (ret != eIMUAbstractionError::eRET_OK)
    {
      LOGERROR("Fail enable buffering in [%s]", imu_data.paths.DeviceBufferEnablePath.c_str());
    }
  }

  return ret;
}

int16_t IMUBufferIIO::ConvertTwosComplementToNum(uint16_t value)
{
  int16_t ret = 0;
  if (value == 0x8000)
  {
    ret = (int16_t)(0x8000);
  }
  else if (!((value & 0x8000) == 0x8000))
  {
    ret = value;
  }
  else
  {
    value = (((~value) + 1) & 0x7FFF);
    ret = -value;
  }
  return ret;
}

IMUBufferIIO::~IMUBufferIIO(void)
{
  this->DeInit();
}
