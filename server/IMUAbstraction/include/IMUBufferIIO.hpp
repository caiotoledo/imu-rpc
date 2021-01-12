#pragma once

#include "IMUIndustrialIO.hpp"

namespace IMUAbstraction
{

  class IMUBufferIIO : public IMUIndustrialIO
  {
  private:
    /**
     * @brief Configure Industrial IO buffering
     *
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eIMUAbstractionError
     */
    eIMUAbstractionError ConfigureBuffering(void);

    /**
     * @brief Convert 16 bits Twos Complement
     *
     * @param value Input value
     * @return int16_t Converted value
     */
    int16_t ConvertTwosComplementToNum(uint16_t value);

    /**
     * @brief Sampling IMU Values to be used in thread execution
     *
     * @param sample_freq Sample frequency in milliseconds
     */
    void SamplingIMUValuesThread(uint16_t sample_freq);

  public:
    IMUBufferIIO(
      const char *path,
      int device_index,
      eAccelScale accelScale=eAccelScale::Accel_2g,
      eGyroScale gyroScale=eGyroScale::Gyro_250dps,
      eSampleFreq sampleFreq=eSampleFreq::Freq_500ms
    );

    eIMUAbstractionError Init(void) override;

    virtual ~IMUBufferIIO();
  };

} // namespace IMUServer
