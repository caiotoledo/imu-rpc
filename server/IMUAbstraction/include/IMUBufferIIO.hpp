#pragma once

#include "IMUIndustrialIO.hpp"

namespace IMUAbstraction
{

  class IMUBufferIIO : public IMUIndustrialIO
  {
  private:
    eIMUAbstractionError ConfigureBuffering(void);

    /**
     * @brief Convert 16 bits Twos Complement
     *
     * @param value Input value
     * @return int16_t Converted value
     */
    int16_t ConvertTwosComplementToNum(uint16_t value);

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
