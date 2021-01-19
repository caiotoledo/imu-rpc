#include <map>

#include <LogInstance.h>

#include "argvalidator.hpp"

const std::map<int, IMUAbstraction::eAccelScale> mapAccelScale =
{
  {2,  IMUAbstraction::eAccelScale::Accel_2g},
  {4,  IMUAbstraction::eAccelScale::Accel_4g},
  {8,  IMUAbstraction::eAccelScale::Accel_8g},
  {16, IMUAbstraction::eAccelScale::Accel_16g},
};

const std::map<int, IMUAbstraction::eGyroScale> mapGyroScale =
{
  {250,  IMUAbstraction::eGyroScale::Gyro_250dps},
  {500,  IMUAbstraction::eGyroScale::Gyro_500dps},
  {1000, IMUAbstraction::eGyroScale::Gyro_1000dps},
  {2000, IMUAbstraction::eGyroScale::Gyro_2000dps},
};

const std::map<int, IMUAbstraction::eSampleFreq> mapSampleFreq =
{
  {10,  IMUAbstraction::eSampleFreq::Freq_10ms},
  {20,  IMUAbstraction::eSampleFreq::Freq_20ms},
  {50,  IMUAbstraction::eSampleFreq::Freq_50ms},
  {100, IMUAbstraction::eSampleFreq::Freq_100ms},
  {200, IMUAbstraction::eSampleFreq::Freq_200ms},
  {500, IMUAbstraction::eSampleFreq::Freq_500ms},
};

bool ArgValidator::IsValidArgs(const ArgParser::arguments &args)
{
  auto ret = 0;

  if (mapAccelScale.find(args.accel_scale) == mapAccelScale.end())
  {
    ret++;
  }

  if (mapGyroScale.find(args.gyro_scale) == mapGyroScale.end())
  {
    ret++;
  }

  if (mapSampleFreq.find(args.sample_rate) == mapSampleFreq.end())
  {
    ret++;
  }

  return (ret == 0) ? true : false;
}
