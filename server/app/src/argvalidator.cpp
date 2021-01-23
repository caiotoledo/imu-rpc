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
  auto ret = true;

  if (mapAccelScale.find(args.accel_scale) == mapAccelScale.end())
  {
    ret = false;
  }

  if (mapGyroScale.find(args.gyro_scale) == mapGyroScale.end())
  {
    ret = false;
  }

  if (mapSampleFreq.find(args.sample_rate) == mapSampleFreq.end())
  {
    ret = false;
  }

  return ret;
}

int ArgValidator::ConvertArgs(const ArgParser::arguments &args,
                               IMUAbstraction::eAccelScale &accel,
                               IMUAbstraction::eGyroScale &gyro,
                               IMUAbstraction::eSampleFreq &freq)
{
  auto ret = 0;

  if (ArgValidator::IsValidArgs(args))
  {
    accel = mapAccelScale.at(args.accel_scale);
    gyro = mapGyroScale.at(args.gyro_scale);
    freq = mapSampleFreq.at(args.sample_rate);
  }
  else
  {
    ret = -1;
    LOGERROR("Invalid Arguments Value!", args.accel_scale);
  }

  return ret;
}
