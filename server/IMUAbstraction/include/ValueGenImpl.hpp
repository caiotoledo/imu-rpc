#pragma once

#include "IValueGenerator.hpp"

namespace IMUAbstraction
{

  class RandomValueGenerator : public IValueGenerator
  {
  private:
    double GetRandomValue(int scale);

  public:
    RandomValueGenerator();

    virtual eIMUAbstractionError GetRawAccel(DBusTypes::eAxis axis, double &val, double scale) override;
    virtual eIMUAbstractionError GetRawGyro(DBusTypes::eAxis axis, double &val, double scale) override;

    ~RandomValueGenerator() = default;
  };

  class SinWaveValueGenerator : public IValueGenerator
  {
  private:
    double freq_sin;
    double phaseshift_axis;

    double SinValue(double amplitude, double freq, double timepoint, double phaseshift=0);

  public:
    SinWaveValueGenerator(double freq, double phaseshift_deg);

    virtual eIMUAbstractionError GetRawAccel(DBusTypes::eAxis axis, double &val, double scale) override;
    virtual eIMUAbstractionError GetRawGyro(DBusTypes::eAxis axis, double &val, double scale) override;

    ~SinWaveValueGenerator() = default;
  };

} // namespace IMUAbstraction
