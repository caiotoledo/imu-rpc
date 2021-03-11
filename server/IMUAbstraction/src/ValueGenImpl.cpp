#include <cmath>

#include <TimerInstance.hpp>

#include <ValueGenImpl.hpp>

using namespace IMUAbstraction;

#define DEG_TO_RAD(x)     (((double)x)*(M_PIl/(double)180.0))
#define GET_CURR_MILLI()  (TIMER_INSTANCE.GetSeconds())

/* RANDOM VALUE GENERATOR CLASS */

RandomValueGenerator::RandomValueGenerator()
{
  /* Random Seed Initialization */
  srand((unsigned) time(0));
}

double RandomValueGenerator::GetRandomValue(int scale)
{
  bool signal = ((bool)(rand() % 2));

  double ret = (rand() % (int)scale);
  ret = signal ? ret : (-ret);

  return ret;
}

eIMUAbstractionError RandomValueGenerator::GetRawAccel(DBusTypes::eAxis axis, double &val, double scale)
{
  auto ret = this->GetRandomValue(scale);
  val = ret;
  return eIMUAbstractionError::eRET_OK;
}
eIMUAbstractionError RandomValueGenerator::GetRawGyro(DBusTypes::eAxis axis, double &val, double scale)
{
  auto ret = this->GetRandomValue(scale);
  val = ret;
  return eIMUAbstractionError::eRET_OK;
}

/* SIN WAVE VALUE GENERATOR CLASS */

SinWaveValueGenerator::SinWaveValueGenerator(double freq, double phaseshift_deg) :
  freq_sin(freq),
  phaseshift_axis(phaseshift_deg)
{
}

double SinWaveValueGenerator::SinValue(double amplitude, double freq, double timepoint, double phaseshift)
{
  auto phaseshift_rad = DEG_TO_RAD(phaseshift);
  auto ret = amplitude * sin(2L * M_PIl * freq * timepoint + phaseshift_rad);
  return ret;
}

eIMUAbstractionError SinWaveValueGenerator::GetRawAccel(DBusTypes::eAxis axis, double &val, double scale)
{
  double amp = scale;
  double freq = freq_sin;
  double timepoint = GET_CURR_MILLI();
  auto phaseshift = (double)axis * phaseshift_axis;

  double ret = this->SinValue(amp, freq, timepoint, phaseshift);
  val = ret;

  return eIMUAbstractionError::eRET_OK;
}
eIMUAbstractionError SinWaveValueGenerator::GetRawGyro(DBusTypes::eAxis axis, double &val, double scale)
{
  double amp = scale;
  double freq = freq_sin;
  double timepoint = GET_CURR_MILLI();
  auto phaseshift = ((double)axis + 1) * phaseshift_axis;

  double ret = this->SinValue(amp, freq, timepoint, phaseshift);
  val = ret;

  return eIMUAbstractionError::eRET_OK;
}
