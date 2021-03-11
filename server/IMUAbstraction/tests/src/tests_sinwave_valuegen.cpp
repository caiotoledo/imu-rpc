#include <cmath>

#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <TimerInstance.hpp>

#include <ValueGenImpl.hpp>

using ::testing::Return;
using ::testing::DoAll;
using ::testing::AtLeast;

#define DEG_TO_RAD(x)   (((double)x)*(M_PIl/(double)180.0))

inline double round( double val )
{
  if( val < 0 ) return ceil(val - 0.5);
  return floor(val + 0.5);
}

inline double sinVal(double amplitude, double freq, double timepoint, double phaseshift_rad)
{
  auto ret = amplitude * sin(2L * M_PIl * freq * timepoint + phaseshift_rad);
  return ret;
}

/**
 * TESTING SIN WAVE GENERATOR
 */
class GetSinWaveValueTestsParameterized : public
  ::testing::TestWithParam<
    std::tuple<
      double, /* Freq */
      double, /* PhaseShift */
      double  /* Amplitude */
      >
    >
  {};

TEST_P(GetSinWaveValueTestsParameterized, GetSinWaveValue)
{
  /* Get Parameters */
  auto freq = std::get<0>(GetParam());
  auto phaseshift = std::get<1>(GetParam());
  auto amp = std::get<2>(GetParam());

  /* Construct objects */
  std::shared_ptr<IMUAbstraction::IValueGenerator> valueGen;
  valueGen = std::make_shared<IMUAbstraction::SinWaveValueGenerator>(freq,phaseshift);

  /* Prepare local variables */
  double timestamp[] = {0,0,0};
  double valAccel[] = {0,0,0};
  double valGyro[] = {0,0,0};
  const auto count_arr = sizeof(valAccel)/sizeof(valAccel[0]);

  /* Prepare mock env */
  EXPECT_CALL(TIMER_INSTANCE, GetSeconds())
    .Times(3*(count_arr))
    .WillRepeatedly(Return(10));

  /* Perform test */
  for (size_t i = 0; i < count_arr; i++)
  {
    auto axis = static_cast<DBusTypes::eAxis>(i);
    timestamp[i] = TIMER_INSTANCE.GetSeconds();
    valueGen->GetRawAccel(axis, valAccel[i], amp);
    valueGen->GetRawGyro(axis, valGyro[i], amp);
  }

  /* Check Results */
  for (size_t i = 0; i < count_arr; i++)
  {
    auto phaseshift_rad = DEG_TO_RAD(i*phaseshift);
    auto expectedAccelVal = sinVal(amp, freq, timestamp[i], phaseshift_rad);

    phaseshift_rad = DEG_TO_RAD((i+1)*phaseshift);
    auto expectedGyroVal = sinVal(amp, freq, timestamp[i], phaseshift_rad);

    EXPECT_EQ(round(expectedAccelVal), round(valAccel[i]));
    EXPECT_EQ(round(expectedGyroVal), round(valGyro[i]));
  }
}

INSTANTIATE_TEST_CASE_P(
    GetSinWaveValueTests,
    GetSinWaveValueTestsParameterized,
    ::testing::Values(
      /* Freq, PhaseShift, Amp */
      std::make_tuple(0.5,90,1000), /* Value used in IMU-Daemon application */
      std::make_tuple(10,45,10),
      std::make_tuple(10,90,10),
      std::make_tuple(1,45,100),
      std::make_tuple(0.1,90,1000),
      std::make_tuple(0.05,45,1000),
      std::make_tuple(0.01,90,1000)
    )
);
