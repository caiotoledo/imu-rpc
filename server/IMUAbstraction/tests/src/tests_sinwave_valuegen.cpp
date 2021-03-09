#include <cmath>

#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ValueGenImpl.hpp>

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
  constexpr auto TOLERANCE = 0.05;
  double valAccel[] = {0,0,0};
  double valGyro[] = {0,0,0};

  /* Perform test */
  for (size_t i = 0; i < sizeof(valAccel)/sizeof(valAccel[0]); i++)
  {
    auto axis = static_cast<DBusTypes::eAxis>(i);
    valueGen->GetRawAccel(axis, valAccel[i], amp);
    valueGen->GetRawGyro(axis, valGyro[i], amp);
  }

  /* Check Results */
  for (size_t i = 0; i < 2; i++)
  {
    auto gyro = abs(valGyro[i]);
    auto accel = abs(valAccel[i+1]);

    EXPECT_LE(gyro, accel*(1+TOLERANCE));
    EXPECT_GE(gyro, accel*(1-TOLERANCE));
  }
}

INSTANTIATE_TEST_CASE_P(
    GetSinWaveValueTests,
    GetSinWaveValueTestsParameterized,
    ::testing::Values(
      /* Freq, PhaseShift, Amp */
      std::make_tuple(0.1,45,1000),
      std::make_tuple(0.1,90,1000),
      std::make_tuple(0.05,90,1000),
      std::make_tuple(0.01,45,1000),
      std::make_tuple(0.01,45,10000)
    )
);
