#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <DBusTypes.hpp>
#include <SingletonInstancer.hpp>
#include <ValueGenImpl.hpp>

#include <MockRand.hpp>

using ::testing::Return;
using ::testing::DoAll;
using ::testing::AtLeast;

/**
 * TESTING RANDOM GENERATOR
 */
class GetRandomValueTestsParameterized : public
  ::testing::TestWithParam<
    std::tuple<
      int,  /* ExpectedValue */
      int,  /* RandomValue */
      int   /* Scale */
      >
    >
  {};

TEST_P(GetRandomValueTestsParameterized, GetRandomValue)
{
  /* Get Parameters */
  auto expectedVal = std::get<0>(GetParam());
  auto randVal = std::get<1>(GetParam());
  auto scale = std::get<2>(GetParam());

  /* Construct objects */
  auto env = singleton::SingletonInstancer<MockRand>::GetInstance();

  std::shared_ptr<IMUAbstraction::IValueGenerator> valueGen;
  valueGen = std::make_shared<IMUAbstraction::RandomValueGenerator>();

  /* Prepare local variables */
  auto valAccel = 0.0;
  auto valGyro = 0.0;

  /* Prepare mock env */
  EXPECT_CALL(*env, rand())
    .Times(4)
    .WillRepeatedly(Return(randVal));

  /* Perform test */
  valueGen->GetRawAccel(DBusTypes::eAxis::X, valAccel, scale);
  valueGen->GetRawGyro(DBusTypes::eAxis::X, valGyro, scale);

  /* Check Results */
  EXPECT_EQ(valAccel, expectedVal);
  EXPECT_EQ(valGyro, expectedVal);
}

INSTANTIATE_TEST_CASE_P(
    GetRandomValueTests,
    GetRandomValueTestsParameterized,
    ::testing::Values(
      /* ExpectedValue, RandomValue, Scale */
      std::make_tuple(3,3,1000),
      std::make_tuple(-2,2,1000),
      std::make_tuple(1,101,10)
    )
);
