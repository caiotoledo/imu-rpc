#include <limits>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MockIMUAbstraction.hpp>

#include <IMUMathImpl.hpp>

using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::AnyNumber;

constexpr auto SAMPLERATE = 5; /* ms */

inline double round( double val )
{
  if( val < 0 ) return ceil(val - 0.5);
  return floor(val + 0.5);
}

TEST(imumathimpl, imumath_init)
{
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock);

  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AnyNumber())
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(0),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
      )
    );

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*imuMock, DeInit())
    .Times(1);

  auto ret = imuMath->Init();

  EXPECT_EQ(ret, IMUMath::eIMUMathError::eRET_OK);
}

/**
 * TESTING EULER ANGLE
 */
class GetEulerAngleTestsParameterized : public ::testing::TestWithParam<std::tuple<double,double,double,double,double,double>> {};

TEST_P(GetEulerAngleTestsParameterized, GetEulerAngle)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock);

  /* Prepare local variables */
  double accel[] = {
    std::get<0>(GetParam()),
    std::get<1>(GetParam()),
    std::get<2>(GetParam()),
  };
  double angle[] = {
    std::get<3>(GetParam()),
    std::get<4>(GetParam()),
    std::get<5>(GetParam()),
  };

  auto funcGetRawAccel = [accel](DBusTypes::eAxis axis, double &val)
  {
    auto axis_index = static_cast<int>(axis);
    val = accel[axis_index];
    return IMUAbstraction::eIMUAbstractionError::eRET_OK;
  };

  /* Prepare mock env */
  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AnyNumber());

  EXPECT_CALL(*imuMock, GetRawGyro(_,_))
    .Times(AnyNumber())
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, GetSampleFrequency())
    .Times(AnyNumber())
    .WillRepeatedly(Return(SAMPLERATE));

  EXPECT_CALL(*imuMock, DeInit())
    .Times(1);

  ON_CALL(*imuMock, GetRawAccel(_,_))
    .WillByDefault(Invoke(funcGetRawAccel));

  /* Test Init IMUMath */
  auto retInit = imuMath->Init();
  EXPECT_EQ(retInit, IMUMath::eIMUMathError::eRET_OK);

  /* Test Euler Angle */
  auto nan = std::numeric_limits<double>::quiet_NaN();
  double eulerAngle[IMUAbstraction::NUM_AXIS] = {nan, nan, nan};
  for (size_t i = 0; i < IMUAbstraction::NUM_AXIS; i++)
  {
    auto axis = static_cast<DBusTypes::eAxis>(i);
    auto retEuler = imuMath->GetEulerAngle(eulerAngle[i], axis, DBusTypes::eAngleUnit::eDegrees);
    EXPECT_EQ(retEuler, IMUMath::eIMUMathError::eRET_OK);
  }
  EXPECT_EQ(round(eulerAngle[0]), angle[0]);
  EXPECT_EQ(round(eulerAngle[1]), angle[1]);
  EXPECT_EQ(round(eulerAngle[2]), angle[2]);
}

INSTANTIATE_TEST_CASE_P(
    GetEulerAngleTests,
    GetEulerAngleTestsParameterized,
    ::testing::Values(
      /* AccelX, AccelY, AccelZ, AngleX, AngleY, AngleZ */
      std::make_tuple(0,0,0,0,0,0),
      std::make_tuple(500,500,500,45,45,45),
      std::make_tuple(0,500,500,45,0,90),
      std::make_tuple(0,0,-1000,180,180,0),
      std::make_tuple(-1000,0,0,0,270,180)
    )
);
