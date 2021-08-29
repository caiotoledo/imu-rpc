#include <limits>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MockIMUAbstraction.hpp>

#include <IMUEulerAngle.hpp>

using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::AtLeast;

/**
 * TESTING EULER ANGLE
 */
class GetEulerAngleTestsParameterized : public
  ::testing::TestWithParam<
    std::tuple<
      double,double,double, /* Accel */
      double,double,double, /* Angle */
      DBusTypes::eAngleUnit
      >
    >
  {};

TEST_P(GetEulerAngleTestsParameterized, GetEulerAngle)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUAngle::IIMUAngle> imuAngle;
  imuAngle = std::make_shared<IMUAngle::IMUEulerAngle>(imuMock);

  /* Prepare local variables */
  const double accel[] = {
    std::get<0>(GetParam()),
    std::get<1>(GetParam()),
    std::get<2>(GetParam()),
  };
  const double angle[] = {
    std::get<3>(GetParam()),
    std::get<4>(GetParam()),
    std::get<5>(GetParam()),
  };

  auto angleUnit = std::get<6>(GetParam());

  auto funcGetRawAccel = [&accel](DBusTypes::eAxis axis, double &val)
  {
    auto axis_index = static_cast<int>(axis);
    val = accel[axis_index];
    return IMUAbstraction::eIMUAbstractionError::eRET_OK;
  };

  /* Prepare mock env */
  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AtLeast(1));

  ON_CALL(*imuMock, GetRawAccel(_,_))
    .WillByDefault(Invoke(funcGetRawAccel));

  /* Test Euler Angle */
  auto nan = std::numeric_limits<double>::quiet_NaN();
  double eulerAngle[IMUAbstraction::NUM_AXIS] = {nan, nan, nan};
  for (size_t i = 0; i < IMUAbstraction::NUM_AXIS; i++)
  {
    auto axis = static_cast<DBusTypes::eAxis>(i);
    auto retEuler = imuAngle->GetEulerAngle(eulerAngle[i], axis, angleUnit);
    EXPECT_EQ(retEuler, IMUAngle::eIMUAngleError::eRET_OK);
  }
  EXPECT_EQ(round(eulerAngle[0]), angle[0]);
  EXPECT_EQ(round(eulerAngle[1]), angle[1]);
  EXPECT_EQ(round(eulerAngle[2]), angle[2]);
}

INSTANTIATE_TEST_CASE_P(
    GetEulerAngleTests,
    GetEulerAngleTestsParameterized,
    ::testing::Values(
      /* AccelX, AccelY, AccelZ, AngleX, AngleY, AngleZ, eAngleUnit */
      std::make_tuple(0,0,0,0,0,0, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(500,500,500,45,45,45, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(0,500,500,45,0,90, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(0,0,-1000,180,180,0, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(-1000,0,0,0,270,180, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(0,0,0,0,0,0, DBusTypes::eAngleUnit::eRadians)
    )
);

TEST(IMUMathImpl, GetEulerAngleAbstractionError)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUAngle::IIMUAngle> imuAngle;
  imuAngle = std::make_shared<IMUAngle::IMUEulerAngle>(imuMock);

  /* Prepare mock env */
  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_ERROR));

  /* Test Euler Angle */
  auto nan = std::numeric_limits<double>::quiet_NaN();
  double eulerAngle[IMUAbstraction::NUM_AXIS] = {nan, nan, nan};
  for (size_t i = 0; i < IMUAbstraction::NUM_AXIS; i++)
  {
    auto axis = static_cast<DBusTypes::eAxis>(i);
    auto retEuler = imuAngle->GetEulerAngle(eulerAngle[i], axis, DBusTypes::eAngleUnit::eDegrees);
    EXPECT_EQ(retEuler, IMUAngle::eIMUAngleError::eRET_ERROR);
  }
}

TEST(IMUMathImpl, GetEulerAngleInvalidParameters)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUAngle::IIMUAngle> imuAngle;
  imuAngle = std::make_shared<IMUAngle::IMUEulerAngle>(imuMock);

  /* Prepare mock env */
  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(0),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
      )
    );

  /* Test Euler Angle */
  double eulerAngle;
  /* Set invalid Axis */
  auto axis = static_cast<DBusTypes::eAxis>(100);
  /* Expected Error for the invalid axis */
  auto retEuler = imuAngle->GetEulerAngle(eulerAngle, axis, DBusTypes::eAngleUnit::eDegrees);
  EXPECT_EQ(retEuler, IMUAngle::eIMUAngleError::eRET_INVALID_PARAMETER);

  /* Use Invalid Angle Unit */
  auto angleUnit = static_cast<DBusTypes::eAngleUnit>(100);
  /* Expected Error for the invalid angle unit */
  retEuler = imuAngle->GetEulerAngle(eulerAngle, DBusTypes::eAxis::X, angleUnit);
  EXPECT_EQ(retEuler, IMUAngle::eIMUAngleError::eRET_INVALID_PARAMETER);
}
