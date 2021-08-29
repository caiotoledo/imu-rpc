#include <limits>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MockIMUAbstraction.hpp>
#include <MockIMUAngle.hpp>

#include <IMUMathImpl.hpp>

using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::AtLeast;
using ::testing::AnyNumber;

constexpr auto SAMPLERATE = 5; /* ms */
/* Constant used in Complementary Filter */
constexpr double ALPHA = 0.7143;

inline double round( double val )
{
  if( val < 0 ) return ceil(val - 0.5);
  return floor(val + 0.5);
}

TEST(IMUMathImpl, imumath_init)
{
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();
  auto angleMock = std::make_shared<IMUAngle::MockIMUAngle>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, angleMock, ALPHA);

  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*angleMock, GetEulerAngle(_,_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(
      DoAll(
        SetArgReferee<0>(0),
        Return(IMUAngle::eIMUAngleError::eRET_OK)
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
 * TESTING COMPLEMENTARY FILTER
 */
class ComplFilterAngleTestsParameterized : public
  ::testing::TestWithParam<
    std::tuple<
      double,double,double, /* AngleEuler */
      double,double,double, /* Gyro */
      double,double,double, /* AngleFilter */
      DBusTypes::eAngleUnit
      >
  > {};

TEST_P(ComplFilterAngleTestsParameterized, ComplFilterAngle)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();
  auto angleMock = std::make_shared<IMUAngle::MockIMUAngle>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, angleMock, ALPHA);

  /* Prepare local variables */
  const double euler[] = {
    std::get<0>(GetParam()),
    std::get<1>(GetParam()),
    std::get<2>(GetParam()),
  };
  double gyro[] = {
    std::get<3>(GetParam()),
    std::get<4>(GetParam()),
    std::get<5>(GetParam()),
  };
  const double angle[] = {
    std::get<6>(GetParam()),
    std::get<7>(GetParam()),
    std::get<8>(GetParam()),
  };

  auto angleUnit = std::get<9>(GetParam());

  auto funcGetEulerAngle = [&euler](double &value, DBusTypes::eAxis axis, const DBusTypes::eAngleUnit &unit)
  {
    auto axis_index = static_cast<int>(axis);
    value = euler[axis_index];
    return IMUAngle::eIMUAngleError::eRET_OK;
  };
  auto funcGetRawGyro = [&gyro](DBusTypes::eAxis axis, double &val)
  {
    auto axis_index = static_cast<int>(axis);
    if (gyro[axis_index] > 0)
    {
      gyro[axis_index] -= 10;
      gyro[axis_index] = (gyro[axis_index] < 0) ? 0 : gyro[axis_index];
    }
    val = gyro[axis_index];
    return IMUAbstraction::eIMUAbstractionError::eRET_OK;
  };

  std::thread thCallbackManager;
  auto bCallbackManager = false;
  auto funcAddUpdateDataCallback = [&thCallbackManager, &bCallbackManager](std::function<void()> cb)
  {
    if (!thCallbackManager.joinable())
    {
      bCallbackManager = true;
      thCallbackManager = std::thread(
        [cb, &bCallbackManager]()
        {
          while (bCallbackManager)
          {
            cb();
            std::this_thread::sleep_for(std::chrono::milliseconds(SAMPLERATE));
          }
        }
      );
    }
  };

  /* Prepare mock env */
  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*angleMock, GetEulerAngle(_,_,_))
    .Times(AtLeast(1));

  EXPECT_CALL(*imuMock, GetRawGyro(_,_))
    .Times(AtLeast(1));

  EXPECT_CALL(*imuMock, DeInit())
    .Times(1);

  ON_CALL(*angleMock, GetEulerAngle(_,_,_))
    .WillByDefault(Invoke(funcGetEulerAngle));

  ON_CALL(*imuMock, GetRawGyro(_,_))
    .WillByDefault(Invoke(funcGetRawGyro));

  ON_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .WillByDefault(Invoke(funcAddUpdateDataCallback));

  /* Test Init IMUMath */
  auto retInit = imuMath->Init();
  EXPECT_EQ(retInit, IMUMath::eIMUMathError::eRET_OK);

  /* Wait complementary filter initialization and filter stabilization */
  std::this_thread::sleep_for(std::chrono::milliseconds(10*SAMPLERATE));

  /* Test Complementary Filter Angle */
  auto nan = std::numeric_limits<double>::quiet_NaN();
  double complFilterAngle[IMUAbstraction::NUM_AXIS] = {nan, nan, nan};
  for (size_t i = 0; i < IMUAbstraction::NUM_AXIS; i++)
  {
    auto axis = static_cast<DBusTypes::eAxis>(i);
    auto retComplFilter = imuMath->GetComplFilterAngle(complFilterAngle[i], axis, angleUnit);
    EXPECT_EQ(retComplFilter, IMUMath::eIMUMathError::eRET_OK);
  }
  EXPECT_EQ(round(complFilterAngle[0]), angle[0]);
  EXPECT_EQ(round(complFilterAngle[1]), angle[1]);
  EXPECT_EQ(round(complFilterAngle[2]), angle[2]);

  /* Finish callback manager thread */
  if (thCallbackManager.joinable())
  {
    bCallbackManager = false;
    thCallbackManager.join();
  }
}

INSTANTIATE_TEST_CASE_P(
    ComplFilterAngleTests,
    ComplFilterAngleTestsParameterized,
    ::testing::Values(
      /* EulerX, EulerY, EulerZ, GyroX, GyroY, GyroZ, AngleX, AngleY, AngleZ, eAngleUnit */
      std::make_tuple(0,0,0,0,0,0,0,0,0, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(45,45,45,45,45,45,45,45,45, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(0,270,180,45,45,45,0,270,180, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(0,0,0,0,0,0,0,0,0, DBusTypes::eAngleUnit::eRadians)
    )
);

TEST(IMUMathImpl, ComplFilterAngleInvalidParameters)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();
  auto angleMock = std::make_shared<IMUAngle::MockIMUAngle>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, angleMock, ALPHA);

  /* Prepare local variables */
  std::thread thCallbackManager;
  auto bCallbackManager = false;
  auto funcAddUpdateDataCallback = [&thCallbackManager, &bCallbackManager](std::function<void()> cb)
  {
    if (!thCallbackManager.joinable())
    {
      bCallbackManager = true;
      thCallbackManager = std::thread(
        [cb, &bCallbackManager]()
        {
          while (bCallbackManager)
          {
            cb();
            std::this_thread::sleep_for(std::chrono::milliseconds(SAMPLERATE));
          }
        }
      );
    }
  };

  /* Prepare mock env */
  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(AnyNumber());

  EXPECT_CALL(*angleMock, GetEulerAngle(_,_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(
      DoAll(
        SetArgReferee<0>(0),
        Return(IMUAngle::eIMUAngleError::eRET_OK)
      )
    );

  EXPECT_CALL(*imuMock, GetRawGyro(_,_))
    .Times(AnyNumber())
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(0),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
      )
    );

  EXPECT_CALL(*imuMock, DeInit())
    .Times(1);

  ON_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .WillByDefault(Invoke(funcAddUpdateDataCallback));

  /* Test Init IMUMath */
  auto retInit = imuMath->Init();
  EXPECT_EQ(retInit, IMUMath::eIMUMathError::eRET_OK);

  std::this_thread::sleep_for(std::chrono::milliseconds(3*SAMPLERATE));

  /* Test Complementary Filter Angle */
  double complFilterAngle;
  /* Set Invalid Axis */
  auto invalidAxis = static_cast<DBusTypes::eAxis>(100);
  /* Set Invalid Angle Unit */
  auto invalidAngleUnit = static_cast<DBusTypes::eAngleUnit>(100);

  /* Expected Error for the invalid axis */
  auto retComplFilter = imuMath->GetComplFilterAngle(complFilterAngle, invalidAxis, DBusTypes::eAngleUnit::eDegrees);
  EXPECT_EQ(retComplFilter, IMUMath::eIMUMathError::eRET_INVALID_PARAMETER);
  /* Expected Error for the invalid angle unit */
  retComplFilter = imuMath->GetComplFilterAngle(complFilterAngle, DBusTypes::eAxis::X, invalidAngleUnit);
  EXPECT_EQ(retComplFilter, IMUMath::eIMUMathError::eRET_INVALID_PARAMETER);

  /* Finish callback manager thread */
  if (thCallbackManager.joinable())
  {
    bCallbackManager = false;
    thCallbackManager.join();
  }
}
