#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MockIMUAbstraction.hpp>

#include <IMUMathImpl.hpp>

using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::AtLeast;
using ::testing::AnyNumber;

typedef struct cv_s
{
  bool flag;
  std::mutex mtx;
  std::condition_variable cv;
} cv_t;

constexpr auto WAITTIMEOUT = std::chrono::milliseconds(500);
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

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, ALPHA);

  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AtLeast(1))
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

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, ALPHA);

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
  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AtLeast(1));

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
    auto retEuler = imuMath->GetEulerAngle(eulerAngle[i], axis, angleUnit);
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

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, ALPHA);

  /* Prepare mock env */
  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_ERROR));

  EXPECT_CALL(*imuMock, DeInit())
    .Times(1);

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
    EXPECT_EQ(retEuler, IMUMath::eIMUMathError::eRET_ERROR);
  }
}

TEST(IMUMathImpl, GetEulerAngleInvalidParameters)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, ALPHA);

  /* Prepare mock env */
  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(0),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
      )
    );

  EXPECT_CALL(*imuMock, DeInit())
    .Times(1);

  /* Test Init IMUMath */
  auto retInit = imuMath->Init();
  EXPECT_EQ(retInit, IMUMath::eIMUMathError::eRET_OK);

  /* Test Euler Angle */
  double eulerAngle;
  /* Set invalid Axis */
  auto axis = static_cast<DBusTypes::eAxis>(100);
  /* Expected Error for the invalid axis */
  auto retEuler = imuMath->GetEulerAngle(eulerAngle, axis, DBusTypes::eAngleUnit::eDegrees);
  EXPECT_EQ(retEuler, IMUMath::eIMUMathError::eRET_INVALID_PARAMETER);

  /* Use Invalid Angle Unit */
  auto angleUnit = static_cast<DBusTypes::eAngleUnit>(100);
  /* Expected Error for the invalid angle unit */
  retEuler = imuMath->GetEulerAngle(eulerAngle, DBusTypes::eAxis::X, angleUnit);
  EXPECT_EQ(retEuler, IMUMath::eIMUMathError::eRET_INVALID_PARAMETER);
}

/**
 * TESTING COMPLEMENTARY FILTER
 */
class ComplFilterAngleTestsParameterized : public
  ::testing::TestWithParam<
    std::tuple<
      double,double,double, /* Accel */
      double,double,double, /* Gyro */
      double,double,double, /* Angle */
      DBusTypes::eAngleUnit
      >
  > {};

TEST_P(ComplFilterAngleTestsParameterized, ComplFilterAngle)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, ALPHA);

  /* Prepare local variables */
  const double accel[] = {
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

  auto funcGetRawAccel = [&accel](DBusTypes::eAxis axis, double &val)
  {
    auto axis_index = static_cast<int>(axis);
    val = accel[axis_index];
    return IMUAbstraction::eIMUAbstractionError::eRET_OK;
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

  /* Conditional Variable to notify enough callback notifications */
  cv_t cvCallbackCounter;
  cvCallbackCounter.flag = false;

  std::thread thCallbackManager;
  auto bCallbackManager = false;
  auto funcAddUpdateDataCallback = [&cvCallbackCounter, &thCallbackManager, &bCallbackManager](std::function<void()> cb)
  {
    if (!thCallbackManager.joinable())
    {
      bCallbackManager = true;
      thCallbackManager = std::thread(
        [cb, &cvCallbackCounter, &bCallbackManager]()
        {
          /* Wait 10 iterations for Complementary Filter Stabilization */
          constexpr auto maxCountCallbackIteration = 10;
          auto countCallbackIteration = 0;
          while (bCallbackManager)
          {
            cb();
            countCallbackIteration++;
            if (countCallbackIteration >= maxCountCallbackIteration)
            {
              std::lock_guard<std::mutex> lock(cvCallbackCounter.mtx);
              cvCallbackCounter.flag = true;
              cvCallbackCounter.cv.notify_one();
            }
            /* Allow other threads to run */
            std::this_thread::yield();
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

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AtLeast(1));

  EXPECT_CALL(*imuMock, GetRawGyro(_,_))
    .Times(AtLeast(1));

  EXPECT_CALL(*imuMock, DeInit())
    .Times(1);

  ON_CALL(*imuMock, GetRawAccel(_,_))
    .WillByDefault(Invoke(funcGetRawAccel));

  ON_CALL(*imuMock, GetRawGyro(_,_))
    .WillByDefault(Invoke(funcGetRawGyro));

  ON_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .WillByDefault(Invoke(funcAddUpdateDataCallback));

  /* Test Init IMUMath */
  auto retInit = imuMath->Init();
  EXPECT_EQ(retInit, IMUMath::eIMUMathError::eRET_OK);

  {
    std::unique_lock<std::mutex> lock(cvCallbackCounter.mtx);
    cvCallbackCounter.cv.wait_for(
      lock,
      WAITTIMEOUT,
      [&cvCallbackCounter](){return cvCallbackCounter.flag;}
    );
  }

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
      /* AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, AngleX, AngleY, AngleZ, eAngleUnit */
      std::make_tuple(0,0,0,0,0,0,0,0,0, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(500,500,500,45,45,45,45,45,45, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(-1000,0,0,45,45,45,0,270,180, DBusTypes::eAngleUnit::eDegrees),
      std::make_tuple(0,0,0,0,0,0,0,0,0, DBusTypes::eAngleUnit::eRadians)
    )
);

TEST(IMUMathImpl, ComplFilterAngleInvalidParameters)
{
  /* Construct objects */
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();

  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuMock, ALPHA);

  /* Prepare local variables */

  /* Conditional Variable to notify enough callback notifications */
  cv_t cvCallback;
  cvCallback.flag = false;

  std::thread thCallbackManager;
  auto bCallbackManager = false;
  auto funcAddUpdateDataCallback = [&cvCallback, &thCallbackManager, &bCallbackManager](std::function<void()> cb)
  {
    if (!thCallbackManager.joinable())
    {
      bCallbackManager = true;
      thCallbackManager = std::thread(
        [cb, &cvCallback, &bCallbackManager]()
        {
          while (bCallbackManager)
          {
            cb();
            {
              std::lock_guard<std::mutex> lock(cvCallback.mtx);
              cvCallback.flag = true;
              cvCallback.cv.notify_one();
            }
            /* Allow other threads to run */
            std::this_thread::yield();
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

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(AnyNumber())
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(0),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
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

  {
    std::unique_lock<std::mutex> lock(cvCallback.mtx);
    cvCallback.cv.wait_for(
      lock,
      WAITTIMEOUT,
      [&cvCallback](){return cvCallback.flag;}
    );
  }

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
