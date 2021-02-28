#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MockValueGenerator.hpp>

#include <IMUStub.hpp>

using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::AtLeast;

TEST(IMUStub, GetRaw_Accel_Gyro_RET_OK)
{
  /* Construct objects */
  auto valueGen = std::make_shared<IMUAbstraction::MockValueGenerator>();

  std::shared_ptr<IMUAbstraction::IIMUAbstraction> imuStub;
  imuStub = std::make_shared<IMUAbstraction::IMUStub>(
    valueGen,
    IMUAbstraction::eAccelScale::Accel_2g,
    IMUAbstraction::eGyroScale::Gyro_250dps,
    IMUAbstraction::eSampleFreq::Freq_10ms
  );

  /* Prepare local variables */
  const double expectedAccel = 1;
  const double expectedGyro = 2;
  double accelValue = 0;
  double gyroValue = 0;

  auto funcCallback = [&imuStub, &accelValue, &gyroValue]()
  {
    auto ret = imuStub->GetRawAccel(DBusTypes::eAxis::X, accelValue);
    EXPECT_EQ(ret, IMUAbstraction::eIMUAbstractionError::eRET_OK);

    ret = imuStub->GetRawGyro(DBusTypes::eAxis::X, gyroValue);
    EXPECT_EQ(ret, IMUAbstraction::eIMUAbstractionError::eRET_OK);
  };

  /* Prepare mock env */
  EXPECT_CALL(*valueGen, GetRawAccel(_,_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(expectedAccel*100),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
      )
    );

  EXPECT_CALL(*valueGen, GetRawGyro(_,_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(expectedGyro*100),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
      )
    );

  /* Perform test */
  auto retInit = imuStub->Init();

  imuStub->AddUpdateDataCallback(funcCallback);

  auto sample_rate = imuStub->GetSampleFrequency();
  std::this_thread::sleep_for(std::chrono::milliseconds(5*sample_rate));

  imuStub->DeInit();

  /* Check Results */
  EXPECT_EQ(retInit, IMUAbstraction::eIMUAbstractionError::eRET_OK);
  EXPECT_EQ(accelValue, expectedAccel);
  EXPECT_EQ(gyroValue, expectedGyro);
}

TEST(IMUStub, GetRaw_Accel_Gyro_RET_INVALID_PARAMETER)
{
  /* Construct objects */
  auto valueGen = std::make_shared<IMUAbstraction::MockValueGenerator>();

  std::shared_ptr<IMUAbstraction::IIMUAbstraction> imuStub;
  imuStub = std::make_shared<IMUAbstraction::IMUStub>(
    valueGen,
    IMUAbstraction::eAccelScale::Accel_2g,
    IMUAbstraction::eGyroScale::Gyro_250dps,
    IMUAbstraction::eSampleFreq::Freq_10ms
  );

  /* Prepare mock env */
  EXPECT_CALL(*valueGen, GetRawAccel(_,_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*valueGen, GetRawGyro(_,_,_))
    .Times(AtLeast(1))
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  /* Perform test */
  auto retInit = imuStub->Init();

  double value = 0;
  auto invalidAxis = static_cast<DBusTypes::eAxis>(100);
  auto retAccel = imuStub->GetRawAccel(invalidAxis, value);
  auto retGyro = imuStub->GetRawGyro(invalidAxis, value);

  /* Check Results */
  EXPECT_EQ(retInit, IMUAbstraction::eIMUAbstractionError::eRET_OK);
  EXPECT_EQ(retAccel, IMUAbstraction::eIMUAbstractionError::eRET_INVALID_PARAMETER);
  EXPECT_EQ(retGyro, IMUAbstraction::eIMUAbstractionError::eRET_INVALID_PARAMETER);
}
