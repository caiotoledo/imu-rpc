#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MockIMUAbstraction.hpp>
#include <MockIMUAngle.hpp>
#include <MockIMUMath.hpp>
#include <MockRPCServer.hpp>

#include <IMURPCServer.hpp>

using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::AtLeast;

TEST(IMURPCServer, StartServer_RET_OK)
{
  /* Construct objects */
  auto rpcMock = std::make_shared<RPCServer::MockRPCServer>();
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();
  auto angleMock = std::make_shared<IMUAngle::MockIMUAngle>();
  auto mathMock = std::make_shared<IMUMath::MockIMUAbstraction>();

  std::shared_ptr<IMUServer::IIMUServer> imuServer;
  imuServer = std::make_shared<IMUServer::IMURPCServer>(rpcMock, imuMock, angleMock, mathMock);

  /* Prepare local variables */
  auto funcAddUpdateDataCallback = [](std::function<void()> cb)
  {
    cb();
  };

  auto ExpectedAccelValue = 1;
  double accelValue = 0;
  auto funcGetRawAccelCallback = [&accelValue](std::function<double(int)> cb)
  {
    accelValue = cb(/* Axis */0);
    return RPCServer::eRPCError::eRET_OK;
  };

  auto ExpectedGyroValue = 2;
  double gyroValue = 0;
  auto funcGetRawGyroCallback = [&gyroValue](std::function<double(int)> cb)
  {
    gyroValue = cb(/* Axis */0);
    return RPCServer::eRPCError::eRET_OK;
  };

  auto ExpectedEulerAngleValue = 3;
  double eulerAngleValue = 0;
  auto funcGetEulerAngleCallback = [&eulerAngleValue](std::function<double(int, int)> cb)
  {
    eulerAngleValue = cb(/* Axis */0, /* Unit */0);
    return RPCServer::eRPCError::eRET_OK;
  };

  auto ExpectedComplFilterAngleValue = 4;
  double complFilterAngleValue = 0;
  auto funcGetComplFilterAngleCallback = [&complFilterAngleValue](std::function<double(int, int)> cb)
  {
    complFilterAngleValue = cb(/* Axis */0, /* Unit */0);
    return RPCServer::eRPCError::eRET_OK;
  };

  /* Prepare mock env */
  EXPECT_CALL(*rpcMock, Init())
    .Times(1)
    .WillRepeatedly(Return(RPCServer::eRPCError::eRET_OK));

  EXPECT_CALL(*rpcMock, setGetRawAccelCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*rpcMock, setGetRawGyroCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*rpcMock, setGetEulerAngleCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*rpcMock, setGetComplFilterAngleCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*rpcMock, NotifyDataUpdate())
    .Times(1)
    .WillRepeatedly(Return(RPCServer::eRPCError::eRET_OK));

  EXPECT_CALL(*rpcMock, DeInit())
    .Times(1);

  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*imuMock, GetRawAccel(_,_))
    .Times(1)
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(ExpectedAccelValue),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
      )
    );

  EXPECT_CALL(*imuMock, GetRawGyro(_,_))
    .Times(1)
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(ExpectedGyroValue),
        Return(IMUAbstraction::eIMUAbstractionError::eRET_OK)
      )
    );

  EXPECT_CALL(*mathMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUMath::eIMUMathError::eRET_OK));

  EXPECT_CALL(*angleMock, GetEulerAngle(_,_,_))
    .Times(1)
    .WillRepeatedly(
      DoAll(
        SetArgReferee<0>(ExpectedEulerAngleValue),
        Return(IMUAngle::eIMUAngleError::eRET_OK)
      )
    );

  EXPECT_CALL(*mathMock, GetComplFilterAngle(_,_,_))
    .Times(1)
    .WillRepeatedly(
      DoAll(
        SetArgReferee<0>(ExpectedComplFilterAngleValue),
        Return(IMUMath::eIMUMathError::eRET_OK)
      )
    );

  ON_CALL(*rpcMock, setGetRawAccelCallback_rv(_))
    .WillByDefault(Invoke(funcGetRawAccelCallback));

  ON_CALL(*rpcMock, setGetRawGyroCallback_rv(_))
    .WillByDefault(Invoke(funcGetRawGyroCallback));

  ON_CALL(*rpcMock, setGetEulerAngleCallback_rv(_))
    .WillByDefault(Invoke(funcGetEulerAngleCallback));

  ON_CALL(*rpcMock, setGetComplFilterAngleCallback_rv(_))
    .WillByDefault(Invoke(funcGetComplFilterAngleCallback));

  ON_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .WillByDefault(Invoke(funcAddUpdateDataCallback));

  /* Perform test */
  auto ret = imuServer->StartServer();

  /* Check Results */
  EXPECT_EQ(ret, IMUServer::eIMUServerError::eRET_OK);

  /* Check Expected Values */
  EXPECT_EQ(ExpectedAccelValue, accelValue);
  EXPECT_EQ(ExpectedGyroValue, gyroValue);
  EXPECT_EQ(ExpectedEulerAngleValue, eulerAngleValue);
  EXPECT_EQ(ExpectedComplFilterAngleValue, complFilterAngleValue);
}

TEST(IMURPCServer, StartServer_RET_ERROR)
{
  /* Construct objects */
  auto rpcMock = std::make_shared<RPCServer::MockRPCServer>();
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();
  auto angleMock = std::make_shared<IMUAngle::MockIMUAngle>();
  auto mathMock = std::make_shared<IMUMath::MockIMUAbstraction>();

  std::shared_ptr<IMUServer::IIMUServer> imuServer;
  imuServer = std::make_shared<IMUServer::IMURPCServer>(rpcMock, imuMock, angleMock, mathMock);

  /* Prepare mock env */
  EXPECT_CALL(*rpcMock, Init())
    .Times(1)
    .WillRepeatedly(Return(RPCServer::eRPCError::eRET_ERROR));

  EXPECT_CALL(*rpcMock, DeInit())
    .Times(AtLeast(1));

  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_ERROR));

  EXPECT_CALL(*mathMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUMath::eIMUMathError::eRET_ERROR));

  /* Perform test */
  auto ret = imuServer->StartServer();

  /* Check Results */
  EXPECT_EQ(ret, IMUServer::eIMUServerError::eRET_ERROR);
}
