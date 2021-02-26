#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MockIMUAbstraction.hpp>
#include <MockIMUMath.hpp>
#include <MockRPCServer.hpp>

#include <IMURPCServer.hpp>

using ::testing::_;
using ::testing::Return;
using ::testing::AtLeast;

TEST(IMURPCServer, StartServer)
{
  /* Construct objects */
  auto rpcMock = std::make_shared<RPCServer::MockRPCServer>();
  auto imuMock = std::make_shared<IMUAbstraction::MockIMUAbstraction>();
  auto mathMock = std::make_shared<IMUMath::MockIMUAbstraction>();

  std::shared_ptr<IMUServer::IIMUServer> imuServer;
  imuServer = std::make_shared<IMUServer::IMURPCServer>(rpcMock, imuMock, mathMock);

  /* Prepare mock env */
  EXPECT_CALL(*rpcMock, Init())
    .Times(1)
    .WillRepeatedly(Return(RPCServer::eRPCError::eRET_OK));

  EXPECT_CALL(*rpcMock, setGetRawAccelCallback_rv(_))
    .Times(1)
    .WillRepeatedly(Return(RPCServer::eRPCError::eRET_OK));

  EXPECT_CALL(*rpcMock, setGetRawGyroCallback_rv(_))
    .Times(1)
    .WillRepeatedly(Return(RPCServer::eRPCError::eRET_OK));

  EXPECT_CALL(*rpcMock, setGetEulerAngleCallback_rv(_))
    .Times(1)
    .WillRepeatedly(Return(RPCServer::eRPCError::eRET_OK));

  EXPECT_CALL(*rpcMock, setGetComplFilterAngleCallback_rv(_))
    .Times(1)
    .WillRepeatedly(Return(RPCServer::eRPCError::eRET_OK));

  EXPECT_CALL(*rpcMock, DeInit())
    .Times(1);

  EXPECT_CALL(*imuMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUAbstraction::eIMUAbstractionError::eRET_OK));

  EXPECT_CALL(*imuMock, AddUpdateDataCallback_rv(_))
    .Times(1);

  EXPECT_CALL(*mathMock, Init())
    .Times(1)
    .WillRepeatedly(Return(IMUMath::eIMUMathError::eRET_OK));

  /* Perform test */
  auto ret = imuServer->StartServer();

  /* Check Results */
  EXPECT_EQ(ret, IMUServer::eIMUServerError::eRET_OK);
}
