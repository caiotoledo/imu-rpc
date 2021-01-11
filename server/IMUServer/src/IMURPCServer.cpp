#include <IMURPCServer.hpp>

#include <LogInstance.h>

using namespace IMUServer;

IMURPCServer::IMURPCServer(std::shared_ptr<RPCServer::IRPCServer> server,
                           std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu) :
instanceServer(server),
instanceImu(imu)
{
  auto cbIMU = [this]()
  {
    auto ret = instanceServer->NotifyDataUpdate();
    if (ret != RPCServer::eRPCError::eRET_OK)
    {
      LOGWARN("Updated Data Notification Failed [%d]", ((int)ret));
    }
  };

  imu->AddUpdateDataCallback(cbIMU);
}

eIMUServerError IMURPCServer::StartServer(void)
{
  auto ret = eIMUServerError::eRET_ERROR;

  /* Initialize Instances */
  auto retImu = static_cast<int>(instanceImu->Init());
  auto retServer = static_cast<int>(instanceServer->Init());
  auto retInstance = retImu + retServer;

  if (retInstance != 0)
  {
    LOGERROR("Failed Server initialization [%d]", retServer);
    ret = eIMUServerError::eRET_ERROR;
  }
  else
  {
    ret = this->InitServer();
  }

  if (ret != eIMUServerError::eRET_OK)
  {
    this->StopServer();
  }

  return ret;
}

eIMUServerError IMURPCServer::InitGetRawAccel(void)
{
  auto ret = eIMUServerError::eRET_OK;

  auto func = [this](int axis)
  {
    double val = 0;
    instanceImu->GetRawAccel((IMUAbstraction::eAxis)axis, val);
    return val;
  };
  auto retServer = instanceServer->setGetRawAccelCallback(func);
  ret = (retServer == RPCServer::eRPCError::eRET_OK) ? eIMUServerError::eRET_OK : eIMUServerError::eRET_ERROR;

  return ret;
}
eIMUServerError IMURPCServer::InitGetRawGyro(void)
{
  auto ret = eIMUServerError::eRET_OK;

  auto func = [this](int axis)
  {
    double val = 0;
    instanceImu->GetRawGyro((IMUAbstraction::eAxis)axis, val);
    return val;
  };
  auto retServer = instanceServer->setGetRawGyroCallback(func);
  ret = (retServer == RPCServer::eRPCError::eRET_OK) ? eIMUServerError::eRET_OK : eIMUServerError::eRET_ERROR;

  return ret;
}

eIMUServerError IMURPCServer::InitServer()
{
  auto ret = eIMUServerError::eRET_OK;

  auto ret1 = this->InitGetRawAccel();
  auto ret2 = this->InitGetRawGyro();

  if (ret1 != eIMUServerError::eRET_OK || ret2 != eIMUServerError::eRET_OK)
  {
    ret = eIMUServerError::eRET_ERROR;
  }

  return ret;
}

void IMURPCServer::StopServer(void)
{
  instanceServer->DeInit();
}

IMURPCServer::~IMURPCServer()
{
  this->StopServer();
}
