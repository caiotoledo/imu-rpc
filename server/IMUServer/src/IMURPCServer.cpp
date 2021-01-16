#include <IMURPCServer.hpp>

#include <LogInstance.h>

using namespace IMUServer;

IMURPCServer::IMURPCServer(std::shared_ptr<RPCServer::IRPCServer> server,
                           std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu,
                           std::shared_ptr<IMUMath::IIMUMath> math) :
instanceServer(server),
instanceImu(imu),
instanceMath(math)
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
  auto retMath = static_cast<int>(instanceMath->Init());
  auto retInstance = retImu + retServer + retMath;

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

eIMUServerError IMURPCServer::InitGetEulerAngle(void)
{
  auto ret = eIMUServerError::eRET_OK;

  auto func = [this](int axis, int unit)
  {
    double val = 0;
    instanceMath->GetEulerAngle(val, (IMUAbstraction::eAxis)axis, (IMUMath::eAngleUnit)unit);
    return val;
  };
  auto retServer = instanceServer->setGetEulerAngleCallback(func);
  ret = (retServer == RPCServer::eRPCError::eRET_OK) ? eIMUServerError::eRET_OK : eIMUServerError::eRET_ERROR;

  return ret;
}

eIMUServerError IMURPCServer::InitServer()
{
  auto ret = eIMUServerError::eRET_OK;

  auto retAccel = static_cast<int>(this->InitGetRawAccel());
  auto retGyro =  static_cast<int>(this->InitGetRawGyro());
  auto retAngle = static_cast<int>(this->InitGetEulerAngle());
  auto retInit = retAccel + retGyro + retAngle;

  if (retInit != 0)
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
