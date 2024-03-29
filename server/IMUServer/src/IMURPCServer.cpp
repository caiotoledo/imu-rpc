#include <IMURPCServer.hpp>

#include <LogInstance.hpp>

using namespace IMUServer;

IMURPCServer::IMURPCServer(std::shared_ptr<RPCServer::IRPCServer> server,
                           std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu,
                           std::shared_ptr<IMUAngle::IIMUAngle> angle,
                           std::shared_ptr<IMUMath::IIMUMath> math) :
instanceServer(server),
instanceImu(imu),
instanceAngle(angle),
instanceMath(math)
{
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
    auto cbIMU = [this]()
    {
      auto ret = instanceServer->NotifyDataUpdate();
      if (ret != RPCServer::eRPCError::eRET_OK)
      {
        LOGWARN("Updated Data Notification Failed [%d]", ((int)ret));
      }
    };
    instanceImu->AddUpdateDataCallback(cbIMU);

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
    instanceImu->GetRawAccel((DBusTypes::eAxis)axis, val);
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
    instanceImu->GetRawGyro((DBusTypes::eAxis)axis, val);
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
    instanceAngle->GetEulerAngle(val, (DBusTypes::eAxis)axis, (DBusTypes::eAngleUnit)unit);
    return val;
  };
  auto retServer = instanceServer->setGetEulerAngleCallback(func);
  ret = (retServer == RPCServer::eRPCError::eRET_OK) ? eIMUServerError::eRET_OK : eIMUServerError::eRET_ERROR;

  return ret;
}

eIMUServerError IMURPCServer::InitGetComplFilterAngle(void)
{
  auto ret = eIMUServerError::eRET_OK;

  auto func = [this](int axis, int unit)
  {
    double val = 0;
    instanceMath->GetComplFilterAngle(val, (DBusTypes::eAxis)axis, (DBusTypes::eAngleUnit)unit);
    return val;
  };
  auto retServer = instanceServer->setGetComplFilterAngleCallback(func);
  ret = (retServer == RPCServer::eRPCError::eRET_OK) ? eIMUServerError::eRET_OK : eIMUServerError::eRET_ERROR;

  return ret;
}

eIMUServerError IMURPCServer::InitServer()
{
  auto ret = eIMUServerError::eRET_OK;

  auto retAccel =       static_cast<int>(this->InitGetRawAccel());
  auto retGyro =        static_cast<int>(this->InitGetRawGyro());
  auto retEulerAngle =  static_cast<int>(this->InitGetEulerAngle());
  auto retComplAngle =  static_cast<int>(this->InitGetComplFilterAngle());
  auto retInit = retAccel + retGyro + retEulerAngle + retComplAngle;

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
