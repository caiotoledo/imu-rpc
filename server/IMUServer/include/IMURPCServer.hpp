#pragma once

#include <memory>

#include <IIMUAbstraction.hpp>
#include <IIMUMath.hpp>

#include "IIMUServer.hpp"

namespace IMUServer
{

  class IMURPCServer : public IIMUServer
  {
  private:
    std::shared_ptr<RPCServer::IRPCServer> instanceServer;
    std::shared_ptr<IMUAbstraction::IIMUAbstraction> instanceImu;
    std::shared_ptr<IMUMath::IIMUMath> instanceMath;

    eIMUServerError InitServer(void);
    eIMUServerError InitGetRawAccel(void);
    eIMUServerError InitGetRawGyro(void);
    eIMUServerError InitGetEulerAngle(void);
    eIMUServerError InitGetComplFilterAngle(void);

  public:
    IMURPCServer(
      std::shared_ptr<RPCServer::IRPCServer> server,
      std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu,
      std::shared_ptr<IMUMath::IIMUMath> math);

    /**
     * @brief Start IMU Server
     *
     * @return eIMUServerError Returns #eRET_OK when successful, ref #eIMUServerError
     */
    virtual eIMUServerError StartServer(void) override;

    /**
     * @brief Stop IMU Server
     */
    virtual void StopServer(void) override;

    virtual ~IMURPCServer();
  };

} // namespace IMUServer
