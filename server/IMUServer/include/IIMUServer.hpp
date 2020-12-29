#pragma once

#include <memory>

#include <IRPCServer.hpp>
#include <IIMUAbstraction.hpp>

namespace IMUServer
{

  enum class eIMUServerError {
    eRET_OK,
    eRET_ERROR,
  };

  class IIMUServer
  {
  public:
    IIMUServer() = default;

    /**
     * @brief Start IMU Server
     *
     * @return eIMUServerError Returns #eRET_OK when successful, ref #eIMUServerError
     */
    virtual eIMUServerError StartServer(void) = 0;

    /**
     * @brief Stop IMU Server
     */
    virtual void StopServer(void) = 0;

    virtual ~IIMUServer() = default;
  };

} // namespace IMUServer
