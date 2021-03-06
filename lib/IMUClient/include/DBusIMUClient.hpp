#pragma once

#include <vector>
#include <mutex>
#include <future>

#include <dbus-cxx-0.12/dbus-cxx.h>

#include "IIMUClient.hpp"

namespace IMUClient
{

  class DBusIMUClient : public IIMUClient
  {
  private:
    std::mutex mtxSignalHandler;
    sigc::connection signalHandler;
    std::vector<std::function<void()>> vecCallback;
    std::vector<std::future<void>> vecFutCallback;

    bool isConnected = false;
    DBus::Dispatcher::pointer dispatcher = nullptr;
    DBus::Connection::pointer connection = nullptr;
    DBus::ObjectProxy::pointer object = nullptr;

    bool isInitialized(void);
    eIMUError InitSignalHandler(void);
  public:
    DBusIMUClient();

    virtual eIMUError Init(void) override;
    virtual void AddUpdateDataCallback(std::function<void()> &&cb) override;
    virtual eIMUError GetRawAccel(DBusTypes::eAxis axis, double &val) override;
    virtual eIMUError GetRawGyro(DBusTypes::eAxis axis, double &val) override;
    virtual eIMUError GetEulerAngle(DBusTypes::eAxis axis, DBusTypes::eAngleUnit unit, double &val) override;
    virtual eIMUError GetComplFilterAngle(DBusTypes::eAxis axis, DBusTypes::eAngleUnit unit, double &val) override;
    virtual void DeInit(void) override;

    virtual ~DBusIMUClient();
  };

} // namespace RPCClient
