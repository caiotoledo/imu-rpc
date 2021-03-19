#pragma once

#include <vector>
#include <mutex>
#include <future>
#include <memory>

#include <dbus-cxx-2.0/dbus-cxx.h>

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
    std::shared_ptr<DBus::Dispatcher> dispatcher;
    std::shared_ptr<DBus::Connection> connection;
    std::shared_ptr<DBus::ObjectProxy> object;

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
