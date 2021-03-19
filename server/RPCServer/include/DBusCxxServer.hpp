#pragma once

#include <memory>

#include <dbus-cxx-2.0/dbus-cxx.h>

#include "IRPCServer.hpp"

namespace RPCServer
{

  class DBusCxxServer : public IRPCServer
  {
  private:
    bool isConnected = false;
    std::shared_ptr<DBus::Dispatcher> dispatcher;
    std::shared_ptr<DBus::Connection> conn;
    std::shared_ptr<DBus::Object> object;

    bool isInitialized();

  public:
    DBusCxxServer();

    virtual eRPCError Init(void) override;
    virtual eRPCError NotifyDataUpdate(void) override;
    virtual eRPCError setGetRawAccelCallback(std::function<double(int)> &&cb) override;
    virtual eRPCError setGetRawGyroCallback(std::function<double(int)> &&cb) override;
    virtual eRPCError setGetEulerAngleCallback(std::function<double(int, int)> &&cb) override;
    virtual eRPCError setGetComplFilterAngleCallback(std::function<double(int, int)> &&cb) override;
    virtual void DeInit(void) override;

    virtual ~DBusCxxServer();
  };

} // namespace RPCServer
