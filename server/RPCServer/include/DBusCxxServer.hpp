#pragma once

#include <dbus-cxx-0.12/dbus-cxx.h>

#include "IRPCServer.hpp"

namespace RPCServer
{

  class DBusCxxServer : public IRPCServer
  {
  private:
    bool isConnected = false;
    DBus::Dispatcher::pointer dispatcher = nullptr;
    DBus::Connection::pointer conn = nullptr;
    DBus::Object::pointer object = nullptr;

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
