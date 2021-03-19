#include <DBusCxxServer.hpp>

#include <DBusTypes.hpp>
#include <LogInstance.hpp>

using namespace RPCServer;

DBusCxxServer::DBusCxxServer() : isConnected(false), object(nullptr)
{
}

bool DBusCxxServer::isInitialized()
{
  return this->isConnected;
}

eRPCError DBusCxxServer::Init(void)
{
  auto ret = eRPCError::eRET_OK;

  /* Already initialized */
  if (this->isInitialized())
  {
    return ret;
  }

  try
  {
    this->dispatcher = DBus::StandaloneDispatcher::create();
    this->conn = dispatcher->create_connection(DBus::BusType::SESSION);

    auto retConn = conn->request_name(DBusTypes::DBUS_NAME, DBUSCXX_NAME_FLAG_REPLACE_EXISTING);
    if (DBus::RequestNameResponse::PrimaryOwner != retConn)
    {
      LOGERROR("Connection Busy!");
      return eRPCError::eRET_ERROR;
    }
    this->object = conn->create_object(DBusTypes::DBUS_PATH);

    /* Server Successfully initialized */
    isConnected = true;
  }
  catch(std::shared_ptr<DBus::Error> e)
  {
    LOGERROR("%s", e->what());
    ret = eRPCError::eRET_ERROR;
  }

  return ret;
}

eRPCError DBusCxxServer::NotifyDataUpdate(void)
{
  auto ret = eRPCError::eRET_ERROR;

  if (this->isInitialized())
  {
    auto signal
      = this->conn->create_free_signal<void(void)>(DBusTypes::DBUS_PATH, DBusTypes::DBUS_NAME, DBusTypes::DBUS_SIGNAL_DATAUPDATE);

    signal->emit();
    this->conn->flush();

    ret = eRPCError::eRET_OK;
  }

  return ret;
}

eRPCError DBusCxxServer::setGetRawAccelCallback(std::function<double(int)> &&cb) {
  auto ret = eRPCError::eRET_OK;

  /* Check Connection */
  if (!this->isInitialized()) {
    return eRPCError::eRET_ERROR;
  }

  /* Assing the callback to the method */
  try {
    this->object->create_method<double(int)>(DBusTypes::DBUS_NAME, DBusTypes::DBUS_FUNC_GETRAWACCEL, cb);
  } catch(std::shared_ptr<DBus::Error> e) {
    LOGERROR("%s", e->what());
    ret = eRPCError::eRET_ERROR;
  }

  return ret;
}

eRPCError DBusCxxServer::setGetRawGyroCallback(std::function<double(int)> &&cb)
{
  auto ret = eRPCError::eRET_OK;

  /* Check Connection */
  if (!this->isInitialized()) {
    return eRPCError::eRET_ERROR;
  }

  /* Assing the callback to the method */
  try {
    this->object->create_method<double(int)>(DBusTypes::DBUS_NAME, DBusTypes::DBUS_FUNC_GETRAWGYRO, cb);
  } catch(std::shared_ptr<DBus::Error> e) {
    LOGERROR("%s", e->what());
    ret = eRPCError::eRET_ERROR;
  }

  return ret;
}

eRPCError DBusCxxServer::setGetEulerAngleCallback(std::function<double(int, int)> &&cb)
{
  auto ret = eRPCError::eRET_OK;

  /* Check Connection */
  if (!this->isInitialized()) {
    return eRPCError::eRET_ERROR;
  }

  /* Assing the callback to the method */
  try {
    this->object->create_method<double(int, int)>(DBusTypes::DBUS_NAME, DBusTypes::DBUS_FUNC_GETEULERANGLE, cb);
  } catch(std::shared_ptr<DBus::Error> e) {
    LOGERROR("%s", e->what());
    ret = eRPCError::eRET_ERROR;
  }

  return ret;
}

eRPCError DBusCxxServer::setGetComplFilterAngleCallback(std::function<double(int, int)> &&cb)
{
  auto ret = eRPCError::eRET_OK;

  /* Check Connection */
  if (!this->isInitialized()) {
    return eRPCError::eRET_ERROR;
  }

  /* Assing the callback to the method */
  try {
    this->object->create_method<double(int, int)>(DBusTypes::DBUS_NAME, DBusTypes::DBUS_FUNC_GETCOMPLFILTERANGLE, cb);
  } catch(std::shared_ptr<DBus::Error> e) {
    LOGERROR("%s", e->what());
    ret = eRPCError::eRET_ERROR;
  }

  return ret;
}

void DBusCxxServer::DeInit(void) {
  if (this->isInitialized()) {
    /* Reset DBus pointers */
    this->object.reset();
    this->conn.reset();
    this->dispatcher.reset();

    /* Reset connection flag */
    isConnected = false;
  }
}

DBusCxxServer::~DBusCxxServer() {
  this->DeInit();
}
