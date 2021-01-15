#include <DBusIMUClient.hpp>

#include <DBusTypes.hpp>
#include <LogInstance.h>

using namespace IMUClient;

DBusIMUClient::DBusIMUClient() :
  isConnected(false),
  dispatcher(nullptr),
  connection(nullptr),
  object(nullptr)
{}

bool DBusIMUClient::isInitialized()
{
  return this->isConnected;
}

eIMUError DBusIMUClient::Init(void)
{
  auto ret = eIMUError::eRET_OK;

  /* Already initialized */
  if (this->isInitialized())
  {
    return ret;
  }

  try
  {
    DBus::init();

    dispatcher = DBus::Dispatcher::create();
    connection = dispatcher->create_connection(DBus::BUS_SESSION);
    object = connection->create_object_proxy(DBusTypes::DBUS_NAME, DBusTypes::DBUS_PATH);
    isConnected = true;

    ret = this->InitSignalHandler();
  }
  catch(std::shared_ptr<DBus::Error> e)
  {
    LOGERROR("%s", e->what());
    ret = eIMUError::eRET_ERROR;
  }

  return ret;
}

eIMUError DBusIMUClient::InitSignalHandler(void)
{
  auto ret = eIMUError::eRET_ERROR;

  auto funcSignalHandler = [this](DBus::SignalMessage::const_pointer msg)
  {
    for(auto &cb : vecCallback)
    {
      cb();
    }
    return DBus::HANDLED;
  };

  if (this->isInitialized())
  {
    auto signal
      = connection->create_signal_proxy(DBusTypes::DBUS_PATH, DBusTypes::DBUS_NAME, DBusTypes::DBUS_SIGNAL_DATAUPDATE);
    signal->signal_dbus_incoming().connect(funcSignalHandler);

    ret = eIMUError::eRET_OK;
  }

  return ret;
}

void DBusIMUClient::AddUpdateDataCallback(std::function<void()> &&cb)
{
  vecCallback.push_back(cb);
}

eIMUError DBusIMUClient::GetRawAccel(eAxis axis, double &val)
{
  auto ret = eIMUError::eRET_OK;

  /* Check Connection */
  if (!this->isInitialized())
  {
    return eIMUError::eRET_ERROR;
  }

  /* Initialize Method Proxies */
  auto GetRawAccel_proxy
    = *(this->object->create_method<double, int>(DBusTypes::DBUS_NAME,DBusTypes::DBUS_FUNC_GETRAWACCEL));

  /* Execute Method Requests */
  try
  {
    val = GetRawAccel_proxy((int)axis);
  }
  catch(std::shared_ptr<DBus::Error> e)
  {
    LOGERROR("%s", e->what());
    ret = eIMUError::eRET_ERROR;
  }

  return ret;
}
eIMUError DBusIMUClient::GetRawGyro(eAxis axis, double &val)
{
  auto ret = eIMUError::eRET_OK;

  /* Check Connection */
  if (!this->isInitialized())
  {
    return eIMUError::eRET_ERROR;
  }

  /* Initialize Method Proxies */
  auto GetRawGyro_proxy
    = *(this->object->create_method<double, int>(DBusTypes::DBUS_NAME,DBusTypes::DBUS_FUNC_GETRAWGYRO));

  /* Execute Method Requests */
  try
  {
    val = GetRawGyro_proxy((int)axis);
  }
  catch(std::shared_ptr<DBus::Error> e)
  {
    LOGERROR("%s", e->what());
    ret = eIMUError::eRET_ERROR;
  }

  return ret;
}

void DBusIMUClient::DeInit(void)
{
  if (this->isInitialized())
  {
    /* Close Dispatcher connection */
    this->dispatcher->stop();

    /* Reset DBus pointers */
    this->object = nullptr;
    this->connection = nullptr;
    this->dispatcher = nullptr;

    /* Reset connection flag */
    isConnected = false;
  }
}

DBusIMUClient::~DBusIMUClient()
{
  this->DeInit();
}