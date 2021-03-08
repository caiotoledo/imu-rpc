#include <DBusIMUClient.hpp>

#include <DBusTypes.hpp>
#include <LogInstance.hpp>

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
    std::lock_guard<std::mutex> lck(mtxSignalHandler);

    /* Finish all last futures */
    for (auto &fut : vecFutCallback)
    {
      fut.get();
    }
    vecFutCallback.clear();

    /* Notify callback */
    for(auto &cb : vecCallback)
    {
      auto fut = std::async(std::launch::async, [&cb]()
      {
        cb();
      });

      /* Store future */
      vecFutCallback.push_back(std::move(fut));
    }
    return DBus::HANDLED;
  };

  if (this->isInitialized())
  {
    auto signal
      = connection->create_signal_proxy(DBusTypes::DBUS_PATH, DBusTypes::DBUS_NAME, DBusTypes::DBUS_SIGNAL_DATAUPDATE);
    signalHandler = signal->signal_dbus_incoming().connect(funcSignalHandler);

    ret = eIMUError::eRET_OK;
  }

  return ret;
}

void DBusIMUClient::AddUpdateDataCallback(std::function<void()> &&cb)
{
  vecCallback.push_back(cb);
}

eIMUError DBusIMUClient::GetRawAccel(DBusTypes::eAxis axis, double &val)
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
eIMUError DBusIMUClient::GetRawGyro(DBusTypes::eAxis axis, double &val)
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

eIMUError DBusIMUClient::GetEulerAngle(DBusTypes::eAxis axis, DBusTypes::eAngleUnit unit, double &val)
{
  auto ret = eIMUError::eRET_OK;

  /* Check Connection */
  if (!this->isInitialized())
  {
    return eIMUError::eRET_ERROR;
  }

  /* Initialize Method Proxies */
  auto GetEulerAngle_proxy
    = *(this->object->create_method<double, int, int>(DBusTypes::DBUS_NAME,DBusTypes::DBUS_FUNC_GETEULERANGLE));

  /* Execute Method Requests */
  try
  {
    val = GetEulerAngle_proxy((int)axis, (int)unit);
  }
  catch(std::shared_ptr<DBus::Error> e)
  {
    LOGERROR("%s", e->what());
    ret = eIMUError::eRET_ERROR;
  }

  return ret;
}

eIMUError DBusIMUClient::GetComplFilterAngle(DBusTypes::eAxis axis, DBusTypes::eAngleUnit unit, double &val)
{
  auto ret = eIMUError::eRET_OK;

  /* Check Connection */
  if (!this->isInitialized())
  {
    return eIMUError::eRET_ERROR;
  }

  /* Initialize Method Proxies */
  auto GetEulerAngle_proxy
    = *(this->object->create_method<double, int, int>(DBusTypes::DBUS_NAME,DBusTypes::DBUS_FUNC_GETCOMPLFILTERANGLE));

  /* Execute Method Requests */
  try
  {
    val = GetEulerAngle_proxy((int)axis, (int)unit);
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
  /* Disconnect DBus Signal */
  signalHandler.disconnect();

  /* Avoid deadlock by calling callbacks during DeInitialization */
  std::lock_guard<std::mutex> lck(mtxSignalHandler);
  for (auto &fut : vecFutCallback)
  {
    fut.get();
  }
  vecFutCallback.clear();
  vecCallback.clear();

  /* Disconnect DBus */
  if (this->isInitialized())
  {
    /* Close Dispatcher connection */
    this->dispatcher->stop();

    /* Reset DBus pointers */
    this->object.reset();
    this->connection.reset();
    this->dispatcher.reset();

    /* Reset connection flag */
    isConnected = false;
  }
}

DBusIMUClient::~DBusIMUClient()
{
  this->DeInit();
}
