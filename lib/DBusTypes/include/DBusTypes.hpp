#pragma once

namespace DBusTypes
{

  enum class eAxis
  {
    X,
    Y,
    Z
  };

  enum class eAngleUnit
  {
    eRadians,
    eDegrees,
  };

  constexpr const char *DBUS_NAME = "imu.daemon";
  constexpr const char *DBUS_PATH = "/imu/server";

  constexpr const char *DBUS_FUNC_GETRAWACCEL = "GetRawAccel";
  constexpr const char *DBUS_FUNC_GETRAWGYRO = "GetRawGyro";
  constexpr const char *DBUS_FUNC_GETEULERANGLE = "GetEulerAngle";
  constexpr const char *DBUS_FUNC_GETCOMPLFILTERANGLE = "GetComplFilterAngle";

  constexpr const char *DBUS_SIGNAL_DATAUPDATE = "DataUpdate";

} // namespace DBusTypes
