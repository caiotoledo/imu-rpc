# Server and Client IMU based on RPC Communication

[![Runtime Tests](https://github.com/caiotoledo/imu-rpc/workflows/Runtime%20Tests/badge.svg)](https://github.com/caiotoledo/imu-rpc/actions)
[![Stability Tests](https://github.com/caiotoledo/imu-rpc/workflows/Stability%20Tests/badge.svg)](https://github.com/caiotoledo/imu-rpc/actions)
[![Static Analysis](https://github.com/caiotoledo/imu-rpc/workflows/Static%20Analysis/badge.svg)](https://github.com/caiotoledo/imu-rpc/actions)
[![Unit Test](https://github.com/caiotoledo/imu-rpc/workflows/Unittesting/badge.svg)](https://github.com/caiotoledo/imu-rpc/actions)

[![codecov.io](https://codecov.io/github/caiotoledo/imu-rpc/coverage.svg?branch=master)](https://codecov.io/github/caiotoledo/imu-rpc?branch=master)

[![License](https://img.shields.io/badge/license-MIT_License-blue.svg?style=flat)](LICENSE.md)

Provides a library to interface with IMU using [Industrial IO](https://www.kernel.org/doc/html/v4.14/driver-api/iio/index.html) as a Daemon distributing it via [dbus-cxx](https://dbus-cxx.github.io/) (a [DBus](https://www.freedesktop.org/wiki/Software/dbus/) C++ Wrapper) to its clients.

## Code Documentation
https://caiotoledo.github.io/imu-rpc/

## Dependencies

- [DBus-cxx](https://dbus-cxx.github.io/) version 0.12
  - [sigc++](https://github.com/libsigcplusplus/libsigcplusplus)
  - [dbus](https://www.freedesktop.org/wiki/Software/dbus/)
- [CMake](https://cmake.org/) 3.8 or greater
- [Google Test](https://github.com/google/googletest) version 1.8.1 (Only for Unit Test)

## Build and Install Project

```bash
cd <path_project>
mkdir .build
cd .build
cmake ../
make
sudo make install
```

Default File Location Install:
- Binaries: `/usr/bin/`
- Headers Libraries: `/usr/include/`
- Shared Libraries: `/usr/lib/`

## Build and Execute Unit Test

Same steps as [Build and Install Project](#Build-and-Install-Project) but adding `-DBUILD_UNITTEST=1` in cmake command:

```bash
cd <path_project>
mkdir .build
cd .build
cmake -DBUILD_UNITTEST=1 ../
make
make test
```
