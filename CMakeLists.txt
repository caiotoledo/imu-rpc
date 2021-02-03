cmake_minimum_required (VERSION 3.8)
set(PROJECT_NAME imu-rpc)

project(${PROJECT_NAME})

# PROJECT VERSION
set(IMURPC_VERSION_MAJOR 0)
set(IMURPC_VERSION_MINOR 3)
set(IMURPC_VERSION_PATCH 0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -g0 -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -g0 -s")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g3")

# Suppress Warning for ARM build, for more info check:
# https://stackoverflow.com/questions/48149323/what-does-the-gcc-warning-project-parameter-passing-for-x-changed-in-gcc-7-1-m
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-psabi")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-psabi")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Add -fPIC option
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)
# Add DBus CXX Wrapper
pkg_check_modules(DBUSCXX dbus-cxx-1.0 REQUIRED)

# SERVER APPLICATION
add_subdirectory(server)

# CLIENT APPLICATION
add_subdirectory(client)

# Libraries Folder
add_subdirectory(lib)
