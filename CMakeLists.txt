cmake_minimum_required (VERSION 3.8)
set(PROJECT_NAME imu-rpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g3")

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
