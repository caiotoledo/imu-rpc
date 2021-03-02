cmake_minimum_required (VERSION 3.8)
set(PROJECT_NAME imu-rpc)

project(${PROJECT_NAME})

# PROJECT VERSION
set(IMURPC_VERSION_MAJOR 0)
set(IMURPC_VERSION_MINOR 4)
set(IMURPC_VERSION_PATCH 0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -g0 -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -g0 -s")

set(COVERAGE_FLAGS "-coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g3 ${COVERAGE_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g3 ${COVERAGE_FLAGS}")

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

include (CTest)
# Macro for Google Test
macro(package_gtest TESTNAME)
  cmake_parse_arguments(
    INPUT_ARG
    ""
    ""
    "SRC;INCLUDE;LIB"
    ${ARGN}
  )
  # Execute only for host compilation
  if(NOT CMAKE_CROSSCOMPILING AND BUILD_UNITTEST)
    find_package(GTest REQUIRED)
    set(TEST_PROGRAM test_${TESTNAME})
    message(STATUS "Adding google test: ${TESTNAME}")
    add_executable(${TEST_PROGRAM} ${INPUT_ARG_SRC})
    target_include_directories(${TEST_PROGRAM} PRIVATE ${GTEST_INCLUDE_DIRS} ${INPUT_ARG_INCLUDE})
    target_link_libraries(${TEST_PROGRAM} PRIVATE ${INPUT_ARG_LIB} ${GTEST_LIBRARIES} gtest_main gmock gmock_main pthread)
    # Add test to be run in ctest
    add_test(NAME ${TESTNAME}
      COMMAND ${TEST_PROGRAM}
      WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    # Set unit test timeout in seconds
    if(${TIMEOUT_UNITTEST})
      set_tests_properties(${TESTNAME} PROPERTIES TIMEOUT ${TIMEOUT_UNITTEST})
    endif()
  endif()
endmacro(package_gtest)

# SERVER APPLICATION
add_subdirectory(server)

# CLIENT APPLICATION
add_subdirectory(client)

# Libraries Folder
add_subdirectory(lib)
