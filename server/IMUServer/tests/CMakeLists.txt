set(PROGRAM_NAME imuserver)

file(GLOB_RECURSE src_test_cpp "./*.cpp")
file(GLOB src_app_cpp "./../src/*.cpp")

package_gtest(${PROGRAM_NAME}
    SRC
      ${src_test_cpp} ${src_app_cpp}
    INCLUDE
      ./include/ ./../include/ ./mocks/
    LIB
      imuabstraction
      imuangle
      imumath
      rpcserver
      gtest_mocks
)
