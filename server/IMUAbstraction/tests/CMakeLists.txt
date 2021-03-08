set(PROGRAM_NAME imuabstraction)

file(GLOB_RECURSE src_test_cpp "./*.cpp")
file(GLOB src_app_cpp "./../src/IMUStub.cpp")

package_gtest(${PROGRAM_NAME}
    SRC
      ${src_test_cpp} ${src_app_cpp}
    INCLUDE
      ./include/ ./../include/ ./mocks/
    LIB
      dbustypes
      gtest_mocks
)
