set(PROGRAM_NAME imuangle)

file(GLOB_RECURSE src_test_cpp "./*.cpp")
file(GLOB src_app_cpp "./../src/*.cpp")

package_gtest(${PROGRAM_NAME}
    SRC
      ${src_test_cpp} ${src_app_cpp}
    INCLUDE
      ./include/ ./../include/ ./mocks/
    LIB
      imuabstraction
      gtest_mocks
)
