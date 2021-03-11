set(PROGRAM_NAME safequeue)

file(GLOB_RECURSE src_test_cpp "./*.cpp")

package_gtest(${PROGRAM_NAME}
    SRC
      ${src_test_cpp}
    INCLUDE
      ./include/ ./../include/
    LIB
      gtest_mocks
)
