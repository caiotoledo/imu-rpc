set(PROGRAM_NAME imumath)

file(GLOB_RECURSE src_files "./*.cpp")
file(GLOB src_c "./../src/*.cpp")

package_gtest(${PROGRAM_NAME}
    SRC
      ${src_files} ${src_c}
    INCLUDE
      ./include/ ./../include/ ./mocks/
    LIB
      imuabstraction
)
