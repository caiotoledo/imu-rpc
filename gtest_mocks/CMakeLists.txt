set(LIBRARY_NAME gtest_mocks)
set(inc_dir "./include/")
set(mocks_dir "./mocks/")

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE "${inc_dir}")
target_include_directories(${LIBRARY_NAME} INTERFACE "${mocks_dir}")
