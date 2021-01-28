cmake_minimum_required (VERSION 3.8)
set(LIBRARY_NAME sockethandler)

set(inc_dir "./include/")
set(src_dir "./src/")
file(GLOB src_files "${src_dir}/*.cpp")

add_library(${LIBRARY_NAME} STATIC ${src_files})

target_include_directories(${LIBRARY_NAME} PRIVATE ${src_dir})
target_include_directories(${LIBRARY_NAME} PUBLIC ${inc_dir})

target_link_libraries(${LIBRARY_NAME} PRIVATE logger)
target_link_libraries(${LIBRARY_NAME} PRIVATE pthread)
